
/*
  nano_modbus_slave_as7341_minimal.cpp

  Arduino Nano + MAX485 as Modbus RTU slave + DFRobot AS7341 spectral sensor.

  - Slave ID: 50
  - Baud: 9600, 8N1
  - RS485 transceiver: MAX485 (TX-enable pin on D2)
  - Uses:
      * ModbusRtu library (Modbus-Master-Slave-for-Arduino by smarmengol)
      * DFRobot_AS7341 library for the 11-channel visible light sensor

  Functionality:
  -------------
  Exposes a block of holding registers starting at address 0.

  Holding registers (FC 0x03, 0-based addresses):
  ------------------------------------------------
    Addr  400xx   Description
    0     40001   LED control word (D4):
                  0 = DF Robot LED OFF
                  !=0 = DF Robot LED ON

    1     40002   Relay control word (D6):
                  0 = Relay OFF
                  !=0 = Relay ON

    2     40003   AS7341 F1 (405–425 nm)
    3     40004   AS7341 F2 (435–455 nm)
    4     40005   AS7341 F3 (470–490 nm)
    5     40006   AS7341 F4 (505–525 nm)
    6     40007   AS7341 F5 (545–565 nm)
    7     40008   AS7341 F6 (580–600 nm)
    8     40009   AS7341 F7 (620–640 nm)
    9     40010   AS7341 F8 (670–690 nm)
    10    40011   AS7341 CLEAR channel
    11    40012   AS7341 NIR channel
    12    40013   AS7341 status word
                  0 = OK
                  1 = init failed
                  2 = read error (reserved)

  IMPORTANT:
  ----------
  All sensor access is done in a non-blocking state machine so that Modbus
  slave.poll() can run frequently and respond reliably.
*/

#include <Wire.h>           // I2C for AS7341
#include <ModbusRtu.h>      // Modbus RTU slave library
#include "DFRobot_AS7341.h" // AS7341 spectral sensor library

// ----------------------------
// RS485 / Modbus configuration
// ----------------------------

static const uint8_t MODBUS_SLAVE_ID = 50;
static const uint8_t RS485_TXEN_PIN  = 2;  // connect to RE+DE on MAX485

// I/O pins for external devices
static const uint8_t LED_MODULE_PIN  = 4;  // DF Robot LED module control (D4)
static const uint8_t RELAY_PIN       = 6;  // Relay module control (D6)

// ----------------------------
// Modbus register map
// ----------------------------

enum ModbusRegisterIndex : uint8_t {
  REG_LED_CONTROL = 0,        // 0: 40001 - LED control (0=OFF, !=0=ON, D4)
  REG_RELAY_CONTROL,          // 1: 40002 - Relay control (0=OFF, !=0=ON, D6)

  REG_AS7341_F1,              // 2: 40003
  REG_AS7341_F2,              // 3: 40004
  REG_AS7341_F3,              // 4: 40005
  REG_AS7341_F4,              // 5: 40006
  REG_AS7341_F5,              // 6: 40007
  REG_AS7341_F6,              // 7: 40008
  REG_AS7341_F7,              // 8: 40009
  REG_AS7341_F8,              // 9: 40010
  REG_AS7341_CLEAR,           // 10: 40011
  REG_AS7341_NIR,             // 11: 40012
  REG_AS7341_STATUS_WORD,     // 12: 40013 (0=OK,1=init fail,2=read error)

  REG_COUNT                   // 13 (not a register, just count)
};

// Holding registers exposed via Modbus (0..REG_COUNT-1)
uint16_t au16data[REG_COUNT];

// Create Modbus slave object:
//   - MODBUS_SLAVE_ID: our node id
//   - 0              : serial port index 0 -> Serial
//   - RS485_TXEN_PIN : DE/RE pin for RS485
Modbus slave(MODBUS_SLAVE_ID, 0, RS485_TXEN_PIN);

// ----------------------------
// AS7341 sensor object & state
// ----------------------------

DFRobot_AS7341 as7341;
bool g_as7341_ok = false;   // true if sensor initialised OK

// State machine for non-blocking AS7341 sampling
enum As7341State : uint8_t {
  AS_IDLE = 0,
  AS_WAIT_F1F4,
  AS_WAIT_F5F8
};

As7341State g_as_state = AS_IDLE;

// ----------------------------
// Timing
// ----------------------------

const unsigned long AS7341_PERIOD_MS     = 5000UL;  // new full set every 5s
const unsigned long AS7341_INIT_RETRY_MS = 2000UL;  // retry init every 2s

unsigned long lastAs7341CycleStartMs     = 0;
unsigned long lastAs7341InitAttemptMs    = 0;

// ----------------------------
// AS7341 helper functions
// ----------------------------

/**
 * @brief Try to initialise the AS7341 sensor once (non-blocking pattern).
 *
 * Sets:
 *   g_as7341_ok                       : true if begin() succeeds
 *   au16data[REG_AS7341_STATUS_WORD]  : 0 on success, 1 on failure
 */
void tryInitAs7341()
{
  lastAs7341InitAttemptMs = millis();

  // Use default mode (eSpm). This call itself does I2C traffic but returns quickly.
  if (as7341.begin() == 0) {
    g_as7341_ok = true;

    // Adjust integration time and gain for a balance of speed and resolution.
    as7341.setAtime(29);   // shorter integration
    as7341.setAstep(299);  // fewer steps -> shorter overall time
    as7341.setAGAIN(3);    // modest gain (X4)

    au16data[REG_AS7341_STATUS_WORD] = 0; // OK
  } else {
    g_as7341_ok = false;
    au16data[REG_AS7341_STATUS_WORD] = 1; // init failed
  }
}

/**
 * @brief Non-blocking AS7341 state machine "tick".
 *
 * This is called every loop() iteration. It:
 *   - Occasionally tries to init the sensor if not ready.
 *   - Every AS7341_PERIOD_MS, kicks off a full measurement cycle.
 *   - Uses measureComplete() to know when to read data, without blocking.
 */
void tickAs7341()
{
  unsigned long now = millis();

  // 1) Handle initialisation & retry if needed.
  if (!g_as7341_ok) {
    // Only attempt init every AS7341_INIT_RETRY_MS to avoid hammering the bus.
    if (now - lastAs7341InitAttemptMs >= AS7341_INIT_RETRY_MS) {
      tryInitAs7341();
    }
    // If still not OK, nothing else to do.
    if (!g_as7341_ok) {
      return;
    }
  }

  // 2) Run measurement state machine.
  switch (g_as_state) {

    case AS_IDLE:
      // Start a new full measurement cycle every AS7341_PERIOD_MS.
      if (now - lastAs7341CycleStartMs >= AS7341_PERIOD_MS) {
        lastAs7341CycleStartMs = now;

        // Kick off F1-F4 + Clear/NIR measurement.
        as7341.startMeasure(as7341.eF1F4ClearNIR);
        g_as_state = AS_WAIT_F1F4;
      }
      break;

    case AS_WAIT_F1F4:
      // Poll until measurement is complete (non-blocking).
      if (as7341.measureComplete()) {
        // Read mode-one data: F1..F4, CLEAR, NIR
        DFRobot_AS7341::sModeOneData_t data1 = as7341.readSpectralDataOne();

        au16data[REG_AS7341_F1]    = data1.ADF1;
        au16data[REG_AS7341_F2]    = data1.ADF2;
        au16data[REG_AS7341_F3]    = data1.ADF3;
        au16data[REG_AS7341_F4]    = data1.ADF4;
        // CLEAR/NIR will be overwritten by the second mode so they stay in sync.

        // Now start the second half: F5-F8 + Clear/NIR.
        as7341.startMeasure(as7341.eF5F8ClearNIR);
        g_as_state = AS_WAIT_F5F8;
      }
      break;

    case AS_WAIT_F5F8:
      if (as7341.measureComplete()) {
        DFRobot_AS7341::sModeTwoData_t data2 = as7341.readSpectralDataTwo();

        au16data[REG_AS7341_F5]    = data2.ADF5;
        au16data[REG_AS7341_F6]    = data2.ADF6;
        au16data[REG_AS7341_F7]    = data2.ADF7;
        au16data[REG_AS7341_F8]    = data2.ADF8;
        au16data[REG_AS7341_CLEAR] = data2.ADCLEAR;
        au16data[REG_AS7341_NIR]   = data2.ADNIR;

        au16data[REG_AS7341_STATUS_WORD] = 0; // measurement OK
        g_as_state = AS_IDLE;
      }
      break;

    default:
      g_as_state = AS_IDLE;
      break;
  }
}

// ----------------------------
// Setup
// ----------------------------

void setup()
{
  // Start UART for Modbus
  Serial.begin(9600);

  // I2C for AS7341
  Wire.begin();

  // External DF Robot LED module control pin on D4
  pinMode(LED_MODULE_PIN, OUTPUT);
  digitalWrite(LED_MODULE_PIN, LOW);  // default OFF

  // Relay module on D6
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);       // default OFF

  // Init control registers to OFF
  au16data[REG_LED_CONTROL]   = 0;
  au16data[REG_RELAY_CONTROL] = 0;

  // Clear sensor-related registers
  for (uint8_t i = REG_AS7341_F1; i < REG_COUNT; ++i) {
    au16data[i] = 0;
  }

  // Initial attempt to init AS7341
  lastAs7341InitAttemptMs = millis();
  tryInitAs7341();

  // Start Modbus RTU slave at 9600 8N1 on Serial
  slave.begin(9600);
}

// ----------------------------
// Main loop
// ----------------------------

void loop()
{
  // 1) Non-blocking AS7341 state machine
  tickAs7341();

  // 2) Handle Modbus requests (read/write holding registers)
  slave.poll(au16data, REG_COUNT);

  // 3) Apply LED control (40001) from holding register 0
  bool ledModuleOn = (au16data[REG_LED_CONTROL] != 0);
  digitalWrite(LED_MODULE_PIN, ledModuleOn ? HIGH : LOW);

  // 4) Apply relay control (40002) from holding register 1
  bool relayOn = (au16data[REG_RELAY_CONTROL] != 0);
  digitalWrite(RELAY_PIN, relayOn ? HIGH : LOW);
}
