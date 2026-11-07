#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// If it is missing, we simply skip the manual TX-power setting.
#if __has_include(<esp_gap_ble_api.h>)
  #include <esp_gap_ble_api.h>
  #define HAS_BLE_TX_POWER 1
#else
  #define HAS_BLE_TX_POWER 0
#endif

// ============================================================
// KidsZee Car - ESP32-C3 + DRV8833 + BLE   (18650 version)
// Compatible with DPad_Controller_Interchanged.html
//
// GPIO4/5 = Drive motor
// GPIO6/7 = Steering motor
//
// BLE commands (newline terminated):
//   J,x,y  x = steering (-100..100), y = throttle (-100..100)
//   V,n    speed limit (0..100)
//   S      immediate stop

#define DRIVE_IN1  4
#define DRIVE_IN2  5
#define STEER_IN1  6
#define STEER_IN2  7

#define DEVICE_NAME  "KidsZee Car"
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Second service, same as HM-10 / JDY BLE modules (the Arduino setup).
// Apps written for those modules look for FFE0 / FFE1.
#define SERVICE_UUID_FFE0 "0000FFE0-0000-1000-8000-00805F9B34FB"
#define CHAR_UUID_FFE1    "0000FFE1-0000-1000-8000-00805F9B34FB"

// ---------------- Tunables ----------------
#define DEBUG_SERIAL        0      // 1 = print J/V commands (slows things a bit)

// BLE TX power: ESP_PWR_LVL_P3 (+3dBm) is plenty for a toy car.
// Raise to P6 / P9 only if range is a problem AND power is stable.
#if HAS_BLE_TX_POWER
  #define BLE_TX_LEVEL      ESP_PWR_LVL_P3
#endif

// Optional: disable brownout reset. Only a band-aid; fix power first.
#define DISABLE_BROWNOUT    0

// Optional battery monitor. Set to a GPIO (ADC pin, e.g. 0..4 on C3)
// connected to the battery through a 2x100k divider. -1 = disabled.
// NOTE: GPIO4 is used by motor, so use GPIO0..GPIO3 for ADC.
#define BAT_ADC_PIN         -1
#define BAT_DIV_RATIO       2.0f   // 100k/100k divider
#define BAT_LOW_MV          3000   // motors cut below this (under load, filtered)
#define BAT_RECOVER_MV      3350   // re-enable above this

const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_BITS = 8;

const int MIN_PWM        = 65;
const int MAX_PWM        = 230;    // drive motor max duty

// Steering: needs strong torque to turn the wheels, so it uses a high
// minimum duty and is NOT scaled by the speed limit. To protect the
// battery/BMS from stall current, duty drops to STEER_HOLD_PWM when the
// steering is held in one direction longer than STEER_FULL_MS.
const int STEER_MIN_PWM  = 150;
const int STEER_MAX_PWM  = 255;
const int STEER_HOLD_PWM = 130;
const unsigned long STEER_FULL_MS = 700;
const int STEER_RAMP_STEP = 50;    // steering reacts almost instantly

const unsigned long COMMAND_TIMEOUT_MS = 700;
const unsigned long RAMP_INTERVAL_MS   = 10;

// Percent change per RAMP_INTERVAL_MS. 4 => 0 to 100% in ~250ms.
const int RAMP_UP_STEP   = 4;
const int RAMP_DOWN_STEP = 10;

const bool INVERT_DRIVE = false;
const bool INVERT_STEER = false;

#if DISABLE_BROWNOUT
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

BLEServer *bleServer = nullptr;
BLECharacteristic *txCharacteristic = nullptr;

volatile bool bleConnected = false;
volatile bool stopRequested = true;
volatile bool charMode = false;   // single-letter commands (U/D/L/R/S) in use

volatile int steerX = 0;
volatile int throttleY = 0;
volatile int speedLimit = 100;

int driveOutput = 0;   // current (ramped) output, percent
int steerOutput = 0;

volatile unsigned long lastValidCommandMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastRampMs = 0;
String rxBuffer;
bool rxOverflow = false;

bool lowBattery = false;
float batFilteredMv = 0;
unsigned long lowBatSinceMs = 0;
unsigned long lastBatMs = 0;

const uint8_t pwmPins[4] = {
  DRIVE_IN1, DRIVE_IN2, STEER_IN1, STEER_IN2
};

void pwmInit() {
  for (int i = 0; i < 4; i++) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(pwmPins[i], PWM_FREQ, PWM_BITS);
    ledcWrite(pwmPins[i], 0);
#else
    ledcSetup(i, PWM_FREQ, PWM_BITS);
    ledcAttachPin(pwmPins[i], i);
    ledcWrite(i, 0);
#endif
  }
}

void pwmWriteIndex(int index, int duty) {
  duty = constrain(duty, 0, 255);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pwmPins[index], duty);
#else
  ledcWrite(index, duty);
#endif
}

// Force all H-bridge inputs low (coast) immediately, no ramp.
void hardMotorOff() {
  for (int i = 0; i < 4; i++) pwmWriteIndex(i, 0);
  driveOutput = 0;
  steerOutput = 0;
}

void setMotor(uint8_t motorIndex, int percent, bool invert, int minPwm, int maxPwm) {
  int pinA = motorIndex * 2;
  int pinB = pinA + 1;

  if (invert) percent = -percent;
  percent = constrain(percent, -100, 100);

  if (percent == 0) {
    pwmWriteIndex(pinA, 0);
    pwmWriteIndex(pinB, 0);
    return;
  }

  int duty = minPwm + (abs(percent) * (maxPwm - minPwm)) / 100;

  if (percent > 0) {
    pwmWriteIndex(pinA, duty);
    pwmWriteIndex(pinB, 0);
  } else {
    pwmWriteIndex(pinA, 0);
    pwmWriteIndex(pinB, duty);
  }
}

// Move 'current' toward 'target' by a limited step.
int rampToward(int current, int target, int upStep, int downStep) {
  if (current == target) return current;

  // Decelerating (towards zero) or reversing is allowed faster.
  bool speedingUp = abs(target) > abs(current) && (current == 0 || (current > 0) == (target > 0));
  int step = speedingUp ? upStep : downStep;

  if (target > current) return min(current + step, target);
  return max(current - step, target);
}

void emergencyStop() {
  stopRequested = true;
  charMode = false;
  steerX = 0;
  throttleY = 0;
  hardMotorOff();
}

// Called every RAMP_INTERVAL_MS from loop().
void updateMotors() {
  if (!bleConnected || stopRequested || lowBattery) {
    hardMotorOff();
    return;
  }

  int driveTarget = throttleY * speedLimit / 100;
  int steerTarget = steerX;   // steering independent of speed limit

  driveOutput = rampToward(driveOutput, driveTarget, RAMP_UP_STEP, RAMP_DOWN_STEP);
  steerOutput = rampToward(steerOutput, steerTarget, STEER_RAMP_STEP, STEER_RAMP_STEP);

  // Reduce steering power if held against the end stop (stall protection).
  static int lastSteerSign = 0;
  static unsigned long steerSinceMs = 0;
  int sign = (steerTarget > 0) - (steerTarget < 0);
  if (sign != lastSteerSign) {
    lastSteerSign = sign;
    steerSinceMs = millis();
  }
  int steerMax = (sign != 0 && millis() - steerSinceMs > STEER_FULL_MS)
                   ? STEER_HOLD_PWM : STEER_MAX_PWM;
  int steerMin = min(STEER_MIN_PWM, steerMax);

  setMotor(0, driveOutput, INVERT_DRIVE, MIN_PWM, MAX_PWM);
  setMotor(1, steerOutput, INVERT_STEER, steerMin, steerMax);
}

// Single-letter commands, same as the Arduino version (no newline needed):
//   U = forward, D = backward, L = left, R = right, S = stop
bool handleSingleChar(char c) {
  switch (c) {
    case 'U': throttleY = 100;  break;
    case 'D': throttleY = -100; break;
    case 'L': steerX = -100;    break;
    case 'R': steerX = 100;     break;
    case 'S':
      steerX = 0;
      throttleY = 0;
      hardMotorOff();
      break;
    default: return false;
  }
  charMode = true;               // these apps send once per press, so no 700ms timeout
  lastValidCommandMs = millis();
  stopRequested = false;
  return true;
}

bool parseIntStrict(const String &s, int &value) {
  if (s.length() == 0) return false;
  char *endPtr = nullptr;
  long v = strtol(s.c_str(), &endPtr, 10);
  if (endPtr == s.c_str() || *endPtr != '\0') return false;
  value = (int)v;
  return true;
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  charMode = false;

  if (line == "S") {
    steerX = 0;
    throttleY = 0;
    hardMotorOff();
    lastValidCommandMs = millis();
#if DEBUG_SERIAL
    Serial.println("STOP command");
#endif
    return;
  }

  char type = line.charAt(0);

  if (type == 'J') {
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    if (c1 < 0 || c2 < 0) return;

    int x, y;
    if (!parseIntStrict(line.substring(c1 + 1, c2), x)) return;
    if (!parseIntStrict(line.substring(c2 + 1), y)) return;

    steerX = constrain(x, -100, 100);
    throttleY = constrain(y, -100, 100);

    lastValidCommandMs = millis();
    stopRequested = false;
#if DEBUG_SERIAL
    Serial.printf("J x=%d y=%d\n", steerX, throttleY);
#endif
  }
  else if (type == 'V') {
    int comma = line.indexOf(',');
    if (comma < 0) return;

    int v;
    if (!parseIntStrict(line.substring(comma + 1), v)) return;

    speedLimit = constrain(v, 0, 100);
    lastValidCommandMs = millis();
    stopRequested = false;
#if DEBUG_SERIAL
    Serial.printf("Speed=%d%%\n", speedLimit);
#endif
  }
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    bleConnected = true;
    rxBuffer = "";
    rxOverflow = false;
    emergencyStop();
    lastValidCommandMs = millis();
    Serial.println("BLE connected; motors stopped until command");
  }

  void onDisconnect(BLEServer *server) override {
    bleConnected = false;
    rxBuffer = "";
    rxOverflow = false;
    emergencyStop();
    Serial.println("BLE disconnected: HARD STOP");
    // Advertising restarted from loop().
  }
};

class RXCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    auto value = characteristic->getValue();

    for (size_t i = 0; i < value.length(); i++) {
      char c = (char)value[i];

      if (c == '\n') {
        if (!rxOverflow && rxBuffer.length()) {
          handleCommand(rxBuffer);
        }
        rxBuffer = "";
        rxOverflow = false;
      }
      else if (c != '\r') {
        // Single letter at start of a packet = Arduino-style command
        if (rxBuffer.length() == 0 && !rxOverflow && handleSingleChar(c)) {
          continue;
        }
        if (rxBuffer.length() < 40 && !rxOverflow) {
          rxBuffer += c;
        } else {
          rxBuffer = "";
          rxOverflow = true;
        }
      }
    }
  }
};

// ---------------- Battery monitor (optional) ----------------
void updateBattery(unsigned long now) {
#if BAT_ADC_PIN >= 0
  if (now - lastBatMs < 50) return;
  lastBatMs = now;

  float mv = analogReadMilliVolts(BAT_ADC_PIN) * BAT_DIV_RATIO;
  if (batFilteredMv == 0) batFilteredMv = mv;
  batFilteredMv = batFilteredMv * 0.95f + mv * 0.05f;

  if (!lowBattery) {
    if (batFilteredMv < BAT_LOW_MV) {
      if (lowBatSinceMs == 0) lowBatSinceMs = now;
      // Must stay low for 1.5s so short load sags don't trigger it.
      if (now - lowBatSinceMs > 1500) {
        lowBattery = true;
        hardMotorOff();
        Serial.println("LOW BATTERY: motors disabled");
      }
    } else {
      lowBatSinceMs = 0;
    }
  } else if (batFilteredMv > BAT_RECOVER_MV) {
    lowBattery = false;
    lowBatSinceMs = 0;
  }
#endif
}

void setup() {
  // All DRV8833 inputs LOW first.
  pinMode(DRIVE_IN1, OUTPUT);
  pinMode(DRIVE_IN2, OUTPUT);
  pinMode(STEER_IN1, OUTPUT);
  pinMode(STEER_IN2, OUTPUT);
  digitalWrite(DRIVE_IN1, LOW);
  digitalWrite(DRIVE_IN2, LOW);
  digitalWrite(STEER_IN1, LOW);
  digitalWrite(STEER_IN2, LOW);

#if DISABLE_BROWNOUT
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif

  Serial.begin(115200);
  delay(100);

#if BAT_ADC_PIN >= 0
  analogReadResolution(12);
  pinMode(BAT_ADC_PIN, INPUT);
#endif

  pwmInit();
  hardMotorOff();
  stopRequested = true;

  BLEDevice::init(DEVICE_NAME);
#if HAS_BLE_TX_POWER
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, BLE_TX_LEVEL);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, BLE_TX_LEVEL);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, BLE_TX_LEVEL);
#endif

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService *service = bleServer->createService(SERVICE_UUID);

  txCharacteristic = service->createCharacteristic(
    TX_UUID, BLECharacteristic::PROPERTY_NOTIFY
  );
  txCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *rx = service->createCharacteristic(
    RX_UUID,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  rx->setCallbacks(new RXCallbacks());

  service->start();

  // Extra HM-10 style service so apps that use FFE0/FFE1 connect too
  BLEService *serviceFfe = bleServer->createService(SERVICE_UUID_FFE0);
  BLECharacteristic *ffe1 = serviceFfe->createCharacteristic(
    CHAR_UUID_FFE1,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  ffe1->addDescriptor(new BLE2902());
  ffe1->setCallbacks(new RXCallbacks());
  serviceFfe->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->addServiceUUID(SERVICE_UUID_FFE0);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("KidsZee Car BLE ready (18650 build, soft-ramp enabled)");
}

void loop() {
  unsigned long now = millis();

  // Watchdog: no valid packets => outputs forced off.
  bool timedOut = !charMode && (now - lastValidCommandMs > COMMAND_TIMEOUT_MS);
  if ((throttleY != 0 || steerX != 0) && (!bleConnected || timedOut)) {
    emergencyStop();
    Serial.println("WATCHDOG: command lost, motors OFF");
  }

  // Restart advertising after disconnect.
  static bool previousConnected = false;
  if (!bleConnected && previousConnected) {
    emergencyStop();
    delay(100);
    BLEDevice::startAdvertising();
    Serial.println("Advertising restarted");
  }
  previousConnected = bleConnected;

  updateBattery(now);

  // Soft-ramped motor update.
  if (now - lastRampMs >= RAMP_INTERVAL_MS) {
    lastRampMs = now;
    updateMotors();
  }

  // Telemetry (same format as before).
  if (bleConnected && txCharacteristic && now - lastTelemetryMs >= 100) {
    lastTelemetryMs = now;
    char packet[32];
    int len = snprintf(packet, sizeof(packet), "T,%d,%d\n",
                       driveOutput, steerOutput);
    txCharacteristic->setValue((uint8_t *)packet, len);
    txCharacteristic->notify();
  }

  delay(2);
}
