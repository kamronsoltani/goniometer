#include <Arduino.h>
#include <Adafruit_LSM6DSOX.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Wire.h>

constexpr uint8_t COIN_BUZZER_MOSFET_PIN = 4;
constexpr uint8_t SPEAKER_BUZZER_PIN = 5;

#if defined(PIN_WIRE_SDA) && defined(PIN_WIRE_SCL)
constexpr uint8_t I2C_SDA_PIN = PIN_WIRE_SDA;
constexpr uint8_t I2C_SCL_PIN = PIN_WIRE_SCL;
#else
constexpr uint8_t I2C_SDA_PIN = 6;
constexpr uint8_t I2C_SCL_PIN = 7;
#endif

constexpr uint8_t FOREARM_IMU_ADDRESS = 0x6A;
constexpr uint8_t BICEP_IMU_ADDRESS = 0x6B;

// Your bicep IMU is mounted 180 degrees around its X axis.
// Rotation around X by 180 degrees means: x stays same, y and z flip signs.
constexpr bool BICEP_MOUNT_ROTATED_180_X = true;

constexpr uint32_t STATUS_INTERVAL_MS = 2000;
constexpr uint32_t DATA_INTERVAL_MS = 50;

constexpr char BLE_DEVICE_NAME[] = "GonioPro-XIAO";
constexpr char BLE_SERVICE_UUID[] = "6f4d0001-7c4d-4b8c-9a7a-3f4c2b6d0001";
constexpr char BLE_DATA_UUID[] = "6f4d0002-7c4d-4b8c-9a7a-3f4c2b6d0001";
constexpr char BLE_COMMAND_UUID[] = "6f4d0003-7c4d-4b8c-9a7a-3f4c2b6d0001";
constexpr uint8_t COMMAND_QUEUE_SIZE = 16;

struct Vec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct ImuSample {
  Vec3 accel;
  Vec3 gyro;
  float tempC = 0.0f;
};

Adafruit_LSM6DSOX forearmIMU;
Adafruit_LSM6DSOX bicepIMU;
BLEServer *bleServer = nullptr;
BLECharacteristic *bleDataCharacteristic = nullptr;
BLEAdvertising *bleAdvertising = nullptr;

bool forearmReady = false;
bool bicepReady = false;
bool bleClientConnected = false;
bool liveStatusEnabled = true;
bool showRawBicep = false;
volatile uint8_t commandQueueHead = 0;
volatile uint8_t commandQueueTail = 0;
volatile bool commandQueueOverflow = false;
char commandQueue[COMMAND_QUEUE_SIZE] = {};
uint32_t lastStatusMs = 0;
uint32_t lastDataMs = 0;

bool calibrationGuideActive = false;
bool humanGuideActive = false;
bool angleZeroCaptured = false;
uint8_t calibrationStep = 0;
uint8_t humanGuideStep = 0;
float zeroOffsetDeg = 0.0f;
float angleScale = 1.0f;
float humanMeasured90RawDeg = 0.0f;
float capturedAnglesDeg[5] = {0, 0, 0, 0, 0};
constexpr float CALIBRATION_TARGETS_DEG[5] = {0, 45, 90, 135, 180};

void processCommand(char command);

bool enqueueCommand(char command) {
  const uint8_t nextHead = (commandQueueHead + 1) % COMMAND_QUEUE_SIZE;
  if (nextHead == commandQueueTail) {
    commandQueueOverflow = true;
    return false;
  }

  commandQueue[commandQueueHead] = command;
  commandQueueHead = nextHead;
  return true;
}

bool dequeueCommand(char &command) {
  if (commandQueueTail == commandQueueHead) {
    return false;
  }

  command = commandQueue[commandQueueTail];
  commandQueueTail = (commandQueueTail + 1) % COMMAND_QUEUE_SIZE;
  return true;
}

void printHex(uint8_t value) {
  Serial.print(F("0x"));
  if (value < 16) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

float magnitude(const Vec3 &v) {
  return sqrtf((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
}

float dotProduct(const Vec3 &a, const Vec3 &b) {
  return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

float gravityVectorAngleDeg(const Vec3 &a, const Vec3 &b) {
  const float magA = magnitude(a);
  const float magB = magnitude(b);
  if (magA < 0.001f || magB < 0.001f) {
    return 0.0f;
  }

  const float cosine = constrain(dotProduct(a, b) / (magA * magB), -1.0f, 1.0f);
  return acosf(cosine) * 180.0f / PI;
}

float pitchFromAccel(const Vec3 &accel) {
  return atan2f(-accel.x, sqrtf((accel.y * accel.y) + (accel.z * accel.z))) * 180.0f / PI;
}

void coinBuzz(uint16_t durationMs) {
  digitalWrite(COIN_BUZZER_MOSFET_PIN, HIGH);
  delay(durationMs);
  digitalWrite(COIN_BUZZER_MOSFET_PIN, LOW);
}

void speakerTone(uint16_t frequency, uint16_t durationMs) {
  tone(SPEAKER_BUZZER_PIN, frequency, durationMs);
  delay(durationMs);
  noTone(SPEAKER_BUZZER_PIN);
}

void playTierFeedback(uint8_t tier) {
  if (tier == 0) {
    return;
  }

  Serial.print(F("[feedback] tier "));
  Serial.println(tier);

  if (tier == 1) {
    // Best zone: one confident haptic dot.
    coinBuzz(95);
  } else if (tier == 2) {
    // Close: two friendly haptic nudges.
    coinBuzz(45);
    delay(42);
    coinBuzz(65);
  } else {
    // Further out: three light breadcrumb taps, still cute instead of scolding.
    for (uint8_t i = 0; i < 3; i++) {
      coinBuzz(34);
      delay(48);
    }
  }
}

void playGoodRepFeedback() {
  Serial.println(F("[feedback] good rep"));

  // Happy major rep stamp: quick, bright, and unmistakably positive.
  speakerTone(784, 55);
  delay(24);
  speakerTone(1047, 70);
  delay(24);
  speakerTone(1319, 82);
  delay(28);
  speakerTone(1568, 130);

  delay(35);
  coinBuzz(75);
  delay(45);
  coinBuzz(125);
}

void playInZoneChime(uint8_t stage) {
  Serial.print(F("[feedback] in-zone chime stage "));
  Serial.println(stage);

  const uint16_t (*notes)[3];
  static const uint16_t stage1[3] = {988, 1245, 1480};   // B5, D#6, F#6
  static const uint16_t stage2[3] = {1047, 1319, 1568};  // C6, E6, G6
  static const uint16_t stage3[3] = {1175, 1480, 1760};  // D6, F#6, A6
  static const uint16_t stage4[3] = {1319, 1661, 1976};  // E6, G#6, B6
  static const uint16_t stage5[3] = {1568, 1976, 2349};  // G6, B6, D7

  if (stage <= 1) {
    notes = &stage1;
  } else if (stage == 2) {
    notes = &stage2;
  } else if (stage == 3) {
    notes = &stage3;
  } else if (stage == 4) {
    notes = &stage4;
  } else {
    notes = &stage5;
  }

  // Short rising major chime: subway-door-ish, but brighter and less alarming.
  speakerTone((*notes)[0], 58);
  delay(28);
  speakerTone((*notes)[1], 58);
  delay(28);
  speakerTone((*notes)[2], 82);
}

void playGuideStartFeedback() {
  Serial.println(F("[feedback] guide start"));
  speakerTone(988, 55);
  delay(24);
  speakerTone(1319, 70);
  delay(28);
  coinBuzz(70);
}

void playCaptureFeedback() {
  Serial.println(F("[feedback] capture ok"));
  speakerTone(1175, 48);
  delay(22);
  speakerTone(1568, 72);
  delay(28);
  coinBuzz(75);
}

void playGuideCompleteFeedback() {
  Serial.println(F("[feedback] guide complete"));
  speakerTone(1047, 58);
  delay(24);
  speakerTone(1319, 58);
  delay(24);
  speakerTone(1760, 92);
  delay(34);
  coinBuzz(95);
}

void playFriendlyErrorFeedback() {
  Serial.println(F("[feedback] try again"));
  speakerTone(784, 60);
  delay(30);
  speakerTone(698, 80);
  delay(35);
  coinBuzz(55);
  delay(35);
  coinBuzz(55);
}

void playSessionCompleteFeedback() {
  Serial.println(F("[feedback] session complete victory"));

  // Original arcade-style victory fanfare, not a copy of any game melody.
  speakerTone(784, 90);
  delay(28);
  speakerTone(988, 90);
  delay(28);
  speakerTone(1175, 90);
  delay(28);
  speakerTone(1568, 130);
  delay(48);
  speakerTone(1319, 90);
  delay(28);
  speakerTone(1568, 90);
  delay(28);
  speakerTone(1976, 165);
  delay(55);
  speakerTone(2093, 210);

  delay(45);
  coinBuzz(75);
  delay(50);
  coinBuzz(75);
  delay(50);
  coinBuzz(170);
}

class GonioBleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    (void)server;
    bleClientConnected = true;
    Serial.println(F("[ble] client connected"));
  }

  void onDisconnect(BLEServer *server) override {
    (void)server;
    bleClientConnected = false;
    Serial.println(F("[ble] client disconnected"));
    if (bleAdvertising != nullptr) {
      bleAdvertising->start();
    }
  }
};

class GonioCommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    const auto value = characteristic->getValue();
    if (value.length() == 0) {
      return;
    }

    for (size_t i = 0; i < value.length(); i++) {
      const char command = static_cast<char>(tolower(value[i]));
      if (command == '\r' || command == '\n') {
        continue;
      }
      enqueueCommand(command);
    }
  }
};

void startBLE() {
  Serial.println();
  Serial.println(F("BLE INIT"));
  Serial.println(F("--------"));

  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(185);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new GonioBleServerCallbacks());

  BLEService *service = bleServer->createService(BLE_SERVICE_UUID);

  bleDataCharacteristic = service->createCharacteristic(
      BLE_DATA_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  bleDataCharacteristic->addDescriptor(new BLE2902());
  bleDataCharacteristic->setValue("DATA,angle=0.00,raw=0.00,calibrated=0,forearm_ready=0,bicep_ready=0,scale=1.0000,zero=0.00");

  BLECharacteristic *commandCharacteristic = service->createCharacteristic(
      BLE_COMMAND_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  commandCharacteristic->setCallbacks(new GonioCommandCallbacks());

  service->start();

  bleAdvertising = BLEDevice::getAdvertising();
  bleAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  bleAdvertising->setScanResponse(true);
  bleAdvertising->setMinPreferred(0x06);
  bleAdvertising->setMinPreferred(0x12);
  bleAdvertising->start();

  Serial.print(F("[ble] advertising as "));
  Serial.println(BLE_DEVICE_NAME);
}

ImuSample correctedBicepSample(const ImuSample &raw) {
  if (!BICEP_MOUNT_ROTATED_180_X) {
    return raw;
  }

  ImuSample corrected = raw;
  corrected.accel.y *= -1.0f;
  corrected.accel.z *= -1.0f;
  corrected.gyro.y *= -1.0f;
  corrected.gyro.z *= -1.0f;
  return corrected;
}

bool readIMU(Adafruit_LSM6DSOX &imu, bool ready, ImuSample &sample) {
  if (!ready) {
    return false;
  }

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  imu.getEvent(&accel, &gyro, &temp);

  sample.accel.x = accel.acceleration.x;
  sample.accel.y = accel.acceleration.y;
  sample.accel.z = accel.acceleration.z;
  sample.gyro.x = gyro.gyro.x;
  sample.gyro.y = gyro.gyro.y;
  sample.gyro.z = gyro.gyro.z;
  sample.tempC = temp.temperature;
  return true;
}

bool readForearm(ImuSample &sample) {
  return readIMU(forearmIMU, forearmReady, sample);
}

bool readBicep(ImuSample &sample) {
  ImuSample raw;
  if (!readIMU(bicepIMU, bicepReady, raw)) {
    return false;
  }

  sample = correctedBicepSample(raw);
  return true;
}

bool readRelativeAngle(float &rawAngleDeg) {
  ImuSample forearm;
  ImuSample bicep;
  if (!readForearm(forearm) || !readBicep(bicep)) {
    return false;
  }

  rawAngleDeg = gravityVectorAngleDeg(forearm.accel, bicep.accel);
  return true;
}

bool readCalibratedAngle(float &angleDeg) {
  float rawAngleDeg = 0.0f;
  if (!readRelativeAngle(rawAngleDeg)) {
    return false;
  }

  angleDeg = constrain(fabsf(rawAngleDeg - zeroOffsetDeg) * angleScale, 0.0f, 180.0f);
  return true;
}

void scanI2C() {
  Serial.println();
  Serial.println(F("I2C SCAN"));
  Serial.println(F("--------"));

  bool foundForearm = false;
  bool foundBicep = false;
  uint8_t found = 0;

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("Found "));
      printHex(address);
      if (address == FOREARM_IMU_ADDRESS) {
        Serial.print(F("  <- forearm IMU expected address"));
        foundForearm = true;
      } else if (address == BICEP_IMU_ADDRESS) {
        Serial.print(F("  <- bicep IMU expected address"));
        foundBicep = true;
      }
      Serial.println();
      found++;
    }
  }

  Serial.print(F("Total I2C devices: "));
  Serial.println(found);
  Serial.print(F("Forearm 0x6A: "));
  Serial.println(foundForearm ? F("FOUND") : F("MISSING"));
  Serial.print(F("Bicep   0x6B: "));
  Serial.println(foundBicep ? F("FOUND") : F("MISSING"));
}

bool startOneIMU(Adafruit_LSM6DSOX &imu, uint8_t address, const __FlashStringHelper *name) {
  Serial.print(name);
  Serial.print(F(" "));
  printHex(address);
  Serial.print(F(": "));

  if (!imu.begin_I2C(address, &Wire)) {
    Serial.println(F("FAIL"));
    return false;
  }

  imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
  imu.setGyroDataRate(LSM6DS_RATE_104_HZ);

  Serial.println(F("OK"));
  return true;
}

void startIMUs() {
  Serial.println();
  Serial.println(F("IMU INIT"));
  Serial.println(F("--------"));
  Serial.print(F("I2C pins: SDA=GPIO"));
  Serial.print(I2C_SDA_PIN);
  Serial.print(F(", SCL=GPIO"));
  Serial.println(I2C_SCL_PIN);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  scanI2C();

  Serial.println();
  Serial.println(F("Driver init"));
  forearmReady = startOneIMU(forearmIMU, FOREARM_IMU_ADDRESS, F("Forearm"));
  bicepReady = startOneIMU(bicepIMU, BICEP_IMU_ADDRESS, F("Bicep"));

  Serial.println();
  Serial.print(F("Overall IMU status: "));
  Serial.println((forearmReady && bicepReady) ? F("PASS") : F("FAIL"));
  Serial.print(F("Bicep mount correction: "));
  Serial.println(BICEP_MOUNT_ROTATED_180_X ? F("180 deg around X ENABLED") : F("none"));
}

void testVibrationMotors() {
  Serial.println();
  Serial.println(F("VIBRATION TEST"));
  Serial.println(F("--------------"));
  Serial.println(F("Expected: all coin buzzers pulse together 3 times."));

  for (uint8_t i = 0; i < 3; i++) {
    Serial.print(F("Pulse "));
    Serial.println(i + 1);
    coinBuzz(180);
    delay(220);
  }
}

void testSpeakerBuzzer() {
  Serial.println();
  Serial.println(F("SPEAKER TEST"));
  Serial.println(F("------------"));
  Serial.println(F("Expected: cute chimes plus a victory fanfare."));

  playInZoneChime(1);
  delay(120);
  playInZoneChime(2);
  delay(120);
  playInZoneChime(3);
  delay(120);
  playInZoneChime(4);
  delay(120);
  playInZoneChime(5);
  delay(180);
  playGoodRepFeedback();
  delay(220);
  playSessionCompleteFeedback();
}

void printSampleLine(const __FlashStringHelper *name, uint8_t address, bool ready, const ImuSample &sample) {
  Serial.print(name);
  Serial.print(F(" "));
  printHex(address);

  if (!ready) {
    Serial.println(F(": NOT READY"));
    return;
  }

  Serial.print(F(": accel=("));
  Serial.print(sample.accel.x, 2);
  Serial.print(',');
  Serial.print(sample.accel.y, 2);
  Serial.print(',');
  Serial.print(sample.accel.z, 2);
  Serial.print(F(") |a|="));
  Serial.print(magnitude(sample.accel), 2);
  Serial.print(F(" gyro=("));
  Serial.print(sample.gyro.x, 3);
  Serial.print(',');
  Serial.print(sample.gyro.y, 3);
  Serial.print(',');
  Serial.print(sample.gyro.z, 3);
  Serial.print(F(") temp="));
  Serial.print(sample.tempC, 1);
  Serial.println(F("C"));
}

void printDetailedIMUs() {
  ImuSample forearm;
  ImuSample bicep;
  const bool forearmOk = readForearm(forearm);
  const bool bicepOk = readBicep(bicep);

  Serial.println();
  Serial.println(F("DETAILED IMU READINGS"));
  Serial.println(F("---------------------"));
  printSampleLine(F("Forearm"), FOREARM_IMU_ADDRESS, forearmOk, forearm);
  printSampleLine(F("Bicep corrected"), BICEP_IMU_ADDRESS, bicepOk, bicep);

  if (showRawBicep && bicepReady) {
    ImuSample rawBicep;
    readIMU(bicepIMU, bicepReady, rawBicep);
    printSampleLine(F("Bicep raw"), BICEP_IMU_ADDRESS, true, rawBicep);
  }
}

void printStatusSummary() {
  ImuSample forearm;
  ImuSample bicep;
  const bool forearmOk = readForearm(forearm);
  const bool bicepOk = readBicep(bicep);
  const bool allOk = forearmOk && bicepOk;

  Serial.print(F("STATUS "));
  Serial.print(allOk ? F("PASS") : F("FAIL"));
  Serial.print(F(" | forearm="));
  Serial.print(forearmOk ? F("OK") : F("MISS"));
  Serial.print(F(" bicep="));
  Serial.print(bicepOk ? F("OK") : F("MISS"));

  if (allOk) {
    const float forearmPitch = pitchFromAccel(forearm.accel);
    const float bicepPitch = pitchFromAccel(bicep.accel);
    const float pitchOnlyAngle = fabsf(forearmPitch - bicepPitch);
    const float gravityAngle = gravityVectorAngleDeg(forearm.accel, bicep.accel);

    Serial.print(F(" | accel_mag "));
    Serial.print(magnitude(forearm.accel), 1);
    Serial.print(F("/"));
    Serial.print(magnitude(bicep.accel), 1);
    Serial.print(F(" m/s^2"));
    Serial.print(F(" | pitch "));
    Serial.print(forearmPitch, 1);
    Serial.print(F("/"));
    Serial.print(bicepPitch, 1);
    Serial.print(F(" deg"));
    Serial.print(F(" | gravity_angle="));
    Serial.print(gravityAngle, 1);
    Serial.print(F(" deg"));
    Serial.print(F(" | old_pitch_angle="));
    Serial.print(pitchOnlyAngle, 1);
    Serial.print(F(" deg"));

    if (angleZeroCaptured) {
      float calibratedAngle = 0.0f;
      if (readCalibratedAngle(calibratedAngle)) {
        Serial.print(F(" | calibrated_angle="));
        Serial.print(calibratedAngle, 1);
        Serial.print(F(" deg"));
      }
    }
  }

  Serial.println();
}

String buildDataLine() {
  float rawAngleDeg = 0.0f;
  float calibratedAngleDeg = 0.0f;
  const bool rawOk = readRelativeAngle(rawAngleDeg);
  const bool calibratedOk = readCalibratedAngle(calibratedAngleDeg);

  String line = F("DATA");
  line += F(",angle=");
  line += String(calibratedOk ? calibratedAngleDeg : 0.0f, 2);
  line += F(",raw=");
  line += String(rawOk ? rawAngleDeg : 0.0f, 2);
  line += F(",calibrated=");
  line += angleZeroCaptured ? '1' : '0';
  line += F(",forearm_ready=");
  line += forearmReady ? '1' : '0';
  line += F(",bicep_ready=");
  line += bicepReady ? '1' : '0';
  line += F(",scale=");
  line += String(angleScale, 4);
  line += F(",zero=");
  line += String(zeroOffsetDeg, 2);
  return line;
}

void sendDataLine() {
  const String line = buildDataLine();
  if (!bleClientConnected) {
    Serial.println(line);
  }

  if (bleClientConnected && bleDataCharacteristic != nullptr) {
    const String bleLine = line + '\n';
    bleDataCharacteristic->setValue(bleLine.c_str());
    bleDataCharacteristic->notify();
  }
}

void printCalibrationTable() {
  Serial.println();
  Serial.println(F("CALIBRATION TABLE"));
  Serial.println(F("-----------------"));
  Serial.println(F("Target | Measured | Error"));

  for (uint8_t i = 0; i < 5; i++) {
    Serial.print(CALIBRATION_TARGETS_DEG[i], 0);
    Serial.print(F(" deg"));
    if (i > calibrationStep || (calibrationGuideActive && i == calibrationStep)) {
      Serial.println(F(" | pending"));
      continue;
    }

    const float error = capturedAnglesDeg[i] - CALIBRATION_TARGETS_DEG[i];
    Serial.print(F(" | "));
    Serial.print(capturedAnglesDeg[i], 1);
    Serial.print(F(" deg | "));
    if (error > 0.0f) {
      Serial.print('+');
    }
    Serial.print(error, 1);
    Serial.println(F(" deg"));
  }
}

void printHumanCalibrationStatus() {
  Serial.println();
  Serial.println(F("HUMAN ARM CALIBRATION"));
  Serial.println(F("---------------------"));

  if (!angleZeroCaptured) {
    Serial.println(F("Zero pose: not captured"));
    Serial.println(F("Send 'u' to start, then hold arm straight down and send 'n'."));
    return;
  }

  Serial.print(F("Zero raw angle: "));
  Serial.print(zeroOffsetDeg, 2);
  Serial.println(F(" deg"));
  Serial.print(F("90-degree raw movement: "));
  Serial.print(humanMeasured90RawDeg, 2);
  Serial.println(F(" deg"));
  Serial.print(F("Scale factor: "));
  Serial.println(angleScale, 4);

  float currentAngle = 0.0f;
  if (readCalibratedAngle(currentAngle)) {
    Serial.print(F("Current corrected angle: "));
    Serial.print(currentAngle, 1);
    Serial.println(F(" deg"));
  }
}

void printCalibrationPrompt() {
  Serial.println();
  Serial.println(F("GUIDED TABLE CALIBRATION"));
  Serial.println(F("------------------------"));

  if (!calibrationGuideActive) {
    Serial.println(F("Not active. Send 'g' to start."));
    return;
  }

  if (calibrationStep == 0) {
    Serial.println(F("Step 1/5: Place both IMUs flat, aligned, and motionless."));
    Serial.println(F("This represents straight elbow extension = 0 degrees."));
    Serial.println(F("Send 'n' when the sensors are still."));
  } else {
    Serial.print(F("Step "));
    Serial.print(calibrationStep + 1);
    Serial.print(F("/5: Keep the bicep/main IMU still. Rotate ONLY the forearm IMU to "));
    Serial.print(CALIBRATION_TARGETS_DEG[calibrationStep], 0);
    Serial.println(F(" degrees relative to it."));
    Serial.println(F("Send 'n' when it is positioned. Send 'q' to quit the guide."));
  }
}

void startCalibrationGuide() {
  calibrationGuideActive = true;
  humanGuideActive = false;
  angleZeroCaptured = false;
  calibrationStep = 0;
  zeroOffsetDeg = 0.0f;
  angleScale = 1.0f;
  humanMeasured90RawDeg = 0.0f;
  for (uint8_t i = 0; i < 5; i++) {
    capturedAnglesDeg[i] = 0.0f;
  }

  liveStatusEnabled = false;
  playGuideStartFeedback();
  printCalibrationPrompt();
}

void printHumanGuidePrompt() {
  Serial.println();
  Serial.println(F("GUIDED HUMAN ARM CALIBRATION"));
  Serial.println(F("----------------------------"));

  if (!humanGuideActive) {
    Serial.println(F("Not active. Send 'u' to start."));
    return;
  }

  if (humanGuideStep == 0) {
    Serial.println(F("Step 1/2: Hold arm straight down at your side."));
    Serial.println(F("Keep elbow extended as comfortably as possible."));
    Serial.println(F("Keep upper arm still. Send 'n' when still."));
  } else {
    Serial.println(F("Step 2/2: Bend elbow to a real 90-degree angle."));
    Serial.println(F("Keep upper arm/bicep IMU still at your side."));
    Serial.println(F("Send 'n' when you are at 90 degrees."));
  }
}

void startHumanGuide() {
  humanGuideActive = true;
  calibrationGuideActive = false;
  humanGuideStep = 0;
  angleZeroCaptured = false;
  zeroOffsetDeg = 0.0f;
  angleScale = 1.0f;
  humanMeasured90RawDeg = 0.0f;
  liveStatusEnabled = false;

  playGuideStartFeedback();
  printHumanGuidePrompt();
}

void captureHumanGuideStep() {
  if (!humanGuideActive) {
    Serial.println(F("[human] Not active. Send 'u' to start human calibration."));
    return;
  }

  float rawAngleDeg = 0.0f;
  if (!readRelativeAngle(rawAngleDeg)) {
    Serial.println(F("[human] Cannot read both IMUs. Check wiring, then send 'r'."));
    return;
  }

  if (humanGuideStep == 0) {
    zeroOffsetDeg = rawAngleDeg;
    angleScale = 1.0f;
    humanMeasured90RawDeg = 0.0f;
    angleZeroCaptured = true;
    humanGuideStep = 1;

    Serial.println();
    Serial.print(F("[human] Zero captured. raw_zero_offset="));
    Serial.print(zeroOffsetDeg, 2);
    Serial.println(F(" deg"));
    playCaptureFeedback();
    printHumanGuidePrompt();
    return;
  }

  humanMeasured90RawDeg = fabsf(rawAngleDeg - zeroOffsetDeg);
  if (humanMeasured90RawDeg < 5.0f) {
    Serial.println();
    Serial.println(F("[human] 90-degree capture was too small."));
    Serial.println(F("Make sure only the forearm bends and the bicep IMU stays still, then send 'n' again."));
    playFriendlyErrorFeedback();
    return;
  }

  angleScale = 90.0f / humanMeasured90RawDeg;
  humanGuideActive = false;
  liveStatusEnabled = true;

  Serial.println();
  Serial.print(F("[human] 90-degree pose captured. raw_movement="));
  Serial.print(humanMeasured90RawDeg, 2);
  Serial.println(F(" deg"));
  Serial.print(F("[human] scale_factor=90/raw_movement="));
  Serial.println(angleScale, 4);
  Serial.println(F("[human] Complete. Live status now shows calibrated_angle corrected by this scale."));
  printHumanCalibrationStatus();

  playGuideCompleteFeedback();
}

void captureCalibrationStep() {
  if (humanGuideActive) {
    captureHumanGuideStep();
    return;
  }

  if (!calibrationGuideActive) {
    Serial.println(F("[guide] Not active. Send 'g' for table guide or 'u' for human guide."));
    return;
  }

  float rawAngleDeg = 0.0f;
  if (!readRelativeAngle(rawAngleDeg)) {
    Serial.println(F("[guide] Cannot read both IMUs. Check wiring, then send 'r'."));
    return;
  }

  if (calibrationStep == 0) {
    zeroOffsetDeg = rawAngleDeg;
    angleZeroCaptured = true;
    capturedAnglesDeg[0] = 0.0f;
    Serial.println();
    Serial.print(F("[guide] Zero captured. raw_zero_offset="));
    Serial.print(zeroOffsetDeg, 2);
    Serial.println(F(" deg"));
    playCaptureFeedback();
  } else {
    const float calibratedAngle = constrain(fabsf(rawAngleDeg - zeroOffsetDeg) * angleScale, 0.0f, 180.0f);
    capturedAnglesDeg[calibrationStep] = calibratedAngle;

    Serial.println();
    Serial.print(F("[guide] Captured target="));
    Serial.print(CALIBRATION_TARGETS_DEG[calibrationStep], 0);
    Serial.print(F(" deg measured="));
    Serial.print(calibratedAngle, 1);
    Serial.print(F(" deg error="));
    const float error = calibratedAngle - CALIBRATION_TARGETS_DEG[calibrationStep];
    if (error > 0.0f) {
      Serial.print('+');
    }
    Serial.print(error, 1);
    Serial.println(F(" deg"));
    playCaptureFeedback();
  }

  calibrationStep++;

  if (calibrationStep >= 5) {
    calibrationGuideActive = false;
    liveStatusEnabled = true;
    printCalibrationTable();
    Serial.println(F("[guide] Complete. Send me this table if the numbers look wrong."));
    playGuideCompleteFeedback();
    return;
  }

  printCalibrationPrompt();
}

void quitCalibrationGuide() {
  calibrationGuideActive = false;
  humanGuideActive = false;
  liveStatusEnabled = true;
  Serial.println(F("[guide] Stopped. Live status is back on."));
}

void runFullCheckout() {
  Serial.println();
  Serial.println(F("========== HARDWARE CHECKOUT =========="));
  startIMUs();
  testVibrationMotors();
  testSpeakerBuzzer();
  printDetailedIMUs();
  Serial.println(F("RESULT: If both IMUs are OK, buzzers vibrated, and speaker beeped, wiring is ready to glue."));
  Serial.println(F("======================================="));
}

void printHelp() {
  Serial.println();
  Serial.println(F("Hardware checkout commands"));
  Serial.println(F("  a = run all tests"));
  Serial.println(F("  i = print detailed IMU readings once"));
  Serial.println(F("  g = start guided flat-table calibration test"));
  Serial.println(F("  u = start guided human arm calibration"));
  Serial.println(F("  n = capture current guided-calibration step"));
  Serial.println(F("  q = quit guided calibration"));
  Serial.println(F("  m = print calibration table / human calibration status"));
  Serial.println(F("  l = toggle compact live status"));
  Serial.println(F("  x = toggle bicep raw reading in detailed output"));
  Serial.println(F("  r = rescan I2C and reinitialize IMUs"));
  Serial.println(F("  v = test vibration motors on GPIO4"));
  Serial.println(F("  s = test speaker buzzer on GPIO5"));
  Serial.println(F("  1/2/3 = play tier vibration feedback"));
  Serial.println(F("  c/d/e/f/z = play in-zone countdown chime stages"));
  Serial.println(F("  b = play good-rep success feedback"));
  Serial.println(F("  w = play session-complete victory feedback"));
  Serial.println(F("  h = show help"));
  Serial.println();
  Serial.println(F("What to look for before hot glue:"));
  Serial.println(F("  STATUS PASS"));
  Serial.println(F("  accel_mag near 9.8/9.8 when still"));
  Serial.println(F("  gravity_angle should match your physical table-test angle"));
  Serial.println(F("  old_pitch_angle is shown only to prove why the first method failed"));
  Serial.println(F("  forearm numbers change when forearm IMU moves"));
  Serial.println(F("  bicep numbers change when bicep IMU moves"));
  Serial.println(F("  vibration pulses and speaker tones are physically obvious"));
  Serial.println();
  Serial.println(F("Guided calibration test:"));
  Serial.println(F("  1. Send g."));
  Serial.println(F("  2. Put both IMUs flat/aligned. Send n for zero."));
  Serial.println(F("  3. Rotate forearm IMU to 45, 90, 135, 180."));
  Serial.println(F("  4. Send n at each angle."));
  Serial.println();
  Serial.println(F("Guided human calibration:"));
  Serial.println(F("  1. Send u."));
  Serial.println(F("  2. Hold arm straight down at side. Send n for zero."));
  Serial.println(F("  3. Bend elbow to a real 90 degrees. Send n for scale."));
  Serial.println(F("  4. Live status shows calibrated_angle."));
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);

  pinMode(COIN_BUZZER_MOSFET_PIN, OUTPUT);
  pinMode(SPEAKER_BUZZER_PIN, OUTPUT);
  digitalWrite(COIN_BUZZER_MOSFET_PIN, LOW);
  digitalWrite(SPEAKER_BUZZER_PIN, LOW);

  delay(1500);
  Serial.println();
  Serial.println(F("XIAO ESP32C3 HARDWARE CHECKOUT"));
  Serial.println(F("Expected: vibration MOSFET=GPIO4, speaker=GPIO5."));
  Serial.println(F("Expected: forearm IMU=0x6A, bicep IMU=0x6B."));
  Serial.println(F("Bicep IMU correction: 180 degree rotation around X axis."));
  printHelp();
  startBLE();
  runFullCheckout();
}

void processCommand(char command) {
  switch (command) {
    case 'a':
      runFullCheckout();
      break;
    case 'i':
      printDetailedIMUs();
      break;
    case 'g':
      startCalibrationGuide();
      break;
    case 'u':
      startHumanGuide();
      break;
    case 'n':
      captureCalibrationStep();
      break;
    case 'q':
      quitCalibrationGuide();
      break;
    case 'm':
      printCalibrationTable();
      printHumanCalibrationStatus();
      break;
    case 'l':
      liveStatusEnabled = !liveStatusEnabled;
      Serial.print(F("Live status: "));
      Serial.println(liveStatusEnabled ? F("ON") : F("OFF"));
      break;
    case 'x':
      showRawBicep = !showRawBicep;
      Serial.print(F("Raw bicep detail: "));
      Serial.println(showRawBicep ? F("ON") : F("OFF"));
      break;
    case 'r':
      startIMUs();
      break;
    case 'v':
      testVibrationMotors();
      break;
    case 's':
      testSpeakerBuzzer();
      break;
    case '1':
      playTierFeedback(1);
      break;
    case '2':
      playTierFeedback(2);
      break;
    case '3':
      playTierFeedback(3);
      break;
    case 'b':
      playGoodRepFeedback();
      break;
    case 'w':
      playSessionCompleteFeedback();
      break;
    case 'c':
      playInZoneChime(1);
      break;
    case 'd':
      playInZoneChime(2);
      break;
    case 'e':
      playInZoneChime(3);
      break;
    case 'f':
      playInZoneChime(4);
      break;
    case 'z':
      playInZoneChime(5);
      break;
    case 'h':
    case '?':
      printHelp();
      break;
    case '\r':
    case '\n':
      break;
    default:
      Serial.print(F("[cmd] Unknown command: "));
      Serial.println(command);
      printHelp();
      break;
  }
}

void loop() {
  const uint32_t now = millis();
  if (commandQueueOverflow) {
    commandQueueOverflow = false;
    Serial.println(F("[cmd] Command queue overflow; dropped command."));
  }

  char queuedCommand = '\0';
  if (dequeueCommand(queuedCommand)) {
    processCommand(queuedCommand);
    return;
  }

  if (liveStatusEnabled && now - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = now;
    printStatusSummary();
  }

  if (!calibrationGuideActive && !humanGuideActive && now - lastDataMs >= DATA_INTERVAL_MS) {
    lastDataMs = now;
    sendDataLine();
  }

  if (Serial.available() == 0) {
    return;
  }

  enqueueCommand(static_cast<char>(tolower(Serial.read())));
}
