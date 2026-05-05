#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "mega_servo_sequence.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ================= IMU =================
const int MPU = 0x69;

int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;

float gx_offset = 0, gy_offset = 0, gz_offset = 0;

float roll = 0, pitch = 0, yaw = 0;

unsigned long lastTime;
float dt;

// ================= SENSORES CORRIENTE =================
#define PIN_ACS712 A0   // Arduino + IMU + PCA
#define PIN_ACS758 A1   // Servos

float offset712 = 2.5;
float offset758 = 2.5;

// ================= SERVOS =================
#define MIN_PULSE_WIDTH 600
#define MAX_PULSE_WIDTH 2600
#define FREQUENCY 50

static const uint8_t NUM_CHANNELS = 16;
static const float kWalkSpeedScale = 1.2f;

// ================= LIMITES =================
int safeMin[NUM_CHANNELS] = {
  85, 40, 40, 0,
  74, 30, 40, 0,
   0, 83, 5, 30,
  77, 5, 20, 0
};

int safeMax[NUM_CHANNELS] = {
  115, 175, 145, 180,
  104, 175, 145, 180,
  180, 113, 150, 140,
  113, 147, 150, 180
};

int neutralPose[NUM_CHANNELS] = {
  100, 90, 120, 90,
   89, 120, 120, 90,
   90, 98, 60, 60,
   93, 90, 60, 90
};

// ================= CALIBRACIÓN =================
void calibrarSensores() {
  Serial.println("Calibrando sensores...");

  float sum712 = 0;
  float sum758 = 0;

  for (int i = 0; i < 500; i++) {
    sum712 += analogRead(PIN_ACS712);
    sum758 += analogRead(PIN_ACS758);
    delay(2);
  }

  offset712 = (sum712 / 500.0) * (5.0 / 1023.0);
  offset758 = (sum758 / 500.0) * (5.0 / 1023.0);

  Serial.println("Sensores listos");
}

void calibrarGyro() {
  Serial.println("Calibrando IMU...");
  delay(2000);

  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);

    if (Wire.requestFrom(MPU, 6, true) == 6) {
      gx_offset += Wire.read() << 8 | Wire.read();
      gy_offset += Wire.read() << 8 | Wire.read();
      gz_offset += Wire.read() << 8 | Wire.read();
    }
    delay(2);
  }

  gx_offset /= 500;
  gy_offset /= 500;
  gz_offset /= 500;

  Serial.println("IMU lista");
}

// ================= IMU =================
void leerIMU() {

  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  if (dt <= 0 || dt > 0.1) dt = 0.01;

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) return;

  Wire.requestFrom(MPU, 14, true);
  if (Wire.available() < 14) return;

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  Wire.read(); Wire.read();

  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  float ax = AcX / 16384.0;
  float ay = AcY / 16384.0;
  float az = AcZ / 16384.0;

  float gx = (GyX - gx_offset) / 131.0;
  float gy = (GyY - gy_offset) / 131.0;
  float gz = (GyZ - gz_offset) / 131.0;

  float roll_acc  = atan2(ay, az) * 180 / PI;
  float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

  roll  = 0.96 * (roll  + gx * dt) + 0.04 * roll_acc;
  pitch = 0.96 * (pitch + gy * dt) + 0.04 * pitch_acc;

  yaw += gz * dt;
}

// ================= CORRIENTE =================
float leerCorriente712() {
  float voltage = analogRead(PIN_ACS712) * (5.0 / 1023.0);
  return (voltage - offset712) / 0.100;
}

float leerCorriente758() {
  float voltage = analogRead(PIN_ACS758) * (5.0 / 1023.0);
  return (voltage - offset758) / 0.040;
}

// ================= SERVOS =================
int pulseWidth(int angle) {
  int pulse = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  return int(float(pulse) / 1000000 * FREQUENCY * 4096);
}

int clampAngle(uint8_t channel, int angle) {
  if (angle < safeMin[channel]) return safeMin[channel];
  if (angle > safeMax[channel]) return safeMax[channel];
  return angle;
}

void writeServo(uint8_t channel, int angle) {
  pwm.setPWM(channel, 0, pulseWidth(clampAngle(channel, angle)));
}

void moveToNeutral() {
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
    writeServo(ch, neutralPose[ch]);
  }
}

// 🔥 ESTA ERA LA QUE TE FALTABA
void applyPoseFromProgmem(uint16_t rowIndex) {
  for (uint8_t idx = 0; idx < 8; idx++) {
    uint8_t channel = pgm_read_byte(&kActiveChannels[idx]);
    uint8_t angle = pgm_read_byte(&gaitSequence8[rowIndex][idx]);
    writeServo(channel, angle);
  }
}

// ================= LOG =================
void playFullSequenceOnce() {

  Serial.println("t,row,roll,pitch,yaw,i_arduino,i_servos");

  unsigned long lastIMU = 0;

  for (uint16_t row = 0; row < kGaitRows; row++) {

    applyPoseFromProgmem(row);

    unsigned long tStart = millis();

    while (millis() - tStart < (uint32_t)(kGaitDtMs * kWalkSpeedScale)) {

      if (millis() - lastIMU >= 50) {

        leerIMU();

        float i1 = leerCorriente712();
        float i2 = leerCorriente758();

        Serial.print(millis()); Serial.print(",");
        Serial.print(row); Serial.print(",");
        Serial.print(roll); Serial.print(",");
        Serial.print(pitch); Serial.print(",");
        Serial.print(yaw); Serial.print(",");
        Serial.print(i1); Serial.print(",");
        Serial.println(i2);

        lastIMU = millis();
      }
    }
  }

  moveToNeutral();
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);

  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  calibrarGyro();
  calibrarSensores();

  lastTime = millis();

  moveToNeutral();

  Serial.println("Sistema listo");
}

// ================= LOOP =================
void loop() {

  if (Serial.available()) {

    char cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r') return;

    if (cmd == 'w') playFullSequenceOnce();
    if (cmd == 'n') moveToNeutral();
  }
}