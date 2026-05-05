#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "esp32_servo_sequence.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define MIN_PULSE_WIDTH 600
#define MAX_PULSE_WIDTH 2600
#define FREQUENCY 50

static const int NUM_CHANNELS = 16;

int safeMin[NUM_CHANNELS] = {
  85, 50, 40, 0,
  74, 50, 40, 0,
   0, 83, 20, 43,
  77, 27, 50, 0
};

int safeMax[NUM_CHANNELS] = {
  115, 160, 140, 180,
  104, 160, 140, 180,
  180, 113, 125, 143,
  113, 147, 150, 180
};

int neutralPose[NUM_CHANNELS] = {
  100, 90, 90, 90,
   89, 86, 90, 90,
   90, 98, 85, 93,
   93, 97, 100, 90
};

int currentPos[NUM_CHANNELS] = {
  100, 90, 90, 90,
   89, 86, 90, 90,
   90, 98, 85, 93,
   93, 97, 100, 90
};

int pulseWidth(int angle) {
  int pulse = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  return int(float(pulse) / 1000000 * FREQUENCY * 4096);
}

int clampAngle(int channel, int angle) {
  if (angle < safeMin[channel]) {
    return safeMin[channel];
  }
  if (angle > safeMax[channel]) {
    return safeMax[channel];
  }
  return angle;
}

void writeServo(int channel, int angle) {
  int safeAngle = clampAngle(channel, angle);
  pwm.setPWM(channel, 0, pulseWidth(safeAngle));
  currentPos[channel] = safeAngle;
}

void applyPose(const uint16_t *poseRow) {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    writeServo(ch, (int)poseRow[ch + 1]);
  }
}

void moveToNeutral() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    writeServo(ch, neutralPose[ch]);
  }
}

void playSequenceOnce() {
  if (kGaitRows < 1) {
    return;
  }

  applyPose(gaitSequence[0]);

  for (int row = 1; row < kGaitRows; row++) {
    uint16_t previousTime = gaitSequence[row - 1][0];
    uint16_t currentTime = gaitSequence[row][0];
    uint16_t dt = currentTime - previousTime;

    applyPose(gaitSequence[row]);
    delay(dt);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);

  moveToNeutral();

  Serial.println("ESP32 listo para reproducir secuencia local.");
  Serial.print("Filas cargadas: ");
  Serial.println(kGaitRows);

  delay(2000);
}

void loop() {
  playSequenceOnce();
  delay(1000);
  moveToNeutral();
  delay(3000);
}
