#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "BluetoothSerial.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
BluetoothSerial SerialBT;

#define MIN_PULSE_WIDTH 600
#define MAX_PULSE_WIDTH 2600
#define FREQUENCY 50

static const int NUM_CHANNELS = 16;
static const int MAX_LINE_LEN = 256;

char lineBuffer[MAX_LINE_LEN];
int lineLength = 0;

int currentPos[NUM_CHANNELS] = {
  100, 90, 90, 90,
   89, 86, 90, 90,
   90, 98, 85, 93,
   93, 97, 100, 90
};

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

unsigned long lastPacketMs = 0;
const unsigned long communicationTimeoutMs = 1500;

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

void applyPose(const int target[NUM_CHANNELS]) {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    writeServo(ch, target[ch]);
  }
}

void moveToNeutral() {
  int neutral[NUM_CHANNELS] = {
    100, 90, 90, 90,
     89, 86, 90, 90,
     90, 98, 85, 93,
     93, 97, 100, 90
  };
  applyPose(neutral);
}

bool parsePacket(char *line, int &timeMs, int target[NUM_CHANNELS]) {
  char *token = strtok(line, ",");
  if (token == NULL) {
    return false;
  }

  timeMs = atoi(token);

  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    token = strtok(NULL, ",");
    if (token == NULL) {
      return false;
    }
    target[ch] = atoi(token);
  }

  return true;
}

void processLine(char *line) {
  if (strncmp(line, "time_ms", 7) == 0) {
    return;
  }

  int target[NUM_CHANNELS];
  int packetTimeMs = 0;
  if (!parsePacket(line, packetTimeMs, target)) {
    Serial.println("Paquete invalido");
    return;
  }

  applyPose(target);
  lastPacketMs = millis();

  Serial.print("Aplicado paquete t=");
  Serial.print(packetTimeMs);
  Serial.println(" ms");
}

void readBluetoothPackets() {
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      lineBuffer[lineLength] = '\0';
      if (lineLength > 0) {
        processLine(lineBuffer);
      }
      lineLength = 0;
      continue;
    }

    if (lineLength < MAX_LINE_LEN - 1) {
      lineBuffer[lineLength++] = c;
    } else {
      lineLength = 0;
      Serial.println("Linea demasiado larga, descartada");
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);

  moveToNeutral();

  if (!SerialBT.begin("QuadrupedESP32")) {
    Serial.println("No se pudo iniciar Bluetooth");
    while (true) {
      delay(1000);
    }
  }

  lastPacketMs = millis();
  Serial.println("ESP32 listo. Bluetooth: QuadrupedESP32");
}

void loop() {
  readBluetoothPackets();

  if (millis() - lastPacketMs > communicationTimeoutMs) {
    moveToNeutral();
    lastPacketMs = millis();
  }
}
