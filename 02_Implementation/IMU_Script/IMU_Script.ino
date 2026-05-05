#include <Wire.h>

const int MPU = 0x69;  // tu dirección

// Datos crudos
int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;

// Offsets
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

// Ángulos
float roll = 0;
float pitch = 0;
float yaw = 0;

unsigned long lastTime;
float dt;

// ===== CALIBRACIÓN =====
void calibrarGyro() {
  Serial.println("Calibrando IMU... NO MOVER");

  for (int i = 0; i < 1000; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    gx_offset += Wire.read() << 8 | Wire.read();
    gy_offset += Wire.read() << 8 | Wire.read();
    gz_offset += Wire.read() << 8 | Wire.read();

    delay(2);
  }

  gx_offset /= 1000;
  gy_offset /= 1000;
  gz_offset /= 1000;

  Serial.println("Calibración lista");
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  // Despertar MPU
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  calibrarGyro();

  lastTime = millis();
}

// ===== LOOP =====
void loop() {

  // Tiempo
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // Leer IMU
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  Wire.read(); Wire.read(); // temp

  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  // Convertir
  float ax = AcX / 16384.0;
  float ay = AcY / 16384.0;
  float az = AcZ / 16384.0;

  float gx = (GyX - gx_offset) / 131.0;
  float gy = (GyY - gy_offset) / 131.0;
  float gz = (GyZ - gz_offset) / 131.0;

  // ===== ÁNGULOS (ajustables según montaje) =====
  float roll_acc  = atan2(ay, az) * 180 / PI;
  float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

  // ===== FILTRO COMPLEMENTARIO =====
  roll  = 0.96 * (roll  + gx * dt) + 0.04 * roll_acc;
  pitch = 0.90 * (pitch + gy * dt) + 0.10 * pitch_acc;
  yaw  += gz * dt;  // relativo

  // ===== SALIDA =====
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" | Pitch: ");
  Serial.print(pitch);
  Serial.print(" | Yaw: ");
  Serial.println(yaw);

  delay(10);
}