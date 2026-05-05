/***************************************************************
 * uROS - Firmware ESP32-S3 para robot cuadrúpedo
 *
 * Funciones:
 * 1. Recibe 12 ángulos del nodo de IK via micro-ROS
 * 2. Convierte ángulos a PWM y los envía a servos DS3240 via PCA9685
 * 3. Publica pseudo-feedback opcional
 * 4. Log de corriente usando sensor ASC712 30A
 * 5. Log de orientación y aceleración con IMU MPU-9250
 ***************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <micro_ros_arduino.h>
#include <std_msgs/msg/float32_multi_array.h>

// Librería para MPU-9250
#include <MPU9250_asukiaaa.h>

// ===================== CONFIGURACIÓN PCA9685 =====================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define MIN_PULSE 120   // Ajusta según tus servos DS3240
#define MAX_PULSE 620
#define NUM_SERVOS 12

// ===================== SENSOR DE CORRIENTE =====================
#define CURRENT_PIN 34   // Pin analógico del ASC712 30A
float currentValue = 0;

// ===================== IMU =====================
MPU9250_asukiaaa myIMU;
float ax, ay, az, gx, gy, gz;

// ===================== VARIABLES ROS =====================
rcl_subscription_t subscriber;
rcl_publisher_t publisher;   // Pseudo-feedback
std_msgs__msg__Float32MultiArray angles_msg;   // Mensaje feedback
std_msgs__msg__Float32MultiArray command_msg;  // Mensaje recibido (12 ángulos)

// micro-ROS Agent
#define MICRO_ROS_AGENT_IP "192.168.1.100"  // IP Jetson Nano
#define MICRO_ROS_AGENT_PORT 8888
#define WIFI_SSID "TuSSID"
#define WIFI_PASS "TuPassword"

// ===================== FUNCIONES AUXILIARES =====================
uint16_t angleToPWM(float angle) {
  return map(angle, 0, 180, MIN_PULSE, MAX_PULSE);
}

// ===================== CALLBACK DE SUSCRIPCIÓN =====================
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;

  // Enviar a servos
  for(int i = 0; i < NUM_SERVOS; i++){
      float angle = msg->data.data[i];
      uint16_t pwm_value = angleToPWM(angle);
      pwm.setPWM(i, 0, pwm_value);
  }

  // Publicación opcional de feedback
  angles_msg.data = *msg;
  rcl_publish(&publisher, &angles_msg, NULL);
}

// ===================== CONFIGURACIÓN MICRO-ROS =====================
void setupMicroROS() {
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, MICRO_ROS_AGENT_IP, MICRO_ROS_AGENT_PORT);
  delay(2000);

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rclc_executor_t executor;
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  // Suscripción a comandos de IK
  rclc_subscription_init_default(
      &subscriber,
      &support.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "quad_jointController/commands"
  );
  rclc_executor_add_subscription(
      &executor,
      &subscriber,
      &command_msg,
      &subscription_callback,
      ON_NEW_DATA
  );

  // Publisher opcional
  rclc_publisher_init_default(
      &publisher,
      &support.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "quad_hw_feedback"
  );
}

// ===================== CONFIGURACIÓN PCA9685 =====================
void setupPCA9685() {
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50); // Frecuencia típica de servos 50Hz
}

// ===================== CONFIGURACIÓN IMU =====================
void setupIMU() {
  Wire.begin();
  myIMU.setWire(&Wire);
  myIMU.beginAccel();
  myIMU.beginGyro();
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  setupPCA9685();
  setupIMU();
  setupMicroROS();
}

// ===================== LOOP =====================
void loop() {
  // 1️⃣ Ejecuta micro-ROS executor para recibir mensajes
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // 2️⃣ Leer corriente del ASC712
  int raw = analogRead(CURRENT_PIN);
  // Convertir a corriente (aprox para sensor 30A, ajuste según datasheet)
  currentValue = ((raw / 4095.0) * 3.3 - 2.5) * 30.0 / 0.185;
  Serial.print("Corriente [A]: ");
  Serial.println(currentValue);

  // 3️⃣ Leer IMU
  myIMU.accelUpdate();
  myIMU.gyroUpdate();
  ax = myIMU.accelX();
  ay = myIMU.accelY();
  az = myIMU.accelZ();
  gx = myIMU.gyroX();
  gy = myIMU.gyroY();
  gz = myIMU.gyroZ();

  Serial.print("IMU Accel: "); Serial.print(ax); Serial.print(", "); Serial.print(ay); Serial.print(", "); Serial.println(az);
  Serial.print("IMU Gyro: "); Serial.print(gx); Serial.print(", "); Serial.print(gy); Serial.print(", "); Serial.println(gz);

  delay(50); // 20Hz aprox
}

