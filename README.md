# 🤖 Quad_WS v1.0
### Robot cuadrúpedo basado en la plataforma open-source **Hyperdog**
Proyecto de grado desarrollado con **ROS 2 Humble**, **micro-ROS**, **Jetson Nano**, y **ESP32-S3**.

---

## 🧠 Descripción general

**Quad_WS** es un workspace ROS 2 adaptado para controlar un robot cuadrúpedo de diseño propio, con un enfoque modular y open-source.  
La arquitectura está basada en la plataforma **Hyperdog**, pero con modificaciones específicas para integrar:

- **Firmware uROS** en ESP32-S3
- Control de servos DS3240 mediante PCA9685
- Lectura de sensores IMU MPU-9250 y sensor de corriente ACS712
- Comunicación completa ROS ↔ micro-ROS vía UDP

El objetivo del proyecto es lograr un control estable, escalable y probado en hardware real.  
Esta versión 1.0 constituye la **base funcional del sistema**.

---