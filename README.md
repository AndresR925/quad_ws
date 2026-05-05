# Robot Cuadrúpedo de Locomoción Polinómica con Validación Dinámica

Proyecto de grado para el programa de Ingeniería Mecatrónica - Universidad Mariana.

## 🚀 Resumen del Proyecto
Implementación de un robot cuadrúpedo de 12 GDL enfocado en la generación de trayectorias suaves mediante curvas de Bézier de 5to grado y validación experimental de errores en campo.

## 📁 Estructura del Repositorio
* **01_Simulation:** Modelado cinemático y agente DDPG (Reinforcement Learning).
* **02_Implementation:** Firmware de control en C++ y puente de comunicación Python-UART.
* **03_Data_Analysis:** Logs crudos de pruebas de laboratorio y scripts de procesamiento de datos.
* **Legacy_ROS2:** Fase de exploración inicial en ROS2 Humble.

## 📊 Resultados Técnicos
* **Algoritmo:** Trayectorias Bézier G5 (Continuidad C2).
* **Hardware:** Control distribuido con estabilidad temporal superior a la arquitectura ROS2 inicial.
* **Error de Posicionamiento:** 11.4 cm promedio en pruebas de desplazamiento lineal.
