# Robot Balancín — Sir Balance III
### Diseño, Fabricación y Control PID de un Péndulo Invertido Autobalanceado

Proyecto final del curso **ME4250-1 Mecatrónica** · Universidad de Chile · FCFM · DIMEC · 2025

---

## Integrantes

| Nombre |
|--------|
| A. Benjamín Castillo V. |
| Ignacio Saavedra |
| Jorge Brahim |
| Pedro Gómez |

---

## Prototipo
![Sir Balance III](Avance%203%20Mecatrónica/Fotos%20Prototipo%20Avance%203/PHOTO-2026-06-22-11-16-01%20(1).jpg)

---

## Descripción del proyecto

Sir Balance III es un robot autobalanceado de dos ruedas que resuelve el problema clásico del péndulo invertido: un sistema inherentemente inestable que requiere control activo y continuo para mantenerse en pie. Su desarrollo integró disciplinas de diseño mecánico, electrónica embebida y teoría de control, constituyendo una plataforma de aprendizaje mecatrónico completa.

El sistema utiliza un sensor **MPU-6050** (acelerómetro + giroscopio) para medir el ángulo de inclinación en tiempo real mediante fusión sensorial con filtro complementario. Un **Arduino UNO R3** ejecuta un algoritmo **PID a 100 Hz** para mantener el equilibrio, comandando dos motores DC a través de un driver L298N. La estructura física fue diseñada en Fusion 360 y construida en madera con accesorios impresos en 3D, incluyendo un sombrero de copa y mostacho como estética de época.

### Hardware

| Componente | Descripción |
|------------|-------------|
| Arduino UNO R3 | Microcontrolador principal (ATmega328P, 16 MHz) |
| MPU-6050 (GY-521) | IMU: acelerómetro ±16g + giroscopio ±2000°/s, protocolo I²C |
| Driver L298N | Puente H doble para control independiente de ambos motores |
| Motor DC 6V × 2 | Con caja reductora 48:1 para torque suficiente a bajas velocidades |
| Baterías 18650 × 2 | 3350 mAh c/u, autonomía estimada ~6h 37min |

### Parámetros de control (configuración final)

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| Kp | 2,700 | Ganancia proporcional |
| Ki | 0,002 | Ganancia integral |
| Kd | 0,029 | Ganancia derivativa |
| Setpoint | 1° | Ángulo objetivo de equilibrio |
| ANGLE_DEADBAND | 0,5° | Banda muerta angular para evitar correcciones espurias |
| DEAD_ZONE_PWM | 100/255 | PWM mínimo para vencer la inercia de los motores |
| Frecuencia de loop | 100 Hz | Período de muestreo y control |

### Métricas de desempeño

| Métrica | Valor |
|---------|-------|
| Sobrepaso | ~15% |
| Error estacionario | ~0,4° |
| Tiempo de establecimiento | ~2,5 s |

---

## Conclusiones

Se diseñó, fabricó y sintonizó exitosamente Sir Balance III, un robot balancín de dos ruedas capaz de mantener equilibrio de forma autónoma mediante control PID.

El método de Ziegler–Nichols proveyó un punto de partida útil (Kp=2,7), aunque los valores teóricos resultaron inestables y requirieron ajuste iterativo manual hasta llegar a Ki=0,002 y Kd=0,029. La zona muerta de los motores (PWM≈100/255) fue un desafío crítico, resuelto con mapeo lineal de la salida PID al rango [100–255] y una banda muerta angular de 0,5°.

El proceso iterativo de tres avances demostró que el diseño mecánico, la elección de componentes y el algoritmo de control deben concebirse como un sistema unificado, donde cada decisión afecta el desempeño global.

---

## Referencias

[1] Mechatronic Store. *Motor DC 3V–6V con motorreductor.*
https://www.mechatronicstore.cl/motor-dc-3v-6v-con-motorreductor/

[2] Motorba. *Driver Motor Arduino L298N.*
https://motorba.com.ar/driver-motor-arduino-l298n/

[3] Descubre Arduino. *MPU6050, Diagrama de Pines, Circuito y Conexión con Arduino.*
https://descubrearduino.com/mpu6050/

[4] Descubre Arduino. *Arduino UNO R3: Qué es, Características, Precio y Pinout.*
https://descubrearduino.com/arduino-uno/

[5] PDFs Clases Auxiliares ME4250-1, 2025.
