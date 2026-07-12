#include <Wire.h>

// ═══════════════════════════════════════════════════════════
// VARIABLES DE CONTROL
// ═══════════════════════════════════════════════════════════

const unsigned long LOOP_DT_MS = 10;

const float COMP_ALPHA = 1;

float Kp = 35;
float Ki = 40;;
float Kd = 0.5;
float setpoint = 2.31;
const float INTEGRAL_LIMIT = 100;
const int DEAD_ZONE_PWM = 0;
const float ANGLE_DEADBAND = 0;
const float MAX_ANGLE = 25.0;

// --- Anti-windup (v7) ---
// Solo se integra cuando el error es pequeno (cerca del equilibrio).
// En errores grandes, P y D hacen el trabajo y la I no acumula sesgo.
const float INTEGRAL_ACTIVE_BAND = 5.0;   // grados
// Si el pitch se pasa de este angulo, se descarta la integral acumulada.
const float INTEGRAL_RESET_ANGLE = 5.0;  // grados

// ═══════════════════════════════════════════════════════════
// VARIABLES DE HARDWARE
// ═══════════════════════════════════════════════════════════

// MPU-6050
const int MPU_ADDR = 0x68;
const float GYRO_SENS = 131.0;

// L298N — Motor A (izquierdo)
const int enA = 9;
const int in1 = 8;
const int in2 = 7;

// L298N — Motor B (derecho)
const int enB = 3;
const int in3 = 5;
const int in4 = 4;

// PWM

const int MAX_PWM = 255;

// ═══════════════════════════════════════════════════════════
// ESTADO GLOBAL
// ═══════════════════════════════════════════════════════════

int16_t ax, ay, az;
int16_t gx, gy, gz;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

float pitch = 0.0;
float gyroPitchRate = 0.0;

float integral = 0.0;
float errorPrev = 0.0;

unsigned long lastTime;
unsigned long nextLoop = 0;
float dt;

int currentSpeed = 0;

// ═══════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);

  setupMotors();
  setupMPU();

  Serial.println(F("Calibrando... manten el robot quieto."));
  delay(1500);
  calibrateGyro();
  Serial.println(F("Calibracion completa."));

  readMPU6050();
  pitch = computeAccelPitch();
  errorPrev = setpoint - pitch;

  lastTime = millis();
  nextLoop = millis() + LOOP_DT_MS;
}

// ═══════════════════════════════════════════════════════════
// LOOP PRINCIPAL
// ═══════════════════════════════════════════════════════════

void loop() {
  if (millis() < nextLoop) return;
  nextLoop += LOOP_DT_MS;

  readMPU6050();

  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  updatePitch();

  // Corte de seguridad por ángulo extremo
  if (abs(pitch) > MAX_ANGLE) {
    stopMotors();
    integral = 0;
    errorPrev = setpoint - pitch;
    logState();
    return;
  }

  // Reset de integral en ángulo crítico: si ya estamos muy inclinados,
  // el windup acumulado juega en contra. Lo soltamos.
  if (abs(pitch) > INTEGRAL_RESET_ANGLE) {
    integral = 0;
  }

  float error = setpoint - pitch;

  // Zona muerta angular: no corregir desvíos pequeños
  if (abs(error) < ANGLE_DEADBAND) {
    stopMotors();
    errorPrev = error;
    logState();
    return;
  }

  float output = computePID(error);
  applyMotors(output);
  logState();
}

// ═══════════════════════════════════════════════════════════
// PID
// ═══════════════════════════════════════════════════════════

float computePID(float error) {
  // Conditional integration (anti-windup): solo acumular cuando
  // el error es pequeño. En desvíos grandes, P y D hacen el trabajo.
  if (abs(error) < INTEGRAL_ACTIVE_BAND) {
    integral += error * dt;
    integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  }

  float derivative = (error - errorPrev) / dt;
  errorPrev = error;

  return Kp * error + Ki * integral + Kd * derivative;
}

// ═══════════════════════════════════════════════════════════
// MOTORES
// ═══════════════════════════════════════════════════════════

void setupMotors() {
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  stopMotors();
}

void applyMotors(float output) {
  float mag = abs(output);
  if (mag > 255.0) mag = 255.0;

  // Mapeo lineal [0, 255] → [DEAD_ZONE_PWM, MAX_PWM]
  int speed = DEAD_ZONE_PWM + (int)(mag * (MAX_PWM - DEAD_ZONE_PWM) / 255.0);
  speed = constrain(speed, DEAD_ZONE_PWM, MAX_PWM);

  if (output > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
    currentSpeed = speed;
  } else {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);  digitalWrite(in4, LOW);
    currentSpeed = -speed;
  }

  analogWrite(enA, speed);
  analogWrite(enB, speed);
}

void stopMotors() {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  currentSpeed = 0;
}

// ═══════════════════════════════════════════════════════════
// MPU-6050
// ═══════════════════════════════════════════════════════════

void setupMPU() {
  Wire.begin();
  Wire.setClock(400000);

  // Despertar el MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Activar DLPF interno (~44 Hz BW)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);
}

void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

void calibrateGyro() {
  long sumX = 0, sumY = 0, sumZ = 0;
  const int samples = 3000;

  for (int i = 0; i < samples; i++) {
    readMPU6050();
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(3);
  }

  gx_offset = (float)sumX / samples;
  gy_offset = (float)sumY / samples;
  gz_offset = (float)sumZ / samples;

  Serial.print(F("Offsets - gx: ")); Serial.print(gx_offset);
  Serial.print(F("  gy: "));         Serial.print(gy_offset);
  Serial.print(F("  gz: "));         Serial.println(gz_offset);
}

float computeAccelPitch() {
  float ax_f = (float)ax;
  float ay_f = (float)ay;
  float az_f = (float)az;

  float denom = sqrt(ay_f * ay_f + az_f * az_f);
  if (denom < 0.0001) denom = 0.0001;

  return atan2(-ax_f, denom) * 180.0 / PI;
}

void updatePitch() {
  gyroPitchRate = ((float)gy - gy_offset) / GYRO_SENS;
  float accelPitch = computeAccelPitch();
  pitch = COMP_ALPHA * (pitch + gyroPitchRate * dt)
        + (1.0 - COMP_ALPHA) * accelPitch;
}

// ═══════════════════════════════════════════════════════════
// LOG
// ═══════════════════════════════════════════════════════════

void logState() {
  static byte counter = 0;
  if (++counter < 5) return;
  counter = 0;

  Serial.print(F("Pitch:"));  Serial.print(pitch, 2);
  Serial.print(F(" gRate:")); Serial.print(gyroPitchRate, 2);
  Serial.print(F(" PWM:"));   Serial.println(currentSpeed);
  Serial.print(F(" Setpo:"));   Serial.println(setpoint);
}
