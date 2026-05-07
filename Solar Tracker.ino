// ================= PIN =================
#define LDR1 34
#define LDR2 35
#define LDR3 32
#define LDR4 33

#define STEP_X 18
#define DIR_X 19

#define STEP_Y 21
#define DIR_Y 22

// ================= PID PARAMETER =================
float Kp = 0.2;
float Ki = 0.0;
float Kd = 0.1;

// PID Variable X
float errorX, prevErrorX = 0;
float integralX = 0;
float derivativeX;
float outputX;

// PID Variable Y
float errorY, prevErrorY = 0;
float integralY = 0;
float derivativeY;
float outputY;

// Timing PID
unsigned long prevTime = 0;

// Timing gerak motor
unsigned long lastMoveX = 0;
unsigned long lastMoveY = 0;

// Interval gerak motor
const unsigned long moveIntervalX = 700;
const unsigned long moveIntervalY = 700;

// Deadband
const int deadband = 100;

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(STEP_X, OUTPUT);
  pinMode(DIR_X, OUTPUT);

  pinMode(STEP_Y, OUTPUT);
  pinMode(DIR_Y, OUTPUT);
}

// ================= STEP MOTOR =================
void stepMotor(int stepPin, int steps, int delayMicros) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delayMicros);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayMicros);
  }
}

// ================= PID FUNCTION =================
float PID_Control(float error, float &prevError, float &integral, float &derivative, float dt) {
  if (dt < 0.05) dt = 0.05;

  integral += error * dt;
  integral = constrain(integral, -1000, 1000);

  derivative = (error - prevError) / dt;

  float output = (Kp * error) +
                 (Ki * integral) +
                 (Kd * derivative);

  prevError = error;

  return output;
}

// ================= LOOP =================
void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;

  if (dt <= 0) dt = 0.01;
  prevTime = currentTime;

  // ===== READ LDR =====
  int ldr1 = analogRead(LDR1);
  int ldr2 = analogRead(LDR2);
  int ldr3 = analogRead(LDR3);
  int ldr4 = analogRead(LDR4);

  // ===== MAPPING LDR =====
  int kiri  = ldr1 + ldr2;
  int kanan = ldr3 + ldr4;

  int atas  = ldr2 + ldr3;
  int bawah = ldr1 + ldr4;

  // ===== HITUNG ERROR =====
  errorX = kiri - kanan;
  errorY = atas - bawah;

  // ===== DEADBAND =====
  if (abs(errorX) < deadband) errorX = 0;
  if (abs(errorY) < deadband) errorY = 0;

  // ===== PID OUTPUT =====
  outputX = PID_Control(errorX, prevErrorX, integralX, derivativeX, dt);
  outputY = PID_Control(errorY, prevErrorY, integralY, derivativeY, dt);

  // ===== KONVERSI PID KE JUMLAH STEP =====
  int stepsX = map(abs(outputX), 0, 2000, 0, 10);
  int stepsY = map(abs(outputY), 0, 2000, 0, 10);

  stepsX = constrain(stepsX, 0, 10);
  stepsY = constrain(stepsY, 0, 10);

  // ===== KONVERSI PID KE KECEPATAN MOTOR =====
  int speedX = map(abs(outputX), 0, 2000, 3000, 1200);
  int speedY = map(abs(outputY), 0, 2000, 3000, 1200);

  speedX = constrain(speedX, 1200, 3000);
  speedY = constrain(speedY, 1200, 3000);

  // ===== GERAK MOTOR X BERBASIS PID =====
  if (stepsX > 0) {
    if (currentTime - lastMoveX >= moveIntervalX) {
      digitalWrite(DIR_X, outputX > 0);
      stepMotor(STEP_X, stepsX, speedX);
      lastMoveX = currentTime;
    }
  }

  // ===== GERAK MOTOR Y BERBASIS PID =====
  if (stepsY > 0) {
    if (currentTime - lastMoveY >= moveIntervalY) {
      digitalWrite(DIR_Y, outputY > 0);
      stepMotor(STEP_Y, stepsY, speedY);
      lastMoveY = currentTime;
    }
  }

  // ===== DEBUG SERIAL =====
  Serial.print("LDR1: "); Serial.print(ldr1);
  Serial.print(" | LDR2: "); Serial.print(ldr2);
  Serial.print(" | LDR3: "); Serial.print(ldr3);
  Serial.print(" | LDR4: "); Serial.print(ldr4);

  Serial.print(" || ErrorX: "); Serial.print(errorX);
  Serial.print(" | PID_X: "); Serial.print(outputX);
  Serial.print(" | Steps_X: "); Serial.print(stepsX);

  Serial.print(" || ErrorY: "); Serial.print(errorY);
  Serial.print(" | PID_Y: "); Serial.print(outputY);
  Serial.print(" | Steps_Y: "); Serial.println(stepsY);

  delay(100);
}