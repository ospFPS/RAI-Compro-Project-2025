

#include <Wire.h>
#include <Servo.h>

// ------------------ Motor driver pins ------------------

#define L_IN1 10
#define L_IN2 12
#define L_PWM 11
#define R_IN1 8
#define R_IN2 4
#define R_PWM 6


// Encoder pins.
#define L_ENC 2   
#define R_ENC 3   

#define SERVO_PIN 9

// Ultrasonic sensor pins.

#define US_TRIG 7
#define US_ECHO 5


const int SERVO_UP_POS   = 180;  
const int SERVO_DOWN_POS = 60;  

const uint8_t TURN_SPEED = 150;


const int PPR = 20;
const float WHEEL_DIAMETER_CM = 6.5f;
const float CM_PER_PULSE = (3.14159265f * WHEEL_DIAMETER_CM) / (float)PPR;
const float WALL_THRESHOLD_CM = 4.5f;

// ------------------ BMI160 register definitions ------------------

#define BMI160_ADDR_LOW   0x68
#define BMI160_ADDR_HIGH  0x69
#define BMI160_REG_CHIP_ID 0x00
#define BMI160_REG_GYR_X_L 0x0C
#define BMI160_REG_GYR_X_H 0x0D
#define BMI160_REG_GYR_Y_L 0x0E
#define BMI160_REG_GYR_Y_H 0x0F
#define BMI160_REG_GYR_Z_L 0x10
#define BMI160_REG_GYR_Z_H 0x11
#define BMI160_REG_CMD     0x7E
#define BMI160_CMD_SOFTRESET   0xB6
#define BMI160_CMD_GYRO_NORMAL 0x15
#define BMI160_REG_GYR_RANGE   0x43
// ------------------ BMI160 minimal driver ------------------
struct BMI160 {
  uint8_t addr = BMI160_ADDR_LOW;

  bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
  }

  bool readMulti(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((int)addr, (int)len);
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
  }

  bool beginAt(uint8_t a) {
    addr = a;
    writeReg(BMI160_REG_CMD, BMI160_CMD_SOFTRESET);
    delay(100);
    writeReg(BMI160_REG_CMD, BMI160_CMD_GYRO_NORMAL);
    delay(100);
    writeReg(BMI160_REG_GYR_RANGE, 0x00); // ±2000 dps
    uint8_t id;
    Wire.beginTransmission(addr);
    Wire.write(BMI160_REG_CHIP_ID);
    Wire.endTransmission(false);
    Wire.requestFrom((int)addr, 1);
    id = Wire.read();
    return id == 0xD1;
  }

  bool beginAuto() {
    return beginAt(BMI160_ADDR_LOW) || beginAt(BMI160_ADDR_HIGH);
  }

  bool readGyroRaw(int16_t g[3]) {
    uint8_t buf[6];
    if (!readMulti(BMI160_REG_GYR_X_L, buf, 6)) return false;
    g[0] = (int16_t)((buf[1] << 8) | buf[0]);
    g[1] = (int16_t)((buf[3] << 8) | buf[2]);
    g[2] = (int16_t)((buf[5] << 8) | buf[4]);
    return true;
  }
};

// ------------------ Gyro integrator ------------------
struct GyroIntegrator {
  BMI160 *dev;
  float offset = 0;
  float angle = 0;
  unsigned long lastMs;
  const float LSB_PER_DPS = 16.4f;  // ±2000 dps scale

  bool begin(BMI160 &d) {
    dev = &d;
    lastMs = millis();
    return true;
  }

  bool calibrate(int samples = 200) {
    long sum = 0;
    for (int i = 0; i < samples; i++) {
      int16_t g[3];
      dev->readGyroRaw(g);
      sum += g[2];
      delay(10);
    }
    offset = sum / (float)samples;
    return true;
  }

  void reset() { angle = 0; lastMs = millis(); }

  bool update() {
    int16_t g[3];
    if (!dev->readGyroRaw(g)) return false;
    float w = (g[2] - offset) / LSB_PER_DPS;  // Z‑axis dps
    unsigned long now = millis();
    float dt = (now - lastMs) / 1000.0f;
    lastMs = now;
    angle += w * dt;
    return true;
  }
};


BMI160 bmi;
GyroIntegrator gyro;
Servo scoop;



volatile long L_pulses = 0;
volatile long R_pulses = 0;

void L_isr();
void R_isr();

bool missionComplete = false;

// ------------------ Motor helper functions ------------------

void driveForward(uint8_t speed) {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, HIGH);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, HIGH);
  analogWrite(L_PWM, speed);
  analogWrite(R_PWM, speed);
}

void driveBackward(uint8_t speed) {
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, HIGH);
  digitalWrite(R_IN2, LOW);
  analogWrite(L_PWM, speed);
  analogWrite(R_PWM, speed);
}

void driveTurnRight(uint8_t speed) {
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, HIGH);
  analogWrite(L_PWM, speed);
  analogWrite(R_PWM, speed);
}


void driveTurnLeft(uint8_t speed) {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, HIGH);
  digitalWrite(R_IN1, HIGH);
  digitalWrite(R_IN2, LOW);
  analogWrite(L_PWM, speed);
  analogWrite(R_PWM, speed);
}


void stopMotors() {
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
}

// ------------------ Servo helper functions ------------------
void servoDown() {
  scoop.write(SERVO_DOWN_POS);
  delay(600);
}

void servoUp() {
  scoop.write(SERVO_UP_POS);
  delay(600);
}

// ------------------ Encoder and ultrasonic helpers ------------------

void L_isr() {
  L_pulses++;
}

void R_isr() {
  R_pulses++;
}

void resetEncoders() {
  noInterrupts();
  L_pulses = 0;
  R_pulses = 0;
  interrupts();
}


float averageDistanceCm() {
  noInterrupts();
  long l = L_pulses;
  long r = R_pulses;
  interrupts();
  float avgPulses = (l + r) / 2.0f;
  return avgPulses * CM_PER_PULSE;
}


float readDistanceCM() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);
  long duration = pulseIn(US_ECHO, HIGH, 30000);  
  if (duration == 0) return -1.0f;
  return duration * 0.0343f * 0.5f;
}


void moveForwardCm(float cm, uint8_t speed = 180) {
  resetEncoders();
  driveForward(speed);
  while (averageDistanceCm() < cm) {
    gyro.update();
    if (WALL_THRESHOLD_CM > 0) {
      float d = readDistanceCM();
      if (d > 0 && d <= WALL_THRESHOLD_CM) {
        break;  // wall detected
      }
    }
  }
  stopMotors();
  delay(100);
}


void moveBackwardCm(float cm, uint8_t speed = 180) {
  resetEncoders();
  driveBackward(speed);
  while (averageDistanceCm() < cm) {
  
    ;
  }
  stopMotors();
  delay(100);
}

// ------------------ Movement primitives ------------------


void turnRight90() {

  gyro.reset();
  while (gyro.angle < 90.0f) {
    gyro.update();
    driveTurnRight(TURN_SPEED);
  }
  stopMotors();
  delay(200);
}

void turnLeft90() {
  gyro.reset();
  while (gyro.angle > -90.0f) {
    gyro.update();
    driveTurnLeft(TURN_SPEED);
  }
  stopMotors();
  delay(200);
}


void run() {
  moveForwardCm(15);
  turnRight90();
  moveForwardCm(30);
  turnLeft90();
  moveBackwardCm(15);
  servoDown();
  servoUp();
  moveForwardCm(15);
  turnRight90();
  moveForwardCm(15);
  moveForwardCm(15);

  // === Deliver Phase ===
  turnLeft90();
  moveForwardCm(70);
  turnRight90();
  moveForwardCm(15);
  turnRight90();
  moveForwardCm(15);
  turnLeft90();
  moveForwardCm(15);
  turnRight90();
  moveForwardCm(30);
  turnRight90();
  moveForwardCm(15);
  turnLeft90();
  moveForwardCm(15);
  turnRight90();
  moveBackwardCm(15);
  servoDown();
  servoUp();

  // === Return Phase ===
  moveForwardCm(15);
  turnRight90();
  moveForwardCm(20);
  turnRight90();
  moveForwardCm(15);
  turnLeft90();
  moveForwardCm(30);
  turnLeft90();
  moveForwardCm(15);
  turnRight90();
  moveForwardCm(15);
  turnLeft90();
  moveForwardCm(15);
  turnLeft90();
  moveForwardCm(70);
  turnRight90();
  moveForwardCm(15);
  turnLeft90();
  moveForwardCm(15);


  stopMotors();

  scoop.detach();
}


void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  stopMotors();

  attachInterrupt(digitalPinToInterrupt(L_ENC), L_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC), R_isr, RISING);


  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);

  scoop.attach(SERVO_PIN);
  scoop.write(SERVO_UP_POS);
  delay(500);

  Serial.println("Init BMI160...");
  if (!bmi.beginAuto()) {
    Serial.println("BMI160 failed. Halting.");
    while (1);
  }
  Serial.println("BMI160 ready.");
  gyro.begin(bmi);
  Serial.println("Calibrating gyro...");
  gyro.calibrate();
  gyro.reset();
  Serial.println("Calibration done.");
}

void loop() {
  if (!missionComplete) {
    run();
    missionComplete = true;
  }
}
