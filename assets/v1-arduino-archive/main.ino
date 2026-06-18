#include <Wire.h>
#include <Servo.h>

// --- КОНФИГУРАЦИЯ ПИНОВ ---
const int PIN_ENA = 5;  const int PIN_IN1 = 6;  const int PIN_IN2 = 7;
const int PIN_SERVO = 9;
const int PIN_TRIG_F = 8; const int PIN_ECHO_F = 10; // Передний датчик
const int PIN_TRIG_S = 11; const int PIN_ECHO_S = 12; // Боковой (правый) датчик
const int MPU_ADDR = 0x68;

Servo steer;

// --- ПАРАМЕТРЫ ДВИЖЕНИЯ ---
int CENTER = 180;         // Твоя точка прямолинейного движения
int BASE_SPEED = 255;     // Скорость на прямых
int TARGET_DIST = 35;     // Дистанция до стены в см
bool isClockwise = false;  // Поменяй на false, если едешь против часовой

// PID коэффициенты для езды вдоль стены
float kp = 4.0; 
float kd = 1.5;
float lastSideError = 0;

// Переменные гироскопа
float currentAngle = 0;
unsigned long lastTime;

// Состояния робота
enum State { FOLLOW_WALL, TURN };
State currentState = FOLLOW_WALL;
float turnTargetAngle = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  steer.attach(PIN_SERVO);
  steer.write(CENTER);

  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_TRIG_F, OUTPUT); pinMode(PIN_ECHO_F, INPUT);
  pinMode(PIN_TRIG_S, OUTPUT); pinMode(PIN_ECHO_S, INPUT);

  // Старт MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission(true);

  delay(3000); // Время поставить робота на старт
  lastTime = millis();
}

void loop() {
  updateAngle();
  int distFront = getDist(PIN_TRIG_F, PIN_ECHO_F);
  int distSide = getDist(PIN_TRIG_S, PIN_ECHO_S);

  if (currentState == FOLLOW_WALL) {
    // Езда прямо вдоль стены
    driveStraight(distSide);
    
    // Если стена впереди близко — начинаем поворот
    if (distFront > 5 && distFront < 60) {
      currentState = TURN;
      if (isClockwise) turnTargetAngle = currentAngle - 85; // Поворот направо
      else turnTargetAngle = currentAngle + 85;            // Поворот налево
    }
  } 
  else if (currentState == TURN) {
    // Режим поворота
    executeTurn();
  }
}

// Функция езды вдоль стены (PID)
void driveStraight(int currentDist) {
  float error = currentDist - TARGET_DIST;
  float p = error * kp;
  float d = (error - lastSideError) * kd;
  lastSideError = error;

  float correction = p + d;
  
  // Если едем против часовой, инвертируем руль
  if (!isClockwise) correction = -correction;

  int angle = CENTER + constrain(correction, -30, 30);
  steer.write(angle);
  
  moveMotors(BASE_SPEED);
}

// Функция поворота по гироскопу
void executeTurn() {
  int turnSteer;
  if (isClockwise) {
    turnSteer = CENTER - 30; // Выворачиваем руль вправо (учти, CENTER у тебя 210)
    if (currentAngle <= turnTargetAngle) currentState = FOLLOW_WALL;
  } else {
    turnSteer = CENTER + 30; // Выворачиваем руль влево
    if (currentAngle >= turnTargetAngle) currentState = FOLLOW_WALL;
  }
  
  steer.write(turnSteer);
  moveMotors(BASE_SPEED - 20); // Чуть медленнее на повороте
}

// Вспомогательные функции
void moveMotors(int speed) {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_ENA, speed);
}

int getDist(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 30000); // Таймаут 30мс
  int dist = duration * 0.034 / 2;
  if (dist == 0) return 400; // Если ничего не видит
  return dist;
}

void updateAngle() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  int16_t gz = Wire.read()<<8|Wire.read();
  
  float gyroRate = gz / 131.0;
  if (abs(gyroRate) > 0.3) currentAngle += gyroRate * dt;
}