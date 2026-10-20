#include <Adafruit_NeoPixel.h>
#include <math.h>

// =====================================================
// WS2812B LEDS
// =====================================================

Adafruit_NeoPixel strip(8, 35, NEO_GRB + NEO_KHZ800);

const int LED_BRIGHTNESS = 16;
const int LED_DIM = 128;
const int LED_DELAY = 40;

void showLedDot(int pixel) {
  strip.clear();
  strip.setPixelColor(pixel, strip.Color(255, 255, 255));
  strip.show();
}

// =====================================================
// CALIBRATION LED SWEEP
// =====================================================

void calibrationAnimation() {
  static int pixel = 0;
  static int direction = 1;
  static unsigned long lastUpdate = 0;

  if (millis() - lastUpdate >= LED_DELAY) {
    lastUpdate = millis();

    showLedDot(pixel);

    pixel += direction;

    if (pixel >= 7) {
      pixel = 7;
      direction = -1;
    }
    else if (pixel <= 0) {
      pixel = 0;
      direction = 1;
    }
  }
}

void calibrationComplete() {
  strip.clear();
  strip.show();
  delay(100);

  for (int i = 0; i < 2; i++) {
    strip.fill(strip.Color(255, 255, 255));
    strip.show();
    delay(100);

    strip.clear();
    strip.show();
    delay(100);
  }
}

// =====================================================
// LIVE ERROR DISPLAY
// =====================================================

void showError(float error) {
  // Map shaped PID error to LED position.
  // Negative = left side, positive = right side.
  float index = (error + 3500.0) * 7.0 / 7000.0;
  index = constrain(index, 0.0, 7.0);

  showLedDot((int)(index + 0.5));
}

// =====================================================
// SENSORS
// =====================================================

const int sensorPins[8] = {
  A0, A2, A4, A6,
  A8, A10, A12, A14
};

const int positions[8] = {
  0, 1000, 2000, 3000,
  4000, 5000, 6000, 7000
};

int sensorMin[8];
int sensorMax[8];

// =====================================================
// L298N
// =====================================================

const int LEFT_IN1  = 3;
const int LEFT_IN2  = 5;
const int RIGHT_IN1 = 2;
const int RIGHT_IN2 = 4;

// =====================================================
// PID
// =====================================================

float Kp = 1.20;
float Ki = 0.00;
float Kd = 0.00;

const int CENTER = 3500;

float ERROR_EXPONENT = 1.4;

float integral = 0;
float previousError = 0;
unsigned long previousTime;

// =====================================================
// MOTOR
// =====================================================

int baseSpeed = 160;

const int MAX_SPEED = 255;
const int MAX_CORRECTION = 298;

// =====================================================
// CALIBRATION
// =====================================================

void calibrateSensors() {
  for (int i = 0; i < 8; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  Serial.println("================================");
  Serial.println("CALIBRATION");
  Serial.println("Move over WHITE + BLACK");
  Serial.println("10 seconds");
  Serial.println("================================");

  unsigned long startTime = millis();

  while (millis() - startTime < 10000) {
    for (int i = 0; i < 8; i++) {
      int reading = analogRead(sensorPins[i]);

      if (reading < sensorMin[i])
        sensorMin[i] = reading;

      if (reading > sensorMax[i])
        sensorMax[i] = reading;
    }

    calibrationAnimation();
  }

  calibrationComplete();

  Serial.println("CALIBRATION COMPLETE");

  for (int i = 0; i < 8; i++) {
    Serial.print("S");
    Serial.print(i);
    Serial.print(" MIN=");
    Serial.print(sensorMin[i]);
    Serial.print(" MAX=");
    Serial.println(sensorMax[i]);
  }

  Serial.println();
}

// =====================================================
// LINE POSITION
// =====================================================

int getLinePosition() {
  long weightedSum = 0;
  long total = 0;

  for (int i = 0; i < 8; i++) {
    int reading = analogRead(sensorPins[i]);

    int value = map(
      reading,
      sensorMin[i],
      sensorMax[i],
      0,
      127
    );

    value = constrain(value, 0, 127);

    weightedSum += (long)value * positions[i];
    total += value;
  }

  if (total == 0)
    return CENTER;

  return weightedSum / total;
}

// =====================================================
// NONLINEAR ERROR
// =====================================================

float shapeError(float rawError) {
  float normalized = rawError / (float)CENTER;

  normalized = constrain(normalized, -1.0, 1.0);

  float shaped = copysign(
    pow(abs(normalized), ERROR_EXPONENT),
    normalized
  );

  return shaped * CENTER;
}

// =====================================================
// PID
// =====================================================

float calculatePID(int position) {
  unsigned long currentTime = micros();

  float dt = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;

  if (dt <= 0)
    dt = 0.001;

  float rawError = CENTER - position;
  float usedError = shapeError(rawError);

  integral += usedError * dt;
  integral = constrain(integral, -10000, 10000);

  float derivative = (usedError - previousError) / dt;
  previousError = usedError;

  return Kp * usedError + Ki * integral + Kd * derivative;
}

// =====================================================
// MOTOR CONTROL
// =====================================================

void setMotor(int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    analogWrite(in1, speed);
    digitalWrite(in2, LOW);
  }
  else if (speed < 0) {
    digitalWrite(in1, LOW);
    analogWrite(in2, -speed);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  strip.clear();
  strip.show();

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  setMotor(LEFT_IN1, LEFT_IN2, 0);
  setMotor(RIGHT_IN1, RIGHT_IN2, 0);

  delay(1000);

  calibrateSensors();

  previousTime = micros();

  Serial.println("PID START");
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  int position = getLinePosition();

  float rawError = CENTER - position;
  float usedError = shapeError(rawError);

  float correction = calculatePID(position);

  correction = constrain(
    correction,
    -MAX_CORRECTION,
    MAX_CORRECTION
  );

  // Display current shaped PID error
  showError(usedError);

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  setMotor(LEFT_IN1, LEFT_IN2, leftSpeed);
  setMotor(RIGHT_IN1, RIGHT_IN2, rightSpeed);

  Serial.print("POS:");
  Serial.print(position);

  Serial.print(" RAW:");
  Serial.print(rawError);

  Serial.print(" USED:");
  Serial.print(usedError);

  Serial.print(" PID:");
  Serial.print(correction);

  Serial.print(" L:");
  Serial.print(leftSpeed);

  Serial.print(" R:");
  Serial.println(rightSpeed);

  delay(5);
}
