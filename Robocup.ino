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

// Left motor
// IN1 = D3
// IN2 = D5

// Right motor
// IN1 = D2
// IN2 = D4

const int LEFT_IN1  = 3;
const int LEFT_IN2  = 5;

const int RIGHT_IN1 = 2;
const int RIGHT_IN2 = 4;


// =====================================================
// PID
// =====================================================

float Kp = 1.00;
float Ki = 0.00;
float Kd = 0.00;

const int CENTER = 3500;


// =====================================================
// ERROR EXPONENT
// =====================================================

// 1.0 = completely linear
// 1.2 = mild
// 1.4 = moderate
// 2.0 = strong
// 3.0 = VERY strong
//
// The error is normalized first, so:
// +3500 always remains +3500
// -3500 always remains -3500

float ERROR_EXPONENT = 2.0;


float integral = 0;
float previousError = 0;

unsigned long previousTime;


// =====================================================
// MOTOR
// =====================================================

int baseSpeed = 192;

const int MAX_SPEED = 255;
const int MAX_CORRECTION = 320;


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
  }

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
//
// Normalizes error to -1 ... +1
// Applies exponent
// Scales back to -3500 ... +3500
//
// This guarantees the endpoints stay unchanged.
//
// Example with exponent = 3:
//
// raw       used
// ----------------
//  500       10
// 1000       82
// 1500      276
// 2000      653
// 2500     1276
// 3000     2204
// 3500     3500
//
// Negative values are mirrored.
// =====================================================

float shapeError(float rawError) {

  float normalized =
    rawError / (float)CENTER;

  // Keep it inside -1 ... +1
  normalized = constrain(
    normalized,
    -1.0,
    1.0
  );

  float shaped =
    copysign(
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

  float dt =
    (currentTime - previousTime) / 1000000.0;

  previousTime = currentTime;

  if (dt <= 0)
    dt = 0.001;


  // -----------------------------------------------
  // RAW ERROR
  // -----------------------------------------------

  float rawError =
    CENTER - position;


  // -----------------------------------------------
  // NONLINEAR ERROR
  // -----------------------------------------------

  float usedError =
    shapeError(rawError);


  // -----------------------------------------------
  // Integral
  // -----------------------------------------------

  integral += usedError * dt;

  integral = constrain(
    integral,
    -10000,
    10000
  );


  // -----------------------------------------------
  // Derivative
  // -----------------------------------------------

  float derivative =
    (usedError - previousError) / dt;

  previousError = usedError;


  // -----------------------------------------------
  // PID
  // -----------------------------------------------

  return
    Kp * usedError +
    Ki * integral +
    Kd * derivative;
}


// =====================================================
// L298N MOTOR CONTROL
//
// Positive = forward
// Negative = reverse
//
// PWM is applied directly to the active IN pin.
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

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  // Motors off
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

  int position =
    getLinePosition();

  float rawError =
    CENTER - position;

  float usedError =
    shapeError(rawError);

  float correction =
    calculatePID(position);

  correction = constrain(
    correction,
    -MAX_CORRECTION,
    MAX_CORRECTION
  );


  // -----------------------------------------------
  // DIFFERENTIAL DRIVE
  // -----------------------------------------------

  int leftSpeed =
    baseSpeed + correction;

  int rightSpeed =
    baseSpeed - correction;


  leftSpeed =
    constrain(
      leftSpeed,
      -MAX_SPEED,
      MAX_SPEED
    );

  rightSpeed =
    constrain(
      rightSpeed,
      -MAX_SPEED,
      MAX_SPEED
    );


  setMotor(
    LEFT_IN1,
    LEFT_IN2,
    leftSpeed
  );

  setMotor(
    RIGHT_IN1,
    RIGHT_IN2,
    rightSpeed
  );


  // -----------------------------------------------
  // DEBUG
  // -----------------------------------------------

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
