#include <TektiteRotEv.h>

RotEv rotev;

#define WHEELBASE 3.3f / 100.0f
#define DIAMETER 5.252f / 100.0f
#define CIRCUMFERENCE_M (DIAMETER * PI)
#define TPR 16384.0f
#define TPM (TPR / CIRCUMFERENCE_M)
#define MPT (CIRCUMFERENCE_M / TPR)

// ====================== GYRO ======================
float gyroBias = 0.0f;
float targetTheta = 0.0f;

// ====================== LOCALIZATION ======================
float theta = 0.0f;
float encoderBasedTheta = 0.0f;
float axialDist = 0.0f, targetAxialDist = 0.0f;
float lateralDist = 0.0f, targetLateralDist = 0.0f;
float vel = 0.0f;
float omega = 0.0f;

float avgDist = 0.0f;
double dT = 0.0f;

// ====================== VOLTAGE ======================
float vRatio = 0.0f;
float motorVoltage = 6.0f;
float batteryVoltage = 0.0f;

// ====================== STATE ======================
volatile bool goPressed = false;
volatile bool going = false;

// ===========================================================

float normalizeDelta(float delta) {
  if (delta > PI) {
    delta -= 2.0f * PI;
  } else if (delta < -PI) {
    delta += 2.0f * PI;
  }
  return delta;
}

void calcBias(int samples) {
  rotev.ledWrite(0, 0, 0.1f);
  gyroBias = 0.0f;

  for (int i = 0; i < samples; i++) {
    gyroBias += rotev.readYawRate();
    delay(1);
  }

  gyroBias /= samples;
}

void calcDT() {
  static unsigned long prevTime = micros();
  unsigned long currTime = micros();
  dT = (currTime - prevTime) / 1e6f;
  prevTime = currTime;
}

void readIMU() {
  static unsigned long lastIMUTime = micros();
  unsigned long now = micros();

  if ((now - lastIMUTime) >= 1000) {
    omega = rotev.readYawRate() - gyroBias;
    if (fabs(omega) < 0.005f) omega = 0.0f;
    lastIMUTime = now;
  }

  theta += dT * omega;
}
void calcEncoder() {
  float left = rotev.enc1Angle();
  float right = rotev.enc2Angle();

  static float leftPrev = rotev.enc1Angle();
  static float rightPrev = rotev.enc2Angle();

  float dL = normalizeDelta(left - leftPrev);
  float dR = normalizeDelta(rightPrev - right);

  avgDist = (dL + dR) / (2.0f * PI) * CIRCUMFERENCE_M / 2.0f;

  encoderBasedTheta += (dR - dL) / WHEELBASE;

  axialDist += avgDist * cos(theta);
  lateralDist += avgDist * sin(theta);

  vel = avgDist / dT;

  leftPrev = left;
  rightPrev = right;

  Serial.println(axialDist);
}

void updateLocalization() {
  calcDT();
  calcEncoder();
  readIMU();
}

void BRAKE() {
  rotev.motorWrite1(0.0f);
  rotev.motorWrite2(0.0f);
}

void resetLocalization() {
  theta = 0.0f;
  encoderBasedTheta = 0.0f;
  axialDist = 0.0f, targetAxialDist = 0.0f;
  lateralDist = 0.0f, targetLateralDist = 0.0f;


  avgDist = 0.0f;
}

//PID stuff
float kpHeading = 0.27f, kdHeading = 0.2;
float kiVel = 0.25f;

void setup() {
  Serial.begin(9600);
  rotev.begin();

  delay(250);
  calcBias(2000);

  batteryVoltage = rotev.getVoltage();
  vRatio = (motorVoltage / batteryVoltage);
}

/*

Score calc
Score = 100 + distError * 2 -0.5(110 - canDist) + abs(targetTime - actualTime)

UPenn Score 
72.85
// UPenn 7.9m 16.5s

*/

float count = 0.0f;

void ev(float dist = 7.5f, float targetTime = 14.5f, float canDist = 0.025) {
  float timeStart = millis();
  const float initialLateral = 1.0f - canDist / 2.0f;
  float totalDist = (2.0f * sqrt(pow(initialLateral, 2.0f) + pow(0.35f * dist, 2.0f)) + 0.3f * dist);
  float targetVelocity = 1.5f * totalDist / targetTime;

  float distRemaining = dist;
  float lateralTarget = 0.0f;
  float velocityCorrection = 0.0f;
  float off = 0.0f;
  float prevLateral = 0.0f;
  float headingIntegral = 0.0;
  float kiHeading = 0.001;
  if (count > 0) calcBias(2000);
  count++;

  while (true) {

    //  if (rotev.stopButtonPressed()) break;
    if (distRemaining < 0.005f) break;


    if (axialDist < 0.52f * dist) {
      lateralTarget = initialLateral;
    } else {
      headingIntegral = 0.0;
      //kiHeading = 0;
      lateralTarget = 0.0f;
    }

    float lateralError = lateralTarget - lateralDist;
    // use atan if it needs to be dampened
    headingIntegral += (lateralError - theta) * dT;
    float headingCorrection = kpHeading * (lateralError - theta) + headingIntegral * kiHeading + kdHeading * (lateralError - prevLateral) / dT;

    if (distRemaining > 1.0f) {
      // positionalCorrection = pidPosition.ut(dist, axialDist);
      velocityCorrection += dT * kiVel * (targetVelocity - vel);

    } else {
      // positionalCorrection = 0.0f;

      rotev.ledWrite(0.0f, 0.1f, 0.1f);

      float timeElapsed = (millis() - timeStart) / 1000.0f;
      float timeRemaining = targetTime - timeElapsed;

      float targetVelocityFinal = distRemaining / fabs(timeRemaining);

      off += (targetVelocityFinal - vel) * dT;

      velocityCorrection = 0.4 * off;
    }

    float M1 = velocityCorrection + headingCorrection;
    float M2 = velocityCorrection - headingCorrection;

    rotev.motorWrite1(constrain(-M1 * vRatio, -1.0f, 1.0f));
    rotev.motorWrite2(constrain(M2 * vRatio, -1.0f, 1.0f));

    distRemaining = dist - axialDist;
    prevLateral = lateralError;
  }

  BRAKE();
}


void setup1() {
}

void loop1() {
  if (going) {
    updateLocalization();
  } else {
    calcDT();
  }
}
int goPressedCount = 0;
void loop() {
  resetLocalization();

  if (rotev.goButtonPressed()) {
    goPressedCount++;
    goPressed = true;
    rotev.ledWrite(0.0f, 0.1f, 0.0f);

  } else if (goPressed && !rotev.goButtonPressed()) {
    goPressed = false;
    going = true;
    rotev.motorEnable(true);

  } else if (rotev.stopButtonPressed()) {
    going = false;
    rotev.motorEnable(false);
    rotev.ledWrite(0.1f, 0.0f, 0.0f);

  } else if (!going) {
    rotev.ledWrite(0.1f, 0.0f, 0.1f);
  }

  if (going) {
    ev();
    going = false;
  } else {
    BRAKE();
  }
}
