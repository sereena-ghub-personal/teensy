#include <Servo.h>
#include <math.h>    // for fabs()

Servo servoRight;
Servo servoLeft;

// Motor connections (check with your wiring!)
constexpr int pinFeedbackLeft  = 7;
constexpr int pinFeedbackRight = 8;
constexpr int pinControlLeft   = 5;
constexpr int pinControlRight  = 6;

char keyCommand;

// Encoder raw measurements
unsigned long highRight = 0, highLeft = 0;
unsigned long lowRight  = 0, lowLeft  = 0;
unsigned long cycleTimeRight = 0, cycleTimeLeft = 0;
double dutyCycleRight = 0.0, dutyCycleLeft = 0.0;

// Current + previous angles (0–360°)
double thetaRight  = 0.0, thetaLeft  = 0.0;
double thetaRightP = 0.0, thetaLeftP = 0.0;

// Accumulated rotation in degrees (for distance)
double totalDegRight = 0.0;
double totalDegLeft  = 0.0;

// Target wheel rotation for about 2 meters
// Adjust this after a quick calibration if needed
const double DEGREE_TARGET_2M = 3478.0;   // ≈ 9.66 turns * 360°

/********** FUNCTION DECLARATIONS **********/
void calculateThetaRight();
void calculateThetaLeft();
void stopRobot();
void updateEncodersAndAccumulate();
void moveForward2m();
void moveBackward2m();


void setup() {
  Serial.begin(9600);
  pinMode(pinFeedbackRight, INPUT);
  pinMode(pinFeedbackLeft,  INPUT);
}

void loop() {
  if (Serial.available() > 0) {
    keyCommand = Serial.read();
    
    switch (keyCommand) {

      // ----- short forward (~10–20 cm) -----
      case 'f':
        servoRight.attach(pinControlRight);
        servoLeft.attach(pinControlLeft);

        servoRight.write(89);   // your forward calibration
        servoLeft.write(98);

        delay(500);             // distance depends on this

        stopRobot();            // also detaches
        break;

      // ----- short backward -----
      case 'b':
        servoRight.attach(pinControlRight);
        servoLeft.attach(pinControlLeft);

        servoRight.write(98);   // backward
        servoLeft.write(89);

        delay(500);

        stopRobot();
        break;

      // ----- STOP (any time) -----
      case 's':
        stopRobot();
        break;

      // ----- 2 m forward using encoders -----
      case 'F':
        moveForward2m();
        break;

      // ----- 2 m backward using encoders -----
      case 'B':
        moveBackward2m();
        break;

      default:
        break;
    }
  }
}


/***************** BASIC STOP *****************/

void stopRobot() {
  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);

  // Your neutral / stop value from earlier code
  servoRight.write(94);
  servoLeft.write(94);

  delay(100);    // let servos receive stop PWM

  servoRight.detach();
  servoLeft.detach();
}


/********* ENCODER UPDATE + ACCUMULATION *********/

void updateEncodersAndAccumulate() {
  // store previous angles
  thetaRightP = thetaRight;
  thetaLeftP  = thetaLeft;

  // read new angles from encoders
  calculateThetaRight();
  calculateThetaLeft();

  // compute angle change
  double dR = thetaRight - thetaRightP;
  double dL = thetaLeft  - thetaLeftP;

  // handle wrap-around at 0/360 so we get the shortest step
  if (dR > 180.0)  dR -= 360.0;
  if (dR < -180.0) dR += 360.0;

  if (dL > 180.0)  dL -= 360.0;
  if (dL < -180.0) dL += 360.0;

  // accumulate absolute rotation
  totalDegRight += fabs(dR);
  totalDegLeft  += fabs(dL);
}


/***************** 2 METER FORWARD *****************/

void moveForward2m() {
  totalDegRight = 0.0;
  totalDegLeft  = 0.0;

  // init previous angles so first diff is small
  calculateThetaRight();
  calculateThetaLeft();

  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);

  unsigned long startTime = millis();   // safety timeout

  while (totalDegRight < DEGREE_TARGET_2M &&
         totalDegLeft  < DEGREE_TARGET_2M) {

    // --- SAFETY 1: time-out in case encoders fail ---
    if (millis() - startTime > 6000) {   // 6 seconds, adjust if needed
      Serial.println("Timeout in moveForward2m, stopping.");
      break;
    }

    // --- SAFETY 2: allow 's' to stop immediately ---
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's') {
        Serial.println("Stop command (s) received in moveForward2m.");
        stopRobot();
        return;   // exit function immediately
      }
    }

    // forward motion (your calibrated values)
    servoRight.write(89);
    servoLeft.write(98);

    // update encoder-based rotation
    updateEncodersAndAccumulate();

    // debug print
    Serial.print("Fwd2m | totalDegRight: ");
    Serial.print(totalDegRight);
    Serial.print("  totalDegLeft: ");
    Serial.println(totalDegLeft);
  }

  stopRobot();
}


/***************** 2 METER BACKWARD *****************/

void moveBackward2m() {
  totalDegRight = 0.0;
  totalDegLeft  = 0.0;

  // init previous angles
  calculateThetaRight();
  calculateThetaLeft();

  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);

  unsigned long startTime = millis();   // safety timeout

  while (totalDegRight < DEGREE_TARGET_2M &&
         totalDegLeft  < DEGREE_TARGET_2M) {

    // --- SAFETY 1: time-out ---
    if (millis() - startTime > 6000) {
      Serial.println("Timeout in moveBackward2m, stopping.");
      break;
    }

    // --- SAFETY 2: allow 's' to stop immediately ---
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's') {
        Serial.println("Stop command (s) received in moveBackward2m.");
        stopRobot();
        return;
      }
    }

    // backward motion (reverse speeds)
    servoRight.write(98);
    servoLeft.write(89);

    updateEncodersAndAccumulate();

    // debug print
    Serial.print("Back2m | totalDegRight: ");
    Serial.print(totalDegRight);
    Serial.print("  totalDegLeft: ");
    Serial.println(totalDegLeft);
  }

  stopRobot();
}


/***************** ENCODER FUNCTIONS *****************/

// Your original formulas from Parallax datasheet are used here

void calculateThetaRight() {
  highRight = pulseIn(pinFeedbackRight, HIGH);
  lowRight  = pulseIn(pinFeedbackRight, LOW);
  cycleTimeRight = highRight + lowRight;

  if (cycleTimeRight == 0) {
    // no valid pulse → keep previous thetaRight
    return;
  }

  dutyCycleRight = (double)(highRight * 100.0) / (double)cycleTimeRight;

  thetaRight = (360.0 - 1.0)
               - ((dutyCycleRight - 2.9) * 360.0) / (97.1 - 2.9 + 1.0);
}

void calculateThetaLeft() {
  highLeft = pulseIn(pinFeedbackLeft, HIGH);
  lowLeft  = pulseIn(pinFeedbackLeft, LOW);
  cycleTimeLeft = highLeft + lowLeft;

  if (cycleTimeLeft == 0) {
    // no valid pulse → keep previous thetaLeft
    return;
  }

  dutyCycleLeft = (double)(highLeft * 100.0) / (double)cycleTimeLeft;

  thetaLeft = (360.0 - 1.0)
              - ((dutyCycleLeft - 2.9) * 360.0) / (97.1 - 2.9 + 1.0);
}
