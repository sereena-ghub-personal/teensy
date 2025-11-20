#include <Servo.h>

Servo servoRight;
Servo servoLeft;

int pos = 0;
byte pinServoLeft    = 5;
byte pinServoRight   = 6;
byte pinFeedbackLeft = 7;
byte pinFeedbackRight= 8;

unsigned long highRight = 0, highLeft = 0;
unsigned long lowRight  = 0, lowLeft  = 0;
char keyCommand;
double dutyCycleRight = 0.0, dutyCycleLeft = 0.0;
unsigned long cycleTimeRight, cycleTimeLeft;
double totalCycle = 1098.901;
double thetaRight = 0.0, thetaLeft = 0.0;
double thetaRightPrev = 0.0, thetaLeftPrev = 0.0;
int turnsRight = 0, turnsLeft = 0;
double deltaThetaRight = 0, deltaThetaLeft = 0;
double totalThetaRight = 0, totalThetaLeft = 0;
int PcontrolRight = 0, PcontrolLeft = 0;

void calculateThetaRight();
void calculateThetaLeft();
void Pcontroller(int dir);
void printInfos();

// ---- helper functions for movement ----
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopRobot();

void setup() {
  Serial.begin(9600);
  pinMode(pinFeedbackRight, INPUT);
  pinMode(pinFeedbackLeft, INPUT);
} 

void loop() {
  if (Serial.available() > 0) {
    keyCommand = Serial.read();
    
    switch (keyCommand) {
      case 'f':   // forward
        moveForward();
        break;

      case 'b':   // backward
        moveBackward();
        break;

      case 'l':   // turn left
        turnLeft();
        break;

      case 'r':   // turn right
        turnRight();
        break;

      case 's':   // stop
        stopRobot();
        break;

      default:
        break;
    }
  }
}

// ---------------- MOVEMENT ----------------

// forward ~10–20 cm (adjust delay)
void moveForward() {
  servoRight.attach(pinServoRight);
  servoLeft.attach(pinServoLeft);

  // Your prof’s calibration: forward
  servoRight.write(89 + PcontrolRight);  // slightly <90
  servoLeft.write(98 + PcontrolLeft);    // slightly >90

  delay(500);   // time = distance

  servoRight.detach();
  servoLeft.detach();

  Pcontroller(1);   // direction +1 (forward)
}

// backward
void moveBackward() {
  servoRight.attach(pinServoRight);
  servoLeft.attach(pinServoLeft);

  // reverse of forward
  servoRight.write(98 + PcontrolRight);
  servoLeft.write(89 + PcontrolLeft);

  delay(500);

  servoRight.detach();
  servoLeft.detach();

  Pcontroller(-1);  // direction -1 (backward)
}

// rotate left in place (~45°)
void turnLeft() {
  servoRight.attach(pinServoRight);
  servoLeft.attach(pinServoLeft);

  // for turning: both to same "side" of stop value
  // here: both > stop → one goes fwd, one back (since mounted opposite)
  servoRight.write(98 + PcontrolRight);
  servoLeft.write(98 + PcontrolLeft);

  delay(350);   // adjust for ~45° (increase/decrease)

  servoRight.detach();
  servoLeft.detach();

  Pcontroller(0);   // just print infos for now
}

// rotate right in place (~45°)
void turnRight() {
  servoRight.attach(pinServoRight);
  servoLeft.attach(pinServoLeft);

  // both < stop value
  servoRight.write(89 + PcontrolRight);
  servoLeft.write(89 + PcontrolLeft);

  delay(350);   // adjust for ~45°

  servoRight.detach();
  servoLeft.detach();

  Pcontroller(0);
}

// stop robot (hold still)
void stopRobot() {
  servoRight.attach(pinServoRight);
  servoLeft.attach(pinServoLeft);

  // your stop value ~94 (from your code)
  servoRight.write(94);
  servoLeft.write(94);

  delay(200);

  servoRight.detach();
  servoLeft.detach();
}

// -------------- CONTROL PART (STILL SIMPLE) --------------

void Pcontroller(int dir) {
  // later you’ll:
  // 1) call calculateThetaRight/Left()
  // 2) compute error and adjust PcontrolRight/Left
  // For now: just print debug values.
  printInfos();
}

void printInfos() {
  Serial.print("ThetaRight: ");
  Serial.print(thetaRight);
  Serial.print(" , ThetaLeft: ");
  Serial.println(thetaLeft);

  Serial.print("deltaThetaRight: ");       
  Serial.print(deltaThetaRight);
  Serial.print(" , deltaThetaLeft: ");       
  Serial.println(deltaThetaLeft);

  Serial.print("totalThetaRight: ");         
  Serial.print(totalThetaRight);
  Serial.print(" , totalThetaLeft: ");       
  Serial.println(totalThetaLeft);

  Serial.print("PcontrolRight: ");         
  Serial.print(PcontrolRight);
  Serial.print(" , PcontrolLeft: ");       
  Serial.println(PcontrolLeft);
}

void calculateThetaRight() {
  // later: copy code from Parallax 360 datasheet
}

void calculateThetaLeft() {
  // later: copy code from Parallax 360 datasheet
}
