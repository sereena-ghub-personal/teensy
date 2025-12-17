#include <Servo.h>
#include <math.h>

Servo servoRight;
Servo servoLeft;

// Motor connections
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

// Accumulated rotation
double totalDegRight = 0.0;
double totalDegLeft  = 0.0;
double totalTurnsRight = 0.0;
double totalTurnsLeft  = 0.0;

// --- Wheel geometry ---
const double wheelDiameter   = 0.07;                    // m
const double wheelCirc       = 3.14159 * wheelDiameter; // m

// --- Distance parameters ---
const double DEGREE_TARGET_2M = 3500.0;  // For 2m forward/backward
const double squareSide_m     = 0.40;    // 40 cm square

// --- Turn parameters ---
const double TURN_90_TURNS        = 0.50;
const double TURN_135_TURNS_RIGHT = TURN_90_TURNS * 1.5;
const double TURN_135_TURNS_LEFT  = TURN_90_TURNS * 1.0;

// --- Neutral PWM values ---
const int PWM_NEUTRAL_RIGHT = 94;
const int PWM_NEUTRAL_LEFT  = 94;

// ========== 2M MOTION PARAMETERS ==========
double KP_2M = 0.05;
const double MAX_ERROR = 80.0;
const int BASE_RIGHT_FWD  = 88;
const int BASE_LEFT_FWD   = 97;
const int BASE_RIGHT_BACK = 98;
const int BASE_LEFT_BACK  = 87;
const int SPEED_MIN = 83;
const int SPEED_MAX = 105;

// ========== SQUARE/DIAGONAL PARAMETERS ==========
const double K_RIGHT_PWM_FWD  = 4.4;
const double K_LEFT_PWM_FWD   = 4.3;
const double K_RIGHT_PWM_BACK = 4.3;
const double K_LEFT_PWM_BACK  = 4.7;
const double Kp_turns_FWD     = 0.3;
const double S_BASE_FWD_RIGHT = 1.08;
const double S_BASE_FWD_LEFT  = 0.98;
const double S_TURN_RIGHT     = 1.1;
const double S_TURN_LEFT      = 1.1;

// Function declarations
void calculateThetaRight();
void calculateThetaLeft();
void stopRobot();
void updateEncodersAndAccumulate();
void updateEncodersAndAccumulate_Turns();
double getAverageDistance();

// 2m motion functions
void moveForward2m();
void moveBackward2m();

// Square/Diagonal motion functions
void moveForwardDistance(double distance_m);
void turnRight90();
void turnRightByTurns(double targetTurns);
void turnLeftByTurns(double targetTurns);
void runSquare40cm();
void runDiagonal40cm();
void calibrateTurn90();

void setup() {
  Serial.begin(9600);
  pinMode(pinFeedbackRight, INPUT);
  pinMode(pinFeedbackLeft,  INPUT);

  Serial.println("========================================");
  Serial.println("   TEENSY ROBOT - COMBINED CONTROL");
  Serial.println("========================================");
  Serial.println("Commands:");
  Serial.println("  F = Move 2m forward");
  Serial.println("  B = Move 2m backward");
  Serial.println("  q = Run 40cm square");
  Serial.println("  d = Run diagonal pattern");
  Serial.println("  c = Calibrate 90° turn");
  Serial.println("  s = STOP");
  Serial.println("========================================\n");
}

void loop() {
  if (Serial.available() > 0) {
    keyCommand = Serial.read();
   
    switch (keyCommand) {
      case 's':
      case 'S':
        stopRobot();
        Serial.println("STOPPED");
        break;

      case 'F':
        Serial.println("\n[STARTING FORWARD 2m]");
        moveForward2m();
        break;

      case 'B':
        Serial.println("\n[STARTING BACKWARD 2m]");
        moveBackward2m();
        break;

      case 'q':
        Serial.println("\n>>> Running 40cm square path...");
        runSquare40cm();
        Serial.println(">>> Square path finished!\n");
        break;

      case 'd':
        Serial.println("\n>>> Running diagonal path...");
        runDiagonal40cm();
        Serial.println(">>> Diagonal path finished!\n");
        break;

      case 'c':
        Serial.println("\n>>> Starting 90° turn calibration...");
        calibrateTurn90();
        Serial.println(">>> Calibration run finished!\n");
        break;

      default:
        break;
    }
  }
}

/***************** STOP *****************/
void stopRobot() {
  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);
 
  servoRight.write(PWM_NEUTRAL_RIGHT);
  servoLeft.write(PWM_NEUTRAL_LEFT);
 
  delay(100);
 
  servoRight.detach();
  servoLeft.detach();
}

/********* ENCODER UPDATE (for 2m motion) *********/
void updateEncodersAndAccumulate() {
  thetaRightP = thetaRight;
  thetaLeftP  = thetaLeft;
 
  calculateThetaRight();
  calculateThetaLeft();
 
  double dR = thetaRight - thetaRightP;
  double dL = thetaLeft  - thetaLeftP;
 
  if (dR > 180.0)  dR -= 360.0;
  if (dR < -180.0) dR += 360.0;
  if (dL > 180.0)  dL -= 360.0;
  if (dL < -180.0) dL += 360.0;
 
  if (fabs(dR) < 90.0) {
    totalDegRight += fabs(dR);
  }
  if (fabs(dL) < 90.0) {
    totalDegLeft  += fabs(dL);
  }
}

/********* ENCODER UPDATE (for square/diagonal motion) *********/
void updateEncodersAndAccumulate_Turns() {
  thetaRightP = thetaRight;
  thetaLeftP  = thetaLeft;
 
  calculateThetaRight();
  calculateThetaLeft();
 
  double dR = thetaRight - thetaRightP;
  double dL = thetaLeft  - thetaLeftP;
 
  if (dR > 180.0)  dR -= 360.0;
  if (dR < -180.0) dR += 360.0;
  if (dL > 180.0)  dL -= 360.0;
  if (dL < -180.0) dL += 360.0;
 
  const double MAX_DEG_PER_UPDATE = 15.0;
  if(abs(dR) > MAX_DEG_PER_UPDATE) dR = 0;
  if(abs(dL) > MAX_DEG_PER_UPDATE) dL = 0;
 
  totalTurnsRight += fabs(dR) / 360.0;
  totalTurnsLeft  += fabs(dL) / 360.0;
}

double getAverageDistance() {
  double avgTurns = 0.5 * (totalTurnsRight + totalTurnsLeft);
  return avgTurns * wheelCirc;
}

/***************** 2 METER FORWARD *****************/
void moveForward2m() {
  totalDegRight = 0.0;
  totalDegLeft  = 0.0;
 
  calculateThetaRight();
  calculateThetaLeft();
 
  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);
 
  unsigned long startTime = millis();
  unsigned long lastPrintTime = millis();
 
  while (totalDegRight < DEGREE_TARGET_2M &&
         totalDegLeft  < DEGREE_TARGET_2M) {
   
    if (millis() - startTime > 50000) {
      Serial.println("Timeout in moveForward2m");
      break;
    }
   
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's' || c == 'S') {
        stopRobot();
        Serial.println("EMERGENCY STOP");
        return;
      }
    }
   
    double error = totalDegRight - totalDegLeft;
   
    if (error > MAX_ERROR) {
      error = MAX_ERROR;
    } else if (error < -MAX_ERROR) {
      error = -MAX_ERROR;
    }
   
    double correction = KP_2M * error;
   
    int speedRight = BASE_RIGHT_FWD + (int)correction;
    int speedLeft  = BASE_LEFT_FWD - (int)correction;
   
    speedRight = constrain(speedRight, SPEED_MIN, SPEED_MAX);
    speedLeft  = constrain(speedLeft, SPEED_MIN, SPEED_MAX);
   
    servoRight.write(speedRight);
    servoLeft.write(speedLeft);
   
    delay(8);
   
    updateEncodersAndAccumulate();
   
    if (millis() - lastPrintTime > 300) {
      double avgDeg = (totalDegRight + totalDegLeft) / 2.0;
      double progress = (avgDeg / DEGREE_TARGET_2M) * 100.0;
     
      Serial.print("FWD | Progress: ");
      Serial.print(progress, 1);
      Serial.print("% | R=");
      Serial.print(totalDegRight, 0);
      Serial.print("° L=");
      Serial.print(totalDegLeft, 0);
      Serial.print("° | Err=");
      Serial.print(error, 1);
      Serial.print("° | Speed R/L: ");
      Serial.print(speedRight);
      Serial.print("/");
      Serial.println(speedLeft);
     
      lastPrintTime = millis();
    }
  }
 
  stopRobot();
  Serial.println("\n--- FORWARD MOTION COMPLETE ---");
  Serial.print("Right wheel: ");
  Serial.print(totalDegRight, 1);
  Serial.println("°");
  Serial.print("Left wheel:  ");
  Serial.print(totalDegLeft, 1);
  Serial.println("°");
  Serial.print("Difference:  ");
  Serial.print(fabs(totalDegRight - totalDegLeft), 1);
  Serial.println("°");
  Serial.println("-------------------------------\n");
}

/***************** 2 METER BACKWARD *****************/
void moveBackward2m() {
  totalDegRight = 0.0;
  totalDegLeft  = 0.0;
 
  calculateThetaRight();
  calculateThetaLeft();
 
  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);
 
  unsigned long startTime = millis();
  unsigned long lastPrintTime = millis();
 
  while (totalDegRight < DEGREE_TARGET_2M &&
         totalDegLeft  < DEGREE_TARGET_2M) {
   
    if (millis() - startTime > 50000) {
      Serial.println("Timeout in moveBackward2m");
      break;
    }
   
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's' || c == 'S') {
        stopRobot();
        Serial.println("EMERGENCY STOP");
        return;
      }
    }
   
    double error = totalDegRight - totalDegLeft;
   
    if (error > MAX_ERROR) {
      error = MAX_ERROR;
    } else if (error < -MAX_ERROR) {
      error = -MAX_ERROR;
    }
   
    double correction = KP_2M * error;
   
    int speedRight = BASE_RIGHT_BACK - (int)correction;
    int speedLeft  = BASE_LEFT_BACK - (int)correction;
   
    speedRight = constrain(speedRight, SPEED_MIN, SPEED_MAX);
    speedLeft  = constrain(speedLeft, SPEED_MIN, SPEED_MAX);
   
    servoRight.write(speedRight);
    servoLeft.write(speedLeft);
   
    delay(8);
   
    updateEncodersAndAccumulate();
   
    if (millis() - lastPrintTime > 300) {
      double avgDeg = (totalDegRight + totalDegLeft) / 2.0;
      double progress = (avgDeg / DEGREE_TARGET_2M) * 100.0;
     
      Serial.print("BACK | Progress: ");
      Serial.print(progress, 1);
      Serial.print("% | R=");
      Serial.print(totalDegRight, 0);
      Serial.print("° L=");
      Serial.print(totalDegLeft, 0);
      Serial.print("° | Err=");
      Serial.print(error, 1);
      Serial.print("° | Speed R/L: ");
      Serial.print(speedRight);
      Serial.print("/");
      Serial.println(speedLeft);
     
      lastPrintTime = millis();
    }
  }
 
  stopRobot();
  Serial.println("\n--- BACKWARD MOTION COMPLETE ---");
  Serial.print("Right wheel: ");
  Serial.print(totalDegRight, 1);
  Serial.println("°");
  Serial.print("Left wheel:  ");
  Serial.print(totalDegLeft, 1);
  Serial.println("°");
  Serial.print("Difference:  ");
  Serial.print(fabs(totalDegRight - totalDegLeft), 1);
  Serial.println("°");
  Serial.println("--------------------------------\n");
}

/***************** FORWARD DISTANCE (for square/diagonal) *****************/
void moveForwardDistance(double distance_m) {
  double targetRot = distance_m / wheelCirc;
 
  Serial.print("\n--- FORWARD ");
  Serial.print(distance_m, 3);
  Serial.println(" m START ---");
 
  totalTurnsRight = 0.0;
  totalTurnsLeft  = 0.0;
 
  calculateThetaRight();
  calculateThetaLeft();
  thetaRightP = thetaRight;
  thetaLeftP  = thetaLeft;
 
  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);
 
  unsigned long startTime = millis();
 
  while (totalTurnsRight < targetRot &&
         totalTurnsLeft  < targetRot) {
   
    if (millis() - startTime > 20000) {  
      Serial.println("!! Timeout in moveForwardDistance, stopping.");
      break;
    }
   
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's') {
        Serial.println("!! Stop command (s) received.");
        stopRobot();
        return;
      }
    }
   
    updateEncodersAndAccumulate_Turns();
   
    double errorTurns = totalTurnsLeft - totalTurnsRight;
    double correction = Kp_turns_FWD * errorTurns;
   
    double S_right = S_BASE_FWD_RIGHT + correction;
    double S_left  = S_BASE_FWD_LEFT  - correction;
   
    S_right = constrain(S_right, -1.5, 1.5);
    S_left  = constrain(S_left,  -1.5, 1.5);
   
    int pwmRight = (int)round(PWM_NEUTRAL_RIGHT - K_RIGHT_PWM_FWD * S_right);
    int pwmLeft  = (int)round(PWM_NEUTRAL_LEFT  + K_LEFT_PWM_FWD  * S_left);
   
    pwmLeft = min(pwmLeft, PWM_NEUTRAL_LEFT + 5);
   
    pwmRight = constrain(pwmRight, 60, 130);
    pwmLeft  = constrain(pwmLeft,  60, 130);
   
    servoRight.write(pwmRight);
    servoLeft.write(pwmLeft);
   
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 400) {
      double dist = getAverageDistance();
      Serial.print("FWDd | dist ≈ ");
      Serial.print(dist, 3);
      Serial.print(" m | R: ");
      Serial.print(totalTurnsRight, 3);
      Serial.print(" L: ");
      Serial.print(totalTurnsLeft, 3);
      Serial.println(" turns");
      lastPrint = millis();
    }
  }
 
  double finalDist = getAverageDistance();
  Serial.println("\n--- FORWARD distance END ---");
  Serial.print("Final distance ≈ ");
  Serial.print(finalDist, 3);
  Serial.println(" m\n");
 
  stopRobot();
}

/***************** TURN RIGHT 90° *****************/
void turnRight90() {
  Serial.println("\n--- TURN RIGHT 90° START ---");
 
  totalTurnsRight = 0.0;
  totalTurnsLeft  = 0.0;
 
  calculateThetaRight();
  calculateThetaLeft();
  thetaRightP = thetaRight;
  thetaLeftP  = thetaLeft;
 
  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);
 
  unsigned long startTime = millis();
 
  while (totalTurnsRight < TURN_90_TURNS &&
         totalTurnsLeft  < TURN_90_TURNS) {
   
    if (millis() - startTime > 10000) {
      Serial.println("!! Timeout in turnRight90, stopping.");
      break;
    }
   
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's') {
        Serial.println("!! Stop command received.");
        stopRobot();
        return;
      }
    }
   
    updateEncodersAndAccumulate_Turns();
   
    double S_turn_R = S_TURN_RIGHT;
    double S_turn_L = S_TURN_LEFT;
   
    int pwmRight = (int)round(PWM_NEUTRAL_RIGHT + K_RIGHT_PWM_BACK * S_turn_R);
    int pwmLeft  = (int)round(PWM_NEUTRAL_LEFT  + K_LEFT_PWM_FWD  * S_turn_L);
   
    pwmRight = constrain(pwmRight, 60, 130);
    pwmLeft  = constrain(pwmLeft,  60, 130);
   
    servoRight.write(pwmRight);
    servoLeft.write(pwmLeft);
  }
 
  Serial.println("--- TURN RIGHT 90° END ---\n");
  stopRobot();
}

/***************** TURN RIGHT BY TURNS *****************/
void turnRightByTurns(double targetTurns) {
  Serial.println("\n--- TURN RIGHT (generic) START ---");
 
  totalTurnsRight = 0.0;
  totalTurnsLeft  = 0.0;
 
  calculateThetaRight();
  calculateThetaLeft();
  thetaRightP = thetaRight;
  thetaLeftP  = thetaLeft;
 
  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);
 
  unsigned long startTime = millis();
 
  while (totalTurnsRight < targetTurns &&
         totalTurnsLeft  < targetTurns) {
   
    if (millis() - startTime > 10000) {
      Serial.println("!! Timeout in turnRightByTurns.");
      break;
    }
   
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's') {
        stopRobot();
        return;
      }
    }
   
    updateEncodersAndAccumulate_Turns();
   
    int pwmRight = (int)round(PWM_NEUTRAL_RIGHT + K_RIGHT_PWM_BACK * S_TURN_RIGHT);
    int pwmLeft  = (int)round(PWM_NEUTRAL_LEFT  + K_LEFT_PWM_FWD  * S_TURN_LEFT);
   
    pwmRight = constrain(pwmRight, 60, 130);
    pwmLeft  = constrain(pwmLeft,  60, 130);
   
    servoRight.write(pwmRight);
    servoLeft.write(pwmLeft);
  }
 
  Serial.println("--- TURN RIGHT (generic) END ---\n");
  stopRobot();
}

/***************** TURN LEFT BY TURNS *****************/
void turnLeftByTurns(double targetTurns) {
  Serial.println("\n--- TURN LEFT (generic) START ---");
 
  totalTurnsRight = 0.0;
  totalTurnsLeft  = 0.0;
 
  calculateThetaRight();
  calculateThetaLeft();
  thetaRightP = thetaRight;
  thetaLeftP  = thetaLeft;
 
  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);
 
  unsigned long startTime = millis();
 
  while (totalTurnsRight < targetTurns) {
   
    if (millis() - startTime > 10000) {
      Serial.println("!! Timeout in turnLeftByTurns.");
      break;
    }
   
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's') {
        stopRobot();
        return;
      }
    }
   
    updateEncodersAndAccumulate_Turns();
   
    double S_turn_R = 1.4; //was 1.5;
    double S_turn_L = 2.0; //was 0.7;
   
    int pwmRight = (int)round(PWM_NEUTRAL_RIGHT - K_RIGHT_PWM_FWD * S_turn_R);
    int pwmLeft  = (int)round(PWM_NEUTRAL_LEFT  - K_LEFT_PWM_BACK * S_turn_L);
   
    pwmRight = constrain(pwmRight, 60, 130);
    pwmLeft  = constrain(pwmLeft,  60, 130);
   
    servoRight.write(pwmRight);
    servoLeft.write(pwmLeft);
  }
 
  Serial.println("--- TURN LEFT (generic) END ---\n");
  stopRobot();
}

/***************** RUN 40CM SQUARE *****************/
void runSquare40cm() {
  for (int i = 0; i < 4; ++i) {
    Serial.print("\n=== SQUARE: side ");
    Serial.print(i + 1);
    Serial.println(" of 4 ===");
   
    moveForwardDistance(squareSide_m);
    delay(500);
   
    if (i < 3) {
      turnRight90();
      delay(500);
    }
  }
 
  Serial.println("Final 90° turn to restore starting orientation...");
  turnRight90();
  delay(500);
}

/***************** RUN DIAGONAL PATTERN *****************/
void runDiagonal40cm() {
  Serial.println("\n--- DIAGONAL PATH START ---");
 
  const double side_m     = squareSide_m;
  const double diagonal_m = squareSide_m * 1.41421356237;
 
  // 1) First straight 40cm
  Serial.println("Segment 1: straight side");
  moveForwardDistance(side_m);
  delay(500);
 
  // 2) Turn RIGHT 135°
  Serial.println("Turn RIGHT 135°");
  turnRightByTurns(TURN_135_TURNS_RIGHT);
  delay(500);
 
  // 3) First diagonal
  Serial.println("Segment 2: diagonal");
  moveForwardDistance(diagonal_m);
  delay(500);
 
  // 4) Turn LEFT 135°
  Serial.println("Turn LEFT 135°");
  turnLeftByTurns(TURN_135_TURNS_LEFT);
  delay(500);
 
  // 5) Second straight 40cm
  Serial.println("Segment 3: straight side");
  moveForwardDistance(side_m);
  delay(500);
 
  // 6) Turn LEFT 135°
  Serial.println("Turn LEFT 135°");
  turnLeftByTurns(TURN_135_TURNS_LEFT);
  delay(500);
 
  // 7) Second diagonal
  Serial.println("Segment 4: diagonal");
  moveForwardDistance(diagonal_m);
  delay(500);
 
  // 8) Final RIGHT 135°
  Serial.println("Final turn RIGHT 135°");
  turnRightByTurns(TURN_135_TURNS_RIGHT);
 
  Serial.println("--- DIAGONAL PATH END ---\n");
}

/***************** CALIBRATE TURN 90° *****************/
void calibrateTurn90() {
  Serial.println("Robot will turn RIGHT on the spot.");
  Serial.println("Send 's' when it reaches ~90°.");
 
  totalTurnsRight = 0.0;
  totalTurnsLeft  = 0.0;
 
  calculateThetaRight();
  calculateThetaLeft();
  thetaRightP = thetaRight;
  thetaLeftP  = thetaLeft;
 
  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);
 
  unsigned long startTime = millis();
 
  while (true) {
    if (millis() - startTime > 15000) {
      Serial.println("!! Timeout in calibration.");
      break;
    }
   
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's') {
        Serial.println("!! Stop command received.");
        break;
      }
    }
   
    updateEncodersAndAccumulate_Turns();
   
    int pwmRight = (int)round(PWM_NEUTRAL_RIGHT + K_RIGHT_PWM_BACK * S_TURN_RIGHT);
    int pwmLeft  = (int)round(PWM_NEUTRAL_LEFT  + K_LEFT_PWM_FWD  * S_TURN_LEFT);
   
    pwmRight = constrain(pwmRight, 60, 130);
    pwmLeft  = constrain(pwmLeft,  60, 130);
   
    servoRight.write(pwmRight);
    servoLeft.write(pwmLeft);
   
    static unsigned long lastPrintCal = 0;
    if (millis() - lastPrintCal > 400) {
      Serial.print("CAL | R: ");
      Serial.print(totalTurnsRight, 3);
      Serial.print(" turns, L: ");
      Serial.print(totalTurnsLeft, 3);
      Serial.println(" turns");
      lastPrintCal = millis();
    }
  }
 
  stopRobot();
 
  double avgTurns = 0.5 * (totalTurnsRight + totalTurnsLeft);
  Serial.println("\n--- CALIBRATION RESULT ---");
  Serial.print("Average turns: ");
  Serial.println(avgTurns, 4);
  Serial.println("-> Use this as TURN_90_TURNS");
  Serial.println("--------------------------\n");
}

/***************** ENCODER FUNCTIONS *****************/
void calculateThetaRight() {
  highRight = pulseIn(pinFeedbackRight, HIGH);
  lowRight  = pulseIn(pinFeedbackRight, LOW);
  cycleTimeRight = highRight + lowRight;
 
  if (cycleTimeRight == 0) return;
 
  dutyCycleRight = (double)(highRight * 100.0) / (double)cycleTimeRight;
 
  thetaRight = (360.0 - 1.0)
               - ((dutyCycleRight - 2.9) * 360.0) / (97.1 - 2.9 + 1.0);
}

void calculateThetaLeft() {
  highLeft = pulseIn(pinFeedbackLeft, HIGH);
  lowLeft  = pulseIn(pinFeedbackLeft, LOW);
  cycleTimeLeft = highLeft + lowLeft;
 
  if (cycleTimeLeft == 0) return;
 
  dutyCycleLeft = (double)(highLeft * 100.0) / (double)cycleTimeLeft;
 
  thetaLeft = (360.0 - 1.0)
              - ((dutyCycleLeft - 2.9) * 360.0) / (97.1 - 2.9 + 1.0);
}