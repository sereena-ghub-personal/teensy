#include <Servo.h>
#include <math.h>   

Servo servoRight;
Servo servoLeft;
constexpr int pinFeedbackLeft  = 9;
constexpr int pinFeedbackRight = 10;
constexpr int pinControlLeft   = 5;
constexpr int pinControlRight  = 6;

char keyCommand;

unsigned long highRight = 0, highLeft = 0;
unsigned long lowRight  = 0, lowLeft  = 0;
unsigned long cycleTimeRight = 0, cycleTimeLeft = 0;
double dutyCycleRight = 0.0, dutyCycleLeft = 0.0;

double thetaRight  = 0.0, thetaLeft  = 0.0;
double thetaRightP = 0.0, thetaLeftP = 0.0;

double totalTurnsRight = 0.0;
double totalTurnsLeft  = 0.0;

// --- Wheel geometry ---
const double wheelDiameter   = 0.07;                    // m
const double wheelCirc       = 3.14159 * wheelDiameter; // m

// --- 40 cm square parameters ---
const double squareSide_m    = 0.40;                    // 40 cm
const double squareSideTurns = squareSide_m / wheelCirc; // (info only)

// --- 90° & 135° turn parameters (to be tuned) ---
// After running calibration ('c'), replace this with the measured value.
const double TURN_90_TURNS   = 0.50;              // initial guess, tune via calibration
const double TURN_135_TURNS_RIGHT  = TURN_90_TURNS * 1.5;  // 135° = 90° + 45°
const double TURN_135_TURNS_LEFT  = TURN_90_TURNS * 2.95;  // for LEFT 135° (will tune separately)

// --- Per-wheel neutral PWM (adjust these if a wheel creeps at "stop") ---
const int PWM_NEUTRAL_RIGHT = 94;
const int PWM_NEUTRAL_LEFT  = 94;

// --- Forward gains ---
// Mapping S -> PWM: 
//   right  : PWM_NEUTRAL_RIGHT - K_RIGHT_PWM_FWD * S_right
//   left   : PWM_NEUTRAL_LEFT  + K_LEFT_PWM_FWD  * S_left
const double K_RIGHT_PWM_FWD = 4.4;
const double K_LEFT_PWM_FWD  = 4.3;

// P-gain for distance difference between wheels when going straight
// (reduced for calmer behaviour; tune 0.3–1.0)
const double Kp_turns_FWD    = 0.3;

// --- Backward gains (used for turning one wheel backward) ---
const double K_RIGHT_PWM_BACK = 4.3;
const double K_LEFT_PWM_BACK  = 4.7;

// --- Base forward speed per wheel ---
const double S_BASE_FWD_RIGHT = 1.08; //1.06
const double S_BASE_FWD_LEFT  = 0.98; //0.99

// --- Turn speed per wheel ---
const double S_TURN_RIGHT = 1.1;
const double S_TURN_LEFT  = 1.1;

// Prototypes
void calculateThetaRight();
void calculateThetaLeft();
void stopRobot();
void updateEncodersAndAccumulate();
double getAverageDistance();

void moveForwardDistance(double distance_m);
void turnRight90();
void runSquare40cm();
void calibrateTurn90();

// Diagonal part
void runDiagonal40cm();          // full diagonal sequence
void turnRightByTurns(double targetTurns);
void turnLeftByTurns(double targetTurns);

void setup() {
  Serial.begin(9600);
  pinMode(pinFeedbackRight, INPUT);
  pinMode(pinFeedbackLeft,  INPUT);

  Serial.println("===== ROBOT READY =====");
  Serial.print("Wheel diameter: ");
  Serial.print(wheelDiameter * 100.0, 1);
  Serial.println(" cm");

  Serial.print("Wheel circumference: ");
  Serial.print(wheelCirc, 4);
  Serial.println(" m");

  Serial.print("Square side (m): ");
  Serial.println(squareSide_m, 3);
  Serial.print("Square side rotations (approx): ");
  Serial.println(squareSideTurns, 3);

  Serial.print("TURN_90_TURNS (current): ");
  Serial.println(TURN_90_TURNS, 4);
  Serial.print("TURN_135_TURNS_RIGHT (current): ");
  Serial.println(TURN_135_TURNS_RIGHT, 4);
  Serial.print("TURN_135_TURNS_LEFT  (current): ");
  Serial.println(TURN_135_TURNS_LEFT, 4);


  Serial.print("PWM_NEUTRAL_RIGHT: ");
  Serial.println(PWM_NEUTRAL_RIGHT);
  Serial.print("PWM_NEUTRAL_LEFT : ");
  Serial.println(PWM_NEUTRAL_LEFT);

  Serial.println("\nCommands:");
  Serial.println("  'q' - 40cm square ONLY");
  Serial.println("  'd' - Run ONLY diagonal pattern (from current pose)");
  Serial.println("  'c' - Calibrate 90° turn (press 's' when you see 90°)");
  Serial.println("  's' - Emergency stop (works during motion)");
  Serial.println("=======================\n");
}

void loop() {
  if (Serial.available() > 0) {
    keyCommand = Serial.read();

    switch (keyCommand) {
      case 'q':
        Serial.println("\n>>> Running 40cm square path...");
        runSquare40cm();
        Serial.println(">>> Square path finished! (no diagonal)\n");
        break;

      case 'd':
        Serial.println("\n>>> Running diagonal path for 40cm square (from current pose)...");
        runDiagonal40cm();
        Serial.println(">>> Diagonal path finished!\n");
        break;

      case 'c':
        Serial.println("\n>>> Starting 90° turn calibration...");
        calibrateTurn90();
        Serial.println(">>> Calibration run finished!\n");
        break;  

      case 'l':
        Serial.println("\n>>> TEST: LEFT 135° TURN ONLY <<<");
        turnLeftByTurns(TURN_135_TURNS_LEFT);
        Serial.println(">>> LEFT 135° test done!\n");
        break;
  

      default:
        break;
    }
  }
}

void stopRobot() {
  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);

  servoRight.write(PWM_NEUTRAL_RIGHT);
  servoLeft.write(PWM_NEUTRAL_LEFT);

  delay(100);   

  servoRight.detach();
  servoLeft.detach();
}

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

  // ---GLITCH FILTER (BACKWARD FIX) ----
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

// ------------------- Generic forward distance -------------------
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
        Serial.println("!! Stop command (s) received in moveForwardDistance.");
        stopRobot();
        return;
      }
    }

    updateEncodersAndAccumulate();
    
    double errorTurns = totalTurnsLeft - totalTurnsRight;
    double correction = Kp_turns_FWD * errorTurns;

    // Per-wheel base speeds
    double S_right = S_BASE_FWD_RIGHT + correction;
    double S_left  = S_BASE_FWD_LEFT  - correction;

    S_right = constrain(S_right, -1.5, 1.5);
    S_left  = constrain(S_left,  -1.5, 1.5);

    int pwmRight = (int)round(PWM_NEUTRAL_RIGHT - K_RIGHT_PWM_FWD * S_right);
    int pwmLeft  = (int)round(PWM_NEUTRAL_LEFT  + K_LEFT_PWM_FWD  * S_left);

    // protect left from going too far above neutral
    pwmLeft = min(pwmLeft, PWM_NEUTRAL_LEFT + 5);

    pwmRight = constrain(pwmRight, 60, 130);
    pwmLeft  = constrain(pwmLeft,  60, 130);

    servoRight.write(pwmRight);
    servoLeft.write(pwmLeft);

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 400) {
      double dist = getAverageDistance();
      Serial.print("FWDd | t = ");
      Serial.print((millis() - startTime) / 1000.0, 2);
      Serial.print(" s  | R: ");
      Serial.print(totalTurnsRight, 3);
      Serial.print(" turns, L: ");
      Serial.print(totalTurnsLeft, 3);
      Serial.print(" turns  | avg dist ≈ ");
      Serial.print(dist, 3);
      Serial.print(" m  | error(turns) = ");
      Serial.print(errorTurns, 4);
      Serial.print("  | PWM R/L = ");
      Serial.print(pwmRight);
      Serial.print("/");
      Serial.println(pwmLeft);
      lastPrint = millis();
    }
  }

  double finalDist = getAverageDistance();
  Serial.println("\n--- FORWARD distance END ---");
  Serial.print("Final turns  R: ");
  Serial.print(totalTurnsRight, 3);
  Serial.print("  L: ");
  Serial.print(totalTurnsLeft, 3);
  Serial.print("  | avg distance ≈ ");
  Serial.print(finalDist, 3);
  Serial.println(" m\n");

  stopRobot();
}

// ------------------- In-place right turn using TURN_90_TURNS -------------------
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
        Serial.println("!! Stop command (s) received in turnRight90.");
        stopRobot();
        return;
      }
    }

    updateEncodersAndAccumulate();

    // Per-wheel turn speed
    double S_turn_R = S_TURN_RIGHT;
    double S_turn_L = S_TURN_LEFT;

    int pwmRight = (int)round(PWM_NEUTRAL_RIGHT + K_RIGHT_PWM_BACK * S_turn_R);  // backward
    int pwmLeft  = (int)round(PWM_NEUTRAL_LEFT  + K_LEFT_PWM_FWD  * S_turn_L);   // forward

    pwmRight = constrain(pwmRight, 60, 130);
    pwmLeft  = constrain(pwmLeft,  60, 130);

    servoRight.write(pwmRight);
    servoLeft.write(pwmLeft);

    static unsigned long lastPrintTurn = 0;
    if (millis() - lastPrintTurn > 400) {
      Serial.print("TURN | t = ");
      Serial.print((millis() - startTime) / 1000.0, 2);
      Serial.print(" s  | R: ");
      Serial.print(totalTurnsRight, 3);
      Serial.print(" turns, L: ");
      Serial.print(totalTurnsLeft, 3);
      Serial.print(" turns  | target per wheel ≈ ");
      Serial.println(TURN_90_TURNS, 3);
      lastPrintTurn = millis();
    }
  }

  Serial.println("--- TURN RIGHT 90° END ---\n");
  stopRobot();
}

// ------------------- 40cm square path -------------------
void runSquare40cm() {
  for (int i = 0; i < 4; ++i) {
    Serial.print("\n=== SQUARE: side ");
    Serial.print(i + 1);
    Serial.println(" of 4 ===");

    // Move one side (40 cm)
    moveForwardDistance(squareSide_m);
    delay(500);

    // Turn right 90° except after the last side
    if (i < 3) {
      turnRight90();
      delay(500);
    }
  }
}

// ------------------- 90° turn calibration -------------------
void calibrateTurn90() {
  Serial.println("Place robot on floor.");
  Serial.println("It will start turning RIGHT on the spot.");
  Serial.println("When you THINK it has turned ~90°, send 's' in Serial Monitor.");
  Serial.println("Then read the measured turns and update TURN_90_TURNS.\n");

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
      Serial.println("!! Timeout in calibrateTurn90, stopping.");
      break;
    }

    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's') {
        Serial.println("!! Stop command (s) received in calibrateTurn90.");
        break;  // exit loop, then print measured turns
      }
    }

    updateEncodersAndAccumulate();

    double S_turn_R = S_TURN_RIGHT;
    double S_turn_L = S_TURN_LEFT;

    int pwmRight = (int)round(PWM_NEUTRAL_RIGHT + K_RIGHT_PWM_BACK * S_turn_R);  // backward
    int pwmLeft  = (int)round(PWM_NEUTRAL_LEFT  + K_LEFT_PWM_FWD  * S_turn_L);   // forward

    pwmRight = constrain(pwmRight, 60, 130);
    pwmLeft  = constrain(pwmLeft,  60, 130);

    servoRight.write(pwmRight);
    servoLeft.write(pwmLeft);

    static unsigned long lastPrintCal = 0;
    if (millis() - lastPrintCal > 400) {
      Serial.print("CAL | t = ");
      Serial.print((millis() - startTime) / 1000.0, 2);
      Serial.print(" s  | R: ");
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
  Serial.print("Measured turns RIGHT: ");
  Serial.println(totalTurnsRight, 4);
  Serial.print("Measured turns LEFT : ");
  Serial.println(totalTurnsLeft, 4);
  Serial.print("Average turns       : ");
  Serial.println(avgTurns, 4);
  Serial.println("-> Use this AVERAGE value as TURN_90_TURNS in the code.");
  Serial.println("--------------------------\n");
}

// ------------------- Encoder calculations -------------------
void calculateThetaRight() {
  highRight = pulseIn(pinFeedbackRight, HIGH);
  lowRight  = pulseIn(pinFeedbackRight, LOW);
  cycleTimeRight = highRight + lowRight;

  if (cycleTimeRight == 0) {
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
    return;
  }

  dutyCycleLeft = (double)(highLeft * 100.0) / (double)cycleTimeLeft;

  thetaLeft = (360.0 - 1.0)
              - ((dutyCycleLeft - 2.9) * 360.0) / (97.1 - 2.9 + 1.0);
}

// --------------- Right turn by given wheel turns ---------------
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
      Serial.println("!! Timeout in turnRightByTurns, stopping.");
      break;
    }

    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's') {
        Serial.println("!! Stop command (s) received in turnRightByTurns.");
        stopRobot();
        return;
      }
    }

    updateEncodersAndAccumulate();

    double S_turn_R = S_TURN_RIGHT;
    double S_turn_L = S_TURN_LEFT;

    // Right turn: right wheel backward, left wheel forward
    int pwmRight = (int)round(PWM_NEUTRAL_RIGHT + K_RIGHT_PWM_BACK * S_turn_R); // backward
    int pwmLeft  = (int)round(PWM_NEUTRAL_LEFT  + K_LEFT_PWM_FWD  * S_turn_L);  // forward

    pwmRight = constrain(pwmRight, 60, 130);
    pwmLeft  = constrain(pwmLeft,  60, 130);

    servoRight.write(pwmRight);
    servoLeft.write(pwmLeft);

    static unsigned long lastPrintTurnR = 0;
    if (millis() - lastPrintTurnR > 400) {
      Serial.print("TURN_R | t = ");
      Serial.print((millis() - startTime) / 1000.0, 2);
      Serial.print(" s  | R: ");
      Serial.print(totalTurnsRight, 3);
      Serial.print(" turns, L: ");
      Serial.print(totalTurnsLeft, 3);
      Serial.print(" turns  | target per wheel ≈ ");
      Serial.println(targetTurns, 3);
      lastPrintTurnR = millis();
    }
  }

  Serial.println("--- TURN RIGHT (generic) END ---\n");
  stopRobot();
}

// --------------- Left turn by given wheel turns ---------------
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
      Serial.println("!! Timeout in turnLeftByTurns, stopping.");
      break;
    }

    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's') {
        Serial.println("!! Stop command (s) received in turnLeftByTurns.");
        stopRobot();
        return;
      }
    }

    updateEncodersAndAccumulate();

    double S_turn_R = 1.5; //1.3
    double S_turn_L = 0.7;

    // Left turn: right wheel forward, left wheel backward
    int pwmRight = (int)round(PWM_NEUTRAL_RIGHT - K_RIGHT_PWM_FWD * S_turn_R); // forward
    int pwmLeft  = (int)round(PWM_NEUTRAL_LEFT  - K_LEFT_PWM_BACK * S_turn_L); // backward

    pwmRight = constrain(pwmRight, 60, 130);
    pwmLeft  = constrain(pwmLeft,  60, 130);

    servoRight.write(pwmRight);
    servoLeft.write(pwmLeft);

    static unsigned long lastPrintTurnL = 0;
    if (millis() - lastPrintTurnL > 400) {
      Serial.print("TURN_L | t = ");
      Serial.print((millis() - startTime) / 1000.0, 2);
      Serial.print(" s  | R: ");
      Serial.print(totalTurnsRight, 3);
      Serial.print(" turns, L: ");
      Serial.print(totalTurnsLeft, 3);
      Serial.print(" turns  | target per wheel ≈ ");
      Serial.println(targetTurns, 3);
      lastPrintTurnL = millis();
    }
  }

  Serial.println("--- TURN LEFT (generic) END ---\n");
  stopRobot();
}

// --------------- Diagonal path for 40cm square ---------------
void runDiagonal40cm() {
  Serial.println("\n--- DIAGONAL PATH START ---");

  const double side_m       = squareSide_m;                     // 40 cm straight
  const double diagonal_m   = squareSide_m * 1.41421356237;     // side * sqrt(2)
  const double turn135TurnsRight = TURN_135_TURNS_RIGHT;        // 135° in wheel turns right
  const double turn135TurnsLeft = TURN_135_TURNS_LEFT;        // 135° in wheel turns left

  // 1) First: go 40 cm straight
  Serial.print("Segment 1: straight side ~");
  Serial.println(side_m, 3);
  moveForwardDistance(side_m);
  delay(500);

  // 2) Turn RIGHT 135°
  Serial.println("Turn RIGHT 135°");
  turnRightByTurns(turn135TurnsRight);
  delay(500);

  // 3) Move along first diagonal
  Serial.print("Segment 2: diagonal ~");
  Serial.println(diagonal_m, 3);
  moveForwardDistance(diagonal_m);
  delay(500);

  // 4) Turn LEFT 135°
  Serial.println("Turn LEFT 135°");
  turnLeftByTurns(turn135TurnsLeft);
  delay(500);

  // 5) Move second straight 40 cm
  Serial.print("Segment 3: straight side ~");
  Serial.println(side_m, 3);
  moveForwardDistance(side_m);
  delay(500);

  // 6) Turn LEFT 135° //Was turn right first
  Serial.println("Turn LEFT 135°");
  turnLeftByTurns(turn135TurnsLeft);
  delay(500);

  // 7) Move along second diagonal
  Serial.print("Segment 4: diagonal ~");
  Serial.println(diagonal_m, 3);
  moveForwardDistance(diagonal_m);
  delay(500);

  // 8) Final 135° turn (RIGHT) and stop
  Serial.println("Final turn RIGHT 135°");
  turnRightByTurns(turn135TurnsRight);

  Serial.println("--- DIAGONAL PATH END ---\n");
}
