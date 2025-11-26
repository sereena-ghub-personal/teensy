#include <Servo.h>
#include <math.h>    // for fabs()

/* ---------------------------------------------------------
   SERVO OBJECTS
--------------------------------------------------------- */
Servo servoRight;
Servo servoLeft;

/* ---------------------------------------------------------
   PIN ASSIGNMENTS (CHECK WITH YOUR WIRING!)
   Feedback: yellow wires from Parallax 360°
   Control:  white wires from Parallax 360°
--------------------------------------------------------- */
constexpr int pinFeedbackLeft  = 7;
constexpr int pinFeedbackRight = 8;
constexpr int pinControlLeft   = 5;
constexpr int pinControlRight  = 6;

char keyCommand;

/* ---------------------------------------------------------
   ENCODER RAW MEASUREMENTS
--------------------------------------------------------- */
unsigned long highRight = 0, highLeft = 0;
unsigned long lowRight  = 0, lowLeft  = 0;
unsigned long cycleTimeRight = 0, cycleTimeLeft = 0;
double dutyCycleRight = 0.0, dutyCycleLeft = 0.0;

/* ---------------------------------------------------------
   ANGLES AND ROTATIONS
   thetaX     = current angle (0..360)
   thetaXP    = previous angle
   totalTurnsX = accumulated wheel rotations (0.5, 1.25, ...)
--------------------------------------------------------- */
double thetaRight  = 0.0, thetaLeft  = 0.0;
double thetaRightP = 0.0, thetaLeftP = 0.0;

double totalTurnsRight = 0.0;
double totalTurnsLeft  = 0.0;

/* ---------------------------------------------------------
   WHEEL GEOMETRY & TARGET DISTANCE
   Diameter = 7 cm = 0.07 m
--------------------------------------------------------- */
const double wheelDiameter   = 0.07;                       // meters
const double wheelCirc       = 3.14159 * wheelDiameter;     // meters per 1 rotation
const double targetDistance  = 2.0;                         // meters
const double targetRotations = targetDistance / wheelCirc;  // turns needed for ~2m

/* ---------------------------------------------------------
   SERVO MAPPING
   We control a "signed speed" S:
     S > 0  => forward
     S < 0  => backward
     S = 0  => stop
   and convert S -> PWM for each wheel using a linear map
   fitted to your earlier calibration.
--------------------------------------------------------- */
const int    PWM_NEUTRAL  = 94;   // stop position
const double K_RIGHT_PWM  = 4.5;  // how much 1.0 "speed" changes right PWM
const double K_LEFT_PWM   = 4.5;  // how much 1.0 "speed" changes left PWM

// Base speeds (in "S" units) for forward & backward
const double S_BASE_FWD   = 1.1;   // ~forward speed (tune if too fast/slow)
const double S_BASE_BACK  = 1.1;  // ~backward speed

// P-controller gain to equalize wheel speeds (in turns)
const double Kp_turns     = 0.25;   // if left is ahead in turns, slow left / speed right

/* ---------------------------------------------------------
   HELPER FUNCTION DECLARATIONS
--------------------------------------------------------- */
void calculateThetaRight();
void calculateThetaLeft();
void stopRobot();
void updateEncodersAndAccumulate();
void moveForward2m();
void moveBackward2m();
double getAverageDistance();

/* ---------------------------------------------------------
   SETUP
--------------------------------------------------------- */
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

  Serial.print("Target rotations for 2m: ");
  Serial.println(targetRotations, 3);

  Serial.println("\nCommands:");
  Serial.println("  'f' - Move forward 2m, wait 3s, then backward 2m");
  Serial.println("  's' - Emergency stop");
  Serial.println("  't' - Test LEFT motor only (3s)");
  Serial.println("=======================\n");
}

/* ---------------------------------------------------------
   MAIN LOOP
--------------------------------------------------------- */
void loop() {
  if (Serial.available() > 0) {
    keyCommand = Serial.read();

    switch (keyCommand) {

      case 'f':
        Serial.println("\n>>> Starting forward-backward cycle...");
        moveForward2m();
        Serial.println(">>> Waiting 3 seconds before moving backward...");
        delay(3000);  // *** 3 second pause before going backward ***
        moveBackward2m();
        Serial.println(">>> Cycle complete!\n");
        break;

      case 's':
        Serial.println("\n>>> Emergency stop!");
        stopRobot();
        break;

      case 't':  // test left motor only
        Serial.println("\n>>> Testing LEFT motor only (3s)...");
        servoLeft.attach(pinControlLeft);
        // S = +1.0 forward on left: PWM = 94 + 4*1.0 = 98
        servoLeft.write(PWM_NEUTRAL + (int)round(K_LEFT_PWM * S_BASE_FWD));
        delay(3000);
        servoLeft.write(PWM_NEUTRAL);
        delay(200);
        servoLeft.detach();
        Serial.println(">>> Left motor test complete\n");
        break;

      default:
        // ignore other keys
        break;
    }
  }
}

/* ---------------------------------------------------------
   BASIC STOP
--------------------------------------------------------- */
void stopRobot() {
  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);

  servoRight.write(PWM_NEUTRAL);
  servoLeft.write(PWM_NEUTRAL);

  delay(100);    // let servos receive stop PWM

  servoRight.detach();
  servoLeft.detach();
}

/* ---------------------------------------------------------
   ENCODER UPDATE + ACCUMULATION (IN TURNS)
--------------------------------------------------------- */
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

  // accumulate absolute rotation in TURNS (deg / 360)
  totalTurnsRight += fabs(dR) / 360.0;
  totalTurnsLeft  += fabs(dL) / 360.0;
}

/* ---------------------------------------------------------
   GET AVERAGE DISTANCE (m) FROM TURNS
--------------------------------------------------------- */
double getAverageDistance() {
  double avgTurns = 0.5 * (totalTurnsRight + totalTurnsLeft);
  return avgTurns * wheelCirc;   // meters
}

/* ---------------------------------------------------------
   MOVE FORWARD ~2 METERS (WITH P-CONTROL FOR EQUAL SPEED)
--------------------------------------------------------- */
void moveForward2m() {
  Serial.println("\n--- FORWARD 2m START ---");

  totalTurnsRight = 0.0;
  totalTurnsLeft  = 0.0;

  // init previous angles so first diff is small
  calculateThetaRight();
  calculateThetaLeft();
  thetaRightP = thetaRight;
  thetaLeftP  = thetaLeft;

  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);

  unsigned long startTime = millis();

  while (totalTurnsRight < targetRotations &&
         totalTurnsLeft  < targetRotations) {

    // --- SAFETY 1: time-out in case encoders fail ---
    if (millis() - startTime > 20000) {   // 8 seconds timeout
      Serial.println("!! Timeout in moveForward2m, stopping.");
      break;
    }

    // --- SAFETY 2: allow 's' to stop immediately ---
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's') {
        Serial.println("!! Stop command (s) received in moveForward2m.");
        stopRobot();
        return;
      }
    }

    // --- UPDATE ENCODERS ---
    updateEncodersAndAccumulate();

    // --- P-CONTROL TO EQUALIZE TURNS (SPEEDS) ---
    // errorTurns > 0 => left wheel has more turns (ahead)
    double errorTurns = totalTurnsLeft - totalTurnsRight;
    double correction = Kp_turns * errorTurns;

    // base forward speed
    double S_right = S_BASE_FWD + correction;
    double S_left  = S_BASE_FWD - correction;

    // limit S so PWM stays in safe range
    S_right = constrain(S_right, -1.5, 1.5);
    S_left  = constrain(S_left,  -1.5, 1.5);

    // map S to PWM for each wheel
    int pwmRight = (int)round(PWM_NEUTRAL - K_RIGHT_PWM * S_right);
    int pwmLeft  = (int)round(PWM_NEUTRAL + K_LEFT_PWM  * S_left);

    // clamp PWM to valid servo range
    pwmRight = constrain(pwmRight, 60, 130);
    pwmLeft  = constrain(pwmLeft,  60, 130);

    // send commands
    servoRight.write(pwmRight);
    servoLeft.write(pwmLeft);

    // --- DEBUG OUTPUT every 400ms ---
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 400) {
      double dist = getAverageDistance();
      Serial.print("FWD | t = ");
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
  Serial.println("\n--- FORWARD 2m END ---");
  Serial.print("Final turns  R: ");
  Serial.print(totalTurnsRight, 3);
  Serial.print("  L: ");
  Serial.print(totalTurnsLeft, 3);
  Serial.print("  | avg distance ≈ ");
  Serial.print(finalDist, 3);
  Serial.println(" m\n");

  stopRobot();
}

/* ---------------------------------------------------------
   MOVE BACKWARD ~2 METERS (WITH P-CONTROL FOR EQUAL SPEED)
--------------------------------------------------------- */
void moveBackward2m() {
  Serial.println("\n--- BACKWARD 2m START ---");

  totalTurnsRight = 0.0;
  totalTurnsLeft  = 0.0;

  // init previous angles
  calculateThetaRight();
  calculateThetaLeft();
  thetaRightP = thetaRight;
  thetaLeftP  = thetaLeft;

  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);

  unsigned long startTime = millis();

  while (totalTurnsRight < targetRotations &&
         totalTurnsLeft  < targetRotations) {

    // --- SAFETY 1: time-out ---
    if (millis() - startTime > 40000) {
      Serial.println("!! Timeout in moveBackward2m, stopping.");
      break;
    }

    // --- SAFETY 2: allow 's' to stop immediately ---
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's') {
        Serial.println("!! Stop command (s) received in moveBackward2m.");
        stopRobot();
        return;
      }
    }

    // --- UPDATE ENCODERS ---
    updateEncodersAndAccumulate();

    // --- P-CONTROL TO EQUALIZE TURNS (SPEEDS) ---
    // Same idea as forward: errorTurns > 0 => left ahead
    double errorTurns = totalTurnsLeft - totalTurnsRight;
    const double Kp_turns_local = 0.25;
    double correction = Kp_turns_local * errorTurns;


    // base backward speed
    double S_right = S_BASE_BACK + correction;
    double S_left  = S_BASE_BACK - correction;

    // limit S
    S_right = constrain(S_right, 0.4, 1.5);
    S_left  = constrain(S_left,  0.4, 1.5);

    // map S to PWM
    int pwmRight = (int)round(PWM_NEUTRAL + K_RIGHT_PWM * S_right);
    int pwmLeft  = (int)round(PWM_NEUTRAL - K_LEFT_PWM  * S_left);

    pwmRight = constrain(pwmRight, 60, 130);
    pwmLeft  = constrain(pwmLeft,  60, 130);

    servoRight.write(pwmRight);
    servoLeft.write(pwmLeft);

    // --- DEBUG OUTPUT every 400ms ---
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 400) {
      double dist = getAverageDistance();
      Serial.print("BACK | t = ");
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
  Serial.println("\n--- BACKWARD 2m END ---");
  Serial.print("Final turns  R: ");
  Serial.print(totalTurnsRight, 3);
  Serial.print("  L: ");
  Serial.print(totalTurnsLeft, 3);
  Serial.print("  | avg distance ≈ ");
  Serial.print(finalDist, 3);
  Serial.println(" m\n");

  stopRobot();
}

/* ---------------------------------------------------------
   ENCODER FUNCTIONS – Parallax datasheet mapping
   Convert duty cycle → angle 0..360
--------------------------------------------------------- */
void calculateThetaRight() {
  highRight = pulseIn(pinFeedbackRight, HIGH);
  lowRight  = pulseIn(pinFeedbackRight, LOW);
  cycleTimeRight = highRight + lowRight;

  if (cycleTimeRight == 0) {
    // no valid pulse → keep previous thetaRight
    return;
  }

  dutyCycleRight = (double)(highRight * 100.0) / (double)cycleTimeRight;

  // Map 2.9%..97.1% duty to ~0..360 degrees (slightly adjusted)
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




