/*
 * Teensy 4.0 - Servo Controller for IMU Straight Line (ROS2)
 * Uses proven parameters from 2m straight line code
 * 
 * Pin Connections:
 * - Pin 5: Left Servo Signal
 * - Pin 6: Right Servo Signal
 * - Pin 7: Left Feedback (not used in this version)
 * - Pin 8: Right Feedback (not used in this version)
 * 
 * Serial Protocol from ROS2:
 * - Receives: "left_angle right_angle" (e.g., "88 97")
 * - Angles are direct PWM values (0-180)
 * 
 * IMPORTANT: Uses your proven servo parameters!
 */

#include <Servo.h>

// ==========================================
// PIN DEFINITIONS
// ==========================================
constexpr int pinControlLeft = 5;
constexpr int pinControlRight = 6;
constexpr int pinFeedbackLeft = 7;   // Not used
constexpr int pinFeedbackRight = 8;  // Not used

// ==========================================
// SERVO PARAMETERS (from your working 2m code!)
// ==========================================

// Neutral PWM values (STOP)
const int PWM_NEUTRAL_RIGHT = 94;
const int PWM_NEUTRAL_LEFT = 94;

// Forward motion base speeds (from your BASE_RIGHT_FWD/BASE_LEFT_FWD)
const int BASE_RIGHT_FWD = 88;   // From your working code
const int BASE_LEFT_FWD = 97;    // From your working code

// Backward motion base speeds (from your BASE_RIGHT_BACK/BASE_LEFT_BACK)
const int BASE_RIGHT_BACK = 98;  // From your working code
const int BASE_LEFT_BACK = 87;   // From your working code

// Speed limits (from your SPEED_MIN/SPEED_MAX)
const int SPEED_MIN = 83;
const int SPEED_MAX = 105;

// ==========================================
// GLOBAL VARIABLES
// ==========================================
Servo servoRight;
Servo servoLeft;

int current_left = PWM_NEUTRAL_LEFT;
int current_right = PWM_NEUTRAL_RIGHT;

unsigned long last_command_time = 0;
const unsigned long TIMEOUT = 1000;  // Stop if no command for 1 second

bool servos_attached = false;

// ==========================================

void setup() {
  Serial.begin(115200);
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pinFeedbackRight, INPUT);
  pinMode(pinFeedbackLeft, INPUT);
  
  // Startup indication
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  
  Serial.println("========================================");
  Serial.println("   TEENSY SERVO CONTROLLER - ROS2");
  Serial.println("   (Using proven 2m motion parameters)");
  Serial.println("========================================");
  Serial.println("Servo Parameters:");
  Serial.print("  Neutral: R=");
  Serial.print(PWM_NEUTRAL_RIGHT);
  Serial.print(" L=");
  Serial.println(PWM_NEUTRAL_LEFT);
  Serial.print("  Forward: R=");
  Serial.print(BASE_RIGHT_FWD);
  Serial.print(" L=");
  Serial.println(BASE_LEFT_FWD);
  Serial.println("========================================");
  Serial.println("Waiting for ROS2 commands...");
  Serial.println("Format: left_pwm right_pwm");
  Serial.println("========================================\n");
}

void loop() {
  // Read incoming commands from ROS2
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    parseCommand(cmd);
    last_command_time = millis();
  }
  
  // Safety timeout - stop if no command received
  if (millis() - last_command_time > TIMEOUT) {
    if (current_left != PWM_NEUTRAL_LEFT || current_right != PWM_NEUTRAL_RIGHT) {
      stopServos();
      Serial.println("TIMEOUT - Servos stopped");
    }
  }
  
  // Update servo positions
  updateServos();
  
  // LED blink when moving
  if (abs(current_left - PWM_NEUTRAL_LEFT) > 2 || 
      abs(current_right - PWM_NEUTRAL_RIGHT) > 2) {
    digitalWrite(LED_BUILTIN, (millis() / 250) % 2);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void parseCommand(String cmd) {
  /*
   * Parse command in format: "left_pwm right_pwm"
   * Example: "88 97" (both forward)
   * Example: "94 94" (both stop)
   */
  cmd.trim();
  
  int space_index = cmd.indexOf(' ');
  if (space_index == -1) {
    Serial.println("ERROR: Invalid format. Use: left right");
    return;
  }
  
  int left = cmd.substring(0, space_index).toInt();
  int right = cmd.substring(space_index + 1).toInt();
  
  // Validate range (using your SPEED_MIN/SPEED_MAX)
  if (left < SPEED_MIN || left > SPEED_MAX || 
      right < SPEED_MIN || right > SPEED_MAX) {
    // Allow neutral values
    if (!((left == PWM_NEUTRAL_LEFT || left == PWM_NEUTRAL_RIGHT) &&
          (right == PWM_NEUTRAL_LEFT || right == PWM_NEUTRAL_RIGHT))) {
      Serial.print("WARN: Speed out of range [");
      Serial.print(SPEED_MIN);
      Serial.print("-");
      Serial.print(SPEED_MAX);
      Serial.println("]");
      // Still apply, but warn
    }
  }
  
  current_left = left;
  current_right = right;
  
  // Debug output
  Serial.print("CMD: L=");
  Serial.print(current_left);
  Serial.print(" R=");
  Serial.println(current_right);
}

void updateServos() {
  /*
   * Update servo positions based on current speed values
   */
  
  // Attach servos if not already attached
  if (!servos_attached) {
    servoLeft.attach(pinControlLeft);
    servoRight.attach(pinControlRight);
    servos_attached = true;
  }
  
  // Write positions to servos
  servoLeft.write(current_left);
  servoRight.write(current_right);
}

void stopServos() {
  /*
   * Stop both servos
   */
  current_left = PWM_NEUTRAL_LEFT;
  current_right = PWM_NEUTRAL_RIGHT;
  
  if (!servos_attached) {
    servoLeft.attach(pinControlLeft);
    servoRight.attach(pinControlRight);
    servos_attached = true;
  }
  
  servoLeft.write(PWM_NEUTRAL_LEFT);
  servoRight.write(PWM_NEUTRAL_RIGHT);
  
  delay(100);
  
  // Optionally detach to reduce jitter (comment out if causes issues)
  // servoLeft.detach();
  // servoRight.detach();
  // servos_attached = false;
}
