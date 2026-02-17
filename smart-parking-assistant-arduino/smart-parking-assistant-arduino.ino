/*
  Smart Parking Assistant Arduino
  
  An intelligent line-following robot with integrated obstacle detection and 
  real-time LCD status display. Perfect for automated parking systems, 
  warehouse automation, or educational robotics projects.
  
  Features:
  - Dual IR sensor line following capability
  - HC-SR04 ultrasonic obstacle detection with hysteresis
  - L298N motor driver for smooth DC motor control
  - 16x2 LCD display for real-time status monitoring
  - Non-blocking code architecture for smooth operation
  - Configurable speed and sensitivity parameters
  
  Components:
  - Arduino Uno/Nano
  - 2x IR Line Following Sensors
  - HC-SR04 Ultrasonic Sensor
  - L298N Motor Driver Module
  - 2x DC Geared Motors
  - 16x2 LCD Display
  - Power supply (7.4V Li-Po or 6x AA batteries recommended)
  
  Author: Your Name
  Version: 2.0
  Date: February 2026
  License: MIT
*/

#include <LiquidCrystal.h>

// ==================== LCD DISPLAY CONFIGURATION ====================
// LCD pins: RS, E, D4, D5, D6, D7
LiquidCrystal lcd(2, 3, 4, 11, A2, A3);

// ==================== SENSOR PIN DEFINITIONS ====================
// Ultrasonic sensor (HC-SR04)
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

// IR line following sensors
const int LINE_SENSOR_LEFT = A0;
const int LINE_SENSOR_RIGHT = A1;

// ==================== MOTOR DRIVER CONFIGURATION ====================
// L298N Motor Driver - Left Motor (Motor A)
const int MOTOR_LEFT_PWM = 5;    // ENA pin (PWM speed control)
const int MOTOR_LEFT_DIR1 = 8;   // IN1 pin
const int MOTOR_LEFT_DIR2 = 7;   // IN2 pin

// L298N Motor Driver - Right Motor (Motor B)
const int MOTOR_RIGHT_PWM = 6;   // ENB pin (PWM speed control)
const int MOTOR_RIGHT_DIR1 = 12; // IN3 pin
const int MOTOR_RIGHT_DIR2 = 13; // IN4 pin

// ==================== ROBOT CONFIGURATION ====================
// Motor speed settings (0-255)
const int SPEED_FORWARD = 175;   // Normal forward speed
const int SPEED_TURN = 150;      // Turning speed (slower for better control)
const int SPEED_SEARCH = 140;    // Search/recovery speed

// Line sensor configuration
// Set to true if your sensors output HIGH when detecting black line
// Set to false if they output LOW when detecting black line
const bool BLACK_LINE_IS_HIGH = false;

// Obstacle detection with hysteresis (prevents oscillation)
const int OBSTACLE_STOP_DISTANCE = 15;  // Stop when obstacle is closer than this (cm)
const int OBSTACLE_CLEAR_DISTANCE = 22; // Resume when obstacle is farther than this (cm)

// ==================== TIMING CONFIGURATION ====================
const unsigned long SONAR_UPDATE_INTERVAL = 60;  // Update distance every 60ms
const unsigned long LCD_UPDATE_INTERVAL = 200;   // Update LCD every 200ms
const unsigned long SENSOR_TIMEOUT = 25000;      // Ultrasonic sensor timeout (μs)

// ==================== SYSTEM STATE VARIABLES ====================
enum RobotCommand {
  CMD_STOP,     // Emergency stop or obstacle detected
  CMD_FORWARD,  // Both sensors on line - go straight
  CMD_LEFT,     // Left sensor on line - turn left
  CMD_RIGHT,    // Right sensor on line - turn right
  CMD_SEARCH    // No sensors on line - search for line
};

// Global state variables
RobotCommand currentCommand = CMD_STOP;
unsigned long lastSonarUpdate = 0;
unsigned long lastLCDUpdate = 0;
int distanceCm = 999;
bool obstacleDetected = false;

// ==================== SETUP FUNCTION ====================
void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Initialize LCD display
  lcd.begin(16, 2);
  lcd.clear();
  showStartupMessage();
  
  // Configure sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LINE_SENSOR_LEFT, INPUT);
  pinMode(LINE_SENSOR_RIGHT, INPUT);
  
  // Configure motor driver pins
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);
  
  // Initialize motors in stopped state
  stopMotors();
  
  Serial.println("Smart Parking Assistant Initialized");
  Serial.println("Starting line following with obstacle detection...");
}

// ==================== MAIN CONTROL LOOP ====================
void loop() {
  unsigned long currentTime = millis();
  
  // Update distance sensor (scheduled to avoid blocking)
  updateDistanceSensor(currentTime);
  
  // Read line sensors and determine basic command
  RobotCommand lineFollowCommand = readLineFollowingSensors();
  
  // Apply obstacle override if necessary
  currentCommand = obstacleDetected ? CMD_STOP : lineFollowCommand;
  
  // Execute the determined command
  executeCommand(currentCommand);
  
  // Update LCD display
  updateLCDDisplay(currentTime);
}

// ==================== STARTUP DISPLAY ====================
void showStartupMessage() {
  lcd.setCursor(0, 0);
  lcd.print("Smart Parking");
  lcd.setCursor(0, 1);
  lcd.print("Assistant v2.0");
  delay(2000);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  lcd.setCursor(0, 1);
  lcd.print("Systems Check");
  delay(1500);
  
  lcd.clear();
}

// ==================== SENSOR FUNCTIONS ====================
void updateDistanceSensor(unsigned long currentTime) {
  if (currentTime - lastSonarUpdate >= SONAR_UPDATE_INTERVAL) {
    lastSonarUpdate = currentTime;
    
    // Measure distance
    distanceCm = measureDistance();
    
    // Output distance to serial for monitoring
    Serial.print("Distance: ");
    Serial.print(distanceCm);
    Serial.println("cm");
    
    // Apply hysteresis to prevent oscillation
    updateObstacleState();
  }
}

int measureDistance() {
  // Send ultrasonic pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Measure echo duration with timeout
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, SENSOR_TIMEOUT);
  
  // Convert to distance (speed of sound = 343 m/s)
  if (duration == 0) {
    return 999; // No echo received
  }
  
  int distance = (duration * 0.034) / 2;
  return (distance > 0 && distance <= 400) ? distance : 999;
}

void updateObstacleState() {
  if (!obstacleDetected && distanceCm <= OBSTACLE_STOP_DISTANCE) {
    obstacleDetected = true;
    Serial.println("OBSTACLE DETECTED - STOPPING");
  } 
  else if (obstacleDetected && distanceCm >= OBSTACLE_CLEAR_DISTANCE) {
    obstacleDetected = false;
    Serial.println("OBSTACLE CLEARED - RESUMING");
  }
}

RobotCommand readLineFollowingSensors() {
  // Read digital sensors (adjust based on your sensor type)
  int leftSensor = digitalRead(LINE_SENSOR_LEFT);
  int rightSensor = digitalRead(LINE_SENSOR_RIGHT);
  
  // Determine if sensors detect black line
  bool leftOnLine = (BLACK_LINE_IS_HIGH) ? (leftSensor == HIGH) : (leftSensor == LOW);
  bool rightOnLine = (BLACK_LINE_IS_HIGH) ? (rightSensor == HIGH) : (rightSensor == LOW);
  
  // Determine movement command based on sensor states
  if (leftOnLine && rightOnLine) {
    return CMD_FORWARD;  // Both sensors on line - go straight
  } 
  else if (leftOnLine && !rightOnLine) {
    return CMD_LEFT;     // Only left sensor on line - turn left
  } 
  else if (!leftOnLine && rightOnLine) {
    return CMD_RIGHT;    // Only right sensor on line - turn right
  } 
  else {
    return CMD_SEARCH;   // No sensors on line - search for line
  }
}

// ==================== MOTOR CONTROL FUNCTIONS ====================
void executeCommand(RobotCommand command) {
  switch (command) {
    case CMD_STOP:
      stopMotors();
      break;
    case CMD_FORWARD:
      moveForward();
      break;
    case CMD_LEFT:
      turnLeft();
      break;
    case CMD_RIGHT:
      turnRight();
      break;
    case CMD_SEARCH:
      searchForLine();
      break;
  }
}

void stopMotors() {
  // Set all motor pins to stop state
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
}

void moveForward() {
  // Both motors forward at normal speed
  analogWrite(MOTOR_LEFT_PWM, SPEED_FORWARD);
  analogWrite(MOTOR_RIGHT_PWM, SPEED_FORWARD);
  
  // Left motor forward
  digitalWrite(MOTOR_LEFT_DIR1, HIGH);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  
  // Right motor forward
  digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
}

void turnLeft() {
  // Reduce left motor speed to create left turn
  analogWrite(MOTOR_LEFT_PWM, SPEED_TURN);
  analogWrite(MOTOR_RIGHT_PWM, SPEED_FORWARD);
  
  // Both motors forward, but left slower
  digitalWrite(MOTOR_LEFT_DIR1, HIGH);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
}

void turnRight() {
  // Reduce right motor speed to create right turn
  analogWrite(MOTOR_LEFT_PWM, SPEED_FORWARD);
  analogWrite(MOTOR_RIGHT_PWM, SPEED_TURN);
  
  // Both motors forward, but right slower
  digitalWrite(MOTOR_LEFT_DIR1, HIGH);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
}

void searchForLine() {
  // Gentle rotation to search for lost line
  analogWrite(MOTOR_LEFT_PWM, SPEED_SEARCH);
  analogWrite(MOTOR_RIGHT_PWM, SPEED_SEARCH);
  
  // Left motor backward, right motor forward (clockwise rotation)
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
}

// ==================== DISPLAY FUNCTIONS ====================
void updateLCDDisplay(unsigned long currentTime) {
  if (currentTime - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
    lastLCDUpdate = currentTime;
    
    // Display current command on first line
    lcd.setCursor(0, 0);
    lcd.print("Cmd: ");
    lcd.print(getCommandString(currentCommand));
    lcd.print("        "); // Clear remaining characters
    
    // Display distance on second line
    lcd.setCursor(0, 1);
    lcd.print("Dist: ");
    if (distanceCm >= 999) {
      lcd.print("---");
    } else {
      lcd.print(distanceCm);
    }
    lcd.print("cm");
    
    // Show obstacle warning if detected
    if (obstacleDetected) {
      lcd.setCursor(11, 1);
      lcd.print(" STOP");
    } else {
      lcd.setCursor(11, 1);
      lcd.print("     ");
    }
  }
}

String getCommandString(RobotCommand command) {
  switch (command) {
    case CMD_STOP:    return "STOP";
    case CMD_FORWARD: return "FORWARD";
    case CMD_LEFT:    return "LEFT";
    case CMD_RIGHT:   return "RIGHT";
    case CMD_SEARCH:  return "SEARCH";
    default:          return "UNKNOWN";
  }
}
