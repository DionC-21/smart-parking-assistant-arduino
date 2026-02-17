# Programming Guide

## Code Structure Overview

The Smart Parking Assistant uses a well-organized, modular code structure designed for maintainability and extensibility.

## Core Functions

### Setup and Initialization
```cpp
void setup() {
  // System initialization sequence
  Serial.begin(9600);           // Debug communication
  lcd.begin(16, 2);            // LCD initialization
  showStartupMessage();         // User feedback
  configurePins();             // Hardware setup
  stopMotors();                // Safe initial state
}
```

### Main Control Loop
```cpp
void loop() {
  unsigned long currentTime = millis();
  
  updateDistanceSensor(currentTime);        // Scheduled sensor reading
  RobotCommand lineCommand = readLineFollowingSensors(); // Line detection
  currentCommand = obstacleDetected ? CMD_STOP : lineCommand; // Override logic
  executeCommand(currentCommand);           // Motor control
  updateLCDDisplay(currentTime);           // Status display
}
```

## Command System

### Robot Commands Enum
```cpp
enum RobotCommand {
  CMD_STOP,     // Emergency stop or obstacle detected
  CMD_FORWARD,  // Both sensors on line - go straight  
  CMD_LEFT,     // Left sensor on line - turn left
  CMD_RIGHT,    // Right sensor on line - turn right
  CMD_SEARCH    // No sensors on line - search for line
};
```

### Line Following Logic
```cpp
RobotCommand readLineFollowingSensors() {
  int leftSensor = digitalRead(LINE_SENSOR_LEFT);
  int rightSensor = digitalRead(LINE_SENSOR_RIGHT);
  
  bool leftOnLine = (BLACK_LINE_IS_HIGH) ? (leftSensor == HIGH) : (leftSensor == LOW);
  bool rightOnLine = (BLACK_LINE_IS_HIGH) ? (rightSensor == HIGH) : (rightSensor == LOW);
  
  // Decision matrix for line following
  if (leftOnLine && rightOnLine) return CMD_FORWARD;
  if (leftOnLine && !rightOnLine) return CMD_LEFT;  
  if (!leftOnLine && rightOnLine) return CMD_RIGHT;
  return CMD_SEARCH;
}
```

## Motor Control System

### PWM Speed Control
```cpp
// Speed constants (0-255)
const int SPEED_FORWARD = 175;   // Normal forward speed
const int SPEED_TURN = 150;      // Reduced speed for turning
const int SPEED_SEARCH = 140;    // Search rotation speed
```

### Movement Functions
```cpp
void moveForward() {
  analogWrite(MOTOR_LEFT_PWM, SPEED_FORWARD);
  analogWrite(MOTOR_RIGHT_PWM, SPEED_FORWARD);
  setMotorDirection(BOTH_FORWARD);
}

void turnLeft() {
  analogWrite(MOTOR_LEFT_PWM, SPEED_TURN);    // Reduce left speed
  analogWrite(MOTOR_RIGHT_PWM, SPEED_FORWARD); // Maintain right speed
  setMotorDirection(BOTH_FORWARD);
}
```

### Motor Direction Control
```cpp
void setMotorDirection(int direction) {
  switch(direction) {
    case BOTH_FORWARD:
      digitalWrite(MOTOR_LEFT_DIR1, HIGH);  digitalWrite(MOTOR_LEFT_DIR2, LOW);
      digitalWrite(MOTOR_RIGHT_DIR1, HIGH); digitalWrite(MOTOR_RIGHT_DIR2, LOW);
      break;
    // Additional direction cases...
  }
}
```

## Sensor Management

### Ultrasonic Distance Measurement
```cpp
int measureDistance() {
  // Trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Measure echo with timeout
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, SENSOR_TIMEOUT);
  
  // Convert to centimeters
  if (duration == 0) return 999; // No echo
  return (duration * 0.034) / 2;
}
```

### Obstacle Detection with Hysteresis
```cpp
void updateObstacleState() {
  if (!obstacleDetected && distanceCm <= OBSTACLE_STOP_DISTANCE) {
    obstacleDetected = true;
    Serial.println("OBSTACLE DETECTED");
  } 
  else if (obstacleDetected && distanceCm >= OBSTACLE_CLEAR_DISTANCE) {
    obstacleDetected = false;
    Serial.println("OBSTACLE CLEARED");
  }
}
```

## Timing and Scheduling

### Non-blocking Timing
```cpp
// Global timing variables
unsigned long lastSonarUpdate = 0;
unsigned long lastLCDUpdate = 0;

// Scheduled updates
void updateDistanceSensor(unsigned long currentTime) {
  if (currentTime - lastSonarUpdate >= SONAR_UPDATE_INTERVAL) {
    lastSonarUpdate = currentTime;
    distanceCm = measureDistance();
    updateObstacleState();
  }
}
```

### Update Intervals
```cpp
const unsigned long SONAR_UPDATE_INTERVAL = 60;  // 60ms for distance sensor
const unsigned long LCD_UPDATE_INTERVAL = 200;   // 200ms for display
```

## Display System

### LCD Status Display
```cpp
void updateLCDDisplay(unsigned long currentTime) {
  if (currentTime - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
    lastLCDUpdate = currentTime;
    
    // Line 1: Current command
    lcd.setCursor(0, 0);
    lcd.print("Cmd: ");
    lcd.print(getCommandString(currentCommand));
    
    // Line 2: Distance and obstacle status  
    lcd.setCursor(0, 1);
    lcd.print("Dist: ");
    lcd.print(distanceCm);
    lcd.print("cm");
    
    if (obstacleDetected) {
      lcd.setCursor(11, 1);
      lcd.print(" STOP");
    }
  }
}
```

## Configuration Parameters

### Sensor Configuration
```cpp
// Line sensor behavior
const bool BLACK_LINE_IS_HIGH = false;  // Adjust based on sensor type

// Distance thresholds
const int OBSTACLE_STOP_DISTANCE = 15;  // Stop when obstacle closer than 15cm
const int OBSTACLE_CLEAR_DISTANCE = 22; // Resume when obstacle farther than 22cm
```

### Motor Tuning
```cpp
// Speed settings (0-255)
const int SPEED_FORWARD = 175;   // Adjust for your motors/battery
const int SPEED_TURN = 150;      // Turning speed (usually slower)
const int SPEED_SEARCH = 140;    // Line search rotation speed
```

## Debugging Features

### Serial Output
```cpp
void setup() {
  Serial.begin(9600);
  Serial.println("Smart Parking Assistant Initialized");
}

void loop() {
  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println("cm");
}
```

### Debug Commands
Add these for advanced debugging:
```cpp
// Print sensor states
void debugSensors() {
  Serial.print("Left: "); Serial.print(digitalRead(LINE_SENSOR_LEFT));
  Serial.print(" Right: "); Serial.println(digitalRead(LINE_SENSOR_RIGHT));
}

// Print motor states  
void debugMotors() {
  Serial.print("Command: "); Serial.println(getCommandString(currentCommand));
}
```

## Customization Options

### Speed Profiles
```cpp
// Create different speed profiles for various conditions
struct SpeedProfile {
  int forward;
  int turn;
  int search;
};

SpeedProfile normalSpeed = {175, 150, 140};
SpeedProfile fastSpeed = {220, 180, 160};
SpeedProfile slowSpeed = {120, 100, 90};
```

### Sensor Sensitivity
```cpp
// Analog sensor support (if using analog IR sensors)
const int LINE_THRESHOLD = 512;  // Adjust based on surface/lighting

bool isOnLine(int sensorValue) {
  return BLACK_LINE_IS_HIGH ? (sensorValue > LINE_THRESHOLD) 
                           : (sensorValue < LINE_THRESHOLD);
}
```

### Advanced Features

#### PID Line Following (Enhancement)
```cpp
// PID constants
float Kp = 1.0;  // Proportional gain
float Ki = 0.0;  // Integral gain  
float Kd = 0.1;  // Derivative gain

int calculatePIDCorrection(int error) {
  static int lastError = 0;
  static int integral = 0;
  
  integral += error;
  int derivative = error - lastError;
  lastError = error;
  
  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}
```

#### Multi-Sensor Line Detection
```cpp
// Support for more sensors
const int NUM_SENSORS = 5;
int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4};

int readLineSensors() {
  int position = 0;
  int sensorsActive = 0;
  
  for(int i = 0; i < NUM_SENSORS; i++) {
    if(isOnLine(analogRead(sensorPins[i]))) {
      position += i * 1000;
      sensorsActive++;
    }
  }
  
  return sensorsActive > 0 ? position / sensorsActive : -1;
}
```

## Error Handling

### Sensor Validation
```cpp
bool validateSensorReadings() {
  // Check for sensor disconnection or failure
  if (distanceCm == 0 && digitalRead(ECHO_PIN) == LOW) {
    Serial.println("WARNING: Ultrasonic sensor may be disconnected");
    return false;
  }
  return true;
}
```

### Motor Protection
```cpp
void checkMotorHealth() {
  // Monitor motor current if current sensors available
  // Implement stall detection
  // Add thermal protection if temperature sensors available
}
```

This programming guide provides a comprehensive overview of the code structure and implementation details for the Smart Parking Assistant Arduino project.
