/*
  Arduino Radar LCD 16-Pin Adaptation
  
  A proximity detection system using an ultrasonic sensor (HC-SR04) with visual 
  and audio feedback through a 16x2 LCD display, RGB LEDs, and buzzer. Features 
  moving average filtering for stable readings and distance-based alert levels.
  
  Components:
  - Arduino Uno/Nano
  - HC-SR04 Ultrasonic Sensor
  - 16x2 LCD Display (HD44780 compatible)
  - 3x LEDs (Red, Blue, Green)
  - Buzzer (Active or Passive)
  - Resistors (220Ω for LEDs, 10kΩ potentiometer for LCD contrast)
  
  Distance Zones:
  - 0-5cm:   Danger (Red LED, Continuous buzzer)
  - 5-10cm:  Critical (Red LED, Fast beeping)
  - 10-15cm: Warning (Red LED, Medium beeping)
  - 15-30cm: Caution (Blue LED, Slow beeping)
  - 30-50cm: Alert (Blue LED, Very slow beeping)
  - >50cm:   Safe (Green LED, No sound)
  
  Author: Your Name
  Version: 1.0
  Date: February 2026
*/

#include <LiquidCrystal.h>

// ==================== PIN DEFINITIONS ====================
// LCD pins: RS, E, D4, D5, D6, D7
LiquidCrystal lcd(2, 3, 4, 5, 6, 11);

// Ultrasonic sensor pins
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

// LED pins
const int LED_RED = 8;     // Danger/Critical/Warning
const int LED_BLUE = 12;   // Caution/Alert
const int LED_GREEN = 13;  // Safe

// Buzzer pin
const int BUZZER_PIN = 7;

// ==================== CONFIGURATION ====================
// Moving average filter
const int NUM_READINGS = 5;
const int MAX_DISTANCE = 200;     // Maximum measurable distance (cm)
const int SENSOR_TIMEOUT = 25000; // Ultrasonic sensor timeout (microseconds)

// Timing intervals
const unsigned long LCD_UPDATE_INTERVAL = 200;  // Update LCD every 200ms
const unsigned long SENSOR_DELAY = 30;          // Main loop delay

// Distance thresholds (cm)
const int DISTANCE_DANGER = 5;
const int DISTANCE_CRITICAL = 10;
const int DISTANCE_WARNING = 15;
const int DISTANCE_CAUTION = 30;
const int DISTANCE_ALERT = 50;

// Buzzer timing
const int BEEP_DURATION = 10;     // Short beep duration (ms)
const int BUZZ_CONTINUOUS = 1;   // Continuous buzzer flag
const int BUZZ_FAST = 100;       // Fast beeping interval
const int BUZZ_MEDIUM = 300;     // Medium beeping interval
const int BUZZ_SLOW = 700;       // Slow beeping interval
const int BUZZ_VERY_SLOW = 1200; // Very slow beeping interval

// ==================== GLOBAL VARIABLES ====================
// Distance measurement
long duration = 0;
int currentDistance = 0;
int readings[NUM_READINGS];
int readIndex = 0;
long total = 0;
int averageDistance = 0;

// Timing control
unsigned long previousBuzzerMillis = 0;
unsigned long lastLCDUpdate = 0;
bool buzzerState = false;

// ==================== SETUP ====================
void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Initialize LCD
  lcd.begin(16, 2);
  lcd.clear();
  
  // Show startup message
  lcd.setCursor(0, 0);
  lcd.print("Arduino Radar");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(2000);
  lcd.clear();
  
  // Initialize pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Initialize moving average array
  for (int i = 0; i < NUM_READINGS; i++) {
    readings[i] = MAX_DISTANCE;
  }
  total = MAX_DISTANCE * NUM_READINGS;
  
  // Initial LED state (safe)
  setLEDState(LED_GREEN);
  
  Serial.println("Arduino Radar System Ready");
  Serial.println("Distance readings will be displayed...");
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long currentMillis = millis();
  
  // Read distance from ultrasonic sensor
  currentDistance = measureDistance();
  
  // Apply moving average filter
  averageDistance = updateMovingAverage(currentDistance);
  
  // Control LEDs based on distance
  updateLEDs(averageDistance);
  
  // Control buzzer based on distance
  updateBuzzer(averageDistance, currentMillis);
  
  // Update LCD display
  updateLCD(currentMillis);
  
  // Send data to Serial for monitoring/logging
  Serial.println(averageDistance);
  
  delay(SENSOR_DELAY);
}

// ==================== SENSOR FUNCTIONS ====================
int measureDistance() {
  // Trigger ultrasonic pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Measure echo duration with timeout
  duration = pulseIn(ECHO_PIN, HIGH, SENSOR_TIMEOUT);
  
  // Calculate distance in centimeters
  if (duration == 0) {
    return MAX_DISTANCE; // Return max distance if no echo received
  }
  
  int distance = (duration * 0.034) / 2;
  
  // Validate reading
  if (distance <= 0 || distance > MAX_DISTANCE) {
    distance = MAX_DISTANCE;
  }
  
  return distance;
}

int updateMovingAverage(int newReading) {
  // Remove oldest reading from total
  total -= readings[readIndex];
  
  // Add new reading
  readings[readIndex] = newReading;
  total += readings[readIndex];
  
  // Advance index (circular buffer)
  readIndex = (readIndex + 1) % NUM_READINGS;
  
  // Calculate and return average
  return total / NUM_READINGS;
}

// ==================== LED CONTROL ====================
void updateLEDs(int distance) {
  if (distance <= DISTANCE_WARNING && distance > 0) {
    // Danger/Critical/Warning zones - Red LED
    setLEDState(LED_RED);
  } 
  else if (distance > DISTANCE_WARNING && distance <= DISTANCE_ALERT) {
    // Caution/Alert zones - Blue LED
    setLEDState(LED_BLUE);
  } 
  else {
    // Safe zone - Green LED
    setLEDState(LED_GREEN);
  }
}

void setLEDState(int activeLED) {
  digitalWrite(LED_RED, activeLED == LED_RED ? HIGH : LOW);
  digitalWrite(LED_BLUE, activeLED == LED_BLUE ? HIGH : LOW);
  digitalWrite(LED_GREEN, activeLED == LED_GREEN ? HIGH : LOW);
}

// ==================== BUZZER CONTROL ====================
void updateBuzzer(int distance, unsigned long currentMillis) {
  int buzzerInterval = getBuzzerInterval(distance);
  
  if (buzzerInterval == BUZZ_CONTINUOUS) {
    // Continuous buzzer for extreme danger
    digitalWrite(BUZZER_PIN, HIGH);
  } 
  else if (buzzerInterval == 0) {
    // No buzzer for safe distances
    digitalWrite(BUZZER_PIN, LOW);
    buzzerState = false;
  } 
  else {
    // Intermittent beeping
    handleIntermittentBuzzer(currentMillis, buzzerInterval);
  }
}

int getBuzzerInterval(int distance) {
  if (distance <= DISTANCE_DANGER && distance > 0) {
    return BUZZ_CONTINUOUS;
  } 
  else if (distance <= DISTANCE_CRITICAL) {
    return BUZZ_FAST;
  } 
  else if (distance <= DISTANCE_WARNING) {
    return BUZZ_MEDIUM;
  } 
  else if (distance <= DISTANCE_CAUTION) {
    return BUZZ_SLOW;
  } 
  else if (distance <= DISTANCE_ALERT) {
    return BUZZ_VERY_SLOW;
  } 
  else {
    return 0; // No buzzer
  }
}

void handleIntermittentBuzzer(unsigned long currentMillis, int interval) {
  if (buzzerState) {
    // Buzzer is currently ON - check if beep duration has elapsed
    if (currentMillis - previousBuzzerMillis >= BEEP_DURATION) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerState = false;
      previousBuzzerMillis = currentMillis;
    }
  } 
  else {
    // Buzzer is currently OFF - check if pause interval has elapsed
    if (currentMillis - previousBuzzerMillis >= interval) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerState = true;
      previousBuzzerMillis = currentMillis;
    }
  }
}

// ==================== LCD DISPLAY ====================
void updateLCD(unsigned long currentMillis) {
  if (currentMillis - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
    lastLCDUpdate = currentMillis;
    
    // Display distance on first line
    lcd.setCursor(0, 0);
    lcd.print("Distance:       "); // Clear old digits
    lcd.setCursor(10, 0);
    if (averageDistance >= MAX_DISTANCE) {
      lcd.print("--");
    } else {
      lcd.print(averageDistance);
    }
    lcd.print("cm");
    
    // Display status on second line
    lcd.setCursor(0, 1);
    lcd.print(getStatusMessage(averageDistance));
  }
}

String getStatusMessage(int distance) {
  if (distance <= DISTANCE_DANGER && distance > 0) {
    return "Status: DANGER! ";
  } 
  else if (distance <= DISTANCE_CRITICAL) {
    return "Status: CRITICAL";
  } 
  else if (distance <= DISTANCE_WARNING) {
    return "Status: WARNING ";
  } 
  else if (distance <= DISTANCE_CAUTION) {
    return "Status: CAUTION ";
  } 
  else if (distance <= DISTANCE_ALERT) {
    return "Status: ALERT   ";
  } 
  else {
    return "Status: CLEAR   ";
  }
}
