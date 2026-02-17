# Assembly Guide

## Step-by-Step Build Instructions

### 1. Chassis Preparation
- Use acrylic robot chassis or 3D printed frame
- Mount Arduino Uno/Nano securely with standoffs
- Install L298N motor driver module
- Ensure adequate space for battery placement

### 2. Motor Installation
```
Left Motor (Motor A) ←→ L298N Terminal A
Right Motor (Motor B) ←→ L298N Terminal B

Motor Specifications:
- Voltage: 6-12V DC
- Type: Geared motors recommended
- RPM: 100-300 RPM optimal
- Torque: Sufficient for robot weight + payload
```

### 3. Sensor Mounting
**IR Line Sensors:**
- Mount at front of chassis
- Height: 2-5cm above ground
- Spacing: Match expected line width (typically 5-10cm apart)
- Angle: Point slightly downward for optimal detection

**HC-SR04 Ultrasonic:**
- Mount at front, centered
- Height: 10-15cm from ground
- Ensure clear field of view (no obstructions)
- Secure mounting to prevent vibration

### 4. LCD Display Mounting
- Position for easy viewing during operation
- Consider tilt angle for readability
- Secure all connections to prevent disconnection during movement

### 5. Power System
```
Recommended Power Setup:
┌─────────────┐
│ 7.4V Li-Po │
│ Battery     │
└─────┬───────┘
      │
      ├── L298N VIN (Motor Power)
      └── Arduino VIN (Logic Power)

Ground Connection: All GND pins connected together
```

## Wiring Checklist

### Motor Connections
- [ ] Left motor to L298N Motor A terminals
- [ ] Right motor to L298N Motor B terminals  
- [ ] Motor polarity noted for direction calibration

### Arduino to L298N
- [ ] ENA → Pin 5 (PWM)
- [ ] IN1 → Pin 8
- [ ] IN2 → Pin 7
- [ ] ENB → Pin 6 (PWM)
- [ ] IN3 → Pin 12
- [ ] IN4 → Pin 13

### Sensors
- [ ] Left IR sensor → A0
- [ ] Right IR sensor → A1
- [ ] HC-SR04 Trig → Pin 9
- [ ] HC-SR04 Echo → Pin 10

### LCD (16x2)
- [ ] RS → Pin 2
- [ ] E → Pin 3
- [ ] D4 → Pin 4
- [ ] D5 → Pin 11
- [ ] D6 → Pin A2
- [ ] D7 → Pin A3
- [ ] VSS → GND
- [ ] VDD → 5V
- [ ] V0 → Contrast pot center

### Power Distribution
- [ ] Battery positive to L298N VIN
- [ ] Battery positive to Arduino VIN
- [ ] All ground connections made
- [ ] Power switch installed (recommended)

## Testing Protocol

### 1. Power Test
```cpp
// Upload basic test code to verify connections
void setup() {
  Serial.begin(9600);
  Serial.println("Power test - all systems");
}
void loop() {
  Serial.println("System running...");
  delay(1000);
}
```

### 2. Motor Test
- Test each motor individually
- Verify rotation direction
- Check for smooth operation
- Adjust if motors run backwards

### 3. Sensor Calibration
- Test line sensors on actual track surface
- Verify ultrasonic distance readings
- Adjust sensor sensitivity if needed

### 4. LCD Verification
- Confirm display shows startup message
- Check contrast adjustment
- Verify all characters display properly

### 5. Integration Test
- Upload full program
- Test on simple straight line
- Verify obstacle detection
- Test line recovery behavior

## Troubleshooting Assembly Issues

| Issue | Check | Solution |
|-------|-------|----------|
| No LCD display | Contrast pot, wiring | Adjust potentiometer, verify connections |
| Motors don't run | Power, L298N enable pins | Check battery, verify ENA/ENB connections |
| Wrong motor direction | Motor wiring | Swap motor wires or modify code |
| Erratic sensor readings | Mounting, interference | Secure sensors, check for electrical noise |
| System resets randomly | Power supply | Check battery capacity, connections |

## Final Assembly Tips

1. **Cable Management**: Use zip ties to secure all wiring
2. **Weight Distribution**: Place battery for balanced weight
3. **Accessibility**: Keep USB port accessible for programming
4. **Protection**: Consider adding bumpers or guards
5. **Expansion**: Leave space for additional sensors/modules

## Performance Tuning

### Line Following Optimization
```cpp
// Adjust these parameters in code:
const int SPEED_FORWARD = 175;    // Base speed
const int SPEED_TURN = 150;       // Turning speed  
const bool BLACK_LINE_IS_HIGH = false; // Sensor logic
```

### Obstacle Detection Tuning
```cpp
const int OBSTACLE_STOP_DISTANCE = 15;  // Stop distance
const int OBSTACLE_CLEAR_DISTANCE = 22; // Resume distance
```

### Track Requirements
- **Line Width**: 1.5-3cm black tape
- **Surface**: Light colored, non-reflective
- **Curves**: Gentle curves work best
- **Intersections**: T-junctions and crossings supported with modifications
