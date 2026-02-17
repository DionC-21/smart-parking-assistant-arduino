# Smart Parking Assistant Arduino

An intelligent autonomous robot combining line-following capabilities with advanced obstacle detection. Perfect for automated parking systems, warehouse navigation, or educational robotics projects.

![Arduino Robot](https://img.shields.io/badge/Arduino-Robot-blue.svg)
![Version](https://img.shields.io/badge/Version-2.0-green.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)
![Motors](https://img.shields.io/badge/Motors-L298N-red.svg)

## 🤖 Features

- **Intelligent Line Following**: Dual IR sensor system with smooth path tracking
- **Obstacle Avoidance**: HC-SR04 ultrasonic sensor with hysteresis-based detection
- **Motor Control**: L298N driver with PWM speed control for precise movement
- **Real-time Display**: 16x2 LCD showing command status and distance readings
- **Smart Recovery**: Automatic line search when path is lost
- **Non-blocking Architecture**: Efficient timing system for smooth operation
- **Serial Monitoring**: Real-time data output for debugging and analysis

## 📋 Components Required

### Core Components
| Component | Quantity | Specifications |
|-----------|----------|----------------|
| Arduino Uno/Nano | 1 | Main controller |
| IR Line Sensors | 2 | Digital output type |
| HC-SR04 Ultrasonic | 1 | Distance measurement |
| L298N Motor Driver | 1 | Dual H-Bridge |
| DC Geared Motors | 2 | 6V, with wheels |
| 16x2 LCD Display | 1 | HD44780 compatible |

### Power & Mechanical
| Component | Quantity | Notes |
|-----------|----------|-------|
| Robot Chassis | 1 | Acrylic or 3D printed |
| Wheels | 2-4 | Depending on chassis |
| Caster Wheel | 1 | Front support wheel |
| Li-Po Battery | 1 | 7.4V 2200mAh recommended |
| Jumper Wires | - | Various lengths |
| Breadboard | 1 | For prototyping |

## 🔌 Wiring Configuration

### Arduino to L298N Motor Driver
| L298N Pin | Arduino Pin | Function |
|-----------|-------------|----------|
| ENA | 5 (PWM) | Left motor speed |
| IN1 | 8 | Left motor direction 1 |
| IN2 | 7 | Left motor direction 2 |
| ENB | 6 (PWM) | Right motor speed |
| IN3 | 12 | Right motor direction 1 |
| IN4 | 13 | Right motor direction 2 |

### Sensors and Display
| Component | Arduino Pin | Notes |
|-----------|-------------|-------|
| Left IR Sensor | A0 | Line detection |
| Right IR Sensor | A1 | Line detection |
| HC-SR04 Trig | 9 | Ultrasonic trigger |
| HC-SR04 Echo | 10 | Ultrasonic echo |
| LCD RS | 2 | Register select |
| LCD E | 3 | Enable |
| LCD D4-D7 | 4,11,A2,A3 | Data pins |

### Power Distribution
```
Battery 7.4V ──┬── L298N VIN (Motor Power)
               └── Arduino VIN (Logic Power)

Common Ground: Arduino GND ←→ L298N GND ←→ Battery GND
```

## 🛣️ Operation Modes

### Line Following Logic
| Left Sensor | Right Sensor | Action | Description |
|-------------|--------------|--------|-------------|
| ⚫ ON | ⚫ ON | FORWARD | Both sensors detect line |
| ⚫ ON | ⚪ OFF | LEFT | Turn left to follow line |
| ⚪ OFF | ⚫ ON | RIGHT | Turn right to follow line |
| ⚪ OFF | ⚪ OFF | SEARCH | Rotate to find line |

### Obstacle Detection
- **Stop Distance**: 15cm (robot stops when obstacle detected)
- **Clear Distance**: 22cm (robot resumes when obstacle moves away)
- **Hysteresis**: Prevents oscillation between stop/go states

### Speed Settings
```cpp
SPEED_FORWARD = 175;  // Normal forward movement
SPEED_TURN = 150;     // Turning speed (reduced for control)
SPEED_SEARCH = 140;   // Line search rotation
```

## 🚀 Installation & Setup

### 1. Hardware Assembly
1. Mount Arduino and L298N on robot chassis
2. Install motors and wheels
3. Mount sensors at front of robot (2-5cm from ground)
4. Connect LCD display for status monitoring
5. Wire according to connection diagram

### 2. Software Installation
1. Install Arduino IDE
2. Open `smart-parking-assistant-arduino.ino`
3. Select your board type (Arduino Uno/Nano)
4. Upload the code

### 3. Calibration
```cpp
// Adjust these values based on your sensors:
const bool BLACK_LINE_IS_HIGH = false;  // Set true if sensors output HIGH for black line
const int SPEED_FORWARD = 175;           // Adjust for your motors/battery
const int OBSTACLE_STOP_DISTANCE = 15;  // Adjust for desired stopping distance
```

### 4. Testing Procedure
1. **Power Test**: Verify all components receive power
2. **Sensor Test**: Check IR sensors detect line properly
3. **Motor Test**: Ensure motors rotate in correct directions
4. **Distance Test**: Verify ultrasonic sensor readings
5. **Integration Test**: Run complete system on test track

## 📊 Performance Optimization

### Line Detection Tips
- **Sensor Height**: Mount 2-5cm above surface
- **Line Width**: Works best with 1.5-3cm wide black tape
- **Surface**: Light-colored, non-reflective surfaces ideal
- **Lighting**: Avoid direct sunlight or very bright lights

### Motor Tuning
```cpp
// Fine-tune these values for your specific setup:
const int SPEED_FORWARD = 175;   // Increase for faster movement
const int SPEED_TURN = 150;      // Decrease for sharper turns
const int SPEED_SEARCH = 140;    // Adjust search rotation speed
```

### Power Management
- **Recommended**: 7.4V Li-Po battery for optimal performance
- **Alternative**: 6x AA batteries (9V) with voltage regulator
- **Current Draw**: ~1-2A during operation
- **Run Time**: 2-4 hours depending on battery capacity

## 🔧 Troubleshooting

### Common Issues

| Problem | Possible Cause | Solution |
|---------|----------------|----------|
| Robot doesn't move | Power/wiring | Check battery charge, verify motor connections |
| Erratic line following | Sensor calibration | Adjust `BLACK_LINE_IS_HIGH` setting |
| Motors run backwards | Wiring reversed | Swap motor connections or modify code |
| LCD blank/garbled | Contrast/wiring | Check connections, adjust contrast pot |
| No obstacle detection | Sensor alignment | Ensure HC-SR04 has clear line of sight |
| Robot loses line often | Sensor height/sensitivity | Adjust sensor mounting height |

### Debug Features
- **Serial Monitor**: View real-time distance and command data
- **LCD Display**: Shows current command and obstacle distance
- **LED Indicators**: Can be added to motor driver for status

## 📈 Advanced Features

### Custom Behaviors
```cpp
// Add these enhancements to the base code:
- Speed ramping for smooth acceleration
- PID control for precise line following  
- Multiple line intersection handling
- Remote control via Bluetooth/WiFi
- Automated parking sequence
- Obstacle mapping and avoidance
```

### Sensor Upgrades
- **Encoders**: Add wheel encoders for precise distance measurement
- **IMU**: Include gyroscope for heading control
- **Camera**: OpenCV-based line detection
- **Multiple Ultrasonics**: 360-degree obstacle detection

## 🎯 Applications

- **Automated Parking**: Self-parking car prototypes
- **Warehouse Automation**: Guided inventory robots  
- **Educational Projects**: Robotics competitions and learning
- **Home Automation**: Autonomous cleaning robots
- **Security Patrol**: Automated perimeter monitoring

## 📝 Code Architecture

### Main Functions
- `setup()`: Hardware initialization and startup
- `loop()`: Main control cycle (non-blocking)
- `readLineFollowingSensors()`: Process IR sensor inputs
- `updateDistanceSensor()`: Scheduled ultrasonic readings
- `executeCommand()`: Motor control dispatcher
- `updateLCDDisplay()`: Status information display

### State Management
- **Command States**: STOP, FORWARD, LEFT, RIGHT, SEARCH
- **Obstacle Detection**: Boolean flag with hysteresis
- **Timing Control**: Non-blocking `millis()` based scheduling

## 🔄 Future Enhancements

- [ ] **WiFi Connectivity**: Remote monitoring and control
- [ ] **Mobile App**: Smartphone interface for parameters
- [ ] **Path Recording**: Save and replay routes
- [ ] **Multiple Robots**: Coordination between units
- [ ] **Machine Learning**: Adaptive line-following behavior
- [ ] **Voice Control**: Audio command interface

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for:
- Code optimizations
- New features
- Bug fixes  
- Documentation improvements
- Hardware compatibility updates

## 🆘 Support

For help with this project:
- **GitHub Issues**: Report bugs or request features
- **Serial Monitor**: Use for real-time debugging
- **Community**: Share your builds and modifications

---

**Built with ❤️ for autonomous robotics enthusiasts**

*Transform any surface into a smart navigation system with this versatile line-following robot!*
