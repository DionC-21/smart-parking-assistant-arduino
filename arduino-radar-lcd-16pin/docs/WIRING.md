# Wiring Diagrams and Schematics

## Circuit Schematic

```
                    Arduino Uno/Nano
                         ┌─────┐
                    VIN──┤     ├──D13 (Green LED)
                    GND──┤     ├──D12 (Blue LED) 
                     5V──┤     ├──D11 (LCD D7)
                     3V3─┤     ├──D10 (Echo)
                   RESET─┤     ├──D9  (Trig)
                   IOREF─┤     ├──D8  (Red LED)
                      NC─┤     ├──D7  (Buzzer)
                      A0─┤     ├──D6  (LCD D6)
                      A1─┤     ├──D5  (LCD D5)
                      A2─┤     ├──D4  (LCD D4)
                      A3─┤     ├──D3  (LCD E)
                   SDA/A4─┤     ├──D2  (LCD RS)
                   SCL/A5─┤     ├──GND
                         └─────┘

HC-SR04 Ultrasonic Sensor:
VCC ──── 5V
GND ──── GND
Trig ─── D9
Echo ─── D10

16x2 LCD Display:
VSS ──── GND
VDD ──── 5V
V0  ──── Potentiometer center pin
RS  ──── D2
E   ──── D3
D4  ──── D4
D5  ──── D5
D6  ──── D6
D7  ──── D11
A   ──── 5V (Backlight)
K   ──── GND (Backlight)

LEDs (with 220Ω resistors):
Red LED   ── D8 ──[220Ω]── GND
Blue LED  ── D12 ──[220Ω]── GND
Green LED ── D13 ──[220Ω]── GND

Buzzer:
Positive ── D7
Negative ── GND

Potentiometer (10kΩ for LCD contrast):
Pin 1 ── 5V
Pin 2 ── LCD V0
Pin 3 ── GND
```

## Power Requirements

- **Arduino**: 5V via USB or external power supply
- **Total Current**: ~100-150mA (depending on LCD backlight)
- **Recommended Power**: USB power or 7-12V external adapter

## Assembly Notes

1. **LCD Contrast**: Adjust the 10kΩ potentiometer to get clear text visibility
2. **LED Resistors**: Use 220Ω resistors to limit current and prevent LED burnout
3. **Buzzer Type**: Works with both active and passive buzzers
4. **Wiring**: Use solid core wire for breadboard, stranded for final assembly
5. **Power**: Ensure stable 5V supply - voltage drops can cause erratic behavior

## Testing Points

- **Power**: Check 5V and GND distribution
- **Sensor**: Verify 5V at HC-SR04 VCC pin
- **LCD**: Adjust contrast for clear display
- **LEDs**: Test each LED individually
- **Serial**: Monitor output at 9600 baud for debugging
