# 3-Wheel Holonomic Robot - PS4 Controlled

A fully holonomic 3-wheel robot with high-torque RS775 motors, controlled via PS4 Bluetooth controller using Teensy 4.1 and inverse kinematics.

**This project was developed under the guidance of SRM Team Robocon.**

## 🚀 Features

- **True Holonomic Movement**: Move in any direction while rotating simultaneously
- **PS4 Bluetooth Control**: Wireless control with PS4 controller
- **High-Torque Drive**: RS775 motors (6000RPM) with 4:1 gear reduction
- **24V Power System**: High-performance operation
- **Real-time Control**: Teensy 4.1 microcontroller with inverse kinematics
- **Safety Features**: Speed limiting, deadzone, auto-stop on disconnect

## 📋 Hardware Specifications

### Power System
- **Battery**: 24V LiPo
- **Motors**: 3x RS775 (6000 RPM) with 4:1 gear reduction
- **Motor Drivers**: 3x Cytron MDD20A (20A continuous)
- **Final Wheel Speed**: 1500 RPM

### Control System
- **Microcontroller**: Teensy 4.1
- **Bluetooth**: TP-Link USB Bluetooth module
- **USB Host**: HW-766 USB 2.0 Adapter
- **Connectors**: SB Connectors

### Wheel Configuration
- **3 Omni Wheels** at 120° intervals
- **Wheel 1**: 0° (pointing forward)
- **Wheel 2**: 120°
- **Wheel 3**: 240°

## 🔧 Wiring Diagram

```
Teensy 4.1 Pinout:
├── Motor 1: PWM=2, DIR=4    → Wheel 1 (0°)
├── Motor 2: PWM=3, DIR=5    → Wheel 2 (120°)  
├── Motor 3: PWM=23, DIR=21  → Wheel 3 (240°)
├── Bluetooth: USB Host Port
└── Power: 24V to MDD20A drivers
```

### Cytron MDD20A Connections
- **DIR** → Teensy Direction pins
- **PWM** → Teensy PWM pins  
- **VCC** → 24V+
- **GND** → 24V-
- **Motor Outputs** → RS775 motors

## 🧮 Mathematical Foundation

Core Inverse Kinematics Formula
The fundamental equation for each wheel's velocity is:

```
V_wheel = -sin(θ) * Vx + cos(θ) * Vy + ω
```
Where:

θ = Wheel angle from forward direction

Vx = Forward/backward velocity (-128 to +128)

Vy = Left/right strafing velocity (-128 to +128)

ω = Rotational velocity (-128 to +128)

V_wheel = Individual wheel speed output

For wheels at 0°, 120°, 240°:

```
V1 = 0.000 * Vx + 1.000 * Vy + ω      // Wheel 1 (0°)
V2 = -0.866 * Vx - 0.500 * Vy + ω     // Wheel 2 (120°)
V3 = 0.866 * Vx - 0.500 * Vy + ω      // Wheel 3 (240°)
```

### Variable Definitions
- **Vx**: Forward/backward velocity (-128 to +128)
- **Vy**: Left/right strafing velocity (-128 to +128)  
- **ω**: Rotational velocity (-128 to +128)
- **V1, V2, V3**: Individual wheel speeds

## ⚡ Performance Specifications

### Motor Performance
- **Base Speed**: 6000 RPM
- **Gear Reduction**: 4:1
- **Wheel Speed**: 1500 RPM
- **Wheel Diameter**: 100mm
- **Theoretical Speed**: ~7.85 m/s (28.3 km/h)

### Speed Calculation
```
Wheel circumference = π × 100mm = 314mm
Max wheel speed = 1500 RPM = 25 RPS
Theoretical max speed = 25 × 0.314m = 7.85 m/s
Effective speed (35% limit) = 7.85 × 0.35 = 2.75 m/s
```

### Power Consumption
- **Operating Voltage**: 24V
- **Typical Current**: 2-8A per motor
- **Total Power**: ~126W at 35% speed
- **Peak Power**: ~360W at full speed

## 🎮 Control Mapping

### PS4 Controller Layout
- **Left Stick Y-axis**: Forward/Backward motion
- **Left Stick X-axis**: Left/Right strafing
- **Right Stick X-axis**: Rotation

### Movement Examples

**Pure Forward:**
```cpp
Vx = 128, Vy = 0, ω = 0
V1 = 0, V2 = -110.85, V3 = 110.85
```

**Pure Right Strafe:**
```cpp
Vx = 0, Vy = 128, ω = 0
V1 = 128, V2 = -64, V3 = -64
```

**Pure Rotation:**
```cpp
Vx = 0, Vy = 0, ω = 128  
V1 = 128, V2 = 128, V3 = 128
```

## 🛠 Installation Guide

### 1. Hardware Assembly
1. Mount RS775 motors with 4:1 gearboxes
2. Install 100mm omni wheels at precise 120° intervals
3. Connect motors to Cytron MDD20A drivers
4. Wire drivers to Teensy 4.1 PWM and DIR pins
5. Connect USB Bluetooth module
6. Connect 24V battery with proper fusing

### 2. Software Setup
1. Install Arduino IDE with Teensyduino
2. Install required libraries:
   ```cpp
   USBHost_t36
   PS4 Controller libraries
   ```
3. Upload the provided code to Teensy 4.1
4. Pair PS4 controller via Bluetooth

### 3. Code Configuration
Adjust these constants in the code as needed:

```cpp
const float MaxSpeed = 0.35;    // Speed limit (0.0 to 1.0)
const int DEADZONE = 15;        // Joystick deadzone
```

## 📁 Source Code

The main control code implements:

- **PS4 controller reading** with deadzone
- **Inverse kinematics calculation** 
- **Motor control** with direction handling
- **Safety features** and error handling

[View complete source code](src/holonomic_robot.ino)

## 🎯 Usage Instructions

1. **Power On**: Connect 24V battery
2. **Pair Controller**: Put PS4 controller in pairing mode
3. **Connect**: Wait for "Controller Connected" message
4. **Operate**:
   - Left stick: Move robot (forward/back, strafe)
   - Right stick: Rotate robot
   - Combined: Complex holonomic maneuvers

## ⚠️ Safety Notes

### Mechanical Safety
- Keep hands away from moving 100mm wheels and gears
- Operate on clear, flat surfaces
- Secure long hair and loose clothing
- Supervise around children and pets

### Electrical Safety  
- Use proper LiPo battery charging procedures
- Install fuse protection (20-30A recommended)
- Avoid short circuits in wiring
- Disconnect battery when not in use

### Operational Safety
- Start with low speed settings (MaxSpeed = 0.35)
- Test in open areas away from obstacles
- Be aware of tip-over risks during sharp turns
- Stop immediately if any abnormal behavior occurs

## 🔧 Troubleshooting

### Common Issues

**Controller Not Connecting**
- Verify Bluetooth module is powered
- Ensure PS4 controller is in pairing mode
- Check USB host initialization in serial monitor

**Motors Not Spinning**
- Confirm 24V power to MDD20A drivers
- Check PWM and DIR wiring to Teensy
- Verify motor connections to drivers

**Incorrect Movement Directions**
- Check wheel alignment (0°, 120°, 240°)
- Verify motor direction settings
- Confirm inverse kinematics formulas match wheel layout

### Performance Tuning

**Increase Responsiveness:**
```cpp
const float MaxSpeed = 0.6;  // More speed
```

**Better Low-Speed Control:**
```cpp
const int DEADZONE = 10;    // Smaller deadzone
```

## 🔄 Calibration Procedure

1. **Verify Wheel Directions**:
   - Forward motion should move robot forward
   - Right strafe should move robot right
   - Rotation should spin counter-clockwise

2. **Adjust Motor 3 Direction**:
   - The code inverts Motor 3 (`-V3`)
   - Remove negation if movement is incorrect

3. **Test All Motion Patterns**:
   - Pure translation (forward, back, left, right)
   - Pure rotation (clockwise, counter-clockwise)
   - Combined motions (diagonal + rotation)

## 📈 Future Enhancements

- [ ] PID speed control for precise movement
- [ ] Encoder feedback for closed-loop control

## 📚 References & Resources

### Technical Documentation
- **PS4 Controller with Teensy**: [PJRC Forum Thread](https://forum.pjrc.com/index.php?threads/ps4-dual-shock-connected-to-teensy-4-0.73259/)
- **Holonomic Drive Theory**: [Research Paper](https://www.internationaljournalssrg.org/IJEEE/2019/Volume6-Issue12/IJEEE-V6I12P101.pdf)

### Video Tutorials
- **Omnidirectional Robot Basics**: [YouTube Tutorial](https://youtu.be/ULQLD6VvXio?si=zd_4OOjbjtM2_eLP)
- **Inverse Kinematics Explained**: [YouTube Tutorial](https://youtu.be/oSbK4zDSTL8?si=KpNk8-_cX2U9Pxpw)

### Libraries Used
- `USBHost_t36` - USB host functionality for Teensy
- PS4 Controller libraries - Bluetooth communication

## 🤝 Contributing

We welcome contributions! Please:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request


## 🙏 Acknowledgments

- **SRM Team Robocon** for guidance and support
- **Teensy Development Team** for excellent microcontroller support
- **Cytron Technologies** for robust MDD20A motor drivers
- **PS4 Controller Library Contributors** for Bluetooth integration
- **Open Source Community** for continuous inspiration

---

**Happy Building!** 🚀

For questions or support, please open an issue in the repository.

