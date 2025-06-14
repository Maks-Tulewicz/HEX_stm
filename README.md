# 🕷️ Advanced Hexapod Robot Control System

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/Platform-STM32F446RE-blue.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html)
[![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen.svg)]()

> **Advanced 6-legged walking robot with sophisticated inverse kinematics and multiple gait algorithms**

A comprehensive hexapod robot control system featuring 18 DOF, dual I2C architecture, and three distinct walking gaits optimized for different performance characteristics.

![494859441_1119192829969747_5225182293932756698_n](https://github.com/user-attachments/assets/7e354689-5dd5-4ebb-bd17-253abfef4211)

## 📋 Table of Contents

- [Features](#-features)
- [Hardware Architecture](#-hardware-architecture)
- [Gait Algorithms](#-gait-algorithms)
- [Inverse Kinematics](#-inverse-kinematics)
- [Performance Metrics](#-performance-metrics)
- [Installation](#-installation)
- [Usage](#-usage)
- [Technical Details](#-technical-details)
- [Future Work](#-future-work)
- [Contributing](#-contributing)

## ✨ Features

- **🦿 18 DOF System**: 6 legs × 3 joints each with MG996R servomotors
- **🧠 Advanced IK**: Geometric inverse kinematics with workspace validation
- **🚶 Multiple Gaits**: Wave (stability), Tripod (speed), Bipedal (balance)
- **⚡ High Performance**: Up to 50 cm/s theoretical speed
- **🔧 Dual I2C**: Independent control of left/right leg groups
- **📊 Diagnostics**: Comprehensive testing and calibration tools
- **🎯 Precision**: ±0.1° mechanical accuracy

## 🔧 Hardware Architecture

### Core Components
- **MCU**: STM32F446RE (ARM Cortex-M4, 180MHz)
- **Servo Controllers**: 2× PCA9685 (16-channel PWM)
- **Actuators**: 18× MG996R servomotors (180° range)
- **Communication**: Dual I2C buses (100kHz)

### Mechanical Specifications
```
Leg Segments:
├── Hip (L₁): 5.8 cm
├── Thigh (L₂): 10.5 cm
└── Shin (L₃): 21.5 cm

Working Range: 3.0 - 28.0 cm radius
Joint Limits: Extracted from ROS hexapod_description
```

### I2C Architecture
```
I2C1 (0x40) ─── Left Legs  (1, 3, 5)
I2C2 (0x41) ─── Right Legs (2, 4, 6)
```

## 🚶 Gait Algorithms

### 1. Wave Gait - Maximum Stability
- **Pattern**: Sequential single-leg movement (1→2→3→4→5→6)
- **Stability**: 5 legs on ground at all times
- **Cycle Time**: 360ms
- **Speed**: ~11.1 cm/s
- **Use Case**: Rough terrain, maximum stability

### 2. Tripod Gait - Maximum Speed
- **Pattern**: Alternating triangular groups (A: 1,4,5 ↔ B: 2,3,6)
- **Stability**: 3 legs on ground minimum
- **Cycle Time**: 120ms (Ultra Speed Mode)
- **Speed**: ~50.0 cm/s
- **Use Case**: Fast locomotion on flat surfaces

### 3. Bipedal Gait - Balanced Performance
- **Pattern**: Paired leg movement (0: 1,4; 1: 2,5; 2: 3,6)
- **Stability**: 4 legs on ground
- **Cycle Time**: 210ms
- **Speed**: ~28.6 cm/s
- **Use Case**: Compromise between speed and stability

## 🧮 Inverse Kinematics

### Mathematical Model

The IK solver uses geometric approach with cosine law:

```math
q₁ = arctan2(y_local, x_local)
D = √(r² + h²), where r = √(x_local² + y_local²) - L₁
γ = arccos((D² - L₂² - L₃²)/(2L₂L₃))
q₂ = -(arctan2(h,r) - arccos((D² + L₂² - L₃²)/(2L₂D)))
q₃ = π - γ
```

### Features
- ✅ **Workspace Validation**: Range checking before computation
- ✅ **Coordinate Transformations**: Individual leg origins from URDF
- ✅ **Mirror Compensation**: Automatic right-leg inversion
- ✅ **Hip Offset Handling**: Mechanical constraint compensation

## 📊 Performance Metrics

| Metric | Wave Gait | Bipedal Gait | Tripod Gait |
|--------|-----------|--------------|-------------|
| **Cycle Time** | 360ms | 210ms | 120ms |
| **Legs on Ground** | 5 | 4 | 3 |
| **Theoretical Speed** | 11.1 cm/s | 28.6 cm/s | 50.0 cm/s |
| **Stability** | Maximum | High | Moderate |
| **Terrain Capability** | Rough | Mixed | Flat |

### Servo Calibration
```c
PWM_MIN = 110  // 0°
PWM_MID = 305  // 90°
PWM_MAX = 500  // 180°
Resolution: ±1 PWM point (~0.1° mechanical)
```

## 🚀 Installation

### Prerequisites
- STM32CubeIDE or compatible ARM toolchain
- STM32F446RE development board
- 2× PCA9685 PWM controllers
- 18× MG996R servomotors
- Appropriate power supply (6V, 10A recommended)

### Build Instructions

1. **Clone the repository**
```bash
git clone https://github.com/yourusername/hexapod-robot
cd hexapod-robot
```

2. **Open in STM32CubeIDE**
```bash
# Import existing project into workspace
File → Import → General → Existing Projects into Workspace
```

3. **Configure hardware connections**
```c
// Update I2C addresses in config.h if needed
#define PCA9685_ADDRESS_LEFT  0x40
#define PCA9685_ADDRESS_RIGHT 0x41
```

4. **Build and flash**
```bash
Project → Build All
Run → Debug As → STM32 MCU Debugging
```

## 🎮 Usage

### Basic Operation

```c
// Initialize the system
hexapod_init();

// Select gait algorithm
gait_set_mode(TRIPOD_GAIT);

// Start walking
gait_start_forward();

// Control commands
gait_step_forward();    // Single step
gait_turn_left();       // Turn in place
gait_stop();           // Stop motion
```

### Diagnostic Tools

```c
// Test inverse kinematics
debugLegIK(leg_id, x, y, z);

// Verify all base positions
testAllBasePositions();

// Calibrate servo range
PCA9685_TestPWMRange(servo_id);

// Print current configuration
printTripodConfig();
```

### Advanced Control

```c
// Custom step parameters
tripod_config.step_height = 8.0f;     // Step height in cm
tripod_config.step_length = 12.0f;    // Step length in cm
tripod_config.interpolation_points = 30; // Ultra speed mode

// Direct position control
setLegPosition(leg_id, x, y, z);
updateAllLegs();
```

## 🔬 Technical Details

### Communication Protocol
- **I2C Frequency**: 100kHz
- **PWM Frequency**: 50Hz (20ms period)
- **Control Loop**: 50Hz update rate
- **Servo Response**: ~20ms typical

### Coordinate Systems
```
Robot Frame:
  X: Forward (+) / Backward (-)
  Y: Left (+) / Right (-)
  Z: Up (+) / Down (-)

Leg Local Frame:
  Origin at hip joint
  Z-axis pointing down
```

### Safety Features
- Workspace boundary checking
- Servo limit validation
- I2C communication error handling
- Graceful failure modes

## 🔮 Future Work

### Planned Enhancements

1. **🧮 Jacobian-based Kinematics**
   - Replace geometric IK with Jacobian method
   - Velocity control for leg endpoints
   - Singularity handling and optimization

2. **🎯 Body Orientation Control**
   - 6-DOF body pose control (±15° roll/pitch, ±30° yaw)
   - Automatic height compensation during walking
   - Terrain adaptation capabilities

3. **📡 Wireless Communication**
   - nRF24L01+ or ESP32 integration
   - Real-time joystick control
   - 50Hz command transmission rate

4. **🧠 Advanced Control**
   - Gait parameter optimization
   - Terrain type detection
   - Energy-efficient locomotion patterns

### Research Applications
- Comparative analysis: Geometric vs. Jacobian IK
- Stability metrics across different gaits
- Energy efficiency optimization
- Rough terrain navigation strategies


## 📞 Contact

**Maksymilian Tulewicz**
- Email: maksymilian.tulewicz@gmail.com
---

⭐ **Star this repository if you found it helpful!**
