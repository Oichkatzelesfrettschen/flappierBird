# FlappierBird 🐦

[![CI/CD Pipeline](https://github.com/Oichkatzelesfrettschen/flappierBird/actions/workflows/ci.yml/badge.svg)](https://github.com/Oichkatzelesfrettschen/flappierBird/actions/workflows/ci.yml)
[![Documentation](https://img.shields.io/badge/docs-latest-blue.svg)](docs/RESEARCH_DEVELOPMENT_REPORT.md)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

An autonomous ornithopter (flapping wing robot) with advanced control systems, formal verification, and comprehensive research documentation.

## 🚀 Features

- **Advanced Control Systems**: Quaternion-based orientation, sensor fusion (IMU, pressure, gyroscope)
- **Machine Learning**: MLP-based situational awareness and adaptive control
- **Formal Verification**: TLA+ specifications and Z3 constraint solving
- **Modern Build System**: PlatformIO and CMake integration
- **Comprehensive Documentation**: Detailed mathematical foundations, aerodynamics, and materials science
- **CI/CD Pipeline**: Automated builds, tests, and verification
- **Hardware**: Teensy 4.0, BNO055 IMU, RFM69 radio, servo control

## 📚 Documentation

- **[Research & Development Report](docs/RESEARCH_DEVELOPMENT_REPORT.md)**: Comprehensive technical analysis covering:
  - Mathematical foundations (quaternions, octonions, Lie algebra)
  - Materials science and structural analysis
  - Fluid mechanics and aerodynamics
  - Machine learning architectures
  - Sensor fusion algorithms
  - Formal verification methods (TLA+, Z3)
  - Hardware interaction mathematics
  - Build system modernization

## 🏗️ Build System

### PlatformIO (Recommended)

```bash
# Install PlatformIO
pip install platformio

# Build onboard firmware
pio run -e teensy40

# Build remote control firmware
pio run -e teensy40_remote

# Upload firmware
pio run -e teensy40 -t upload

# Run tests
pio test -e native

# Monitor serial output
pio device monitor
```

### CMake

```bash
# Configure
cmake -B build -DBUILD_TESTS=ON -DENABLE_TLA_VERIFICATION=ON

# Build
cmake --build build

# Run tests
ctest --test-dir build

# Install
cmake --install build
```

## 🔬 Verification

### TLA+ Formal Verification

```bash
# Install TLA+ tools
wget https://github.com/tlaplus/tlaplus/releases/download/v1.7.1/tla2tools.jar

# Run verification
cd verification/tla
java -cp ../../tla2tools.jar tlc2.TLC FlappierBird.tla
```

### Z3 Constraint Solving

```bash
# Install Z3
sudo apt-get install z3

# Run constraint verification
cd verification/z3
z3 FlappierBird.smt2
```

## 🧪 Testing

The project includes comprehensive unit tests for:
- Quaternion mathematics
- Sensor fusion algorithms
- Control law verification
- Hardware abstraction layers

## 🛠️ Hardware Setup

### Components

- **Microcontroller**: Teensy 4.0 @ 600 MHz
- **IMU**: Adafruit BNO055 (9-DOF)
- **Radio**: RFM69HCW (915 MHz)
- **Servos**: 2x hobby servos for control surfaces
- **ESC**: Electronic Speed Controller for motor
- **Battery**: 2S LiPo (7.4V)

### Connections

See [hardware documentation](docs/hardware.md) for detailed wiring diagrams.

## 📊 Simulations

MATLAB/Octave simulations for:
- Fourbar linkage kinematics
- Torque analysis
- Aerodynamic modeling
- Material stress calculations

```bash
# Run simulations (requires Octave or MATLAB)
cd matlab
octave torques.m
```

## 🤖 Control System

### Architecture

```
Sensors (IMU, Pressure) → Sensor Fusion (EKF) → State Estimation
                                                       ↓
                                                  Controller
                                                       ↓
                                            Actuators (Servos, ESC)
```

### Control Modes

- **Manual**: Direct stick control from remote
- **Stabilized**: Automatic attitude stabilization
- **Autonomous**: AI-based flight control with ML

## 🔐 Safety Features

- Emergency stop on sensor failure
- Battery voltage monitoring
- Geofencing constraints
- Attitude limit protection
- Watchdog timer

## 📦 Project Structure

```
flappierBird/
├── arduino/
│   ├── onboard_active/    # Onboard control firmware
│   ├── remote_active/     # Remote control firmware
│   └── examples/          # Example sketches
├── matlab/                # Simulations and analysis
├── verification/
│   ├── tla/              # TLA+ specifications
│   └── z3/               # Z3 constraints
├── docs/                 # Documentation
├── tests/                # Unit tests
├── platformio.ini        # PlatformIO configuration
├── CMakeLists.txt        # CMake configuration
└── .github/workflows/    # CI/CD pipelines
```

## 🧑‍💻 Development

### Prerequisites

- Python 3.8+
- PlatformIO or Arduino IDE
- CMake 3.20+ (optional)
- Java 11+ (for TLA+)
- Z3 SMT solver (optional)

### Setup Development Environment

```bash
# Clone repository
git clone https://github.com/Oichkatzelesfrettschen/flappierBird.git
cd flappierBird

# Install dependencies
pip install platformio

# Install PlatformIO libraries
pio pkg install
```

## 📖 Mathematical Foundations

The system implements advanced mathematical concepts:

- **Quaternion Rotations**: Singularity-free 3D orientation representation
- **Kalman Filtering**: Optimal state estimation with sensor fusion
- **Lagrangian Mechanics**: Physics-based dynamics modeling
- **Control Theory**: PID, LQR, and adaptive control strategies
- **Aerodynamics**: Unsteady lift/drag modeling with Theodorsen theory

See the [full research report](docs/RESEARCH_DEVELOPMENT_REPORT.md) for detailed mathematics.

## 🤝 Contributing

Contributions welcome! Please read our contributing guidelines and code of conduct.

## 📄 License

This project is licensed under the MIT License - see LICENSE file for details.

## 👥 Authors

- Kirk Boyd - Original firmware and mechanical design
- FlappierBird Development Team - Research, verification, and build system

## 🙏 Acknowledgments

- Adafruit for sensor libraries
- LowPowerLab for RFM69 radio library
- TLA+ and Z3 development teams
- Ornithopter research community

## 📞 Contact

For questions, issues, or collaboration opportunities, please open an issue on GitHub.

---

**Version**: 1.0.0  
**Last Updated**: 2026-01-02  
**Status**: Active Development
