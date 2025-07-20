# Adaptive Flight Controller v2.0 - Modular Architecture

A **production-grade, testable, and maintainable** flight controller with advanced adaptive features.

## 🏗️ Architecture Overview

This flight controller has been completely refactored from a 5000+ line monolith into a **clean, modular architecture** that is:

- **✅ Hardware Agnostic** - Core algorithms work on any platform
- **✅ Unit Testable** - Every module can be tested on desktop without hardware  
- **✅ Maintainable** - Clear interfaces and separation of concerns
- **✅ Fast Development** - Rapid iteration with desktop testing
- **✅ Professional** - CI/CD ready with static analysis and coverage

## 📁 Directory Structure

```
AdaptiveFlightController/
├── src/
│   ├── core/                    # Core framework
│   │   ├── HAL.h               # Hardware Abstraction Layer interface
│   │   ├── TeensyHAL.h         # Teensy implementation of HAL
│   │   ├── Types.h             # Common data types and structures
│   │   └── FlightController.h  # Main coordinator class
│   ├── estimation/              # State estimation modules
│   │   └── StateEstimator.h    # Attitude & airspeed estimation
│   ├── control/                 # Control algorithms
│   │   └── AdaptivePID.h       # 4D adaptive gain scheduling
│   └── mixing/                  # Control surface mixing
│       └── FixedWingMixer.h    # Elevon mixing for fixed-wing
├── test/
│   ├── unit/                   # Unit tests (run on desktop)
│   ├── integration/            # Integration tests
│   └── mocks/                  # Hardware mocks for testing
├── examples/
│   ├── BasicStabilization/     # Simple example
│   └── AdvancedAdaptive/       # Full-featured example
└── AdaptiveFlightController.ino # Minimal Arduino wrapper
```

## 🚀 Quick Start

### 1. Arduino/Teensy Usage (Embedded)

```cpp
#include "src/core/FlightController.h"
#include "src/core/TeensyHAL.h"

// Create hardware abstraction and flight controller
TeensyHAL hal(hw_config);
FlightController flight_controller(&hal, fc_config);

void setup() {
    flight_controller.initialize();
}

void loop() {
    flight_controller.update();  // That's it!
}
```

### 2. Desktop Testing (Development)

```bash
# Install dependencies (Ubuntu/Debian)
sudo apt install cmake catch2 libsdl2-dev lcov

# Build and run tests
mkdir build && cd build
cmake .. -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)
make test

# View code coverage
make coverage
open coverage_report/index.html
```

### 3. Desktop Simulation

```bash
# Build simulation
cmake .. -DBUILD_SIMULATION=ON
make flight_sim
./flight_sim  # Interactive flight simulation
```

## 🧩 Core Modules

### StateEstimator
- **Madgwick attitude filter** with magnetometer fusion
- **Physics-based airspeed estimation** using EKF and thrust-drag dynamics
- **Zone-based drag coefficient learning** with RLS
- **Throttle dithering** for observability

```cpp
StateEstimator estimator(&hal);
estimator.update(dt);
auto attitude = estimator.getAttitude();
auto airspeed = estimator.getAirspeedState();
```

### AdaptivePID  
- **4D gain scheduling** (throttle × AOA × phase × airspeed)
- **Quadrilinear interpolation** between gain schedule points
- **Performance-based adaptive learning** with rollback protection
- **Wind-aware adaptation rate control**

```cpp
AdaptivePID adaptive_pid;
auto gains = adaptive_pid.getGains(flight_regime);
adaptive_pid.updatePerformance(errors, outputs, dt);
```

### FixedWingMixer
- **Elevon mixing** (elevator + aileron combination)
- **Airspeed-dependent control authority scaling**
- **Automatic rudder coordination** for coordinated flight
- **Wind compensation integration**

```cpp
FixedWingMixer mixer(&hal);
auto controls = mixer.mix(pid_outputs, radio_inputs, airspeed, wind_comp);
mixer.applyControls(controls);
```

### FlightController
- **Top-level coordinator** that orchestrates all modules
- **Safety monitoring** and emergency handling
- **Performance statistics** and health monitoring
- **Configuration management** and persistence

## 🧪 Testing Philosophy

### Unit Tests (Desktop)
Every algorithm can be tested in isolation without hardware:

```cpp
TEST_CASE("4D Gain Interpolation") {
    FlightRegime regime{0.5f, 0.0f, 15.0f, FlightPhase::CRUISE};
    auto gains = AdaptivePID::interpolateGains(regime, test_table, config);
    
    REQUIRE(gains.roll.kp == Approx(0.1f).epsilon(0.01f));
}
```

### Integration Tests  
Test module interactions with mocked hardware:

```cpp
MockHAL mock_hal;
mock_hal.setIMUData(synthetic_gyro, synthetic_accel, synthetic_mag);
StateEstimator estimator(&mock_hal);
// Verify attitude estimation with known inputs
```

### Hardware-in-Loop
Test complete system with real hardware but controlled inputs.

## 🔧 Advanced Features

### Intelligent Wind Disturbance Rejection
- **Gust vs steady wind separation** using slow low-pass filtering
- **Adaptive learning rates** based on wind conditions  
- **Feedforward wind compensation** with confidence tracking
- **Learning freeze during severe gusts** for stability

### Physics-Based Airspeed Estimation
- **Extended Kalman Filter** for airspeed and drag coefficient estimation
- **Thrust calibration** from throttle using quadratic lookup table
- **Zone-based drag learning** with Recursive Least Squares per AOA zone
- **Throttle dithering** for observability in steady cruise

### Production-Grade Adaptive Control
- **4D gain scheduling grid** with smooth quadrilinear interpolation
- **Advanced performance metrics**: overshoot, settling time, oscillation frequency
- **State-aware learning** with observing/adapting/frozen/rollback states
- **Graceful rollback** when performance degrades

## 📊 Comparison: Before vs After

| Aspect | Before (Monolithic) | After (Modular) |
|--------|-------------------|-----------------|
| **Lines of code** | 5000+ in one file | ~150 in main sketch |
| **Compilation** | 30+ seconds full rebuild | <2 seconds per module |
| **Testing** | Impossible without hardware | Full desktop unit tests |
| **Debugging** | Global variable chaos | Clean module interfaces |
| **Maintenance** | Nightmare to modify | Easy to extend/modify |
| **Reusability** | Locked to one platform | Portable algorithms |

## 🎯 Development Workflow

### 1. Algorithm Development
```bash
# Develop and test algorithms on desktop
mkdir build && cd build
cmake .. -DBUILD_TESTING=ON
make test_adaptive_pid
./test_adaptive_pid  # Instant feedback!
```

### 2. Hardware Integration
```bash
# Flash to Teensy when ready
arduino-cli compile --fqbn teensy:avr:teensy40 AdaptiveFlightController.ino
arduino-cli upload --fqbn teensy:avr:teensy40 AdaptiveFlightController.ino
```

### 3. Continuous Integration
```yaml
# .github/workflows/ci.yml
- name: Run Tests
  run: |
    cmake .. -DBUILD_TESTING=ON
    make -j$(nproc)
    make test
```

## 🛡️ Safety Features

- **Hardware abstraction** prevents direct hardware access
- **Comprehensive input validation** and bounds checking
- **Emergency stop mechanisms** with graceful degradation
- **Radio timeout handling** and failsafe modes
- **Performance monitoring** with automatic rollback
- **Static analysis integration** catches issues at compile time

## 🚁 Aircraft Configuration

### Fixed-Wing Elevon Setup
```cpp
FlightController::Config config = {
    .mixer_config = {
        .use_elevons = true,
        .enable_airspeed_scaling = true,
        .nominal_airspeed = 15.0f,
        .enable_wind_compensation = true
    }
};
```

### Servo Mapping
- **Channel 1**: Left Elevon  
- **Channel 2**: Right Elevon
- **Channel 3**: Rudder
- **Channel 4**: Throttle

## 📈 Performance Monitoring

The system continuously monitors:
- **Loop timing** and CPU usage
- **Control performance** metrics  
- **Hardware health** status
- **Learning progress** and convergence
- **Wind compensation** effectiveness

## 🎓 Learning Resources

### For Beginners
1. Start with `examples/BasicStabilization/`
2. Understand the HAL concept
3. Explore individual modules

### For Advanced Users  
1. Study the 4D gain scheduling algorithm
2. Implement custom state estimators
3. Add new control modes
4. Contribute to the test suite

## 🤝 Contributing

1. **Fork** the repository
2. **Create tests** for new features
3. **Ensure all tests pass** before submitting
4. **Follow the modular architecture** principles
5. **Submit a pull request** with clear description

## 📜 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🙏 Acknowledgments

- **Original dRehmFlight** project for the foundational flight controller
- **Madgwick** for the excellent attitude estimation algorithm  
- **Community feedback** that led to this architectural improvement

---

**The result**: A flight controller that is simultaneously more advanced AND easier to work with. The complex adaptive algorithms are all still there - they're just properly organized! 🚁✨

