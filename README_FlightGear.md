# FlightGear Integration for dRehm Flight Controller

This integration connects your existing dRehm flight controller code with FlightGear flight simulator for Hardware-in-the-Loop (HITL) testing and development.

## Architecture Overview

```
FlightGear ←UDP→ Interface Program ←→ Your Flight Controller ←→ Control Outputs
   ↑                                         ↑
 Sim Data                              RC/Joystick Input
```

### Components

1. **FlightGear Simulator** - Provides aircraft physics and visualization
2. **UDP Interface Program** (`flightgear_interface`) - Bridges FlightGear and your flight controller
3. **FlightGearHAL** - Hardware abstraction layer that simulates sensors using FlightGear data
4. **Your Flight Controller** - Unmodified dRehm flight controller code
5. **RC Input** - Optional joystick/gamepad for manual control

## Quick Start

### 1. Install Dependencies

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install build-essential cmake libsdl2-dev flightgear

# Or build from the provided script
chmod +x build_flightgear.sh
./build_flightgear.sh
```

### 2. Build the System

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 3. Run the Integration

**Terminal 1 - Start FlightGear:**
```bash
./start_flightgear.sh
```

**Terminal 2 - Start Flight Controller:**
```bash
./start_autopilot.sh
```

## How It Works

### Data Flow

1. **FlightGear → Interface Program**
   - Aircraft attitude (roll, pitch, yaw)
   - Angular rates (roll rate, pitch rate, yaw rate)
   - Accelerations (x, y, z in body frame)

2. **Interface Program → Flight Controller**
   - Converts FlightGear data to simulated IMU readings
   - Adds realistic sensor noise and bias
   - Provides RC inputs from joystick or autopilot commands

3. **Flight Controller → Interface Program**
   - Control surface commands (aileron, elevator, rudder)
   - Throttle commands

4. **Interface Program → FlightGear**
   - Sends control commands to move flight surfaces
   - Updates engine throttle

### Flight Controller Integration

Your existing flight controller code runs completely unchanged. The `FlightGearHAL` class implements the same `HAL` interface as `TeensyHAL`, but instead of reading real sensors, it:

- Simulates IMU data from FlightGear's aircraft state
- Provides RC inputs from joystick or simulated commands
- Outputs control commands to FlightGear instead of real servos/motors

## Configuration

### FlightGear Aircraft

Edit `start_flightgear.sh` to change aircraft:

```bash
AIRCRAFT="c172p"    # Cessna 172 (default)
# AIRCRAFT="f16"    # F-16 Fighter
# AIRCRAFT="j3cub"  # Piper J3 Cub
```

### Flight Controller Settings

The interface creates a `FlightController` with these settings:

```cpp
FlightController::Config fc_config;
fc_config.loop_frequency_hz = 100;  // 100 Hz for simulation
fc_config.default_mode = ControlMode::STABILIZE;
fc_config.enable_debug_output = true;
fc_config.debug_output_rate_hz = 2;  // 2 Hz debug output
```

### Control Modes

The flight controller supports all standard modes:

- **MANUAL** - Direct joystick passthrough
- **STABILIZE** - Attitude stabilization (recommended for testing)
- **ALTITUDE** - Altitude hold (if implemented)
- **POSITION** - Position hold (if implemented)
- **AUTO** - Autonomous waypoint navigation (if implemented)

## RC/Joystick Control

### Supported Controllers

- Xbox/PlayStation controllers
- Flight sim joysticks
- Any USB gamepad recognized by SDL2

### Control Mapping

| Joystick Axis | Flight Control | Range |
|---------------|----------------|-------|
| Axis 0 (Left X) | Aileron/Roll | -1 to +1 |
| Axis 1 (Left Y) | Elevator/Pitch | -1 to +1 |
| Axis 2 (Right Y) | Throttle | 0 to 1 |
| Axis 3 (Right X) | Rudder/Yaw | -1 to +1 |

### Manual Control

1. Connect your controller before starting the interface
2. Use joystick inputs for direct manual control
3. The flight controller will stabilize based on your inputs

### Autopilot Mode

1. Set joystick to neutral position
2. Flight controller will maintain level flight
3. Small joystick movements set attitude targets

## Network Configuration

### UDP Ports

- **Port 5000** - FlightGear → Flight Controller (aircraft state)
- **Port 5010** - Flight Controller → FlightGear (control commands)

### Firewall

Make sure these ports are open:

```bash
# Ubuntu
sudo ufw allow 5000/udp
sudo ufw allow 5010/udp
```

## Troubleshooting

### Common Issues

1. **"No joysticks detected"**
   - This is normal if you don't have a controller
   - System will work in autopilot mode
   - Connect a USB controller for manual control

2. **"Failed to receive from FlightGear"**
   - Make sure FlightGear is running first
   - Check that FlightGear is using the correct UDP ports
   - Verify the protocol XML files are correct

3. **FlightGear crashes on startup**
   - Try a different aircraft: edit `AIRCRAFT` in `start_flightgear.sh`
   - Check FlightGear logs for missing aircraft or scenery

4. **Flight controller doesn't respond**
   - Check that UDP data is being received
   - Look for debug output from the flight controller
   - Verify that IMU data is valid

### Debug Output

The flight controller outputs debug information every 2 seconds:

```
Loop 200: Roll=2.3° Pitch=-1.1° → Ail=-0.042 Elev=0.018 Thr=0.750
```

This shows:
- Current aircraft attitude (roll, pitch from FlightGear)
- Control outputs (aileron, elevator, throttle to FlightGear)

### Performance Tips

1. **Reduce FlightGear graphics** for better performance:
   ```bash
   --prop:/sim/rendering/shaders/quality-level=0
   --prop:/sim/sound/enabled=false
   ```

2. **Run at lower loop rates** if needed:
   ```cpp
   fc_config.loop_frequency_hz = 50;  // Reduce from 100 Hz
   ```

3. **Disable debug output** for maximum performance:
   ```cpp
   fc_config.enable_debug_output = false;
   ```

## No Magnetometer Operation

### What This Means

Without a magnetometer, your flight controller operates like many real-world setups:

**✅ What Still Works Perfectly:**
- **Attitude Stabilization** - Roll and pitch control remain precise
- **Rate Control** - All angular rate control loops work normally  
- **Acrobatic Flight** - Loops, rolls, and complex maneuvers
- **Manual Control** - Full manual flying capability
- **Altitude Hold** - If using barometer/GPS for altitude reference

**⚠️ What Changes:**
- **Heading Drift** - Yaw angle estimate drifts over time (degrees per minute)
- **Navigation Modes** - GPS waypoint navigation needs compass compensation
- **Wind Compensation** - Crosswind estimation may be less accurate

### Practical Impact

**Short Flights (< 5 minutes):**
- **Negligible impact** - heading drift is minimal
- **Perfect for testing** control algorithms and tuning

**Longer Flights:**
- **Manual flying** - pilot controls heading, no issues
- **Stabilization** - still works perfectly for roll/pitch
- **Autopilot** - may need periodic heading correction

### Real-World Equivalent

This setup matches many successful flight controllers:
- **Racing drones** - often run without magnetometer for better performance
- **Simple autopilots** - rely on GPS course-over-ground for heading
- **Aerobatic aircraft** - prioritize attitude control over absolute heading

## Development and Testing

### Adding New Control Modes

1. Implement new modes in your existing flight controller code
2. The FlightGear integration will automatically support them
3. Test in simulation before real hardware

### PID Tuning

1. Use FlightGear to safely test PID parameters
2. Observe aircraft response in real-time
3. The adaptive PID system will learn during simulation

### Sensor Simulation

The FlightGearHAL simulates realistic IMU hardware without magnetometer:

**Sensors Included:**
- **3-Axis Gyroscope** - Angular rates (roll, pitch, yaw rates)
- **3-Axis Accelerometer** - Linear accelerations including gravity

**Realistic Characteristics:**
- **IMU Bias** - Small constant offsets (±0.01 units)
- **Sensor Noise** - Random noise on all measurements  
- **Update Rates** - Realistic timing for sensor data

**No Magnetometer:**
- **Heading drift** - Yaw angle will drift over time without absolute reference
- **Still works great** for stabilization, attitude hold, and most flight modes
- **FlightGear provides ground truth** for comparison and validation

### Custom Aircraft

To test with custom aircraft models:

1. Create or download FlightGear aircraft
2. Update the `AIRCRAFT` variable in `start_flightgear.sh`
3. Adjust control surface mappings in `FlightGearHAL.cpp` if needed

## Integration with Real Hardware

This simulation setup helps you:

1. **Develop algorithms** safely in simulation
2. **Test control modes** before real flights
3. **Tune parameters** without risk
4. **Debug code** with full visibility

When ready for real hardware:

1. Use `TeensyHAL` instead of `FlightGearHAL`
2. Your flight controller code remains unchanged
3. Parameters tuned in simulation transfer to hardware

## Support

For issues with this integration:

1. Check FlightGear documentation for aircraft-specific issues
2. Verify your joystick works with other programs
3. Test with the default C172 aircraft first
4. Check system logs for UDP communication errors

The integration preserves all your existing flight controller functionality while providing a safe, realistic simulation environment for development and testing. 