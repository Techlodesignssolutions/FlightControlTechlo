# FlightGear Integration on Windows

Complete guide for setting up the FlightGear flight controller integration on Windows.

## Quick Start for Windows

### 1. Install Prerequisites

#### **FlightGear Flight Simulator**
Download and install from: https://www.flightgear.org/download/
- Choose the Windows installer
- Make sure it's added to your PATH during installation

#### **C++ Build Tools**
Choose ONE of these options:

**Option A: Visual Studio Community (Recommended)**
- Download from: https://visualstudio.microsoft.com/
- During installation, select "Desktop development with C++"
- This includes CMake and the compiler

**Option B: Build Tools for Visual Studio**
- Download "Build Tools for Visual Studio" 
- Select "C++ build tools" workload

#### **CMake** (if not included with Visual Studio)
- Download from: https://cmake.org/download/
- **Important:** Check "Add CMake to system PATH" during installation

#### **SDL2** (Optional - for joystick support)
- Download from: https://github.com/libsdl-org/SDL/releases
- Extract to `C:\SDL2\` or let CMake download it automatically

### 2. Build the System

Open **Command Prompt** or **PowerShell** in your project directory:

```cmd
# Run the Windows build script
build_flightgear.bat
```

**That's it!** The script will:
- Check for all dependencies
- Download SDL2 if needed  
- Build the complete system
- Create startup scripts

### 3. Run the Integration

#### **Method 1: Using Scripts (Easy)**
**Terminal 1 - Start FlightGear:**
```cmd
start_flightgear.bat
```

**Terminal 2 - Start Flight Controller:**
```cmd
start_autopilot.bat
```

#### **Method 2: Manual Commands (Recommended)**
**Terminal 1 - Start FlightGear with Native FDM:**
```cmd
"C:\Program Files\FlightGear 2024.1\bin\fgfs.exe" --native-fdm=socket,out,10,127.0.0.1,5000,udp --native-ctrls=socket,in,10,127.0.0.1,5010,udp
```

**Terminal 2 - Start Flight Controller:**
```cmd
cd "C:\css\N8N-Data\Jarvis Agent\FlightControlTechlo"
flightgear_interface_final.exe
```

#### **Method 3: Developer Command Prompt (For Building)**
If you need to recompile:
```cmd
# Open "Developer Command Prompt for VS 2022"
cd "C:\css\N8N-Data\Jarvis Agent\FlightControlTechlo"
cl /std:c++17 /EHsc flightgear_interface.cpp src/core/FlightGearHAL.cpp src/core/FlightController.cpp src/estimation/StateEstimator.cpp src/control/AdaptivePID.cpp src/mixing/FixedWingMixer.cpp /Fe:flightgear_interface_final.exe /I. ws2_32.lib
```

### **Successful Output**
When working correctly, you should see:
```
=== FlightGear Flight Controller Interface ===
UDP sockets initialized:
  Input:  Port 5000 (receive from FlightGear)
  Output: Port 5010 (send to FlightGear)

SUCCESS: Parsed Native FDM binary format
Roll=0.00° Pitch=0.00° Heading=0.00°
Rates: -6.18°/s -0.00°/s 0.00°/s
Loop 200: Roll=0.0° Pitch=0.0° → Ail=0.000 Elev=0.355 Thr=0.002
```

## Manual Build (Alternative)

If the automated script doesn't work:

```cmd
# Create build directory
mkdir build
cd build

# Configure with CMake
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build . --config Release
```

## Windows-Specific Features

### **Native Windows Networking**
- Uses Winsock2 for UDP communication
- Automatic WSA initialization/cleanup
- No Linux dependencies required

### **Windows Threading**
- Uses Windows threads instead of pthreads
- Proper signal handling with SetConsoleCtrlHandler
- Compatible with Windows 10/11

### **SDL2 Auto-Detection**
- Automatically finds SDL2 in common locations
- Works with both installed and portable SDL2
- Falls back to manual paths if needed

## Dependencies Explained

### **Required:**
- **FlightGear** - The flight simulator
- **CMake** - Build system  
- **C++ Compiler** - Visual Studio or MinGW

### **Optional:**
- **SDL2** - For joystick/gamepad support
- **Git** - For development (if cloning repository)

## Troubleshooting Windows Issues

### **Build Issues**

**"CMake not found"**
```cmd
# Check if CMake is in PATH
cmake --version

# If not found, reinstall CMake with PATH option checked
```

**"No C++ compiler found"**
```cmd
# Check for Visual Studio compiler
where cl

# Check for MinGW compiler  
where gcc

# Install Visual Studio with C++ tools if neither found
```

**"SDL2 not found"**
```cmd
# SDL2 is optional - system will work without joystick support
# To install manually:
# 1. Download SDL2 development libraries
# 2. Extract to C:\SDL2\
# 3. Re-run build_flightgear.bat
```

### **Runtime Issues**

**"FlightGear failed to start"**
```cmd
# Check if FlightGear is installed
"C:\Program Files\FlightGear 2024.1\bin\fgfs.exe" --version

# If not found, reinstall FlightGear and add to PATH
```

**"Failed to parse IMU data"**
```cmd
# Make sure you're using the Native FDM protocol, NOT generic protocols
# Correct command:
"C:\Program Files\FlightGear 2024.1\bin\fgfs.exe" --native-fdm=socket,out,10,127.0.0.1,5000,udp --native-ctrls=socket,in,10,127.0.0.1,5010,udp

# Wrong (will cause parsing errors):
fgfs --generic=socket,out,10,127.0.0.1,5000,udp,output_protocol
```

**"No IMU data received"**
```cmd
# Check if both programs are listening on correct ports
netstat -an | findstr :5000
netstat -an | findstr :5010

# FlightGear should send TO port 5000
# Flight controller should send TO port 5010
```

**"WSAStartup failed"**
- Restart command prompt as Administrator
- Check Windows firewall settings
- Ensure Windows Sockets service is running

**"No joysticks detected"**
- This is normal if no controller is connected
- System works fine without joystick (autopilot mode)
- Connect Xbox/PlayStation controller if desired

### **UDP Communication Issues**

**Windows Firewall:**
```cmd
# Allow UDP ports through firewall
netsh advfirewall firewall add rule name="FlightGear UDP In" dir=in action=allow protocol=UDP localport=5000
netsh advfirewall firewall add rule name="FlightGear UDP Out" dir=out action=allow protocol=UDP localport=5010
```

**Port Already in Use:**
```cmd
# Check what's using the ports
netstat -an | findstr :5000
netstat -an | findstr :5010

# Kill conflicting processes if found
```

## Performance Tips

### **For Better Performance:**
```cmd
# Run FlightGear with reduced graphics
start_flightgear.bat
# (Script already includes performance optimizations)
```

### **Lower CPU Usage:**
- Close unnecessary programs
- Use Task Manager to set FlightGear to "High" priority
- Consider reducing FlightGear's graphics settings

## Directory Structure

After building, your directory will look like:

```
FlightControlTechlo/
├── build/
│   └── Release/
│       └── flightgear_interface.exe    ← Main executable
├── output_protocol.xml                 ← FlightGear input protocol
├── input_protocol.xml                  ← FlightGear output protocol  
├── start_flightgear.bat               ← FlightGear launcher
├── start_autopilot.bat                ← Flight controller launcher
└── build_flightgear.bat               ← Build script
```

## Advanced Configuration

### **Change Aircraft:**
Edit `start_flightgear.bat`:
```batch
set AIRCRAFT=j3cub
REM Options: c172p, j3cub, f16, 737-800, etc.
```

### **Change Airport:**
Edit `start_flightgear.bat`:
```batch
set AIRPORT=KORD
REM Use any ICAO airport code
```

### **Joystick Mapping:**
Controllers are automatically detected. Axis mapping:
- **Left stick X** → Aileron (roll)
- **Left stick Y** → Elevator (pitch)  
- **Right stick X** → Rudder (yaw)
- **Right stick Y** → Throttle

## Development on Windows

### **Visual Studio Integration:**
```cmd
# Generate Visual Studio project files
cmake .. -G "Visual Studio 16 2019"

# Open in Visual Studio
start FlightControlTechlo.sln
```

### **Debugging:**
- Set `flightgear_interface` as startup project
- Set working directory to project root
- Start FlightGear first, then debug the interface

### **Adding Features:**
- Edit C++ files in `src/` directory
- Rebuild with `cmake --build . --config Release`
- Your flight controller code remains unchanged

## Windows vs Linux

**What's Different:**
- Uses `.bat` files instead of `.sh` scripts
- Uses Winsock instead of Berkeley sockets
- Uses Windows threads instead of pthreads

**What's the Same:**
- All flight controller code unchanged
- Same UDP protocol with FlightGear
- Same control interfaces and functionality

The flight controller behavior is identical between Windows and Linux!

## Need Help?

**Common Solutions:**
1. **Try the automated build script first** - `build_flightgear.bat`
2. **Install Visual Studio with C++ tools** - most build issues are compiler-related
3. **Run as Administrator** - if you get permission errors
4. **Check Windows Defender** - may block networking initially
5. **Restart after installing tools** - PATH changes need fresh terminal

**Still having issues?**
- Check that FlightGear works independently first
- Verify your C++ compiler with: `cl` or `gcc --version`
- Make sure CMake is in PATH: `cmake --version`
- Test UDP connectivity with Windows tools 