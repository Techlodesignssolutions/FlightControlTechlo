#!/bin/bash

# FlightGear Flight Controller Build Script
# This script builds the complete FlightGear integration for your flight controller

set -e  # Exit on any error

echo "=== FlightGear Flight Controller Build Script ==="
echo "Building FlightGear integration for dRehm flight controller"
echo ""

# Check dependencies
echo "Checking dependencies..."

# Check for required packages
MISSING_DEPS=""

if ! pkg-config --exists sdl2; then
    MISSING_DEPS="$MISSING_DEPS libsdl2-dev"
fi

if ! command -v cmake &> /dev/null; then
    MISSING_DEPS="$MISSING_DEPS cmake"
fi

if ! command -v g++ &> /dev/null; then
    MISSING_DEPS="$MISSING_DEPS build-essential"
fi

if [ ! -z "$MISSING_DEPS" ]; then
    echo "Missing dependencies: $MISSING_DEPS"
    echo "Please install them with:"
    echo "sudo apt-get update && sudo apt-get install$MISSING_DEPS"
    exit 1
fi

echo "✓ All dependencies found"

# Create build directory
echo "Creating build directory..."
mkdir -p build
cd build

# Configure with CMake
echo "Configuring build with CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build the project
echo "Building project..."
make -j$(nproc)

echo ""
echo "=== Build Complete ==="
echo "Executable created: ./build/flightgear_interface"
echo ""

# Create fixed XML protocol files
echo "Creating FlightGear protocol files..."

cat > ../output_protocol_fixed.xml << 'EOF'
<PropertyList>
  <generic>
    <output>
      <line_separator>newline</line_separator>
      <var_separator>tab</var_separator>
      
      <chunk>
        <name>roll-deg</name>
        <node>/orientation/roll-deg</node>
        <type>float</type>
        <format>%.6f</format>
      </chunk>
      
      <chunk>
        <name>pitch-deg</name>
        <node>/orientation/pitch-deg</node>
        <type>float</type>
        <format>%.6f</format>
      </chunk>
      
      <chunk>
        <name>heading-deg</name>
        <node>/orientation/heading-deg</node>
        <type>float</type>
        <format>%.6f</format>
      </chunk>
      
      <chunk>
        <name>roll-rate-degps</name>
        <node>/orientation/roll-rate-degps</node>
        <type>float</type>
        <format>%.6f</format>
      </chunk>
      
      <chunk>
        <name>pitch-rate-degps</name>
        <node>/orientation/pitch-rate-degps</node>
        <type>float</type>
        <format>%.6f</format>
      </chunk>
      
      <chunk>
        <name>yaw-rate-degps</name>
        <node>/orientation/yaw-rate-degps</node>
        <type>float</type>
        <format>%.6f</format>
      </chunk>
      
      <chunk>
        <name>x-accel-fps_sec</name>
        <node>/accelerations/pilot/x-accel-fps_sec</node>
        <type>float</type>
        <format>%.6f</format>
      </chunk>
      
      <chunk>
        <name>y-accel-fps_sec</name>
        <node>/accelerations/pilot/y-accel-fps_sec</node>
        <type>float</type>
        <format>%.6f</format>
      </chunk>
      
      <chunk>
        <name>z-accel-fps_sec</name>
        <node>/accelerations/pilot/z-accel-fps_sec</node>
        <type>float</type>
        <format>%.6f</format>
      </chunk>
      
    </output>
  </generic>
</PropertyList>
EOF

cat > ../input_protocol_fixed.xml << 'EOF'
<PropertyList>
  <generic>
    <input>
      <line_separator>newline</line_separator>
      <var_separator>tab</var_separator>
      
      <chunk>
        <name>aileron</name>
        <node>/controls/flight/aileron</node>
        <type>float</type>
        <format>%.6f</format>
      </chunk>
      
      <chunk>
        <name>elevator</name>
        <node>/controls/flight/elevator</node>
        <type>float</type>
        <format>%.6f</format>
      </chunk>
      
      <chunk>
        <name>rudder</name>
        <node>/controls/flight/rudder</node>
        <type>float</type>
        <format>%.6f</format>
      </chunk>
      
      <chunk>
        <name>throttle</name>
        <node>/controls/engines/engine[0]/throttle</node>
        <type>float</type>
        <format>%.6f</format>
      </chunk>
      
    </input>
  </generic>
</PropertyList>
EOF

echo "✓ Protocol files created:"
echo "  - output_protocol_fixed.xml"
echo "  - input_protocol_fixed.xml"

# Create startup scripts
echo ""
echo "Creating startup scripts..."

cat > ../start_flightgear.sh << 'EOF'
#!/bin/bash

# FlightGear startup script with UDP communication
# Run this script to start FlightGear with the flight controller interface

AIRCRAFT="c172p"  # Change this to your preferred aircraft
AIRPORT="KSFO"    # Change this to your preferred airport

echo "Starting FlightGear with UDP interface..."
echo "Aircraft: $AIRCRAFT"
echo "Airport: $AIRPORT"
echo ""

fgfs \
  --aircraft=$AIRCRAFT \
  --airport=$AIRPORT \
  --generic=socket,out,40,127.0.0.1,5000,udp,output_protocol_fixed.xml \
  --generic=socket,in,45,127.0.0.1,5010,udp,input_protocol_fixed.xml \
  --disable-freeze \
  --enable-clouds3d \
  --prop:/sim/rendering/shaders/quality-level=0 \
  --prop:/sim/sound/enabled=false
EOF

cat > ../start_autopilot.sh << 'EOF'
#!/bin/bash

# Flight controller startup script
# Run this AFTER starting FlightGear

echo "Starting flight controller interface..."
echo "Make sure FlightGear is already running!"
echo ""

# Change to the build directory and run the interface
cd build
./flightgear_interface
EOF

chmod +x ../start_flightgear.sh
chmod +x ../start_autopilot.sh

echo "✓ Startup scripts created:"
echo "  - start_flightgear.sh"
echo "  - start_autopilot.sh"

echo ""
echo "=== Setup Complete! ==="
echo ""
echo "To run the system:"
echo "1. Start FlightGear:      ./start_flightgear.sh"
echo "2. Start flight controller: ./start_autopilot.sh (in another terminal)"
echo ""
echo "The flight controller will connect to FlightGear via UDP and provide"
echo "attitude stabilization using your existing dRehm flight controller code."
echo ""
echo "Controls:"
echo "- Connect a USB joystick for manual control"
echo "- Or use keyboard controls in FlightGear for autopilot setpoints"
echo "" 