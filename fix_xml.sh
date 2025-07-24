#!/bin/bash

# Fix XML protocol files for FlightGear
echo "Creating proper FlightGear protocol files..."

# Create output protocol file
cat > output_protocol.xml << 'EOF'
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

# Create input protocol file
cat > input_protocol.xml << 'EOF'
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

echo "✓ Created output_protocol.xml"
echo "✓ Created input_protocol.xml"
echo ""
echo "XML protocol files are now ready for FlightGear!" 