@echo off
echo === Restarting FlightGear with Native FDM ===
echo.
echo Make sure FlightGear is completely closed first!
echo.

"C:\Program Files\FlightGear 2024.1\bin\fgfs.exe" --aircraft=ufo --airport=KSFO --native-fdm=socket,out,120,127.0.0.1,5000,udp --native-ctrls=socket,in,120,127.0.0.1,5010,udp --prop:/sim/time/pause=false --prop:/sim/freeze/fuel=false --prop:/sim/freeze/clock=false --disable-freeze

echo.
echo FlightGear started with:
echo - Native FDM output to port 5000 (120 Hz)
echo - Native Controls input on port 5010 (120 Hz) 
echo - UFO aircraft (more responsive)
echo - Anti-freeze properties enabled
echo. 