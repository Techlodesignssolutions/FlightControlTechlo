@echo off
echo === FlightGear Flight Controller Build Script (Windows) ===
echo Building FlightGear integration for dRehm flight controller
echo.

REM Check for required tools
echo Checking dependencies...

where cmake >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: CMake not found. Please install CMake from https://cmake.org/
    echo Make sure to add it to your PATH during installation
    pause
    exit /b 1
)

where cl >nul 2>nul || where gcc >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: No C++ compiler found. Please install:
    echo - Visual Studio 2019/2022 with C++ tools, OR
    echo - MinGW-w64, OR  
    echo - Build Tools for Visual Studio
    pause
    exit /b 1
)

echo ✓ CMake found
echo ✓ C++ compiler found

REM Check for SDL2 (optional - will be downloaded if needed)
echo Note: SDL2 will be downloaded automatically if not found

REM Create build directory
echo Creating build directory...
if not exist build mkdir build
cd build

REM Configure with CMake
echo Configuring build with CMake...
cmake .. -DCMAKE_BUILD_TYPE=Release

if %ERRORLEVEL% NEQ 0 (
    echo ERROR: CMake configuration failed
    echo Make sure you have Visual Studio or MinGW installed
    pause
    exit /b 1
)

REM Build the project
echo Building project...
cmake --build . --config Release

if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Build failed
    pause
    exit /b 1
)

echo.
echo === Build Complete ===
echo Executable created: .\build\Release\flightgear_interface.exe
echo.

REM Create FlightGear protocol files
echo Creating FlightGear protocol files...

echo ^<PropertyList^> > ..\output_protocol.xml
echo   ^<generic^> >> ..\output_protocol.xml
echo     ^<output^> >> ..\output_protocol.xml
echo       ^<line_separator^>newline^</line_separator^> >> ..\output_protocol.xml
echo       ^<var_separator^>tab^</var_separator^> >> ..\output_protocol.xml
echo       ^<chunk^> >> ..\output_protocol.xml
echo         ^<name^>roll-deg^</name^> >> ..\output_protocol.xml
echo         ^<node^>/orientation/roll-deg^</node^> >> ..\output_protocol.xml
echo         ^<type^>float^</type^> >> ..\output_protocol.xml
echo         ^<format^>%%.6f^</format^> >> ..\output_protocol.xml
echo       ^</chunk^> >> ..\output_protocol.xml
echo       ^<chunk^> >> ..\output_protocol.xml
echo         ^<name^>pitch-deg^</name^> >> ..\output_protocol.xml
echo         ^<node^>/orientation/pitch-deg^</node^> >> ..\output_protocol.xml
echo         ^<type^>float^</type^> >> ..\output_protocol.xml
echo         ^<format^>%%.6f^</format^> >> ..\output_protocol.xml
echo       ^</chunk^> >> ..\output_protocol.xml
echo       ^<chunk^> >> ..\output_protocol.xml
echo         ^<name^>heading-deg^</name^> >> ..\output_protocol.xml
echo         ^<node^>/orientation/heading-deg^</node^> >> ..\output_protocol.xml
echo         ^<type^>float^</type^> >> ..\output_protocol.xml
echo         ^<format^>%%.6f^</format^> >> ..\output_protocol.xml
echo       ^</chunk^> >> ..\output_protocol.xml
echo       ^<chunk^> >> ..\output_protocol.xml
echo         ^<name^>roll-rate-degps^</name^> >> ..\output_protocol.xml
echo         ^<node^>/orientation/roll-rate-degps^</node^> >> ..\output_protocol.xml
echo         ^<type^>float^</type^> >> ..\output_protocol.xml
echo         ^<format^>%%.6f^</format^> >> ..\output_protocol.xml
echo       ^</chunk^> >> ..\output_protocol.xml
echo       ^<chunk^> >> ..\output_protocol.xml
echo         ^<name^>pitch-rate-degps^</name^> >> ..\output_protocol.xml
echo         ^<node^>/orientation/pitch-rate-degps^</node^> >> ..\output_protocol.xml
echo         ^<type^>float^</type^> >> ..\output_protocol.xml
echo         ^<format^>%%.6f^</format^> >> ..\output_protocol.xml
echo       ^</chunk^> >> ..\output_protocol.xml
echo       ^<chunk^> >> ..\output_protocol.xml
echo         ^<name^>yaw-rate-degps^</name^> >> ..\output_protocol.xml
echo         ^<node^>/orientation/yaw-rate-degps^</node^> >> ..\output_protocol.xml
echo         ^<type^>float^</type^> >> ..\output_protocol.xml
echo         ^<format^>%%.6f^</format^> >> ..\output_protocol.xml
echo       ^</chunk^> >> ..\output_protocol.xml
echo       ^<chunk^> >> ..\output_protocol.xml
echo         ^<name^>x-accel-fps_sec^</name^> >> ..\output_protocol.xml
echo         ^<node^>/accelerations/pilot/x-accel-fps_sec^</node^> >> ..\output_protocol.xml
echo         ^<type^>float^</type^> >> ..\output_protocol.xml
echo         ^<format^>%%.6f^</format^> >> ..\output_protocol.xml
echo       ^</chunk^> >> ..\output_protocol.xml
echo       ^<chunk^> >> ..\output_protocol.xml
echo         ^<name^>y-accel-fps_sec^</name^> >> ..\output_protocol.xml
echo         ^<node^>/accelerations/pilot/y-accel-fps_sec^</node^> >> ..\output_protocol.xml
echo         ^<type^>float^</type^> >> ..\output_protocol.xml
echo         ^<format^>%%.6f^</format^> >> ..\output_protocol.xml
echo       ^</chunk^> >> ..\output_protocol.xml
echo       ^<chunk^> >> ..\output_protocol.xml
echo         ^<name^>z-accel-fps_sec^</name^> >> ..\output_protocol.xml
echo         ^<node^>/accelerations/pilot/z-accel-fps_sec^</node^> >> ..\output_protocol.xml
echo         ^<type^>float^</type^> >> ..\output_protocol.xml
echo         ^<format^>%%.6f^</format^> >> ..\output_protocol.xml
echo       ^</chunk^> >> ..\output_protocol.xml
echo     ^</output^> >> ..\output_protocol.xml
echo   ^</generic^> >> ..\output_protocol.xml
echo ^</PropertyList^> >> ..\output_protocol.xml

echo ^<PropertyList^> > ..\input_protocol.xml
echo   ^<generic^> >> ..\input_protocol.xml
echo     ^<input^> >> ..\input_protocol.xml
echo       ^<line_separator^>newline^</line_separator^> >> ..\input_protocol.xml
echo       ^<var_separator^>tab^</var_separator^> >> ..\input_protocol.xml
echo       ^<chunk^> >> ..\input_protocol.xml
echo         ^<name^>aileron^</name^> >> ..\input_protocol.xml
echo         ^<node^>/controls/flight/aileron^</node^> >> ..\input_protocol.xml
echo         ^<type^>float^</type^> >> ..\input_protocol.xml
echo         ^<format^>%%.6f^</format^> >> ..\input_protocol.xml
echo       ^</chunk^> >> ..\input_protocol.xml
echo       ^<chunk^> >> ..\input_protocol.xml
echo         ^<name^>elevator^</name^> >> ..\input_protocol.xml
echo         ^<node^>/controls/flight/elevator^</node^> >> ..\input_protocol.xml
echo         ^<type^>float^</type^> >> ..\input_protocol.xml
echo         ^<format^>%%.6f^</format^> >> ..\input_protocol.xml
echo       ^</chunk^> >> ..\input_protocol.xml
echo       ^<chunk^> >> ..\input_protocol.xml
echo         ^<name^>rudder^</name^> >> ..\input_protocol.xml
echo         ^<node^>/controls/flight/rudder^</node^> >> ..\input_protocol.xml
echo         ^<type^>float^</type^> >> ..\input_protocol.xml
echo         ^<format^>%%.6f^</format^> >> ..\input_protocol.xml
echo       ^</chunk^> >> ..\input_protocol.xml
echo       ^<chunk^> >> ..\input_protocol.xml
echo         ^<name^>throttle^</name^> >> ..\input_protocol.xml
echo         ^<node^>/controls/engines/engine[0]/throttle^</node^> >> ..\input_protocol.xml
echo         ^<type^>float^</type^> >> ..\input_protocol.xml
echo         ^<format^>%%.6f^</format^> >> ..\input_protocol.xml
echo       ^</chunk^> >> ..\input_protocol.xml
echo     ^</input^> >> ..\input_protocol.xml
echo   ^</generic^> >> ..\input_protocol.xml
echo ^</PropertyList^> >> ..\input_protocol.xml

echo ✓ Protocol files created:
echo   - output_protocol.xml
echo   - input_protocol.xml

REM Create startup scripts
echo.
echo Creating startup scripts...

echo @echo off > ..\start_flightgear.bat
echo REM FlightGear startup script with UDP communication >> ..\start_flightgear.bat
echo REM Run this script to start FlightGear with the flight controller interface >> ..\start_flightgear.bat
echo. >> ..\start_flightgear.bat
echo set AIRCRAFT=c172p >> ..\start_flightgear.bat
echo set AIRPORT=KSFO >> ..\start_flightgear.bat
echo. >> ..\start_flightgear.bat
echo echo Starting FlightGear with UDP interface... >> ..\start_flightgear.bat
echo echo Aircraft: %%AIRCRAFT%% >> ..\start_flightgear.bat
echo echo Airport: %%AIRPORT%% >> ..\start_flightgear.bat
echo echo. >> ..\start_flightgear.bat
echo. >> ..\start_flightgear.bat
echo fgfs --aircraft=%%AIRCRAFT%% --airport=%%AIRPORT%% --generic=socket,out,40,127.0.0.1,5000,udp,output_protocol.xml --generic=socket,in,45,127.0.0.1,5010,udp,input_protocol.xml --disable-freeze --enable-clouds3d --prop:/sim/rendering/shaders/quality-level=0 --prop:/sim/sound/enabled=false >> ..\start_flightgear.bat
echo if %%ERRORLEVEL%% NEQ 0 ^( >> ..\start_flightgear.bat
echo     echo ERROR: FlightGear failed to start >> ..\start_flightgear.bat
echo     echo Make sure FlightGear is installed and in your PATH >> ..\start_flightgear.bat
echo     pause >> ..\start_flightgear.bat
echo ^) >> ..\start_flightgear.bat

echo @echo off > ..\start_autopilot.bat
echo REM Flight controller startup script >> ..\start_autopilot.bat
echo REM Run this AFTER starting FlightGear >> ..\start_autopilot.bat
echo. >> ..\start_autopilot.bat
echo echo Starting flight controller interface... >> ..\start_autopilot.bat
echo echo Make sure FlightGear is already running! >> ..\start_autopilot.bat
echo echo. >> ..\start_autopilot.bat
echo. >> ..\start_autopilot.bat
echo REM Change to the build directory and run the interface >> ..\start_autopilot.bat
echo cd build\Release >> ..\start_autopilot.bat
echo flightgear_interface.exe >> ..\start_autopilot.bat
echo if %%ERRORLEVEL%% NEQ 0 ^( >> ..\start_autopilot.bat
echo     echo ERROR: Flight controller interface failed >> ..\start_autopilot.bat
echo     echo Make sure the build completed successfully >> ..\start_autopilot.bat
echo     pause >> ..\start_autopilot.bat
echo ^) >> ..\start_autopilot.bat

echo ✓ Startup scripts created:
echo   - start_flightgear.bat
echo   - start_autopilot.bat

echo.
echo === Setup Complete! ===
echo.
echo To run the system:
echo 1. Start FlightGear:      start_flightgear.bat
echo 2. Start flight controller: start_autopilot.bat (in another terminal)
echo.
echo The flight controller will connect to FlightGear via UDP and provide
echo attitude stabilization using your existing dRehm flight controller code.
echo.
echo Controls:
echo - Connect a USB joystick for manual control
echo - Or use keyboard controls in FlightGear for autopilot setpoints
echo.
pause 