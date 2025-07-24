cmake_minimum_required(VERSION 3.16)
project(FlightControlTechlo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Platform-specific configuration
if(WIN32)
    # Windows configuration
    set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "C:/Program Files/SDL2")
    
    # Find SDL2 on Windows
    find_package(SDL2 QUIET)
    if(NOT SDL2_FOUND)
        # Try to find SDL2 manually
        find_path(SDL2_INCLUDE_DIRS SDL.h PATHS "C:/Program Files/SDL2/include" "C:/SDL2/include")
        find_library(SDL2_LIBRARIES SDL2 PATHS "C:/Program Files/SDL2/lib" "C:/SDL2/lib")
        find_library(SDL2_MAIN_LIBRARIES SDL2main PATHS "C:/Program Files/SDL2/lib" "C:/SDL2/lib")
    endif()
    
    # Add Windows-specific libraries
    set(PLATFORM_LIBS ws2_32)
else()
    # Linux configuration
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(SDL2 REQUIRED sdl2)
    set(PLATFORM_LIBS pthread)
endif()

# Include directories
include_directories(${CMAKE_SOURCE_DIR})
if(SDL2_INCLUDE_DIRS)
    include_directories(${SDL2_INCLUDE_DIRS})
endif()

# Core flight controller library
add_library(flight_controller_core
    src/core/FlightController.cpp
    src/core/FlightGearHAL.cpp
    src/control/AdaptivePID.cpp
    src/estimation/StateEstimator.cpp
    src/mixing/FixedWingMixer.cpp
)

# Link libraries based on platform
if(WIN32)
    if(SDL2_LIBRARIES)
        target_link_libraries(flight_controller_core ${SDL2_LIBRARIES})
        target_compile_definitions(flight_controller_core PRIVATE HAVE_SDL2)
    endif()
else()
    target_link_libraries(flight_controller_core ${SDL2_LIBRARIES})
    target_compile_options(flight_controller_core PRIVATE ${SDL2_CFLAGS_OTHER})
    target_compile_definitions(flight_controller_core PRIVATE HAVE_SDL2)
endif()

# FlightGear interface executable
add_executable(flightgear_interface
    flightgear_interface.cpp
)

# Link platform-specific libraries
if(WIN32)
    target_link_libraries(flightgear_interface 
        flight_controller_core
        ${PLATFORM_LIBS}
    )
    if(SDL2_LIBRARIES)
        target_link_libraries(flightgear_interface ${SDL2_LIBRARIES})
        target_compile_definitions(flightgear_interface PRIVATE HAVE_SDL2)
        message(STATUS "SDL2 support enabled for flightgear_interface")
    endif()
else()
    target_link_libraries(flightgear_interface 
        flight_controller_core
        ${PLATFORM_LIBS}
        ${SDL2_LIBRARIES}
    )
    target_compile_options(flightgear_interface PRIVATE ${SDL2_CFLAGS_OTHER})
    target_compile_definitions(flightgear_interface PRIVATE HAVE_SDL2)
endif()

# Test executables (if testing framework is available and requested)
option(BUILD_TESTING "Build the testing tree" OFF)
if(BUILD_TESTING AND EXISTS "${CMAKE_SOURCE_DIR}/test")
    enable_testing()
    add_subdirectory(test)
endif()

# Install targets
install(TARGETS flightgear_interface DESTINATION bin)
install(FILES output_protocol.xml input_protocol.xml DESTINATION share/flightgear)

# Print configuration summary
message(STATUS "Build Configuration:")
message(STATUS "  Platform: ${CMAKE_SYSTEM_NAME}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
if(WIN32)
    message(STATUS "  SDL2 Include: ${SDL2_INCLUDE_DIRS}")
    message(STATUS "  SDL2 Libraries: ${SDL2_LIBRARIES}")
    message(STATUS "  Platform Libraries: ${PLATFORM_LIBS}")
else()
    message(STATUS "  SDL2 Found: ${SDL2_FOUND}")
    message(STATUS "  SDL2 Libraries: ${SDL2_LIBRARIES}")
endif()
message(STATUS "  Install Prefix: ${CMAKE_INSTALL_PREFIX}") 