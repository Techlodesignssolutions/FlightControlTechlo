cmake_minimum_required(VERSION 3.16)

# Project setup
project(FlightController 
    VERSION 1.0.0
    DESCRIPTION "Advanced Flight Controller with Adaptive PID"
    LANGUAGES CXX
)

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the main flight controller library
add_library(fc_lib
    src/core/FlightController.cpp
    src/control/AdaptivePID.cpp
    src/estimation/StateEstimator.cpp
    src/mixing/FixedWingMixer.cpp
)

# Set up detailed include directories for the library
target_include_directories(fc_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control
    ${CMAKE_CURRENT_SOURCE_DIR}/src/estimation
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mixing
)

# Enable testing
enable_testing()

# Fetch & make Catch2 available
include(FetchContent)
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v2.13.10   # Use stable v2.x branch for compatibility
)
FetchContent_MakeAvailable(Catch2)

# Now pull in your test folder
if(BUILD_TESTING)
    add_subdirectory(test)
endif()

# Print summary
message(STATUS "FlightController ${PROJECT_VERSION}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Testing enabled: ${BUILD_TESTING}") 