# Tell CMake where to find your MockHAL.h
include_directories(${CMAKE_SOURCE_DIR}/test)

# mixer-only test
add_executable(test_mixer_only unit/test_mixer.cpp)
target_link_libraries(test_mixer_only
    PRIVATE
        Catch2::Catch2
        fc_lib
)
add_test(NAME mixer_only COMMAND $<TARGET_FILE:test_mixer_only>)

# adaptive-pid-only test
add_executable(test_adaptive_pid_only unit/test_adaptive_pid.cpp)
target_link_libraries(test_adaptive_pid_only
    PRIVATE
        Catch2::Catch2
        fc_lib
)
add_test(NAME adaptive_pid_only COMMAND $<TARGET_FILE:test_adaptive_pid_only>)

# state-estimator-only test
add_executable(test_state_estimator_only unit/test_state_estimator.cpp)
target_link_libraries(test_state_estimator_only
    PRIVATE
        Catch2::Catch2
        fc_lib
)
add_test(NAME state_estimator_only COMMAND $<TARGET_FILE:test_state_estimator_only>)

# flight-controller integration test
add_executable(test_flight_controller_only integration/test_flight_controller.cpp)
target_link_libraries(test_flight_controller_only
    PRIVATE
        Catch2::Catch2
        fc_lib
)
add_test(NAME flight_controller_only COMMAND $<TARGET_FILE:test_flight_controller_only>) 