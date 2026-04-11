add_executable(so101 src/main.cpp)
target_link_libraries(so101 PRIVATE torq)
target_compile_definitions(so101 PRIVATE PROJECT_ROOT="${CMAKE_SOURCE_DIR}")

add_executable(calibrate src/robot_calibration.cpp)
target_link_libraries(calibrate PRIVATE torq)
target_compile_definitions(calibrate PRIVATE PROJECT_ROOT="${CMAKE_SOURCE_DIR}")
