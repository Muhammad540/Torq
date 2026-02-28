add_executable(so101 src/main.cpp)
target_link_libraries(so101 PRIVATE openmanip)
target_compile_definitions(so101 PRIVATE PROJECT_ROOT="${CMAKE_SOURCE_DIR}")
