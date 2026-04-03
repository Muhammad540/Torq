cmake_minimum_required(VERSION 3.22)
project(torq
        VERSION 0.1
        LANGUAGES CXX C
        DESCRIPTION "High-performance C++ framework for robot control and training")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ENABLE_TRACKING_POINTS "Enable tracking points feature" OFF)
if (ENABLE_TRACKING_POINTS)
    add_compile_definitions(ENABLE_TRACKING_POINTS)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

# ============
# Dependencies
# ============
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

include(FetchContent)

set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    mujoco
    GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
    GIT_TAG        3.5.0
)
FetchContent_MakeAvailable(mujoco) 

find_package(pinocchio REQUIRED)
message(STATUS "Found Pinocchio version: ${pinocchio_VERSION}")

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG docking
)

FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG master
)

find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

FetchContent_MakeAvailable(imgui implot)
add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)

target_include_directories(imgui_lib PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${implot_SOURCE_DIR}
)

target_link_libraries(imgui_lib PUBLIC glfw)
set_target_properties(imgui_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

# =======
# SCServo (ST/STS/SMS protocol)
# =======
set(SCSERVO_SOURCES
    src/hardware/scservo/SCS.cpp
    src/hardware/scservo/SCSPort.cpp
    src/hardware/scservo/SMS_STS_Port.cpp
)

# =======
# Library
# =======
set(CORE_SOURCES
    src/hardware/MujocoDriver.cpp
    src/hardware/ServoDriver.cpp
    ${SCSERVO_SOURCES}
    src/core/RobotSystem.cpp
    src/gui/Gui.cpp
    src/kinematics/PinocchioModel.cpp
    src/tasks/Tasks.cpp
    src/limits/Limits.cpp
    src/barriers/Barriers.cpp
    src/control/Controller.cpp
    src/control/InverseKinematics.cpp
)

add_library(torq SHARED ${CORE_SOURCES})

target_include_directories(torq PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/torq/scservo>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(torq PUBLIC
  glfw
  OpenGL::GL
  GLEW::GLEW
  pinocchio::pinocchio
  mujoco::mujoco
  imgui_lib
  OsqpEigen::OsqpEigen
)

set_target_properties(torq PROPERTIES NO_SYSTEM_FROM_IMPORTED ON)
# ===========
# Application
# ===========
add_subdirectory(workspace/so101)
add_subdirectory(workspace/panda)
add_subdirectory(workspace/arm_ur5e)

message(STATUS "--- Configuration Done ---")
