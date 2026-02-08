cmake_minimum_required(VERSION 3.22)
project(openManip
        VERSION 0.0
        LANGUAGES CXX C
        DESCRIPTION "Open Source framework for Robot Manipulation")

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
    GIT_TAG        main
)
FetchContent_MakeAvailable(mujoco) 

find_package(pinocchio REQUIRED)
message(STATUS "Found Pinocchio version: ${pinocchio_VERSION}")

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG docking
)
FetchContent_MakeAvailable(imgui)
add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui_lib PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)

target_link_libraries(imgui_lib PUBLIC glfw)
set_target_properties(imgui_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

# =======
# Library
# =======
set(CORE_SOURCES
    src/physics/MujocoDriver.cpp
    src/core/RobotSystem.cpp
    src/gui/Gui.cpp
    src/kinematics/PinocchioModel.cpp
    src/control/Controller.cpp
    src/control/InverseKinematics.cpp
)

add_library(openmanip SHARED ${CORE_SOURCES})

target_include_directories(openmanip PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(openmanip PUBLIC
    glfw
    OpenGL::GL
    GLEW::GLEW
    pinocchio::pinocchio
    mujoco::mujoco
    imgui_lib
)

set_target_properties(openmanip PROPERTIES NO_SYSTEM_FROM_IMPORTED ON)
# ===========
# Application
# ===========
add_subdirectory(workspace/so101)

message(STATUS "--- Configuration Done ---")