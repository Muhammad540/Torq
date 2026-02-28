# Installation

## Dependencies

| Dependency | Role | Provided by |
|---|---|---|
| CMake ≥ 3.22 | Build system | `brew` / `apt` |
| Eigen3 | Linear algebra | Pulled in by Pinocchio |
| MuJoCo | Physics simulation | Fetched automatically by CMake (FetchContent) |
| Pinocchio ≥ 3.0 | Rigid-body kinematics & dynamics | `brew` / `apt` / ROS |
| OSQP ≥ 1.0 | QP solver backend | `brew` / `apt` |
| OsqpEigen ≥ 0.8 | Eigen wrapper for OSQP | Build from source |
| GLEW | OpenGL extension wrangler | `brew` / `apt` |
| GLFW ≥ 3.3 | Window / input for the GUI | `brew` / `apt` |
| ImGui (docking) | GUI widgets | Fetched automatically by CMake (FetchContent) |

MuJoCo and ImGui are downloaded at configure time via CMake `FetchContent` — you do not need to install them manually.

---

## macOS (Homebrew)

```bash
brew install cmake eigen glew glfw osqp pinocchio pkg-config
```

**OsqpEigen** is not available in Homebrew and must be built from source:

```bash
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen && mkdir build && cd build
cmake .. && cmake --build . -j$(sysctl -n hw.ncpu)
cmake --install . --prefix /opt/homebrew
```

---

## Ubuntu / Debian

```bash
sudo apt update
sudo apt install cmake libeigen3-dev libglew-dev libglfw3-dev pkg-config
```

**Pinocchio** — install via ROS or from source:

```bash
# Option A: ROS package
sudo apt install ros-${ROS_DISTRO}-pinocchio

# Option B: from upstream
# See https://stack-of-tasks.github.io/pinocchio/
```

**OSQP:**

```bash
sudo apt install libosqp-dev
```

**OsqpEigen:**

```bash
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen && mkdir build && cd build
cmake .. && cmake --build . -j$(nproc)
sudo cmake --install .
```

---

## Verifying the Install

After installing everything, confirm CMake can find the key packages:

```bash
mkdir -p build && cd build
cmake ..
```

You should see output similar to:

```
-- Found Pinocchio version: 3.x.x
-- Found osqp
-- Found OsqpEigen
-- Configuration Done
```

If any `find_package` call fails, double-check that the library is installed and its install prefix is on your `CMAKE_PREFIX_PATH`.
