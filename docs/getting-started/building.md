# Building

## Clone and Build

```bash
git clone <this-repo>
cd OpenManip
mkdir -p build && cd build
cmake ..
cmake --build . -j$(nproc)        # Linux
cmake --build . -j$(sysctl -n hw.ncpu)  # macOS
```

Binaries go to `build/bin/`, shared libraries to `build/lib/`.

## What CMake Does

1. **FetchContent** downloads MuJoCo (from `main`) and ImGui (from `docking` branch).
2. **find_package** locates Pinocchio, OSQP, and OsqpEigen on your system.
3. Builds `imgui_lib` (static) from ImGui sources with GLFW + OpenGL3 backends.
4. Builds `libopenmanip` (shared) from the engine sources and links all dependencies.
5. Builds workspace applications (e.g. `so101`) that link against `libopenmanip`.

## CMake Options

| Option | Description | Default |
|---|---|---|
| `ENABLE_TRACKING_POINTS` | Compile support for drawing tracking points (position, radius, color) in the viewport | `OFF` |

Example:

```bash
cmake -DENABLE_TRACKING_POINTS=ON ..
cmake --build . -j
```

## Incremental Builds

Only modified files recompile — rebuilds are fast:

```bash
cmake --build . -j
```

!!! note
    Changes to model files under `workspace/models/` take effect immediately at runtime without a rebuild.

## Build Targets

| Target | Type | Description |
|---|---|---|
| `openmanip` | Shared library | The core engine (physics, kinematics, control, GUI) |
| `imgui_lib` | Static library | ImGui with GLFW/OpenGL3 backends |
| `so101` | Executable | SO-101 example application |

To build only a specific target:

```bash
cmake --build . --target so101 -j
```

## Troubleshooting

**Pinocchio not found** — Make sure Pinocchio's install prefix is visible to CMake. On macOS with Homebrew this is automatic. On Linux with a custom install:

```bash
cmake -DCMAKE_PREFIX_PATH=/path/to/pinocchio/install ..
```

**OsqpEigen not found** — If you installed to a non-default prefix, pass it explicitly:

```bash
cmake -DCMAKE_PREFIX_PATH="/opt/homebrew;/other/prefix" ..
```

**First build is slow** — MuJoCo is compiled from source via FetchContent on the first run. Subsequent builds only recompile changed files.
