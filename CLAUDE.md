# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

PX4 is a professional-grade open-source autopilot for drones and other autonomous vehicles. It runs on NuttX RTOS for flight controllers and POSIX (Linux/macOS) for simulation. The codebase uses C++14 with CMake build system and supports 100+ hardware boards.

## Build Commands

Build syntax: `make [VENDOR_MODEL_VARIANT] [target]`

**Common SITL builds:**
```bash
make px4_sitl_default              # Build SITL (default target)
make px4_sitl gz_x500              # Build and launch Gazebo with X500 quadrotor
make px4_sitl gz_standard_vtol     # VTOL simulation
make px4_sitl gz_rc_cessna         # Fixed-wing simulation
HEADLESS=1 make px4_sitl gz_x500   # Gazebo headless (no GUI)
```

**Hardware builds:**
```bash
make px4_fmu-v6x_default           # Build for Pixhawk 6X
make px4_fmu-v5_default            # Build for Pixhawk 4/FMUv5
make px4_fmu-v6x_default upload    # Build and flash to board via USB
make list_config_targets           # List all available board targets
```

**Testing:**
```bash
make tests                         # Run all unit tests
make tests TESTFILTER=<regex>      # Run filtered tests
make tests_integration             # MAVSDK integration tests
make px4_sitl_test                 # Build test configuration
```

**Code quality:**
```bash
make format                        # Auto-format code with astyle
make check_format                  # Check code formatting
make clang-tidy                    # Run clang-tidy analysis
make shellcheck_all                # Check shell scripts
make quick_check                   # Quick validation (SITL + NuttX + tests + format)
```

**Metadata generation:**
```bash
make parameters_metadata           # Generate parameter documentation
make airframe_metadata             # Generate airframe metadata
make module_documentation          # Generate module documentation
```

**Cleanup:**
```bash
make clean                         # Clean build directory
make distclean                     # Full clean + submodules
make submodulesupdate              # Update git submodules
```

## Architecture

### Core Components

**uORB Message Bus** (`platforms/common/uORB/`, `msg/`)
- Central publish-subscribe IPC mechanism for all inter-module communication
- Asynchronous, thread-safe messaging with shared memory
- Message definitions in `msg/*.msg` (200+ message types)
- Every message MUST include `uint64 timestamp` field
- View topics: `listener <topic>`, `uorb top`

**Source Code Organization:**
- `src/modules/` - 56 flight stack modules (estimators, controllers, navigation)
- `src/lib/` - 40+ shared libraries (math, control, geo transformations)
- `src/drivers/` - Hardware drivers (IMU, GPS, barometer, mag, etc.)
- `src/systemcmds/` - System commands (param, top, logger, etc.)
- `platforms/{nuttx,posix}/` - OS-specific implementations

**Configuration:**
- `boards/VENDOR/MODEL/*.px4board` - Board configurations (249 board files)
- `ROMFS/px4fmu_common/init.d/` - Initialization shell scripts
- Board configs specify which modules/drivers to compile via `CONFIG_MODULES_*` and `CONFIG_DRIVERS_*`

### Key Subsystems

**EKF2 Estimator** (`src/modules/ekf2/`)
- Sensor fusion for attitude/position estimation
- Fuses IMU, GPS, barometer, magnetometer, optical flow, vision
- Publishes: `vehicle_attitude`, `vehicle_local_position`, `estimator_status`
- 50+ tuning parameters (`EKF2_*`)

**Control Pipeline (Multicopter):**
```
Navigator → Position Setpoint → MC Position Control → Attitude Setpoint
→ MC Rate Control → Control Allocator → Actuator Outputs → Motors/ESCs
```
- `mc_pos_control/` - Position controller (NED frame)
- `mc_rate_control/` - Angular rate controller
- `control_allocator/` - Distributes forces to motors
- Similar pipelines exist for fixed-wing (`fw_*`) and VTOL (`vtol_att_control`)

**Flight Mode Manager** (`src/modules/flight_mode_manager/`)
- Arbitrates between flight modes based on RC input or commands
- Modes: Manual, Altitude, Position, Mission, Hold, Return, Acro, Offboard

**Navigator** (`src/modules/navigator/`)
- Mission execution, RTL (return-to-launch), landing, geofencing
- Sub-modules: mission, rtl, land, precland, geofence

**Sensors** (`src/modules/sensors/`)
- Centralized sensor voting and preprocessing
- Sub-modules: vehicle_imu, vehicle_air_data, vehicle_magnetometer, vehicle_gps_position
- Handles sensor selection with multiple IMUs/GPS units

**Commander** (`src/modules/commander/`)
- System state machine, arming logic, mode management
- Failsafe handling

### Boot Sequence

PX4 boots via shell scripts in `ROMFS/px4fmu_common/init.d/`:
1. `rcS` - Main startup script
2. `rc.base_core` - Core modules (uorb, logger, commander)
3. Vehicle-specific scripts (`rc.mc_apps`, `rc.fw_apps`)
4. Sensor initialization (`rc.sensors`)
5. Autostart configuration based on airframe ID

### Module Development Pattern

Modules typically:
1. Inherit from `ModuleBase<>`, `ModuleParams`, `ScheduledWorkItem`
2. Subscribe to uORB topics: `uORB::Subscription<vehicle_attitude_s> _attitude_sub{ORB_ID(vehicle_attitude)};`
3. Publish to uORB topics: `uORB::Publication<vehicle_attitude_setpoint_s> _setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};`
4. Implement `Run()` method for work queue execution
5. Define parameters in `module.yaml` or param files
6. Register via `CMakeLists.txt` and board config

## Code Standards

**Language Standards:**
- C++14 for C++ code (`CMAKE_CXX_STANDARD 14`)
- C11 for C code (`CMAKE_C_STANDARD 11`)

**Formatting:**
- Use `astyle` for code formatting (enforced via `make check_format`)
- Hard indents (tabs) to match PX4 coding style
- Check with `make check_format`, fix with `make format`

**CMake Coding Standard:**
- Common functions in `px_base.cmake`
- OS/board specific functions in `px_impl_${PX4_PLATFORM}.cmake`
- All function arguments are UPPER CASE
- All local variables are lower case
- All cmake functions are lowercase
- Use `px4_parse_function_args` for argument parsing
- Never use macros (use functions instead)
- All config variables must have `config_` prefix

**Logging:**
- Use `PX4_INFO()`, `PX4_WARN()`, `PX4_ERR()` for console logging
- Use `mavlink_log_info()` for user-facing messages

## Common Development Tasks

**Running SITL:**
```bash
make px4_sitl gz_x500
# In PX4 console:
pxh> commander takeoff
pxh> commander land
pxh> shutdown
```

**Adding a new uORB message:**
1. Create `msg/my_message.msg` with timestamp field
2. CMake automatically generates headers in `build/*/uORB/topics/`
3. Include: `#include <uORB/topics/my_message.h>`
4. Publish: `orb_advertise(ORB_ID(my_message), &data)`
5. Subscribe: `orb_subscribe(ORB_ID(my_message))`

**Creating a new module:**
1. Create directory `src/modules/my_module/`
2. Add `CMakeLists.txt`:
   ```cmake
   px4_add_module(
       MODULE modules__my_module
       MAIN my_module
       SRCS
           my_module.cpp
       DEPENDS
           modules__navigator
   )
   ```
3. Add to board config: `CONFIG_MODULES_MY_MODULE=y` in `.px4board` file
4. Add init script entry in `ROMFS/px4fmu_common/init.d/rcS` or `rc.*`

**Testing on SITL before hardware:**
Always test in SITL first with `make px4_sitl gz_x500` before flashing to hardware. This allows for rapid iteration and debugging without risking hardware.

**Parameter System:**
- Define in `module.yaml` or `.c` param files
- Runtime access: `param get/set PARAM_NAME`
- C++ access: Inherit from `ModuleParams`, use `DEFINE_PARAMETERS()` macro

**Performance profiling:**
- Use `perf_counter` for timing measurements
- View real-time performance: `perf top`
- View uORB message rates: `uorb top`

## Important Notes

**Build system:**
- Uses Ninja (if available) or Unix Makefiles
- CMake generator auto-detected
- Build outputs in `build/VENDOR_MODEL_VARIANT/`
- Compile commands database: `build/*/compile_commands.json`

**Multi-platform support:**
- NuttX RTOS for embedded flight controllers (most boards)
- POSIX for SITL and companion computers (Linux/macOS)
- QURT for Snapdragon boards

**External modules:**
- Can be built outside main tree via `EXTERNAL_MODULES_LOCATION`
- Example: `make px4_sitl EXTERNAL_MODULES_LOCATION=/path/to/modules`

**Working with submodules:**
- PX4 uses git submodules extensively
- Update: `make submodulesupdate`
- Clean: `make submodulesclean`

**Board file structure:**
Format: `boards/VENDOR/MODEL/VARIANT.px4board`
- Vendor: ark, px4, holybro, cubepilot, etc.
- Model: fmu-v6x, fmu-v5, sitl, etc.
- Variant: default, bootloader, test, cyphal, etc.

## Documentation

- Developer Guide: https://docs.px4.io/main/en/development/
- User Guide: https://docs.px4.io/main/en/
- Contributing: `CONTRIBUTING.md` (Github flow, branch off main)
- Flight logs: Upload to https://logs.px4.io (Flight Review)

## Typical Workflow

1. **Setup:** Clone with `git clone --recursive`
2. **Build SITL:** `make px4_sitl_default`
3. **Make changes:** Edit code in `src/modules/` or `src/lib/`
4. **Test SITL:** `make px4_sitl gz_x500`
5. **Run tests:** `make tests`
6. **Format code:** `make format`
7. **Build hardware:** `make px4_fmu-v5_default`
8. **Upload:** `make px4_fmu-v5_default upload`
9. **Flight test:** Upload logs to Flight Review

## Debugging

**Console access:**
- SITL: Interactive console in terminal
- Hardware: Connect via USB serial (MAVLink console in QGroundControl)

**Common commands:**
- `listener <topic>` - Monitor uORB topic
- `uorb top` - Real-time topic rates
- `param show` - List all parameters
- `perf` - Performance counters
- `top` - Task list and CPU usage
- `free` - Memory usage
- `dmesg` - Kernel messages (NuttX)

**GDB debugging:**
- SITL: `make px4_sitl_default jmavsim` then attach with GDB
- Hardware: Use debugger probe (SWD/JTAG)

## 重要提示关于修改错误不要新建文件而是在原理的基础上改好

** 不要新建文件而是在原理的基础上改好**
- 禁止在不原有错误文件额基础上新建fix文件
- 禁止fixed文件而是在原有报错的文件上修改
- 错误文件代码过多难以阅读时可以采用设计模式拆分，而不是重复创建功能相同的替代文件
