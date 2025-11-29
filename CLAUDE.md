# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

PX4 is a professional-grade open-source autopilot for drones and other autonomous vehicles. It runs on NuttX RTOS for flight controllers and POSIX (Linux/macOS) for simulation. The codebase uses C++14 with CMake build system and supports 100+ hardware boards.

## Build Commands

Build syntax: `make [VENDOR_MODEL_VARIANT] [target]`

**Common SITL builds:**
```bash
wsl bash -lc 'cd /mnt/d/code/px4/PX4-Autopilot && make st_nucleo-h743zi-fc_default -j$(nproc)'
不要自动构建，需要构建时提示用户手动构建 反馈给claude日志，因为claude 自动构建等待时间非常长，可能超时也影响客户继续使用clude code
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

**Advanced build options:**
```bash
# Sanitizer builds (SITL only, for debugging memory/threading issues)
PX4_ASAN=1 make px4_sitl_default   # Address Sanitizer (memory errors)
PX4_MSAN=1 make px4_sitl_default   # Memory Sanitizer (uninitialized reads)
PX4_TSAN=1 make px4_sitl_default   # Thread Sanitizer (data races)
PX4_UBSAN=1 make px4_sitl_default  # Undefined Behavior Sanitizer

# Build type override
PX4_CMAKE_BUILD_TYPE=Debug make px4_sitl_default
PX4_CMAKE_BUILD_TYPE=Release make px4_sitl_default

# Parallel build control
make px4_sitl_default -j8          # Force 8 parallel jobs

# Verbose build output
VERBOSE=1 make px4_sitl_default    # Show detailed build commands

# Replay mode (for log replay debugging)
replay=1 make px4_sitl_default     # Build with replay support
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
- Windows: Supported via MSYS2/Cygwin (uses MSYS Makefiles when Ninja unavailable)

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

## Code Modification Guidelines

**IMPORTANT: Modify existing files rather than creating new ones**
- Fix bugs directly in the original files, do not create separate "fix" files
- Refactor existing code rather than creating duplicate replacement files
- If a file becomes too large or complex, use design patterns to split it appropriately
- Maintain the existing architecture and file structure whenever possible

## Custom Development (This Fork)

This fork focuses on **Nucleo-H743ZI flight controller development** with extensive Chinese documentation.

### Custom Board: ST Nucleo-H743ZI-FC

**Board configuration**: `boards/st/nucleo-h743zi-fc/default.px4board`

Key differences from standard PX4:
- **8MHz HSE crystal** (not 25MHz) - requires PLL reconfiguration
- **Custom module**: `dual_imu_fusion` for multi-IMU sensor fusion
- **Minimal configuration**: Core modules only (no EKF2, no navigator, no control by default)
- **Enabled modules**: dataman, logger, mavlink, sensors, dual_imu_fusion
- **Sensors**: ICM42688P IMU, BMM150 magnetometer

**Build commands**:
```bash
make st_nucleo-h743zi-fc_default           # Build for Nucleo board
make st_nucleo-h743zi-fc_default upload    # Flash to board
make list_config_targets | grep nucleo     # List all Nucleo targets
```

### Extensive Technical Documentation

**Location**: `.trae/documents/` (29 technical documents, mostly in Chinese)

**Quick Start**: Read `.trae/documents/QUICKSTART.md` for rapid onboarding (2-4 hours)

**Master Index**: `.trae/documents/INDEX.md` provides complete navigation

**Key documentation areas**:
- `build/build_system_complete_guide.md` - Why PX4 doesn't need CubeMX, CMake workflow
- `build/nucleo_h743zi_step_by_step.md` - Step-by-step Nucleo board bring-up
- `build/nuttx_stm32h7_driver_support.md` - NuttX driver support analysis (answers "Do I need to write drivers?")
- `通用基础系统/stm32h743_minimal_flight_controller_guide.md` - Complete STM32H743 porting guide (26k+ tokens)
- `rtos/` - NuttX RTOS integration and optimization
- `algorithms/` - EKF2, controllers, TECS algorithms deep dive
- `drivers/nuttx_driver_development.md` - Driver development guide (2884 lines)

**Learning paths** (see INDEX.md for details):
1. **Quick Start**: QUICKSTART.md → core concepts → SITL practice
2. **Hardware Porting**: build_system_complete_guide.md → nucleo_h743zi_step_by_step.md → driver development
3. **Algorithm Development**: EKF2 → controllers → TECS
4. **System Integration**: uORB → MAVLink → ROS 2/DDS

### Custom Modules

**Dual IMU Fusion** (`src/modules/dual_imu_fusion/`)
- Custom sensor fusion module for multiple IMU redundancy
- Not present in upstream PX4
- Configured via `CONFIG_MODULES_DUAL_IMU_FUSION=y` in board config

### Development Workflow for Custom Board

1. **Initial setup**:
   ```bash
   git clone --recursive [this-repo]
   make submodulesupdate
   ```

2. **Modify board configuration**:
   - Edit `boards/st/nucleo-h743zi-fc/default.px4board`
   - Enable/disable modules via `CONFIG_MODULES_*` flags
   - Configure drivers via `CONFIG_DRIVERS_*` flags

3. **Test build**:
   ```bash
   make st_nucleo-h743zi-fc_default
   ```

4. **Flash and verify**:
   ```bash
   make st_nucleo-h743zi-fc_default upload
   # Connect via serial console (typically /dev/ttyACM0 or COM port)
   # Verify with MAVLink console in QGroundControl
   ```

5. **Debug via serial**:
   - UART configuration: Typically 115200 baud, 8N1
   - Use `dmesg`, `top`, `uorb top` in NuttShell (nsh>)
   - MAVLink console available via `CONFIG_MODULES_MAVLINK=y`

### Board-Specific Notes

**Clock Configuration**:
- HSE: 8MHz (Nucleo board limitation)
- System clock: 480MHz (via PLL)
- NuttX defconfig handles clock tree (no CubeMX needed)
- Clock configuration in NuttX: `platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/`

**Pin Mapping**:
- See `build/nucleo_h743zi_pinmap.md` for complete pinout
- Custom pin assignments in board-specific files
- USART2 (ST-Link): Default debug console
- SPI/I2C: Configured via NuttX board.h

**NuttX Integration**:
- NuttX submodule: `platforms/nuttx/NuttX/nuttx`
- Board defconfig: Auto-generated from board configuration
- No manual HAL initialization required (PX4 build system handles this)
