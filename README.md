# RCCAR — Remote control car (PlatformIO)

Small PlatformIO project for a remote-control car firmware and logging utilities.

## Overview

This repository contains firmware and helper tools for a remote control car prototype. The firmware is implemented with PlatformIO and PlatformIO-friendly toolchain. It includes IMU support, PID control, SD logging, and a small data parsing utility.

⚠️ **Use at your own risk.** This is experimental firmware for a prototype vehicle.

## Quick Start

Requirements:
- PlatformIO (VS Code extension or PlatformIO Core)
- A compatible ESP/MCU board configured in `platformio.ini`

Build and upload (from project root):

```
platformio run
platformio run -t upload
```

Open serial monitor:

```
platformio device monitor
```

## Project structure

- [src](src): firmware source files, main entry is [src/main.cpp](src/main.cpp)
- [include](include): public headers (IMU, PID, modules)
- [lib](lib): additional libraries and submodules
- [data](data): helper scripts and logs (e.g., `parse_log.py`)
- [test](test): test notes
- `platformio.ini`: PlatformIO project configuration
- `large_storage_16MB.csv`: example data file

## Key files

- [include/IMU_ICM20948.h](include/IMU_ICM20948.h) — IMU driver header
- [include/RC_PID.h](include/RC_PID.h) — PID controller header
- [src/RCModule.cpp](src/RCModule.cpp) — RC control implementation
- [src/SDLogger.cpp](src/SDLogger.cpp) — SD logging implementation

## Development notes

- Edit configuration in `platformio.ini` to match your board and upload settings.
- Use the PlatformIO VS Code extension for convenient build/upload/monitor workflows.

## Data parsing

Use the Python script in [data/parse_log.py](data/parse_log.py) to post-process CSV logs.

## License

MIT License — see LICENSE file for details.
