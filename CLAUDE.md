# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an OpenCV-based 5G autonomous vehicle system designed for real-time lane detection, obstacle avoidance, cone detection, garage parking, and guided navigation. The system is built for small autonomous vehicles/robots with Raspberry Pi deployment capabilities.

**Technology Stack:**
- C++20 with OpenCV 4.x
- CMake build system
- spdlog for logging
- nlohmann/json for configuration
- pigpio for Raspberry Pi GPIO control

## Build Commands

```bash
# Build the project
mkdir build && cd build
cmake ..
cmake --build .

# Run specific executables
./opencv_5g        # Main cone detection with GUI controls
./line_detection  # Lane detection with obstacle avoidance
./guided         # Guided navigation system
./trace          # Tracing/path following system
./demo           # Hardware control demo
```

## Architecture

The system is organized into multiple executable targets, each handling different autonomous driving scenarios:

### Core Components

1. **Cone Detection System** (`src/cone_detector.hpp`)
   - HSV-based red and yellow cone detection
   - Multi-object tracking with distance-based matching
   - Real-time path interpolation and extrapolation
   - GUI controls for HSV parameter adjustment

2. **Lane Detection** (`src/line_detection.cpp`)
   - Edge detection with Canny algorithm
   - Line fitting and filtering
   - Obstacle detection and avoidance logic
   - Zebra crosswalk detection
   - Weighted error calculation for steering

3. **Garage Parking** (`src/garage.hpp`)
   - State machine implementation (SEARCHING → APPROACHING → PARKED)
   - Horizontal/vertical line detection
   - Sliding window filtering for stable detection

4. **Hardware Control** (`src/demo.cpp`, `src/trace.cpp`)
   - PWM motor control (pin 13)
   - Servo control for steering (pin 12)
   - Gimbal control for camera positioning (pins 22-23)
   - PID control loops

### Configuration System

All parameters are managed through `config/config.json` with hierarchical structure:
- PID parameters (kp, ki, kd, output limits)
- Servo and gimbal configurations
- Vision parameters (HSV thresholds, detection settings)
- Motor speed and PWM settings

## Development Workflow

### Testing with Video Files
The project includes multiple test videos in the `img/` directory for different scenarios:
- Left/right turn navigation
- Garage parking sequences
- Guided navigation paths

### Real-time Parameter Adjustment
Use the main `opencv_5g` executable for real-time HSV parameter tuning:
- GUI sliders for threshold adjustment
- Visual feedback on detection quality
- Debug visualization windows

### Hardware-in-the-Loop Testing
1. Test vision algorithms with video files first
2. Use `demo.cpp` for hardware validation
3. Deploy complete system on Raspberry Pi target

## Key Development Patterns

- **Namespace-based organization** (ConeDetector, Garage, Guided)
- **Configuration-driven development** - modify `config.json` instead of hardcoding
- **State machine implementations** for complex behaviors
- **Inline functions** for performance-critical code
- **Header-only implementations** where appropriate

## Important Notes

- This is a Windows development environment with Raspberry Pi deployment target
- Real-time performance requirements - avoid blocking operations in vision loops
- Hardware abstraction layer allows testing without physical hardware
- Multiple executables serve different autonomous driving scenarios
- GUI controls are available for computer vision parameter tuning