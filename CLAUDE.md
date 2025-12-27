# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an OpenCV-based 5G autonomous vehicle system designed for real-time lane detection, obstacle avoidance, cone detection, garage parking, and guided navigation. The system is built for small autonomous vehicles/robots with Raspberry Pi deployment capabilities.

**Technology Stack:**
- C++20 with OpenCV 4.x
- CMake 4.0+ build system
- spdlog for logging
- nlohmann/json for configuration (header-only, `src/json.hpp`)
- ONNX Runtime 1.23.2 for YOLOv5 inference
- CLI11 for command-line argument parsing
- Windows development environment (cross-compiling for Raspberry Pi)

## Build Commands

```bash
# Build the project (Windows)
mkdir build && cd build
cmake ..
cmake --build .

# Run specific executables
.\trace.exe          # Image preprocessing and visualization tool
.\ab_test.exe        # A/B letter detection test
.\test_yolo_infer.exe  # YOLO inference test

# Trace tool usage examples
.\trace.exe -i path/to/image.jpg     # Process single image
.\trace.exe -v path/to/video.mp4     # Process video file
.\trace.exe -c 0                      # Use camera device 0
.\trace.exe                           # Default: camera 0
```

## Architecture

The system is organized into multiple executable targets defined in CMakeLists.txt:

### Current Executables

1. **trace** (`src/trace.cpp`)
   - Image preprocessing and visualization tool for algorithm development
   - Supports multiple preprocessing modes (top-hat+OTSU, adaptive threshold, HSV threshold)
   - Real-time HSV parameter tuning with GUI trackbars
   - Input sources: camera, video file, or static image
   - Command-line interface using CLI11
   - Windows UTF-8 console support for Chinese characters

2. **ab_test** (`src/car/ab_test.cpp`, `src/car/ABDetector.cpp`)
   - A/B letter detection on blue backgrounds
   - Multiple detection methods: hole counting, template matching, projection analysis
   - HSV-based color segmentation for blue background and white letters
   - Perspective correction and ROI warping

3. **test_yolo_infer** (`src/car/test_yolo.cpp`, `src/car/yolo_detector.cpp`)
   - YOLOv5 object detection using ONNX Runtime
   - 320x320 input resolution, 2 classes default
   - Configurable confidence and NMS thresholds
   - Anchor-based detection with 3 detection layers

### Core Detection Modules (in `src/car/`)

- **YoloDetector** (`yolo_detector.hpp/cpp`): ONNX Runtime inference wrapper for YOLOv5
- **ABDetector** (`ABDetector.hpp/cpp`): Multi-method A/B letter recognition
- **ArrowDetector** (`ArrowDetector.hpp/cpp`): Arrow direction detection
- **A4PaperExtractor** (`A4PaperExtractor.hpp/cpp`): Document extraction and perspective correction

### Legacy/Planned Components (headers exist but not in current build)

- **Cone Detection System** (`src/cone_detector.hpp`): HSV-based cone tracking
- **Garage Parking** (referenced in config): State machine for parking maneuvers
- **Guided Navigation** (referenced in config): Path following system

### Configuration System

All parameters are managed through `config/config.json` with hierarchical structure:
- **PID parameters**: kp, ki, kd, output limits, error/output offsets
- **Servo and gimbal**: center positions, max offsets, PWM control values
- **Vision parameters**:
  - `garage`: Line detection (Canny, Hough), HSV thresholds for yellow, close timing
  - `cone_detection`: Area filters, tracking thresholds, left/right turn state machines
  - `guided`: (placeholder for future guided navigation)
- **Motor**: Target speed settings
- **Global**: wait_time, default video path

Configuration is loaded via `Config::load_config()` in `src/config.hpp` (header-only namespace).

## Development Workflow

### Testing with Video Files
The `img/` directory contains test videos for different scenarios:
- **Garage parking**: `garage_hd_left.mp4`, `garage_hd_right.mp4`
- **Guided navigation**: `guided_full.mp4`, `guided_left.mp4`, `guided_right.mp4`
- **Left/right turns**: `left.mp4`, `left - Trim.mp4`, `right.mp4`, `right - Trim.mp4`
- **Camera recordings**: `camera_record1.mp4`, `camera_record2.mp4`
- **Zebra crossing**: `zerba.mp4`
- **Base scenarios**: `base.mp4`, `base2.mp4`

Use the `trace` tool with `-v` flag to test preprocessing algorithms on these videos.

### Real-time Parameter Adjustment
The `trace` executable provides interactive tuning:
- Press keys 0-3 to switch preprocessing modes
- Adjust HSV thresholds via GUI trackbars in "HSV Tuning" window
- Visual feedback with multiple debug windows
- Supports live camera feed, video playback, or static images

### Dataset Structure
The `dataset/turnLR/` directory contains labeled training data:
- `labels/`: YOLO format annotation files (.txt)
- Images organized by timestamp naming convention

## Key Development Patterns

- **Header-only utilities**: `config.hpp`, `json.hpp` (nlohmann/json), `CLI11.hpp`
- **Namespace-based organization**: `Config`, `YoloInfer`, detector classes use PascalCase
- **Configuration-driven development**: Modify `config.json` instead of hardcoding values
- **State machine implementations**: Cone detection uses state-based turn logic
- **Platform abstraction**: `#ifdef _WIN32` guards for Windows-specific code (UTF-8 console setup)
- **ONNX Runtime integration**: Model path in `lib/onnxruntime-win-x64-1.23.2/`

## Important Notes

- **Windows development environment** with Raspberry Pi deployment target (cross-compilation not shown in current CMakeLists.txt)
- Real-time performance requirements for vision loops
- Multiple preprocessing modes available for testing (`preprocessMode` global in `trace.cpp`)
- ONNX models expected in project root or specified path
- Video paths in `config.json` use forward slashes: `"../img/left - Trim.mp4"`
- Console output uses spdlog for structured logging
- HSV color spaces used throughout for robust color detection