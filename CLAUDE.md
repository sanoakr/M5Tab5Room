# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

M5Tab5Room is a C++ application built for the M5Stack Tab5 device that uses the Mooncake application framework with LVGL for UI. The project supports both ESP32 hardware (M5Tab5) and desktop simulation environments.

## Commands

### Development Setup
```bash
# Fetch dependencies first
python ./fetch_repos.py
```

### Desktop Build
```bash
# Tool chains (Ubuntu/Debian)
sudo apt install build-essential cmake libsdl2-dev

# Build
mkdir build && cd build
cmake .. && make -j8

# Run
./desktop/app_desktop_build
```

### ESP-IDF Build (Tab5 Hardware)
```bash
# Navigate to platform directory
cd platforms/tab5

# Build for ESP32-P4
idf.py build

# Flash to device
idf.py flash
```

## Architecture

### Core Layers

1. **Application Layer (`app/`)** - Main application logic using Mooncake framework
2. **Hardware Abstraction Layer (`app/hal/`)** - Platform-specific implementations (Desktop/ESP32)
3. **Shared Data Layer (`app/shared/`)** - Global state management with signal system
4. **Platform Layer (`platforms/`)** - Platform-specific main functions and configurations

### Key Components

- **Mooncake Framework**: Application lifecycle and ability management
- **LVGL**: Graphics library for UI components
- **HAL System**: Abstraction for platform differences (display, audio, power, etc.)
- **Signal System**: Event communication using smooth_ui_toolkit signals
- **App Installer**: Dynamic application loading system

### Application Structure

Apps are organized as independent modules in `app/apps/`:
- `app_launcher/` - Main launcher with hardware test panels
- `app_startup_anim/` - Boot animation
- `app_template/` - Template for new apps
- `utils/` - Shared utilities (audio, UI, math)

Each app follows Mooncake's ability pattern with initialization, update, and cleanup phases.

### Dependencies

External dependencies are managed via `repos.json` and fetched into `dependencies/`:
- **lvgl**: Graphics library (v9.2.2)
- **mooncake**: Application framework (v2.1.0)
- **mooncake_log**: Logging system (v1.0.0)
- **smooth_ui_toolkit**: UI utilities and signals (v2.0.0)

### Platform Differences

The HAL system abstracts platform differences:
- **Desktop**: Uses SDL2 for display/input simulation
- **ESP32**: Direct hardware control for M5Tab5 features (power management, IMU, camera, audio, etc.)

### Configuration

- `lv_conf.h`: LVGL configuration
- `platforms/tab5/sdkconfig.defaults`: ESP-IDF configuration for ESP32-P4
- CMakeLists.txt files: Build configuration per platform

### Japanese Font Support

The project includes Japanese font support with M+ fonts at multiple sizes (32pt, 64pt, 128pt) for internationalization.