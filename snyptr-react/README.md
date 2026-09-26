# SNYPTR — 2-Metre Moving Target Training System

> **Military-Style Moving Target Training System Prototype**  
> Designed for range instructors and Ustaads to conduct precision target drills along a controlled 0.0 m – 2.0 m linear rail.

---

## Overview

SNYPTR is a serious military-grade training system prototype. The physical rig comprises:
- **Target**: 150 mm × 150 mm square target face.
- **Linear Rail**: 0.0 m to 2.0 m total travel range moving towards and away from the shooter.
- **Pop-Up Mechanism**: Actuated via MG90S servo with a 5.0-second auto-drop exposure timer.
- **Detection Hardware (Target Architecture)**:
  - **Camera**: OV5647 for laser point detection.
  - **Processor**: ESP32-P4 Pico M for computer vision.
  - **Wireless Link**: ESP32-S3 for range telemetry.
  - **Rangefinder**: VL53L0X Time-of-Flight (TOF) sensor for continuous distance.

This software prototype provides a high-fidelity **Simulation Engine** where clicks on the target represent laser strikes, coupled with clean hardware abstraction layers (`TargetController`, `ShotDetectionProvider`) ready for physical hardware integration.

---

## Primary Controls

| Command | Key | Description |
| :--- | :---: | :--- |
| **RUN TARGET** | `R` | Starts target travel along the 0.0 m – 2.0 m rail at 0.30 m/s. |
| **STOP TARGET** | `S` | Immediately halts target movement at current position. |
| **POP TARGET** | `0` | Raises target with an automatic 5.0-second exposure window before folding. |
| **EMERGENCY STOP** | `ESC` | Safety halt immediately stopping motor drive and simulation. |

---

## Key Features

- **Realistic Military Target**: 150 × 150 mm target paper with concentric scoring zones (Zone 1 to 4), central bullseye, and calibrated high-value Optimal Area.
- **5-Second Exposure Auto-Drop**: Dynamic countdown bar displayed while raised; auto-drops when time expires.
- **20-Shot Qualification Session**:
  - Continuous telemetry: progression (`SHOT 07 / 20`), hits, misses, accuracy percentage.
  - Live millimeter distance from shooter attached to every shot.
  - Instant hit/miss callout with deviation off-center and directional vector.
- **Comprehensive Session Report**:
  - Automatically compiles after shot 20.
  - 20-shot target dispersion map with visual legend.
  - Performance breakdown across distance brackets (`0.0–0.5 m`, `0.5–1.0 m`, `1.0–1.5 m`, `1.5–2.0 m`).
  - Miss clustering and directional bias diagnostics.
  - Evidence-based training observations and focus areas.
  - JSON session export.
- **Modular Hardware Abstraction**:
  - `TargetController` (`SimulatedTargetController`, `ESP32TargetController`).
  - `ShotDetectionProvider` (`SimulationShotDetectionProvider`, `ESP32CameraShotDetectionProvider`).

---

## Tech Stack

- **Framework**: React 18 + TypeScript
- **Bundler**: Vite
- **Styling**: Tailored military defense aesthetic (Tactical Slate & Sand canvas)

---

## Getting Started

```bash
# Clone the repository
git clone <your-repo-url>
cd snyptr

# Install dependencies
npm install

# Start development server
npm run dev

# Build for production
npm run build

# Preview production build
npm run preview
```

---

## License

Internal / Proprietary Military Training Prototype.
