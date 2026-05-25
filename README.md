# CARLA AEB Autonomous Driving Simulation

## Project Overview

Autonomous driving simulation project based on CARLA 0.9.13, featuring lateral control (Stanley), longitudinal control (PID), and AEB (Automatic Emergency Braking) system validation.

## Tech Stack

- **Simulation**: CARLA 0.9.13
- **Language**: Python 3.7 + C++14
- **Lateral Control**: Stanley Controller (K=0.3, lookahead 8m)
- **Longitudinal Control**: PID (Python simple_pid + C++ extension, 1.7x speedup)
- **Safety System**: AEB 4-state FSM (Normal -> Warning -> Partial Brake -> Full Brake)
- **AI Integration**: ZhipuAI JSON scene -> CARLA auto-configuration
- **Testing**: pytest (54 test cases: 35 AEB + 19 C++ PID)

## Project Structure

```
carla-aeb-autonomous-driving/
+-- controllers/          # Control algorithms
|   +-- stanley_pid_controller.py   # Stanley + PID + AEB + HUD
|   +-- pid_cpp_bridge.py           # C++ PID Python wrapper
|   +-- pid_cpp/                    # C++ PID source
+-- scenarios/            # Test scenarios
|   +-- aeb_validation_test.py      # AEB CCRb + Cut-in validation
|   +-- aeb_ccrb_headless.py        # AEB headless mode
|   +-- run_aeb_ccrb.py             # AEB subprocess wrapper
|   +-- run_kml_demo.py             # KML mountain road driving
+-- tests/                # Unit tests
|   +-- test_aeb.py                 # AEB tests (35/35)
|   +-- test_pid_cpp.py             # C++ PID tests (19/19)
+-- ai_integration/       # AI scene generation
|   +-- carla_ai_scene.py           # ZhipuAI JSON -> CARLA config
|   +-- smart_record.py             # Smart recording with retry
|   +-- auto_record_demo.py         # AI scene demo recording
+-- tools/                # Utility scripts
|   +-- generate_charts.py          # Data visualization
|   +-- screw_shell.kml             # Mountain road KML trajectory (142 points)
+-- docs/                 # Documentation & images
+-- output/               # Runtime output (gitignored)
```

## Quick Start

### Prerequisites

- CARLA 0.9.13 (precompiled)
- Python 3.7
- C++ compiler (MSVC for C++ PID extension)

### Setup

```bash
pip install -r requirements.txt
```

### Build C++ PID Extension (optional)

```bash
cd controllers/pid_cpp
python setup_pid_cpp.py build_ext --inplace
```

### Run AEB Validation Test

```bash
# Start CARLA server first, then run the test
python scenarios/aeb_validation_test.py
```

## Core Features

### Stanley Lateral Controller
- Cross-track error based steering
- Dynamic lookahead distance (8m)
- Steering smoothing (alpha=0.4)

### PID Longitudinal Controller
- Python `simple_pid` and C++ dual implementation
- C++ extension provides 1.7x speedup
- Output clipping [-1.0, 1.0]

### AEB State Machine
- 4 states: Normal -> Warning -> Partial Brake -> Full Brake
- One-way lock (can only escalate, not de-escalate)
- TTC + distance dual protection
- Distance emergency protection (<10m forces full brake)

### Self-Monitoring and Adaptive Repair
- 7 monitoring items: collision, drift, stagnation, AEB failure, pygame, vehicle spawn, target behavior
- Quality adaptive degradation: Epic -> Medium -> Low
- Automatic parameter adjustment with up to 3 retries

## Testing

```bash
cd tests
pytest -v
```

Expected: 54 test cases (35 AEB + 19 C++ PID)

## Demo

### AEB CCRb (Front Vehicle Emergency Brake)
![aeb_ccrb_demo](docs/images/aeb_ccrb_demo.gif)

### AEB Cut-in
![aeb_cutin_demo](docs/images/aeb_cutin_demo.gif)

### KML Mountain Road Driving
![kml_demo](docs/images/kml_demo.gif)

## TODO

- [ ] Add unit tests for Stanley controller
- [ ] Refactor hardcoded paths to environment variables
- [ ] Add CI/CD pipeline
- [ ] Add Docker support
- [ ] Document AEB threshold tuning methodology

## License

MIT License
