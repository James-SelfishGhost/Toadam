# 3D Cable-Driven Positioning System - Python Host

A sophisticated Python host system for controlling 3D cable-driven positioning systems. This system acts as the intelligent "brain" that handles kinematics, path planning, safety, and user interfaces while communicating with an ATmega2560 motor controller client over serial.

## 🚀 Current Status: Phase 1 Complete ✅

**Phase 1 (Foundation Layer)** has been fully implemented and is ready for testing:

- ✅ **Serial Communication Manager** - Robust, thread-safe communication with ATmega2560
- ✅ **Basic Kinematics Engine** - Convert 3D coordinates to motor positions and vice versa  
- ✅ **Motor Controller Interface** - Clean Python wrapper for Arduino command protocol
- ✅ **Position Control System** - Basic 3D positioning functionality
- ✅ **Calibration System** - Accurate motor calibration and management

## System Architecture

- **Python Host** (this codebase): Handles kinematics, path planning, safety, calibration, and user interfaces
- **ATmega2560 Client**: Simple serial-controlled motor driver (complete and documented separately)
- **Hardware Platform**: 4 stepper motors in cable-driven configuration for 3D positioning
- **Platform**: Designed for ARM-based computers (Raspberry Pi and similar)

## System Capabilities

### Core Positioning Features
- **3D Coordinate Positioning**: Move to absolute 3D coordinates with sub-millimeter accuracy
- **Relative Movement**: Move relative to current position
- **Workspace Limits**: Automatic boundary enforcement and safety checks
- **Position Persistence**: Save and restore position state across sessions

### Safety & Reliability
- **Multi-layer Safety**: Software limits, hardware monitoring, and emergency stops
- **Auto-reconnection**: Robust serial communication with automatic recovery
- **Error Handling**: Comprehensive error detection and graceful recovery
- **Emergency Stop**: <100ms response time for critical safety events

### Calibration & Accuracy
- **Semi-automatic Calibration**: Guided calibration procedures with validation
- **Reference Points**: Multiple known positions for accuracy verification
- **Calibration Persistence**: Save calibration data for consistent operation
- **Accuracy Monitoring**: Real-time position error tracking

## Quick Start

### Prerequisites
- Python 3.8+ 
- ATmega2560 motor controller (firmware installed)
- USB serial connection
- 4 stepper motors in cable-driven configuration

### Installation

**Windows:**
```cmd
install.bat
```

**Linux/macOS:**
```bash
./install.sh
```

**Manual Installation:**
```bash
pip install -r requirements.txt
pip install -e .
```

### Basic Usage

```python
import asyncio
from cable_system import CablePositioningSystem

async def main():
    # Create and initialize system
    system = CablePositioningSystem()
    await system.initialize()
    
    # Move to 3D position
    from cable_system.kinematics.kinematics_engine import Point3D
    target = Point3D(100, 200, 300)  # mm
    success = await system.position_controller.move_to_position(target)
    
    # Relative movement
    delta = Point3D(50, 0, 0)  # Move 50mm in X direction
    success = await system.position_controller.move_relative(delta)
    
    # Get current position
    current_pos = await system.position_controller.get_current_position()
    print(f"Current position: {current_pos}")

# Run the system
asyncio.run(main())
```

### Running the Demo

```bash
python main.py
```

This will:
1. Initialize all system components
2. Establish serial communication with ATmega2560
3. Run a demonstration of Phase 1 capabilities
4. Display system status and statistics

## Configuration

The system uses YAML configuration files. Default configuration is in `config/default_config.yaml`:

```yaml
# Serial Communication
serial:
  port: null  # Auto-detect
  baudrate: 115200
  timeout: 1.0

# Kinematics Settings
kinematics:
  steps_per_mm: 80.0
  motor_positions:
    - {x: -500, y: 500, z: 1000}   # Motor 1: Front-left
    - {x: 500, y: 500, z: 1000}    # Motor 2: Front-right
    - {x: 500, y: -500, z: 1000}   # Motor 3: Back-right
    - {x: -500, y: -500, z: 1000}  # Motor 4: Back-left

# Workspace Boundaries
kinematics:
  workspace_x_min: -400.0
  workspace_x_max: 400.0
  workspace_y_min: -400.0
  workspace_y_max: 400.0
  workspace_z_min: 100.0
  workspace_z_max: 800.0
```

## Project Structure

```
cable_system/
├── communication/          # Serial communication with ATmega2560
│   └── serial_manager.py  # Thread-safe serial interface
├── kinematics/            # 3D positioning mathematics
│   └── kinematics_engine.py  # Forward/inverse kinematics
├── control/              # Motor and position control
│   ├── motor_controller.py   # Arduino interface wrapper
│   └── position_control.py   # High-level 3D positioning
├── calibration/          # System calibration
│   └── calibration_system.py # Semi-automatic calibration
├── config/              # Configuration management
│   └── settings.py      # YAML/JSON config system
└── utils/               # Utilities and safety
    ├── safety.py        # Multi-layer safety system
    └── logging_config.py # Structured logging
```

## Development Roadmap

### ✅ Phase 1: Foundation Layer (COMPLETE)
- [x] Serial Communication Manager
- [x] Basic Kinematics Engine  
- [x] Motor Controller Interface
- [x] Position Control System
- [x] Calibration System

### 🔄 Phase 2: Advanced Motion Control (In Development)
- [ ] Advanced Safety and Limits System
- [ ] Path Planning Engine
- [ ] Motion Profiles and Optimization

### 📋 Phase 3: User Interface & Control (Planned)
- [ ] Python API Layer
- [ ] Configuration Management
- [ ] Logging and Monitoring
- [ ] Real-time Visualization

### 🎯 Phase 4: Advanced Features (Planned)
- [ ] Sequence and Automation
- [ ] Camera Control Integration
- [ ] Web Interface and Remote Control
- [ ] Live Stream Integration

## API Examples

### Basic Positioning
```python
# Move to absolute position
target = Point3D(100, 200, 300)
await position_controller.move_to_position(target)

# Relative movement
delta = Point3D(50, 0, 0)
await position_controller.move_relative(delta)

# Move to workspace center
await position_controller.move_to_workspace_center()
```

### Calibration
```python
# Check calibration status
if await calibration_system.is_calibrated():
    print("System is calibrated")
else:
    print("System needs calibration")

# Run full calibration
success = await calibration_system.run_full_calibration()
```

### Safety Monitoring
```python
# Add safety callback
def on_safety_event(event):
    print(f"Safety event: {event}")

safety_manager.add_safety_callback(on_safety_event)

# Emergency stop
await safety_manager.trigger_emergency_stop("Manual trigger")
```

## Performance Targets

- **Positioning Accuracy**: ±0.5mm or better
- **Response Time**: <1 second for positioning commands
- **Emergency Stop**: <100ms response time
- **Communication**: Stable operation with <0.1% command failure rate
- **Uptime**: 99%+ availability for production use

## Platform Requirements

- **Hardware**: ARM-based computer (Raspberry Pi 4+ class)
- **Python**: 3.8+ with full async/await support
- **Memory**: 2GB minimum, 4GB recommended
- **Storage**: 16GB+ for system and logging
- **Network**: Ethernet or WiFi for remote features

## Troubleshooting

### Serial Connection Issues
- Check USB cable connection
- Verify ATmega2560 firmware is loaded
- Check port permissions (Linux: add user to dialout group)
- Try different USB ports

### Calibration Problems
- Ensure motors are properly homed
- Check workspace boundaries in configuration
- Verify motor positions are correctly measured
- Run quick calibration check: `await calibration_system.quick_calibration_check()`

### Performance Issues
- Check system resources (CPU, memory)
- Verify serial communication stability
- Monitor log files for errors
- Adjust configuration parameters as needed

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For issues and questions:
- Check the troubleshooting section above
- Review log files in the `logs/` directory
- Consult the goals.md file for detailed specifications
- Open an issue on the project repository

---

**Status**: Phase 1 Complete ✅ | **Next**: Phase 2 Advanced Motion Control 