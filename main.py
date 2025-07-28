"""
3D Cable-Driven Positioning System - Main Application

Phase 1 demonstration of core functionality:
- Serial communication with ATmega2560
- Basic kinematics and motor control  
- Position control with safety checks
- Calibration system
- Configuration management

This file demonstrates the complete Phase 1 implementation.
"""

import asyncio
import logging
import sys
from pathlib import Path

# Add the current directory to Python path for imports
sys.path.insert(0, str(Path(__file__).parent))

from cable_system import (
    SerialManager, KinematicsEngine, MotorController, 
    PositionController, CalibrationSystem, Settings, SafetyManager
)
from cable_system.utils.logging_config import setup_logging
from cable_system.kinematics.kinematics_engine import Point3D, WorkspaceBounds


class CablePositioningSystem:
    """Main system coordinator for the cable positioning system."""
    
    def __init__(self, config_file: str = "config/default_config.yaml"):
        """Initialize the complete cable positioning system."""
        self.config_file = config_file
        self.logger = None
        
        # Core components
        self.settings = None
        self.serial_manager = None
        self.kinematics = None
        self.motor_controller = None
        self.position_controller = None
        self.calibration_system = None
        self.safety_manager = None
        
        # System state
        self.running = False

    async def initialize(self) -> bool:
        """Initialize all system components."""
        try:
            print("🚀 Initializing 3D Cable-Driven Positioning System...")
            
            # 1. Load configuration
            print("📋 Loading configuration...")
            self.settings = Settings(self.config_file)
            if not self.settings.load_config():
                print("❌ Failed to load configuration")
                return False
            
            # Apply environment overrides
            self.settings.load_environment_overrides()
            
            # 2. Setup logging
            print("📝 Setting up logging...")
            if not setup_logging(
                level=self.settings.logging.level,
                log_file=self.settings.logging.log_file,
                max_file_size_mb=self.settings.logging.max_file_size_mb,
                backup_count=self.settings.logging.backup_count,
                console_output=self.settings.logging.console_output,
                detailed_format=self.settings.logging.detailed_format
            ):
                print("❌ Failed to setup logging")
                return False
            
            self.logger = logging.getLogger(__name__)
            self.logger.info("System initialization started")
            
            # 3. Initialize serial communication
            self.logger.info("Initializing serial communication...")
            self.serial_manager = SerialManager(
                port=self.settings.serial.port,
                baudrate=self.settings.serial.baudrate,
                timeout=self.settings.serial.timeout
            )
            
            if not await self.serial_manager.start():
                self.logger.error("Failed to start serial communication")
                return False
            
            # 4. Initialize kinematics engine
            self.logger.info("Initializing kinematics engine...")
            
            # Convert motor positions from config
            motor_positions = []
            for pos_config in self.settings.kinematics.motor_positions:
                motor_positions.append(Point3D(pos_config['x'], pos_config['y'], pos_config['z']))
            
            # Create workspace bounds
            workspace_bounds = WorkspaceBounds(
                x_min=self.settings.kinematics.workspace_x_min,
                x_max=self.settings.kinematics.workspace_x_max,
                y_min=self.settings.kinematics.workspace_y_min,
                y_max=self.settings.kinematics.workspace_y_max,
                z_min=self.settings.kinematics.workspace_z_min,
                z_max=self.settings.kinematics.workspace_z_max
            )
            
            self.kinematics = KinematicsEngine(
                motor_positions=motor_positions,
                workspace_bounds=workspace_bounds,
                steps_per_mm=self.settings.kinematics.steps_per_mm,
                max_cable_length=self.settings.kinematics.max_cable_length,
                min_cable_length=self.settings.kinematics.min_cable_length
            )
            
            # 5. Initialize motor controller
            self.logger.info("Initializing motor controller...")
            self.motor_controller = MotorController(self.serial_manager)
            
            if not await self.motor_controller.start():
                self.logger.error("Failed to start motor controller")
                return False
            
            # 6. Initialize safety manager
            self.logger.info("Initializing safety manager...")
            self.safety_manager = SafetyManager(workspace_bounds)
            
            # Add safety callbacks
            self.safety_manager.add_safety_callback(self._on_safety_event)
            self.safety_manager.add_emergency_callback(self._on_emergency_stop)
            
            # 7. Initialize position controller
            self.logger.info("Initializing position controller...")
            self.position_controller = PositionController(
                kinematics_engine=self.kinematics,
                motor_controller=self.motor_controller,
                position_file=self.settings.position.position_file,
                max_position_error=self.settings.position.max_position_error
            )
            
            # Configure position controller
            self.position_controller.set_workspace_limits_enabled(
                self.settings.position.enable_workspace_limits
            )
            self.position_controller.set_safety_checks_enabled(
                self.settings.position.enable_safety_checks
            )
            
            # Add position callback
            self.position_controller.add_position_callback(self._on_position_update)
            
            if not await self.position_controller.start():
                self.logger.error("Failed to start position controller")
                return False
            
            # 8. Initialize calibration system
            self.logger.info("Initializing calibration system...")
            self.calibration_system = CalibrationSystem(
                kinematics_engine=self.kinematics,
                motor_controller=self.motor_controller,
                position_controller=self.position_controller,
                calibration_file=self.settings.calibration.calibration_file,
                max_calibration_age_days=self.settings.calibration.max_calibration_age_days
            )
            
            if not await self.calibration_system.start():
                self.logger.error("Failed to start calibration system")
                return False
            
            self.logger.info("✅ System initialization completed successfully")
            return True
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"System initialization failed: {e}")
            else:
                print(f"❌ System initialization failed: {e}")
            return False

    async def run_demo(self):
        """Run a demonstration of Phase 1 functionality."""
        try:
            self.logger.info("🎯 Starting Phase 1 demonstration...")
            
            # Check calibration status
            if await self.calibration_system.is_calibrated():
                self.logger.info("✅ System is calibrated")
            else:
                self.logger.warning("⚠️ System is not calibrated - some functions may be limited")
            
            # Get current position
            current_pos = await self.position_controller.get_current_position(update_from_motors=True)
            self.logger.info(f"📍 Current position: {current_pos}")
            
            # Move to workspace center
            self.logger.info("🎯 Moving to workspace center...")
            success = await self.position_controller.move_to_workspace_center()
            if success:
                self.logger.info("✅ Successfully moved to workspace center")
            else:
                self.logger.error("❌ Failed to move to workspace center")
            
            # Demonstrate relative movement
            self.logger.info("🔄 Demonstrating relative movements...")
            movements = [
                Point3D(50, 0, 0),   # Move +50mm in X
                Point3D(0, 50, 0),   # Move +50mm in Y  
                Point3D(0, 0, 50),   # Move +50mm in Z
                Point3D(-50, -50, -50)  # Return closer to center
            ]
            
            for i, delta in enumerate(movements):
                self.logger.info(f"🔄 Relative movement {i+1}: {delta}")
                success = await self.position_controller.move_relative(
                    delta, wait_for_completion=True
                )
                if success:
                    new_pos = await self.position_controller.get_current_position()
                    self.logger.info(f"✅ New position: {new_pos}")
                else:
                    self.logger.error(f"❌ Movement {i+1} failed")
                
                # Small delay between movements
                await asyncio.sleep(1.0)
            
            # Display system statistics
            await self._display_system_status()
            
            self.logger.info("🎉 Phase 1 demonstration completed successfully!")
            
        except Exception as e:
            self.logger.error(f"Demo failed: {e}")

    async def _display_system_status(self):
        """Display comprehensive system status."""
        self.logger.info("📊 System Status Report:")
        
        # Serial communication stats
        serial_stats = self.serial_manager.get_stats()
        self.logger.info(f"  📡 Serial: {serial_stats['commands_sent']} commands sent, "
                        f"{serial_stats['responses_received']} responses received")
        
        # Motor status
        motor_states = self.motor_controller.get_all_motor_states()
        for motor_id, state in motor_states.items():
            self.logger.info(f"  🔧 Motor {motor_id}: {state.status.value}, pos={state.position}")
        
        # Position status
        pos_state = self.position_controller.get_position_state()
        self.logger.info(f"  📍 Position: current={pos_state.current_position}, "
                        f"error={pos_state.accuracy_error:.2f}mm")
        
        # Safety status
        safety_stats = self.safety_manager.get_safety_statistics()
        self.logger.info(f"  🛡️ Safety: {safety_stats['total_events']} events, "
                        f"emergency_stop={safety_stats['emergency_stop_active']}")
        
        # Calibration status
        calibration_status = self.calibration_system.get_calibration_status()
        self.logger.info(f"  🎯 Calibration: {calibration_status.value}")

    def _on_safety_event(self, event):
        """Handle safety events."""
        self.logger.warning(f"🛡️ Safety Event: {event}")

    def _on_emergency_stop(self, active: bool):
        """Handle emergency stop state changes."""
        if active:
            self.logger.critical("🚨 EMERGENCY STOP ACTIVATED!")
        else:
            self.logger.info("✅ Emergency stop cleared")

    def _on_position_update(self, position_state):
        """Handle position updates."""
        if position_state.is_moving:
            self.logger.debug(f"🔄 Moving to {position_state.target_position}, "
                            f"error={position_state.accuracy_error:.2f}mm")

    async def shutdown(self):
        """Gracefully shutdown all system components."""
        self.logger.info("🛑 Shutting down system...")
        
        try:
            # Stop all components in reverse order
            if self.calibration_system:
                # Calibration system doesn't need explicit stop
                pass
            
            if self.position_controller:
                await self.position_controller.stop()
            
            if self.motor_controller:
                await self.motor_controller.stop()
            
            if self.serial_manager:
                await self.serial_manager.stop()
            
            self.logger.info("✅ System shutdown completed")
            
        except Exception as e:
            self.logger.error(f"Error during shutdown: {e}")

    async def run(self):
        """Run the complete system."""
        self.running = True
        
        try:
            # Initialize system
            if not await self.initialize():
                return False
            
            # Run demonstration
            await self.run_demo()
            
            # Keep system running for interactive use
            self.logger.info("📡 System ready for operation (Ctrl+C to stop)")
            
            try:
                while self.running:
                    # Use shorter sleep for better responsiveness
                    await asyncio.sleep(0.1)
                    
                    # Check for any system issues occasionally
                    if not hasattr(self, '_last_connection_check'):
                        self._last_connection_check = 0
                    
                    import time
                    current_time = time.time()
                    if current_time - self._last_connection_check > 5.0:  # Check every 5 seconds
                        if not self.serial_manager.is_connected():
                            self.logger.warning("⚠️ Serial connection lost - attempting reconnection...")
                        self._last_connection_check = current_time
                        
            except asyncio.CancelledError:
                self.logger.info("👋 Task cancelled, shutting down")
                
        except KeyboardInterrupt:
            self.logger.info("👋 Received Ctrl+C, shutting down")
        except Exception as e:
            self.logger.error(f"System error: {e}")
        finally:
            await self.shutdown()
            return True

    def stop(self):
        """Stop the system (can be called from signal handlers)."""
        self.running = False


async def main():
    """Main entry point."""
    print("=" * 60)
    print("   3D Cable-Driven Positioning System - Phase 1")
    print("=" * 60)
    
    # Create and run system
    system = CablePositioningSystem()
    
    try:
        success = await system.run()
        return 0 if success else 1
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        return 1


if __name__ == "__main__":
    # Run the main async function
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
