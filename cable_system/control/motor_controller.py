"""
Motor Controller Interface for ATmega2560 Communication

Provides a clean Python wrapper for Arduino command protocol with
high-level motor control functions and real-time state tracking.
"""

import asyncio
import logging
from typing import List, Dict, Any, Optional, Callable
from dataclasses import dataclass
from enum import Enum
import time

from ..communication.serial_manager import SerialManager, CommandPriority, SerialResponse


class MotorStatus(Enum):
    """Motor status states."""
    UNKNOWN = "unknown"
    IDLE = "idle"
    MOVING = "moving"
    ERROR = "error"
    CALIBRATING = "calibrating"


@dataclass
class MotorState:
    """Represents the current state of a single motor."""
    motor_id: int
    position: int  # Current position in steps
    target_position: int  # Target position in steps
    status: MotorStatus
    speed: float  # Current speed in steps/second
    acceleration: float  # Current acceleration in steps/second²
    is_homed: bool  # Whether motor has been homed
    error_code: Optional[int] = None
    error_message: Optional[str] = None

    def __str__(self) -> str:
        return f"Motor{self.motor_id}(pos={self.position}, status={self.status.value})"


@dataclass
class SystemStatus:
    """Represents the overall system status."""
    is_connected: bool
    all_motors_idle: bool
    emergency_stop_active: bool
    calibration_complete: bool
    system_errors: List[str]
    uptime_seconds: float
    last_update: float

    def __str__(self) -> str:
        status = "READY" if self.all_motors_idle and self.calibration_complete else "BUSY"
        return f"System({status}, {len(self.system_errors)} errors)"


class MotorController:
    """
    High-level motor controller interface for ATmega2560.
    
    Provides clean Python API for motor operations with real-time
    state tracking and error handling.
    """

    def __init__(self, serial_manager: SerialManager):
        """
        Initialize motor controller.
        
        Args:
            serial_manager: Serial communication manager instance
        """
        self.serial_manager = serial_manager
        self.logger = logging.getLogger(__name__)
        
        # Motor state tracking
        self._motor_states: Dict[int, MotorState] = {}
        self._system_status = SystemStatus(
            is_connected=False,
            all_motors_idle=True,
            emergency_stop_active=False,
            calibration_complete=False,
            system_errors=[],
            uptime_seconds=0.0,
            last_update=0.0
        )
        
        # Status update callbacks
        self._status_callbacks: List[Callable[[SystemStatus], None]] = []
        self._motor_callbacks: List[Callable[[int, MotorState], None]] = []
        
        # Status update task
        self._status_update_task: Optional[asyncio.Task] = None
        self._update_interval = 0.1  # 100ms update interval
        
        # Initialize motor states (4 motors: 0-3)
        for motor_id in range(4):
            self._motor_states[motor_id] = MotorState(
                motor_id=motor_id,
                position=0,
                target_position=0,
                status=MotorStatus.UNKNOWN,
                speed=0.0,
                acceleration=0.0,
                is_homed=False
            )

    async def start(self) -> bool:
        """
        Start the motor controller and begin status monitoring.
        
        Returns:
            bool: True if successfully started
        """
        try:
            # Check connection
            if not self.serial_manager.is_connected():
                self.logger.error("Serial manager not connected")
                return False
            
            # Initialize communication with Arduino using supported commands
            # First, get system status to verify communication
            status_response = await self.serial_manager.send_command("status", {})
            if not status_response.success:
                self.logger.error(f"Failed to get Arduino status: {status_response.error}")
                return False
            
            self.logger.info(f"Arduino status: {status_response.raw_response}")
            
            # Enable motors
            enable_response = await self.serial_manager.send_command("enable", {"state": 1})
            if not enable_response.success:
                self.logger.warning(f"Failed to enable motors: {enable_response.error}")
            else:
                self.logger.info("Motors enabled")
            
            # Get initial system status
            await self._update_system_status()
            
            # Record start time for uptime calculation
            self._start_time = time.time()
            
            # Start status monitoring
            self._status_update_task = asyncio.create_task(self._status_update_loop())
            
            self.logger.info("Motor controller started successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to start motor controller: {e}")
            return False

    async def stop(self):
        """Stop the motor controller and status monitoring."""
        try:
            # Stop status monitoring
            if self._status_update_task:
                self._status_update_task.cancel()
                try:
                    await self._status_update_task
                except asyncio.CancelledError:
                    pass
            
            # Stop all motors using Arduino's STOP command
            await self.serial_manager.send_command("stop", {})
            
            self.logger.info("Motor controller stopped")
            
        except Exception as e:
            self.logger.error(f"Error stopping motor controller: {e}")

    async def emergency_stop(self) -> bool:
        """
        Trigger emergency stop - immediately halt all motors.
        
        Returns:
            bool: True if emergency stop successful
        """
        try:
            response = await self.serial_manager.send_command(
                "emergency_stop", 
                {}, 
                priority=CommandPriority.EMERGENCY,
                timeout=1.0
            )
            
            if response.success:
                self._system_status.emergency_stop_active = True
                self.logger.warning("Emergency stop activated")
                return True
            else:
                self.logger.error(f"Emergency stop failed: {response.error}")
                return False
                
        except Exception as e:
            self.logger.error(f"Emergency stop error: {e}")
            return False

    async def clear_emergency_stop(self) -> bool:
        """
        Clear emergency stop condition.
        
        Returns:
            bool: True if successfully cleared
        """
        try:
            response = await self.serial_manager.send_command("clear_emergency", {})
            
            if response.success:
                self._system_status.emergency_stop_active = False
                self.logger.info("Emergency stop cleared")
                return True
            else:
                self.logger.error(f"Failed to clear emergency stop: {response.error}")
                return False
                
        except Exception as e:
            self.logger.error(f"Clear emergency stop error: {e}")
            return False

    async def home_motor(self, motor_id: int) -> bool:
        """
        Home a single motor to its zero position.
        
        Args:
            motor_id: Motor ID (1-4)
            
        Returns:
            bool: True if homing successful
        """
        if not self._validate_motor_id(motor_id):
            return False
        
        try:
            response = await self.serial_manager.send_command(
                "home_motor",
                {"motor_id": motor_id},
                timeout=30.0  # Homing can take time
            )
            
            if response.success:
                self._motor_states[motor_id].is_homed = True
                self._motor_states[motor_id].position = 0
                self._motor_states[motor_id].target_position = 0
                self.logger.info(f"Motor {motor_id} homed successfully")
                return True
            else:
                self.logger.error(f"Motor {motor_id} homing failed: {response.error}")
                return False
                
        except Exception as e:
            self.logger.error(f"Motor {motor_id} homing error: {e}")
            return False

    async def home_all_motors(self) -> bool:
        """
        Home all motors simultaneously.
        
        Returns:
            bool: True if all motors homed successfully
        """
        try:
            response = await self.serial_manager.send_command(
                "home_all_motors",
                {},
                timeout=60.0  # Homing all motors can take longer
            )
            
            if response.success:
                # Update all motor states
                for motor_id in range(4):
                    self._motor_states[motor_id].is_homed = True
                    self._motor_states[motor_id].position = 0
                    self._motor_states[motor_id].target_position = 0
                
                self.logger.info("All motors homed successfully")
                return True
            else:
                self.logger.error(f"Motor homing failed: {response.error}")
                return False
                
        except Exception as e:
            self.logger.error(f"Motor homing error: {e}")
            return False

    async def move_motor_to_position(self, motor_id: int, position: int, 
                                   speed: float = None, acceleration: float = None) -> bool:
        """
        Move a single motor to absolute position.
        
        Args:
            motor_id: Motor ID (0-3)
            position: Target position in steps
            speed: Movement speed in steps/second (optional)
            acceleration: Acceleration in steps/second² (optional)
            
        Returns:
            bool: True if movement command successful
        """
        if not self._validate_motor_id(motor_id):
            return False
        
        try:
            # Calculate step delta from current position
            current_position = self._motor_states[motor_id].position
            step_delta = position - current_position
            
            # Skip if no movement needed
            if step_delta == 0:
                self.logger.debug(f"Motor {motor_id} already at position {position}")
                return True
            
            # Set speed and acceleration if specified
            if speed is not None:
                speed_response = await self.serial_manager.send_command(
                    "SPEED",
                    {"motor": motor_id, "speed": int(speed)}
                )
                if not speed_response.success:
                    self.logger.warning(f"Failed to set speed for motor {motor_id}")
            
            if acceleration is not None:
                accel_response = await self.serial_manager.send_command(
                    "ACCEL", 
                    {"motor": motor_id, "acceleration": int(acceleration)}
                )
                if not accel_response.success:
                    self.logger.warning(f"Failed to set acceleration for motor {motor_id}")
            
            # Send movement command using Arduino MOVE command with step delta
            response = await self.serial_manager.send_command(
                "MOVE",
                {"motor": motor_id, "steps": step_delta}
            )
            
            if response.success:
                # Update position tracking in Python
                self._motor_states[motor_id].position = position
                self._motor_states[motor_id].target_position = position
                self._motor_states[motor_id].status = MotorStatus.MOVING
                self.logger.debug(f"Motor {motor_id} moving {step_delta} steps to position {position}")
                return True
            else:
                self.logger.error(f"Motor {motor_id} move failed: {response.error}")
                return False
                
        except Exception as e:
            self.logger.error(f"Motor {motor_id} move error: {e}")
            return False

    async def move_all_motors_to_positions(self, positions: List[int],
                                         speed: float = None, acceleration: float = None) -> bool:
        """
        Move all motors to specified positions simultaneously.
        
        Args:
            positions: List of 4 target positions in steps [M0, M1, M2, M3]
            speed: Movement speed in steps/second (optional)
            acceleration: Acceleration in steps/second² (optional)
            
        Returns:
            bool: True if movement command successful
        """
        if len(positions) != 4:
            self.logger.error("Exactly 4 positions required for all motors")
            return False
        
        try:
            # Set speed and acceleration for all motors if specified
            if speed is not None:
                speed_response = await self.serial_manager.send_command(
                    "SPEED",
                    {"motor": "ALL", "speed": int(speed)}
                )
                if not speed_response.success:
                    self.logger.warning("Failed to set speed for all motors")
            
            if acceleration is not None:
                accel_response = await self.serial_manager.send_command(
                    "ACCEL",
                    {"motor": "ALL", "acceleration": int(acceleration)}
                )
                if not accel_response.success:
                    self.logger.warning("Failed to set acceleration for all motors")
            
            # Send individual MOVE commands for each motor (0-based indexing)
            all_success = True
            for motor_id in range(4):
                target_position = positions[motor_id]
                current_position = self._motor_states[motor_id].position
                step_delta = target_position - current_position
                
                # Skip if no movement needed
                if step_delta == 0:
                    self.logger.debug(f"Motor {motor_id} already at position {target_position}")
                    continue
                
                response = await self.serial_manager.send_command(
                    "MOVE",
                    {"motor": motor_id, "steps": step_delta}
                )
                
                if response.success:
                    # Update position tracking in Python
                    self._motor_states[motor_id].position = target_position
                    self._motor_states[motor_id].target_position = target_position
                    self._motor_states[motor_id].status = MotorStatus.MOVING
                else:
                    self.logger.error(f"Motor {motor_id} move to {target_position} failed: {response.error}")
                    all_success = False
            
            if all_success:
                self.logger.debug(f"All motors moving to positions: {positions}")
                return True
            else:
                self.logger.error("Some motors failed to receive movement commands")
                return False
                
        except Exception as e:
            self.logger.error(f"All motors move error: {e}")
            return False

    async def wait_for_movement_complete(self, timeout: float = 30.0) -> bool:
        """
        Wait for all motors to complete their current movements.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            bool: True if all movements completed, False if timeout
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if self._system_status.all_motors_idle:
                return True
            
            await asyncio.sleep(0.1)
        
        self.logger.warning(f"Movement completion timeout after {timeout}s")
        return False

    async def set_motor_parameters(self, motor_id: int, 
                                 max_speed: float = None,
                                 max_acceleration: float = None,
                                 enable: bool = None) -> bool:
        """
        Set motor parameters.
        
        Args:
            motor_id: Motor ID (1-4)
            max_speed: Maximum speed in steps/second
            max_acceleration: Maximum acceleration in steps/second²
            enable: Enable/disable motor
            
        Returns:
            bool: True if parameters set successfully
        """
        if not self._validate_motor_id(motor_id):
            return False
        
        try:
            command_data = {"motor_id": motor_id}
            
            if max_speed is not None:
                command_data["max_speed"] = max_speed
            if max_acceleration is not None:
                command_data["max_acceleration"] = max_acceleration
            if enable is not None:
                command_data["enable"] = enable
            
            response = await self.serial_manager.send_command(
                "set_motor_parameters",
                command_data
            )
            
            if response.success:
                self.logger.debug(f"Motor {motor_id} parameters updated")
                return True
            else:
                self.logger.error(f"Motor {motor_id} parameter update failed: {response.error}")
                return False
                
        except Exception as e:
            self.logger.error(f"Motor {motor_id} parameter error: {e}")
            return False

    def get_motor_state(self, motor_id: int) -> Optional[MotorState]:
        """Get current state of a motor."""
        if not self._validate_motor_id(motor_id):
            return None
        return self._motor_states[motor_id]

    def get_all_motor_states(self) -> Dict[int, MotorState]:
        """Get current states of all motors."""
        return self._motor_states.copy()

    def get_system_status(self) -> SystemStatus:
        """Get current system status."""
        return self._system_status

    def add_status_callback(self, callback: Callable[[SystemStatus], None]):
        """Add callback for system status changes."""
        self._status_callbacks.append(callback)

    def add_motor_callback(self, callback: Callable[[int, MotorState], None]):
        """Add callback for motor state changes."""
        self._motor_callbacks.append(callback)

    async def get_system_info(self) -> Optional[Dict[str, Any]]:
        """Get system information from Arduino."""
        try:
            response = await self.serial_manager.send_command("get_system_info", {})
            
            if response.success:
                return response.data
            else:
                self.logger.error(f"Failed to get system info: {response.error}")
                return None
                
        except Exception as e:
            self.logger.error(f"System info error: {e}")
            return None

    def _validate_motor_id(self, motor_id: int) -> bool:
        """Validate motor ID is in valid range."""
        if not (0 <= motor_id <= 3):
            self.logger.error(f"Invalid motor ID: {motor_id}. Must be 0-3.")
            return False
        return True

    async def _status_update_loop(self):
        """Background task for updating system and motor status."""
        while True:
            try:
                await self._update_system_status()
                await self._update_motor_states()
                await asyncio.sleep(self._update_interval)
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                self.logger.error(f"Status update error: {e}")
                await asyncio.sleep(1.0)  # Wait longer on error

    async def _update_system_status(self):
        """Update system status from Arduino."""
        try:
            response = await self.serial_manager.send_command(
                "status", 
                {},
                timeout=2.0
            )
            
            if response.success:
                # Parse the STATUS response (plain text from Arduino)
                status_text = response.raw_response or ""
                
                # Update system status with basic info from STATUS command
                old_status = self._system_status
                self._system_status = SystemStatus(
                    is_connected=True,
                    all_motors_idle=True,  # Default to idle unless we detect movement
                    emergency_stop_active=False,  # Default to false
                    calibration_complete=True,  # Assume calibrated since Arduino loaded it
                    system_errors=[],  # Will be populated if we detect errors in status
                    uptime_seconds=time.time() - getattr(self, '_start_time', time.time()),
                    last_update=time.time()
                )
                
                # Check status text for any error indicators
                if status_text and any(error_word in status_text.lower() 
                                     for error_word in ['error', 'failed', 'fault']):
                    self._system_status.system_errors.append(status_text)
                
                # Notify callbacks if status changed significantly
                if (old_status.all_motors_idle != self._system_status.all_motors_idle or
                    old_status.emergency_stop_active != self._system_status.emergency_stop_active):
                    self._notify_status_callbacks()
            else:
                # Connection issue
                self._system_status.is_connected = False
                
        except Exception as e:
            self.logger.error(f"System status update error: {e}")
            self._system_status.is_connected = False

    async def _update_motor_states(self):
        """Update motor states from Arduino using STATUS command."""
        try:
            response = await self.serial_manager.send_command(
                "status",
                {},
                timeout=2.0
            )
            
            if response.success:
                # For now, just mark all motors as idle since we don't have detailed state info
                # The Arduino's STATUS command gives system-level status, not per-motor details
                for motor_id in range(4):
                    if motor_id in self._motor_states:
                        old_state = self._motor_states[motor_id]
                        
                        # Update motor state with basic info
                        self._motor_states[motor_id] = MotorState(
                            motor_id=motor_id,
                            position=self._motor_states[motor_id].position,  # Keep last known position
                            target_position=self._motor_states[motor_id].target_position,
                            status=MotorStatus.IDLE,  # Default to idle
                            speed=0.0,  # Default to stopped
                            acceleration=0.0,
                            is_homed=True,  # Arduino says calibration loaded
                            error_code=None,
                            error_message=None
                        )
                        
                        # Notify callbacks if state changed
                        if old_state.status != MotorStatus.IDLE:
                            self._notify_motor_callbacks(motor_id)
                            
        except Exception as e:
            self.logger.error(f"Motor states update error: {e}")

    def _notify_status_callbacks(self):
        """Notify all status callbacks."""
        for callback in self._status_callbacks:
            try:
                callback(self._system_status)
            except Exception as e:
                self.logger.error(f"Status callback error: {e}")

    def _notify_motor_callbacks(self, motor_id: int):
        """Notify all motor callbacks for a specific motor."""
        for callback in self._motor_callbacks:
            try:
                callback(motor_id, self._motor_states[motor_id])
            except Exception as e:
                self.logger.error(f"Motor callback error: {e}")