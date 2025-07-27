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
        
        # Initialize motor states (4 motors)
        for motor_id in range(1, 5):
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
            
            # Initialize communication with Arduino
            response = await self.serial_manager.send_command("init", {})
            if not response.success:
                self.logger.error(f"Failed to initialize Arduino: {response.error}")
                return False
            
            # Get initial system status
            await self._update_system_status()
            
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
            
            # Send shutdown command to Arduino
            await self.serial_manager.send_command("shutdown", {})
            
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
                for motor_id in range(1, 5):
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
            motor_id: Motor ID (1-4)
            position: Target position in steps
            speed: Movement speed in steps/second (optional)
            acceleration: Acceleration in steps/second² (optional)
            
        Returns:
            bool: True if movement command successful
        """
        if not self._validate_motor_id(motor_id):
            return False
        
        try:
            command_data = {
                "motor_id": motor_id,
                "position": position
            }
            
            if speed is not None:
                command_data["speed"] = speed
            if acceleration is not None:
                command_data["acceleration"] = acceleration
            
            response = await self.serial_manager.send_command(
                "move_motor_to_position",
                command_data
            )
            
            if response.success:
                self._motor_states[motor_id].target_position = position
                self._motor_states[motor_id].status = MotorStatus.MOVING
                self.logger.debug(f"Motor {motor_id} moving to position {position}")
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
            positions: List of 4 target positions in steps [M1, M2, M3, M4]
            speed: Movement speed in steps/second (optional)
            acceleration: Acceleration in steps/second² (optional)
            
        Returns:
            bool: True if movement command successful
        """
        if len(positions) != 4:
            self.logger.error("Exactly 4 positions required for all motors")
            return False
        
        try:
            command_data = {"positions": positions}
            
            if speed is not None:
                command_data["speed"] = speed
            if acceleration is not None:
                command_data["acceleration"] = acceleration
            
            response = await self.serial_manager.send_command(
                "move_all_motors_to_positions",
                command_data
            )
            
            if response.success:
                # Update target positions for all motors
                for motor_id in range(1, 5):
                    self._motor_states[motor_id].target_position = positions[motor_id - 1]
                    self._motor_states[motor_id].status = MotorStatus.MOVING
                
                self.logger.debug(f"All motors moving to positions: {positions}")
                return True
            else:
                self.logger.error(f"All motors move failed: {response.error}")
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
        if not (1 <= motor_id <= 4):
            self.logger.error(f"Invalid motor ID: {motor_id}. Must be 1-4.")
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
                "get_system_status", 
                {},
                timeout=2.0
            )
            
            if response.success:
                data = response.data
                
                # Update system status
                old_status = self._system_status
                self._system_status = SystemStatus(
                    is_connected=True,
                    all_motors_idle=data.get('all_motors_idle', False),
                    emergency_stop_active=data.get('emergency_stop_active', False),
                    calibration_complete=data.get('calibration_complete', False),
                    system_errors=data.get('system_errors', []),
                    uptime_seconds=data.get('uptime_seconds', 0.0),
                    last_update=time.time()
                )
                
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
        """Update motor states from Arduino."""
        try:
            response = await self.serial_manager.send_command(
                "get_motor_states",
                {},
                timeout=2.0
            )
            
            if response.success:
                motor_data = response.data.get('motors', [])
                
                for motor_info in motor_data:
                    motor_id = motor_info.get('motor_id')
                    if motor_id and 1 <= motor_id <= 4:
                        old_state = self._motor_states[motor_id]
                        
                        # Update motor state
                        self._motor_states[motor_id] = MotorState(
                            motor_id=motor_id,
                            position=motor_info.get('position', 0),
                            target_position=motor_info.get('target_position', 0),
                            status=MotorStatus(motor_info.get('status', 'unknown')),
                            speed=motor_info.get('speed', 0.0),
                            acceleration=motor_info.get('acceleration', 0.0),
                            is_homed=motor_info.get('is_homed', False),
                            error_code=motor_info.get('error_code'),
                            error_message=motor_info.get('error_message')
                        )
                        
                        # Notify callbacks if state changed
                        if old_state.status != self._motor_states[motor_id].status:
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