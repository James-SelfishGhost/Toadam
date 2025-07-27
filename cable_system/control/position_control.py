"""
Position Control System for 3D Cable-Driven Positioning

Provides high-level 3D positioning functionality with safety checks,
workspace limits, and position tracking persistence.
"""

import asyncio
import logging
import json
import os
from typing import Optional, Callable, List, Dict, Any
from dataclasses import dataclass, asdict
import time

from ..kinematics.kinematics_engine import KinematicsEngine, Point3D, CableLengths, WorkspaceBounds
from .motor_controller import MotorController


@dataclass
class PositionState:
    """Represents the current position state of the system."""
    current_position: Point3D
    target_position: Point3D
    is_moving: bool
    last_updated: float
    accuracy_error: float  # Distance between current and target
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization."""
        return {
            'current_position': {'x': self.current_position.x, 'y': self.current_position.y, 'z': self.current_position.z},
            'target_position': {'x': self.target_position.x, 'y': self.target_position.y, 'z': self.target_position.z},
            'is_moving': self.is_moving,
            'last_updated': self.last_updated,
            'accuracy_error': self.accuracy_error
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'PositionState':
        """Create from dictionary."""
        current_pos = data['current_position']
        target_pos = data['target_position']
        
        return cls(
            current_position=Point3D(current_pos['x'], current_pos['y'], current_pos['z']),
            target_position=Point3D(target_pos['x'], target_pos['y'], target_pos['z']),
            is_moving=data['is_moving'],
            last_updated=data['last_updated'],
            accuracy_error=data['accuracy_error']
        )


class PositionController:
    """
    High-level position control system for 3D cable-driven positioning.
    
    Combines kinematics engine and motor controller to provide intuitive
    3D positioning with safety checks and position persistence.
    """

    def __init__(self, 
                 kinematics_engine: KinematicsEngine,
                 motor_controller: MotorController,
                 position_file: str = "position_state.json",
                 max_position_error: float = 2.0):
        """
        Initialize position controller.
        
        Args:
            kinematics_engine: Kinematics engine for coordinate conversion
            motor_controller: Motor controller for hardware interface
            position_file: File path for position state persistence
            max_position_error: Maximum allowable position error in mm
        """
        self.kinematics = kinematics_engine
        self.motor_controller = motor_controller
        self.position_file = position_file
        self.max_position_error = max_position_error
        
        self.logger = logging.getLogger(__name__)
        
        # Position state
        self._position_state = PositionState(
            current_position=self.kinematics.workspace_center,
            target_position=self.kinematics.workspace_center,
            is_moving=False,
            last_updated=time.time(),
            accuracy_error=0.0
        )
        
        # Position callbacks
        self._position_callbacks: List[Callable[[PositionState], None]] = []
        
        # Safety limits
        self._enable_workspace_limits = True
        self._enable_safety_checks = True
        
        # Position tracking
        self._position_update_task: Optional[asyncio.Task] = None
        self._update_interval = 0.2  # 200ms update interval

    async def start(self) -> bool:
        """
        Start the position controller.
        
        Returns:
            bool: True if started successfully
        """
        try:
            # Load saved position state
            await self._load_position_state()
            
            # Start position tracking
            self._position_update_task = asyncio.create_task(self._position_update_loop())
            
            # Update current position from motors
            await self._update_current_position()
            
            self.logger.info("Position controller started successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to start position controller: {e}")
            return False

    async def stop(self):
        """Stop the position controller."""
        try:
            # Stop position tracking
            if self._position_update_task:
                self._position_update_task.cancel()
                try:
                    await self._position_update_task
                except asyncio.CancelledError:
                    pass
            
            # Save current position state
            await self._save_position_state()
            
            self.logger.info("Position controller stopped")
            
        except Exception as e:
            self.logger.error(f"Error stopping position controller: {e}")

    async def move_to_position(self, target: Point3D, 
                             speed: float = None, 
                             wait_for_completion: bool = True) -> bool:
        """
        Move to absolute 3D position.
        
        Args:
            target: Target 3D position
            speed: Movement speed (optional)
            wait_for_completion: Whether to wait for movement completion
            
        Returns:
            bool: True if movement initiated successfully
        """
        try:
            # Safety checks
            if not self._validate_target_position(target):
                return False
            
            # Calculate required cable lengths
            cable_lengths = self.kinematics.inverse_kinematics(target)
            if cable_lengths is None:
                self.logger.error(f"Cannot reach target position: {target}")
                return False
            
            # Convert to motor steps
            motor_steps = self.kinematics.cable_lengths_to_motor_steps(cable_lengths)
            
            # Send movement command to motors
            success = await self.motor_controller.move_all_motors_to_positions(
                motor_steps, speed=speed
            )
            
            if success:
                # Update target position
                self._position_state.target_position = target
                self._position_state.is_moving = True
                self._position_state.last_updated = time.time()
                
                # Save state
                await self._save_position_state()
                
                # Notify callbacks
                self._notify_position_callbacks()
                
                self.logger.info(f"Moving to position: {target}")
                
                # Wait for completion if requested
                if wait_for_completion:
                    return await self.wait_for_position_reached(target)
                
                return True
            else:
                self.logger.error("Failed to send motor movement command")
                return False
                
        except Exception as e:
            self.logger.error(f"Move to position error: {e}")
            return False

    async def move_relative(self, delta: Point3D, 
                          speed: float = None,
                          wait_for_completion: bool = True) -> bool:
        """
        Move relative to current position.
        
        Args:
            delta: Relative movement vector
            speed: Movement speed (optional)
            wait_for_completion: Whether to wait for movement completion
            
        Returns:
            bool: True if movement initiated successfully
        """
        current_pos = self._position_state.current_position
        target_pos = Point3D(
            current_pos.x + delta.x,
            current_pos.y + delta.y,
            current_pos.z + delta.z
        )
        
        return await self.move_to_position(target_pos, speed, wait_for_completion)

    async def move_to_workspace_center(self, wait_for_completion: bool = True) -> bool:
        """
        Move to workspace center position.
        
        Args:
            wait_for_completion: Whether to wait for movement completion
            
        Returns:
            bool: True if movement successful
        """
        return await self.move_to_position(
            self.kinematics.workspace_center, 
            wait_for_completion=wait_for_completion
        )

    async def wait_for_position_reached(self, target: Point3D, 
                                      timeout: float = 30.0,
                                      accuracy: float = None) -> bool:
        """
        Wait for target position to be reached.
        
        Args:
            target: Target position to wait for
            timeout: Maximum wait time in seconds
            accuracy: Required accuracy in mm (uses max_position_error if None)
            
        Returns:
            bool: True if position reached within accuracy
        """
        if accuracy is None:
            accuracy = self.max_position_error
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # Wait for motors to stop moving
            if not await self.motor_controller.wait_for_movement_complete(timeout=1.0):
                continue
            
            # Update current position
            await self._update_current_position()
            
            # Check if we're within accuracy
            error = self._position_state.current_position.distance_to(target)
            if error <= accuracy:
                self._position_state.is_moving = False
                self._position_state.accuracy_error = error
                self._notify_position_callbacks()
                self.logger.info(f"Position reached with {error:.2f}mm accuracy")
                return True
            
            await asyncio.sleep(0.1)
        
        self.logger.warning(f"Position timeout after {timeout}s")
        return False

    async def get_current_position(self, update_from_motors: bool = False) -> Point3D:
        """
        Get current 3D position.
        
        Args:
            update_from_motors: Whether to update from motor positions first
            
        Returns:
            Point3D: Current position
        """
        if update_from_motors:
            await self._update_current_position()
        
        return self._position_state.current_position

    def get_target_position(self) -> Point3D:
        """Get current target position."""
        return self._position_state.target_position

    def get_position_state(self) -> PositionState:
        """Get complete position state."""
        return self._position_state

    def is_moving(self) -> bool:
        """Check if system is currently moving."""
        return self._position_state.is_moving

    def get_position_error(self) -> float:
        """Get current position accuracy error in mm."""
        return self._position_state.accuracy_error

    def add_position_callback(self, callback: Callable[[PositionState], None]):
        """Add callback for position updates."""
        self._position_callbacks.append(callback)

    def set_workspace_limits_enabled(self, enabled: bool):
        """Enable or disable workspace limit enforcement."""
        self._enable_workspace_limits = enabled
        self.logger.info(f"Workspace limits {'enabled' if enabled else 'disabled'}")

    def set_safety_checks_enabled(self, enabled: bool):
        """Enable or disable safety checks."""
        self._enable_safety_checks = enabled
        self.logger.info(f"Safety checks {'enabled' if enabled else 'disabled'}")

    async def calibrate_position(self, known_position: Point3D) -> bool:
        """
        Calibrate current position to a known reference.
        
        Args:
            known_position: Known reference position
            
        Returns:
            bool: True if calibration successful
        """
        try:
            # Validate known position
            if not self._validate_target_position(known_position):
                return False
            
            # Update current position to known position
            self._position_state.current_position = known_position
            self._position_state.target_position = known_position
            self._position_state.is_moving = False
            self._position_state.accuracy_error = 0.0
            self._position_state.last_updated = time.time()
            
            # Save calibrated position
            await self._save_position_state()
            
            # Notify callbacks
            self._notify_position_callbacks()
            
            self.logger.info(f"Position calibrated to: {known_position}")
            return True
            
        except Exception as e:
            self.logger.error(f"Position calibration error: {e}")
            return False

    def _validate_target_position(self, position: Point3D) -> bool:
        """Validate target position against safety limits."""
        if not self._enable_safety_checks:
            return True
        
        # Check workspace bounds
        if self._enable_workspace_limits:
            if not self.kinematics.workspace_bounds.contains(position):
                self.logger.error(f"Position {position} outside workspace bounds")
                return False
        
        # Check if position is kinematically reachable
        cable_lengths = self.kinematics.inverse_kinematics(position)
        if cable_lengths is None:
            self.logger.error(f"Position {position} not kinematically reachable")
            return False
        
        return True

    async def _update_current_position(self):
        """Update current position from motor positions."""
        try:
            # Get current motor states
            motor_states = self.motor_controller.get_all_motor_states()
            
            # Convert motor positions to cable lengths
            motor_positions = [motor_states[i].position for i in range(1, 5)]
            cable_lengths = self.kinematics.motor_steps_to_cable_lengths(motor_positions)
            
            # Calculate 3D position
            position = self.kinematics.forward_kinematics(cable_lengths)
            
            if position is not None:
                self._position_state.current_position = position
                self._position_state.accuracy_error = position.distance_to(
                    self._position_state.target_position
                )
                self._position_state.last_updated = time.time()
            else:
                self.logger.warning("Failed to calculate current position from motor states")
                
        except Exception as e:
            self.logger.error(f"Position update error: {e}")

    async def _position_update_loop(self):
        """Background task for updating position state."""
        while True:
            try:
                # Update current position from motors
                await self._update_current_position()
                
                # Check if movement completed
                system_status = self.motor_controller.get_system_status()
                if self._position_state.is_moving and system_status.all_motors_idle:
                    # Movement may have completed
                    error = self._position_state.accuracy_error
                    if error <= self.max_position_error:
                        self._position_state.is_moving = False
                        self.logger.debug(f"Movement completed with {error:.2f}mm error")
                
                # Notify callbacks periodically
                self._notify_position_callbacks()
                
                await asyncio.sleep(self._update_interval)
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                self.logger.error(f"Position update loop error: {e}")
                await asyncio.sleep(1.0)

    async def _save_position_state(self):
        """Save current position state to file."""
        try:
            state_data = self._position_state.to_dict()
            
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(self.position_file) if os.path.dirname(self.position_file) else '.', exist_ok=True)
            
            with open(self.position_file, 'w') as f:
                json.dump(state_data, f, indent=2)
                
        except Exception as e:
            self.logger.error(f"Failed to save position state: {e}")

    async def _load_position_state(self):
        """Load position state from file."""
        try:
            if os.path.exists(self.position_file):
                with open(self.position_file, 'r') as f:
                    state_data = json.load(f)
                
                self._position_state = PositionState.from_dict(state_data)
                self.logger.info(f"Loaded position state from {self.position_file}")
            else:
                self.logger.info("No saved position state found, using defaults")
                
        except Exception as e:
            self.logger.error(f"Failed to load position state: {e}")
            # Use default state on error

    def _notify_position_callbacks(self):
        """Notify all position callbacks."""
        for callback in self._position_callbacks:
            try:
                callback(self._position_state)
            except Exception as e:
                self.logger.error(f"Position callback error: {e}") 