"""
Safety Manager for 3D Cable-Driven Positioning System

Comprehensive safety system with multi-layer safety checks,
emergency stop functionality, and error recovery.
"""

import asyncio
import logging
import time
from typing import List, Callable, Optional, Any, Dict
from enum import Enum
from dataclasses import dataclass

from ..kinematics.kinematics_engine import Point3D, WorkspaceBounds


class SafetyLevel(Enum):
    """Safety check severity levels."""
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"
    CRITICAL = "critical"


class SafetyAction(Enum):
    """Actions to take on safety violations."""
    NONE = "none"
    WARN = "warn"
    STOP = "stop"
    EMERGENCY_STOP = "emergency_stop"


@dataclass
class SafetyEvent:
    """Represents a safety event or violation."""
    timestamp: float
    level: SafetyLevel
    category: str
    message: str
    position: Optional[Point3D] = None
    data: Optional[Dict[str, Any]] = None
    
    def __str__(self) -> str:
        return f"SafetyEvent({self.level.value}, {self.category}: {self.message})"


class SafetyManager:
    """
    Comprehensive safety management system.
    
    Features:
    - Multi-layer safety checks (software, hardware, user-defined)
    - Cable tension monitoring and limits
    - Collision detection and avoidance
    - Emergency stop with graceful recovery
    """

    def __init__(self, workspace_bounds: WorkspaceBounds):
        """
        Initialize safety manager.
        
        Args:
            workspace_bounds: Safe workspace boundaries
        """
        self.workspace_bounds = workspace_bounds
        self.logger = logging.getLogger(__name__)
        
        # Safety state
        self._emergency_stop_active = False
        self._safety_enabled = True
        self._last_position: Optional[Point3D] = None
        self._last_check_time = 0.0
        
        # Safety limits
        self._max_velocity = 1000.0  # mm/s
        self._max_acceleration = 2000.0  # mm/s²
        self._max_cable_tension = 100.0  # N (if sensors available)
        self._position_update_rate = 10.0  # Hz
        
        # Safety event tracking
        self._safety_events: List[SafetyEvent] = []
        self._max_event_history = 1000
        
        # Callbacks
        self._safety_callbacks: List[Callable[[SafetyEvent], None]] = []
        self._emergency_callbacks: List[Callable[[bool], None]] = []
        
        # Custom safety checks
        self._custom_checks: List[Callable[[Point3D, Dict[str, Any]], Optional[SafetyEvent]]] = []

    def add_safety_callback(self, callback: Callable[[SafetyEvent], None]):
        """Add callback for safety events."""
        self._safety_callbacks.append(callback)

    def add_emergency_callback(self, callback: Callable[[bool], None]):
        """Add callback for emergency stop state changes."""
        self._emergency_callbacks.append(callback)

    def add_custom_safety_check(self, check_function: Callable[[Point3D, Dict[str, Any]], Optional[SafetyEvent]]):
        """
        Add custom safety check function.
        
        Args:
            check_function: Function that takes (position, context) and returns SafetyEvent or None
        """
        self._custom_checks.append(check_function)

    def set_safety_enabled(self, enabled: bool):
        """Enable or disable safety system."""
        self._safety_enabled = enabled
        self.logger.info(f"Safety system {'enabled' if enabled else 'disabled'}")

    def is_safety_enabled(self) -> bool:
        """Check if safety system is enabled."""
        return self._safety_enabled

    def is_emergency_stop_active(self) -> bool:
        """Check if emergency stop is active."""
        return self._emergency_stop_active

    async def trigger_emergency_stop(self, reason: str = "Manual trigger") -> bool:
        """
        Trigger emergency stop.
        
        Args:
            reason: Reason for emergency stop
            
        Returns:
            bool: True if emergency stop successful
        """
        if self._emergency_stop_active:
            return True
        
        self._emergency_stop_active = True
        
        # Log safety event
        event = SafetyEvent(
            timestamp=time.time(),
            level=SafetyLevel.CRITICAL,
            category="emergency_stop",
            message=f"Emergency stop triggered: {reason}"
        )
        
        self._add_safety_event(event)
        
        # Notify callbacks
        for callback in self._emergency_callbacks:
            try:
                callback(True)
            except Exception as e:
                self.logger.error(f"Emergency callback error: {e}")
        
        self.logger.critical(f"EMERGENCY STOP TRIGGERED: {reason}")
        return True

    async def clear_emergency_stop(self) -> bool:
        """
        Clear emergency stop condition.
        
        Returns:
            bool: True if cleared successfully
        """
        if not self._emergency_stop_active:
            return True
        
        # Perform safety checks before clearing
        if not await self._pre_clear_safety_checks():
            self.logger.error("Cannot clear emergency stop: safety checks failed")
            return False
        
        self._emergency_stop_active = False
        
        # Log safety event
        event = SafetyEvent(
            timestamp=time.time(),
            level=SafetyLevel.INFO,
            category="emergency_stop",
            message="Emergency stop cleared"
        )
        
        self._add_safety_event(event)
        
        # Notify callbacks
        for callback in self._emergency_callbacks:
            try:
                callback(False)
            except Exception as e:
                self.logger.error(f"Emergency callback error: {e}")
        
        self.logger.info("Emergency stop cleared")
        return True

    def check_position_safety(self, position: Point3D, context: Dict[str, Any] = None) -> List[SafetyEvent]:
        """
        Perform comprehensive position safety checks.
        
        Args:
            position: Position to check
            context: Additional context information
            
        Returns:
            List[SafetyEvent]: List of safety events (empty if safe)
        """
        if not self._safety_enabled:
            return []
        
        safety_events = []
        current_time = time.time()
        context = context or {}
        
        # Update context with timing information
        context.update({
            'current_time': current_time,
            'dt': current_time - self._last_check_time if self._last_check_time > 0 else 0.0
        })
        
        # 1. Workspace boundary check
        workspace_event = self._check_workspace_bounds(position)
        if workspace_event:
            safety_events.append(workspace_event)
        
        # 2. Velocity check
        velocity_event = self._check_velocity_limits(position, context)
        if velocity_event:
            safety_events.append(velocity_event)
        
        # 3. Acceleration check
        acceleration_event = self._check_acceleration_limits(position, context)
        if acceleration_event:
            safety_events.append(acceleration_event)
        
        # 4. Cable tension check (if data available)
        tension_event = self._check_cable_tension(position, context)
        if tension_event:
            safety_events.append(tension_event)
        
        # 5. Collision detection
        collision_event = self._check_collision_detection(position, context)
        if collision_event:
            safety_events.append(collision_event)
        
        # 6. Custom safety checks
        for check_function in self._custom_checks:
            try:
                custom_event = check_function(position, context)
                if custom_event:
                    safety_events.append(custom_event)
            except Exception as e:
                self.logger.error(f"Custom safety check error: {e}")
        
        # Update state
        self._last_position = position
        self._last_check_time = current_time
        
        # Log and notify about safety events
        for event in safety_events:
            self._add_safety_event(event)
        
        return safety_events

    def get_safety_events(self, since: Optional[float] = None, 
                         level: Optional[SafetyLevel] = None) -> List[SafetyEvent]:
        """
        Get safety events with optional filtering.
        
        Args:
            since: Only return events after this timestamp
            level: Only return events of this level or higher
            
        Returns:
            List[SafetyEvent]: Filtered safety events
        """
        events = self._safety_events.copy()
        
        if since is not None:
            events = [e for e in events if e.timestamp >= since]
        
        if level is not None:
            level_values = {
                SafetyLevel.INFO: 0,
                SafetyLevel.WARNING: 1,
                SafetyLevel.ERROR: 2,
                SafetyLevel.CRITICAL: 3
            }
            min_level = level_values[level]
            events = [e for e in events if level_values[e.level] >= min_level]
        
        return events

    def clear_safety_events(self):
        """Clear safety event history."""
        self._safety_events.clear()
        self.logger.info("Safety event history cleared")

    def get_safety_statistics(self) -> Dict[str, Any]:
        """Get safety system statistics."""
        total_events = len(self._safety_events)
        
        # Count by level
        level_counts = {level.value: 0 for level in SafetyLevel}
        for event in self._safety_events:
            level_counts[event.level.value] += 1
        
        # Count by category
        category_counts = {}
        for event in self._safety_events:
            category_counts[event.category] = category_counts.get(event.category, 0) + 1
        
        return {
            'total_events': total_events,
            'level_counts': level_counts,
            'category_counts': category_counts,
            'emergency_stop_active': self._emergency_stop_active,
            'safety_enabled': self._safety_enabled,
            'last_check_time': self._last_check_time
        }

    def _check_workspace_bounds(self, position: Point3D) -> Optional[SafetyEvent]:
        """Check if position is within workspace bounds."""
        if self.workspace_bounds.contains(position):
            return None
        
        return SafetyEvent(
            timestamp=time.time(),
            level=SafetyLevel.ERROR,
            category="workspace_bounds",
            message=f"Position {position} outside workspace bounds",
            position=position
        )

    def _check_velocity_limits(self, position: Point3D, context: Dict[str, Any]) -> Optional[SafetyEvent]:
        """Check velocity limits."""
        if self._last_position is None or context.get('dt', 0) <= 0:
            return None
        
        # Calculate velocity
        distance = position.distance_to(self._last_position)
        velocity = distance / context['dt']  # mm/s
        
        if velocity > self._max_velocity:
            return SafetyEvent(
                timestamp=time.time(),
                level=SafetyLevel.WARNING,
                category="velocity_limit",
                message=f"Velocity {velocity:.1f} mm/s exceeds limit {self._max_velocity} mm/s",
                position=position,
                data={'velocity': velocity, 'limit': self._max_velocity}
            )
        
        return None

    def _check_acceleration_limits(self, position: Point3D, context: Dict[str, Any]) -> Optional[SafetyEvent]:
        """Check acceleration limits."""
        # This would require velocity history to calculate acceleration
        # For now, just a placeholder
        return None

    def _check_cable_tension(self, position: Point3D, context: Dict[str, Any]) -> Optional[SafetyEvent]:
        """Check cable tension limits (if sensors available)."""
        # Placeholder for cable tension monitoring
        # Would require actual tension sensors or force calculations
        tension_data = context.get('cable_tensions', [])
        
        if tension_data:
            for i, tension in enumerate(tension_data):
                if tension > self._max_cable_tension:
                    return SafetyEvent(
                        timestamp=time.time(),
                        level=SafetyLevel.ERROR,
                        category="cable_tension",
                        message=f"Cable {i+1} tension {tension:.1f}N exceeds limit {self._max_cable_tension}N",
                        position=position,
                        data={'cable_id': i+1, 'tension': tension, 'limit': self._max_cable_tension}
                    )
        
        return None

    def _check_collision_detection(self, position: Point3D, context: Dict[str, Any]) -> Optional[SafetyEvent]:
        """Check for potential collisions."""
        # Placeholder for collision detection
        # Would implement actual collision detection algorithms
        
        # Simple example: check if position is too close to motor positions
        motor_positions = context.get('motor_positions', [])
        min_distance = 100.0  # mm minimum distance from motors
        
        for i, motor_pos in enumerate(motor_positions):
            if hasattr(motor_pos, 'distance_to'):
                distance = position.distance_to(motor_pos)
                if distance < min_distance:
                    return SafetyEvent(
                        timestamp=time.time(),
                        level=SafetyLevel.WARNING,
                        category="collision_detection",
                        message=f"Position too close to motor {i+1}: {distance:.1f}mm",
                        position=position,
                        data={'motor_id': i+1, 'distance': distance, 'min_distance': min_distance}
                    )
        
        return None

    async def _pre_clear_safety_checks(self) -> bool:
        """Perform safety checks before clearing emergency stop."""
        # Implement checks to ensure it's safe to clear emergency stop
        # For now, just return True
        return True

    def _add_safety_event(self, event: SafetyEvent):
        """Add safety event to history and notify callbacks."""
        # Add to history
        self._safety_events.append(event)
        
        # Trim history if too long
        if len(self._safety_events) > self._max_event_history:
            self._safety_events = self._safety_events[-self._max_event_history:]
        
        # Log event
        log_level = {
            SafetyLevel.INFO: logging.INFO,
            SafetyLevel.WARNING: logging.WARNING,
            SafetyLevel.ERROR: logging.ERROR,
            SafetyLevel.CRITICAL: logging.CRITICAL
        }[event.level]
        
        self.logger.log(log_level, f"Safety: {event}")
        
        # Notify callbacks
        for callback in self._safety_callbacks:
            try:
                callback(event)
            except Exception as e:
                self.logger.error(f"Safety callback error: {e}")
        
        # Trigger emergency stop for critical events
        if event.level == SafetyLevel.CRITICAL and not self._emergency_stop_active:
            asyncio.create_task(self.trigger_emergency_stop(f"Critical safety event: {event.message}")) 