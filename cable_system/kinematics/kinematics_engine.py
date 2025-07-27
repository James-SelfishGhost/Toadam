"""
Basic Kinematics Engine for 3D Cable-Driven Positioning System

Handles coordinate transformations between 3D Cartesian coordinates and
cable lengths/motor positions with workspace boundary validation.
"""

import math
import numpy as np
from typing import Tuple, List, Optional, Dict, Any
from dataclasses import dataclass
import logging


@dataclass
class Point3D:
    """Represents a 3D point in Cartesian coordinates."""
    x: float
    y: float
    z: float

    def __str__(self) -> str:
        return f"Point3D(x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f})"

    def distance_to(self, other: 'Point3D') -> float:
        """Calculate Euclidean distance to another point."""
        return math.sqrt(
            (self.x - other.x) ** 2 +
            (self.y - other.y) ** 2 +
            (self.z - other.z) ** 2
        )

    def to_array(self) -> np.ndarray:
        """Convert to numpy array."""
        return np.array([self.x, self.y, self.z])


@dataclass
class CableLengths:
    """Represents cable lengths for all four motors."""
    motor_1: float  # Front-left
    motor_2: float  # Front-right
    motor_3: float  # Back-right
    motor_4: float  # Back-left

    def to_list(self) -> List[float]:
        """Convert to list for easy iteration."""
        return [self.motor_1, self.motor_2, self.motor_3, self.motor_4]

    def __str__(self) -> str:
        return f"CableLengths(M1={self.motor_1:.3f}, M2={self.motor_2:.3f}, M3={self.motor_3:.3f}, M4={self.motor_4:.3f})"


@dataclass
class WorkspaceBounds:
    """Defines the safe workspace boundaries."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float  
    z_max: float

    def contains(self, point: Point3D) -> bool:
        """Check if point is within workspace bounds."""
        return (self.x_min <= point.x <= self.x_max and
                self.y_min <= point.y <= self.y_max and
                self.z_min <= point.z <= self.z_max)

    def clamp_point(self, point: Point3D) -> Point3D:
        """Clamp point to workspace boundaries."""
        return Point3D(
            x=max(self.x_min, min(self.x_max, point.x)),
            y=max(self.y_min, min(self.y_max, point.y)),
            z=max(self.z_min, min(self.z_max, point.z))
        )


class KinematicsEngine:
    """
    Kinematics engine for 4-cable 3D positioning system.
    
    Assumes rectangular motor arrangement:
    - Motor 1: Front-left (negative X, positive Y)
    - Motor 2: Front-right (positive X, positive Y)
    - Motor 3: Back-right (positive X, negative Y)
    - Motor 4: Back-left (negative X, negative Y)
    
    All motors are at the same height (Z coordinate).
    """

    def __init__(self, 
                 motor_positions: List[Point3D] = None,
                 workspace_bounds: WorkspaceBounds = None,
                 steps_per_mm: float = 80.0,
                 max_cable_length: float = 2000.0,
                 min_cable_length: float = 50.0):
        """
        Initialize kinematics engine.
        
        Args:
            motor_positions: List of 4 motor positions [M1, M2, M3, M4]
            workspace_bounds: Safe workspace boundaries
            steps_per_mm: Motor steps per millimeter of cable
            max_cable_length: Maximum allowed cable length (mm)
            min_cable_length: Minimum allowed cable length (mm)
        """
        
        # Default motor positions (rectangular arrangement, 1m x 1m at 1m height)
        if motor_positions is None:
            motor_positions = [
                Point3D(-500, 500, 1000),   # Motor 1: Front-left
                Point3D(500, 500, 1000),    # Motor 2: Front-right
                Point3D(500, -500, 1000),   # Motor 3: Back-right
                Point3D(-500, -500, 1000)   # Motor 4: Back-left
            ]
        
        if len(motor_positions) != 4:
            raise ValueError("Exactly 4 motor positions required")
        
        self.motor_positions = motor_positions
        
        # Default workspace bounds (conservative safe area)
        if workspace_bounds is None:
            workspace_bounds = WorkspaceBounds(
                x_min=-400, x_max=400,
                y_min=-400, y_max=400,
                z_min=100, z_max=800
            )
        
        self.workspace_bounds = workspace_bounds
        self.steps_per_mm = steps_per_mm
        self.max_cable_length = max_cable_length
        self.min_cable_length = min_cable_length
        
        # Calculate workspace center
        self.workspace_center = Point3D(
            (workspace_bounds.x_min + workspace_bounds.x_max) / 2,
            (workspace_bounds.y_min + workspace_bounds.y_max) / 2,
            (workspace_bounds.z_min + workspace_bounds.z_max) / 2
        )
        
        self.logger = logging.getLogger(__name__)
        
        # Validate configuration
        self._validate_configuration()

    def forward_kinematics(self, cable_lengths: CableLengths) -> Optional[Point3D]:
        """
        Calculate 3D position from cable lengths (forward kinematics).
        
        Uses trilateration to solve for the 3D position given four cable lengths.
        This is an iterative solution since we have an over-constrained system.
        
        Args:
            cable_lengths: Cable lengths for all four motors
            
        Returns:
            Point3D: Calculated 3D position, or None if no solution found
        """
        try:
            # Validate cable lengths
            lengths = cable_lengths.to_list()
            for i, length in enumerate(lengths):
                if not (self.min_cable_length <= length <= self.max_cable_length):
                    self.logger.warning(f"Cable {i+1} length {length:.1f}mm out of range")
                    return None

            # Use least squares approach to solve over-constrained system
            # We have 4 equations (distance to each motor) and 3 unknowns (x, y, z)
            
            # Set up matrices for least squares solution
            # For each motor i: (x - x_i)² + (y - y_i)² + (z - z_i)² = L_i²
            
            # Initial guess - start from workspace center
            position = self.workspace_center.to_array()
            
            # Newton-Raphson iteration
            for iteration in range(50):  # Max iterations
                # Calculate residuals (difference between actual and target distances)
                residuals = []
                jacobian = []
                
                for i, motor_pos in enumerate(self.motor_positions):
                    # Current distance to motor i
                    dx = position[0] - motor_pos.x
                    dy = position[1] - motor_pos.y
                    dz = position[2] - motor_pos.z
                    current_dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                    
                    # Target distance (cable length)
                    target_dist = lengths[i]
                    
                    # Residual
                    residual = current_dist - target_dist
                    residuals.append(residual)
                    
                    # Jacobian (partial derivatives)
                    if current_dist > 1e-6:  # Avoid division by zero
                        jacobian.append([
                            dx / current_dist,  # ∂r/∂x
                            dy / current_dist,  # ∂r/∂y
                            dz / current_dist   # ∂r/∂z
                        ])
                    else:
                        jacobian.append([0, 0, 0])
                
                residuals = np.array(residuals)
                jacobian = np.array(jacobian)
                
                # Check convergence
                rms_error = np.sqrt(np.mean(residuals**2))
                if rms_error < 0.1:  # 0.1mm accuracy
                    break
                
                # Newton-Raphson update: position = position - (J^T J)^-1 J^T r
                try:
                    JtJ = jacobian.T @ jacobian
                    Jtr = jacobian.T @ residuals
                    delta = np.linalg.solve(JtJ, Jtr)
                    position = position - delta
                except np.linalg.LinAlgError:
                    self.logger.warning("Singular matrix in forward kinematics")
                    return None
            
            # Create result point
            result = Point3D(position[0], position[1], position[2])
            
            # Validate result is reasonable
            if not self._is_position_physically_reasonable(result, cable_lengths):
                self.logger.warning("Forward kinematics result failed validation")
                return None
            
            return result
            
        except Exception as e:
            self.logger.error(f"Forward kinematics error: {e}")
            return None

    def inverse_kinematics(self, target_position: Point3D) -> Optional[CableLengths]:
        """
        Calculate cable lengths from 3D position (inverse kinematics).
        
        Args:
            target_position: Target 3D position
            
        Returns:
            CableLengths: Required cable lengths, or None if position invalid
        """
        try:
            # Validate position is within workspace
            if not self.workspace_bounds.contains(target_position):
                self.logger.warning(f"Target position {target_position} outside workspace")
                return None
            
            # Calculate distance from target to each motor
            cable_lengths = []
            for motor_pos in self.motor_positions:
                distance = target_position.distance_to(motor_pos)
                
                # Validate cable length is within limits
                if not (self.min_cable_length <= distance <= self.max_cable_length):
                    self.logger.warning(f"Required cable length {distance:.1f}mm out of range")
                    return None
                
                cable_lengths.append(distance)
            
            result = CableLengths(*cable_lengths)
            
            # Validate by checking forward kinematics
            if not self._validate_inverse_solution(target_position, result):
                self.logger.warning("Inverse kinematics solution failed validation")
                return None
            
            return result
            
        except Exception as e:
            self.logger.error(f"Inverse kinematics error: {e}")
            return None

    def cable_lengths_to_motor_steps(self, cable_lengths: CableLengths) -> List[int]:
        """Convert cable lengths to motor steps."""
        lengths = cable_lengths.to_list()
        return [int(round(length * self.steps_per_mm)) for length in lengths]

    def motor_steps_to_cable_lengths(self, motor_steps: List[int]) -> CableLengths:
        """Convert motor steps to cable lengths."""
        if len(motor_steps) != 4:
            raise ValueError("Exactly 4 motor step values required")
        
        lengths = [steps / self.steps_per_mm for steps in motor_steps]
        return CableLengths(*lengths)

    def calculate_workspace_volume(self, resolution: float = 50.0) -> float:
        """
        Calculate approximate workspace volume by sampling.
        
        Args:
            resolution: Sampling resolution in mm
            
        Returns:
            float: Workspace volume in cubic mm
        """
        valid_points = 0
        total_points = 0
        
        x_range = np.arange(self.workspace_bounds.x_min, 
                           self.workspace_bounds.x_max + resolution, resolution)
        y_range = np.arange(self.workspace_bounds.y_min,
                           self.workspace_bounds.y_max + resolution, resolution)
        z_range = np.arange(self.workspace_bounds.z_min,
                           self.workspace_bounds.z_max + resolution, resolution)
        
        for x in x_range:
            for y in y_range:
                for z in z_range:
                    total_points += 1
                    point = Point3D(x, y, z)
                    cable_lengths = self.inverse_kinematics(point)
                    if cable_lengths is not None:
                        valid_points += 1
        
        volume_per_point = resolution ** 3
        return valid_points * volume_per_point

    def get_motor_configuration(self) -> Dict[str, Any]:
        """Get current motor configuration."""
        return {
            'motor_positions': [
                {'x': pos.x, 'y': pos.y, 'z': pos.z} 
                for pos in self.motor_positions
            ],
            'workspace_bounds': {
                'x_min': self.workspace_bounds.x_min,
                'x_max': self.workspace_bounds.x_max,
                'y_min': self.workspace_bounds.y_min,
                'y_max': self.workspace_bounds.y_max,
                'z_min': self.workspace_bounds.z_min,
                'z_max': self.workspace_bounds.z_max
            },
            'steps_per_mm': self.steps_per_mm,
            'max_cable_length': self.max_cable_length,
            'min_cable_length': self.min_cable_length
        }

    def update_motor_positions(self, motor_positions: List[Point3D]):
        """Update motor positions and recalculate workspace."""
        if len(motor_positions) != 4:
            raise ValueError("Exactly 4 motor positions required")
        
        self.motor_positions = motor_positions
        self._validate_configuration()
        self.logger.info("Motor positions updated")

    def _validate_configuration(self):
        """Validate motor configuration."""
        # Check motor positions are reasonable
        for i, pos in enumerate(self.motor_positions):
            if pos.z <= 0:
                raise ValueError(f"Motor {i+1} must be above Z=0")
        
        # Check motors form a reasonable geometry
        distances = []
        for i in range(4):
            for j in range(i+1, 4):
                dist = self.motor_positions[i].distance_to(self.motor_positions[j])
                distances.append(dist)
        
        if min(distances) < 100:  # Motors too close
            self.logger.warning("Motors may be too close together")
        
        if max(distances) > 5000:  # Motors very far apart
            self.logger.warning("Motors are very far apart")

    def _is_position_physically_reasonable(self, position: Point3D, cable_lengths: CableLengths) -> bool:
        """Check if position/cable length combination is physically reasonable."""
        # Check all cables are under tension (no negative lengths)
        if any(length <= 0 for length in cable_lengths.to_list()):
            return False
        
        # Check position is not too far from motors (basic sanity check)
        max_dist_to_any_motor = max(
            position.distance_to(motor_pos) 
            for motor_pos in self.motor_positions
        )
        if max_dist_to_any_motor > self.max_cable_length * 1.1:
            return False
        
        return True

    def _validate_inverse_solution(self, target_position: Point3D, cable_lengths: CableLengths) -> bool:
        """Validate inverse kinematics solution by checking forward kinematics."""
        # Check forward kinematics gives similar result
        calculated_position = self.forward_kinematics(cable_lengths)
        if calculated_position is None:
            return False
        
        # Check error is small
        error = target_position.distance_to(calculated_position)
        return error < 1.0  # 1mm tolerance 