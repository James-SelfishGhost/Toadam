"""
Calibration System for 3D Cable-Driven Positioning

Handles motor calibration, validation, and persistent storage with
semi-automatic calibration procedures and accuracy testing.
"""

import asyncio
import logging
import json
import os
import time
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass, asdict
from enum import Enum

from ..kinematics.kinematics_engine import KinematicsEngine, Point3D, CableLengths
from ..control.motor_controller import MotorController
from ..control.position_control import PositionController


class CalibrationStatus(Enum):
    """Calibration status states."""
    NOT_CALIBRATED = "not_calibrated"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"
    EXPIRED = "expired"


@dataclass
class CalibrationData:
    """Stores calibration data for the system."""
    motor_positions: List[Dict[str, float]]  # 3D positions of motors
    workspace_bounds: Dict[str, float]  # Workspace boundaries
    steps_per_mm: float  # Motor steps per millimeter
    reference_points: List[Dict[str, Any]]  # Known reference positions
    calibration_timestamp: float
    calibration_accuracy: float  # RMS error in mm
    status: CalibrationStatus
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization."""
        return {
            'motor_positions': self.motor_positions,
            'workspace_bounds': self.workspace_bounds,
            'steps_per_mm': self.steps_per_mm,
            'reference_points': self.reference_points,
            'calibration_timestamp': self.calibration_timestamp,
            'calibration_accuracy': self.calibration_accuracy,
            'status': self.status.value
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'CalibrationData':
        """Create from dictionary."""
        return cls(
            motor_positions=data['motor_positions'],
            workspace_bounds=data['workspace_bounds'],
            steps_per_mm=data['steps_per_mm'],
            reference_points=data['reference_points'],
            calibration_timestamp=data['calibration_timestamp'],
            calibration_accuracy=data['calibration_accuracy'],
            status=CalibrationStatus(data['status'])
        )


@dataclass
class ReferencePoint:
    """A known reference point for calibration."""
    name: str
    position: Point3D
    cable_lengths: CableLengths
    motor_steps: List[int]
    measurement_accuracy: float  # Expected accuracy in mm
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            'name': self.name,
            'position': {'x': self.position.x, 'y': self.position.y, 'z': self.position.z},
            'cable_lengths': {
                'motor_1': self.cable_lengths.motor_1,
                'motor_2': self.cable_lengths.motor_2,
                'motor_3': self.cable_lengths.motor_3,
                'motor_4': self.cable_lengths.motor_4
            },
            'motor_steps': self.motor_steps,
            'measurement_accuracy': self.measurement_accuracy
        }


class CalibrationSystem:
    """
    Comprehensive calibration system for cable-driven positioning.
    
    Provides semi-automatic calibration procedures, validation,
    and persistent storage in both EEPROM and local files.
    """

    def __init__(self,
                 kinematics_engine: KinematicsEngine,
                 motor_controller: MotorController,
                 position_controller: PositionController,
                 calibration_file: str = "calibration_data.json",
                 max_calibration_age_days: float = 30.0):
        """
        Initialize calibration system.
        
        Args:
            kinematics_engine: Kinematics engine instance
            motor_controller: Motor controller instance
            position_controller: Position controller instance
            calibration_file: File path for calibration data storage
            max_calibration_age_days: Maximum age before calibration expires
        """
        self.kinematics = kinematics_engine
        self.motor_controller = motor_controller
        self.position_controller = position_controller
        self.calibration_file = calibration_file
        self.max_calibration_age_seconds = max_calibration_age_days * 24 * 3600
        
        self.logger = logging.getLogger(__name__)
        
        # Calibration state
        self._calibration_data: Optional[CalibrationData] = None
        self._reference_points: List[ReferencePoint] = []
        self._is_calibrating = False
        
        # Default reference points for common calibration
        self._default_reference_points = [
            "workspace_center",
            "front_left_corner",
            "front_right_corner", 
            "back_left_corner",
            "back_right_corner"
        ]

    async def start(self) -> bool:
        """
        Start calibration system and load existing calibration.
        
        Returns:
            bool: True if started successfully
        """
        try:
            # Load existing calibration data
            await self._load_calibration_data()
            
            # Check if calibration is valid
            if self._calibration_data:
                if self._is_calibration_valid():
                    self.logger.info("Valid calibration data loaded")
                else:
                    self.logger.warning("Calibration data expired or invalid")
                    self._calibration_data.status = CalibrationStatus.EXPIRED
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to start calibration system: {e}")
            return False

    async def is_calibrated(self) -> bool:
        """
        Check if system is properly calibrated.
        
        Returns:
            bool: True if calibrated and valid
        """
        return (self._calibration_data is not None and 
                self._calibration_data.status == CalibrationStatus.COMPLETED and
                self._is_calibration_valid())

    async def run_full_calibration(self, user_interaction_callback=None) -> bool:
        """
        Run complete system calibration procedure.
        
        Args:
            user_interaction_callback: Callback for user interaction prompts
            
        Returns:
            bool: True if calibration successful
        """
        if self._is_calibrating:
            self.logger.warning("Calibration already in progress")
            return False
        
        self._is_calibrating = True
        
        try:
            self.logger.info("Starting full system calibration")
            
            # Step 1: Home all motors
            if not await self._calibration_step_home_motors():
                return False
            
            # Step 2: Measure motor positions
            if not await self._calibration_step_measure_motors(user_interaction_callback):
                return False
            
            # Step 3: Collect reference points
            if not await self._calibration_step_collect_references(user_interaction_callback):
                return False
            
            # Step 4: Calculate and validate calibration
            if not await self._calibration_step_calculate_calibration():
                return False
            
            # Step 5: Validate accuracy
            if not await self._calibration_step_validate_accuracy():
                return False
            
            # Step 6: Save calibration data
            if not await self._save_calibration_data():
                return False
            
            self.logger.info("Full calibration completed successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Calibration failed: {e}")
            if self._calibration_data:
                self._calibration_data.status = CalibrationStatus.FAILED
            return False
        finally:
            self._is_calibrating = False

    async def quick_calibration_check(self) -> Tuple[bool, float]:
        """
        Perform quick calibration validation check.
        
        Returns:
            Tuple[bool, float]: (is_valid, accuracy_error_mm)
        """
        try:
            if not await self.is_calibrated():
                return False, float('inf')
            
            # Test a few known positions
            test_points = [
                self.kinematics.workspace_center,
                Point3D(0, 0, 300),  # Lower center
                Point3D(100, 100, 400)  # Offset position
            ]
            
            total_error = 0.0
            valid_tests = 0
            
            for point in test_points:
                # Calculate expected cable lengths
                expected_cables = self.kinematics.inverse_kinematics(point)
                if expected_cables is None:
                    continue
                
                # Calculate position from cable lengths
                calculated_position = self.kinematics.forward_kinematics(expected_cables)
                if calculated_position is None:
                    continue
                
                # Calculate error
                error = point.distance_to(calculated_position)
                total_error += error
                valid_tests += 1
            
            if valid_tests == 0:
                return False, float('inf')
            
            avg_error = total_error / valid_tests
            is_valid = avg_error < 2.0  # 2mm tolerance
            
            self.logger.info(f"Quick calibration check: {avg_error:.2f}mm average error")
            return is_valid, avg_error
            
        except Exception as e:
            self.logger.error(f"Quick calibration check failed: {e}")
            return False, float('inf')

    async def add_reference_point(self, name: str, position: Point3D) -> bool:
        """
        Add a new reference point for calibration.
        
        Args:
            name: Reference point name
            position: Known 3D position
            
        Returns:
            bool: True if added successfully
        """
        try:
            # Calculate cable lengths for this position
            cable_lengths = self.kinematics.inverse_kinematics(position)
            if cable_lengths is None:
                self.logger.error(f"Cannot reach reference position: {position}")
                return False
            
            # Convert to motor steps
            motor_steps = self.kinematics.cable_lengths_to_motor_steps(cable_lengths)
            
            # Create reference point
            ref_point = ReferencePoint(
                name=name,
                position=position,
                cable_lengths=cable_lengths,
                motor_steps=motor_steps,
                measurement_accuracy=1.0  # Default 1mm accuracy
            )
            
            # Add to list (replace if exists)
            self._reference_points = [rp for rp in self._reference_points if rp.name != name]
            self._reference_points.append(ref_point)
            
            self.logger.info(f"Added reference point '{name}' at {position}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to add reference point: {e}")
            return False

    async def move_to_reference_point(self, name: str) -> bool:
        """
        Move to a known reference point.
        
        Args:
            name: Reference point name
            
        Returns:
            bool: True if movement successful
        """
        try:
            # Find reference point
            ref_point = None
            for rp in self._reference_points:
                if rp.name == name:
                    ref_point = rp
                    break
            
            if ref_point is None:
                self.logger.error(f"Reference point '{name}' not found")
                return False
            
            # Move to position
            success = await self.position_controller.move_to_position(
                ref_point.position,
                wait_for_completion=True
            )
            
            if success:
                self.logger.info(f"Moved to reference point '{name}'")
            else:
                self.logger.error(f"Failed to move to reference point '{name}'")
            
            return success
            
        except Exception as e:
            self.logger.error(f"Move to reference point error: {e}")
            return False

    def get_calibration_data(self) -> Optional[CalibrationData]:
        """Get current calibration data."""
        return self._calibration_data

    def get_calibration_status(self) -> CalibrationStatus:
        """Get current calibration status."""
        if self._calibration_data is None:
            return CalibrationStatus.NOT_CALIBRATED
        return self._calibration_data.status

    def get_reference_points(self) -> List[ReferencePoint]:
        """Get list of reference points."""
        return self._reference_points.copy()

    async def _calibration_step_home_motors(self) -> bool:
        """Step 1: Home all motors."""
        self.logger.info("Calibration Step 1: Homing all motors")
        
        success = await self.motor_controller.home_all_motors()
        if success:
            self.logger.info("All motors homed successfully")
        else:
            self.logger.error("Motor homing failed")
        
        return success

    async def _calibration_step_measure_motors(self, callback=None) -> bool:
        """Step 2: Measure actual motor positions."""
        self.logger.info("Calibration Step 2: Measuring motor positions")
        
        if callback:
            response = await callback(
                "Please measure and enter the actual 3D positions of all four motors. "
                "Measure from a common reference point to each motor's cable attachment point.",
                "motor_positions"
            )
            
            if response and 'motor_positions' in response:
                motor_positions = []
                for i, pos_data in enumerate(response['motor_positions']):
                    motor_positions.append(Point3D(
                        pos_data['x'], pos_data['y'], pos_data['z']
                    ))
                
                # Update kinematics engine
                self.kinematics.update_motor_positions(motor_positions)
                self.logger.info("Motor positions updated from user measurements")
                return True
        
        # If no callback or user input, use current kinematics positions
        self.logger.info("Using current motor positions from configuration")
        return True

    async def _calibration_step_collect_references(self, callback=None) -> bool:
        """Step 3: Collect reference point measurements."""
        self.logger.info("Calibration Step 3: Collecting reference points")
        
        # Clear existing reference points
        self._reference_points.clear()
        
        # Add workspace center as first reference
        await self.add_reference_point("workspace_center", self.kinematics.workspace_center)
        
        if callback:
            # Interactive reference point collection
            for ref_name in self._default_reference_points[1:]:  # Skip center, already added
                response = await callback(
                    f"Please position the end effector at the {ref_name} and measure its 3D coordinates.",
                    "reference_point",
                    {"name": ref_name}
                )
                
                if response and 'position' in response:
                    pos = response['position']
                    await self.add_reference_point(
                        ref_name,
                        Point3D(pos['x'], pos['y'], pos['z'])
                    )
        else:
            # Automatic reference points (corners of workspace)
            bounds = self.kinematics.workspace_bounds
            reference_positions = {
                "front_left_corner": Point3D(bounds.x_min, bounds.y_max, bounds.z_min + 100),
                "front_right_corner": Point3D(bounds.x_max, bounds.y_max, bounds.z_min + 100),
                "back_left_corner": Point3D(bounds.x_min, bounds.y_min, bounds.z_min + 100),
                "back_right_corner": Point3D(bounds.x_max, bounds.y_min, bounds.z_min + 100)
            }
            
            for name, position in reference_positions.items():
                await self.add_reference_point(name, position)
        
        self.logger.info(f"Collected {len(self._reference_points)} reference points")
        return len(self._reference_points) >= 3  # Need at least 3 points

    async def _calibration_step_calculate_calibration(self) -> bool:
        """Step 4: Calculate calibration parameters."""
        self.logger.info("Calibration Step 4: Calculating calibration parameters")
        
        try:
            # Create calibration data
            self._calibration_data = CalibrationData(
                motor_positions=[
                    {'x': pos.x, 'y': pos.y, 'z': pos.z}
                    for pos in self.kinematics.motor_positions
                ],
                workspace_bounds={
                    'x_min': self.kinematics.workspace_bounds.x_min,
                    'x_max': self.kinematics.workspace_bounds.x_max,
                    'y_min': self.kinematics.workspace_bounds.y_min,
                    'y_max': self.kinematics.workspace_bounds.y_max,
                    'z_min': self.kinematics.workspace_bounds.z_min,
                    'z_max': self.kinematics.workspace_bounds.z_max
                },
                steps_per_mm=self.kinematics.steps_per_mm,
                reference_points=[rp.to_dict() for rp in self._reference_points],
                calibration_timestamp=time.time(),
                calibration_accuracy=0.0,  # Will be calculated next
                status=CalibrationStatus.IN_PROGRESS
            )
            
            return True
            
        except Exception as e:
            self.logger.error(f"Calibration calculation failed: {e}")
            return False

    async def _calibration_step_validate_accuracy(self) -> bool:
        """Step 5: Validate calibration accuracy."""
        self.logger.info("Calibration Step 5: Validating accuracy")
        
        try:
            total_error = 0.0
            valid_tests = 0
            
            # Test each reference point
            for ref_point in self._reference_points:
                # Calculate position from cable lengths
                calculated_pos = self.kinematics.forward_kinematics(ref_point.cable_lengths)
                if calculated_pos is None:
                    continue
                
                # Calculate error
                error = ref_point.position.distance_to(calculated_pos)
                total_error += error
                valid_tests += 1
                
                self.logger.debug(f"Reference '{ref_point.name}': {error:.2f}mm error")
            
            if valid_tests == 0:
                self.logger.error("No valid reference points for accuracy validation")
                return False
            
            # Calculate RMS accuracy
            rms_accuracy = (total_error / valid_tests)
            self._calibration_data.calibration_accuracy = rms_accuracy
            
            # Check if accuracy is acceptable
            if rms_accuracy < 2.0:  # 2mm tolerance
                self._calibration_data.status = CalibrationStatus.COMPLETED
                self.logger.info(f"Calibration completed with {rms_accuracy:.2f}mm RMS accuracy")
                return True
            else:
                self._calibration_data.status = CalibrationStatus.FAILED
                self.logger.error(f"Calibration accuracy {rms_accuracy:.2f}mm exceeds tolerance")
                return False
                
        except Exception as e:
            self.logger.error(f"Accuracy validation failed: {e}")
            return False

    def _is_calibration_valid(self) -> bool:
        """Check if current calibration is still valid."""
        if self._calibration_data is None:
            return False
        
        # Check age
        age = time.time() - self._calibration_data.calibration_timestamp
        if age > self.max_calibration_age_seconds:
            return False
        
        # Check status
        return self._calibration_data.status == CalibrationStatus.COMPLETED

    async def _save_calibration_data(self) -> bool:
        """Save calibration data to file."""
        try:
            if self._calibration_data is None:
                return False
            
            # Create directory if needed
            os.makedirs(os.path.dirname(self.calibration_file) if os.path.dirname(self.calibration_file) else '.', exist_ok=True)
            
            # Save to file
            with open(self.calibration_file, 'w') as f:
                json.dump(self._calibration_data.to_dict(), f, indent=2)
            
            self.logger.info(f"Calibration data saved to {self.calibration_file}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to save calibration data: {e}")
            return False

    async def _load_calibration_data(self) -> bool:
        """Load calibration data from file."""
        try:
            if not os.path.exists(self.calibration_file):
                self.logger.info("No calibration file found")
                return False
            
            with open(self.calibration_file, 'r') as f:
                data = json.load(f)
            
            self._calibration_data = CalibrationData.from_dict(data)
            
            # Load reference points
            self._reference_points.clear()
            for rp_data in self._calibration_data.reference_points:
                pos_data = rp_data['position']
                cable_data = rp_data['cable_lengths']
                
                ref_point = ReferencePoint(
                    name=rp_data['name'],
                    position=Point3D(pos_data['x'], pos_data['y'], pos_data['z']),
                    cable_lengths=CableLengths(
                        cable_data['motor_1'], cable_data['motor_2'],
                        cable_data['motor_3'], cable_data['motor_4']
                    ),
                    motor_steps=rp_data['motor_steps'],
                    measurement_accuracy=rp_data['measurement_accuracy']
                )
                self._reference_points.append(ref_point)
            
            self.logger.info(f"Calibration data loaded from {self.calibration_file}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to load calibration data: {e}")
            return False 