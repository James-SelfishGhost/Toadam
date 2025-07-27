"""
3D Cable-Driven Positioning System - Python Host

A sophisticated Python host system for controlling 3D cable-driven positioning systems.
This system handles kinematics, path planning, safety, and user interfaces while 
communicating with an ATmega2560 motor controller client over serial.
"""

__version__ = "0.1.0"
__author__ = "Cable System Project"

# Core system imports
from .communication.serial_manager import SerialManager
from .kinematics.kinematics_engine import KinematicsEngine
from .control.motor_controller import MotorController
from .control.position_control import PositionController
from .calibration.calibration_system import CalibrationSystem

# Configuration and utilities
from .config.settings import Settings
from .utils.safety import SafetyManager

__all__ = [
    'SerialManager',
    'KinematicsEngine', 
    'MotorController',
    'PositionController',
    'CalibrationSystem',
    'Settings',
    'SafetyManager'
] 