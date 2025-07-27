"""
Configuration Management System

Handles system settings, configuration loading, validation,
and environment-specific configurations.
"""

import os
import yaml
import json
import logging
from typing import Dict, Any, Optional
from dataclasses import dataclass, asdict
from pathlib import Path


@dataclass
class SerialConfig:
    """Serial communication configuration."""
    port: Optional[str] = None
    baudrate: int = 115200
    timeout: float = 1.0
    auto_detect: bool = True


@dataclass
class KinematicsConfig:
    """Kinematics engine configuration."""
    steps_per_mm: float = 80.0
    max_cable_length: float = 2000.0
    min_cable_length: float = 50.0
    
    # Motor positions (default rectangular arrangement)
    motor_positions: list = None
    
    # Workspace bounds
    workspace_x_min: float = -400.0
    workspace_x_max: float = 400.0
    workspace_y_min: float = -400.0
    workspace_y_max: float = 400.0
    workspace_z_min: float = 100.0
    workspace_z_max: float = 800.0

    def __post_init__(self):
        if self.motor_positions is None:
            # Default rectangular motor arrangement
            self.motor_positions = [
                {"x": -500, "y": 500, "z": 1000},   # Motor 1: Front-left
                {"x": 500, "y": 500, "z": 1000},    # Motor 2: Front-right  
                {"x": 500, "y": -500, "z": 1000},   # Motor 3: Back-right
                {"x": -500, "y": -500, "z": 1000}   # Motor 4: Back-left
            ]


@dataclass
class MotorConfig:
    """Motor controller configuration."""
    default_speed: float = 1000.0  # steps/second
    default_acceleration: float = 2000.0  # steps/second²
    max_speed: float = 5000.0
    max_acceleration: float = 10000.0
    status_update_interval: float = 0.1  # seconds


@dataclass
class PositionConfig:
    """Position control configuration."""
    max_position_error: float = 2.0  # mm
    position_update_interval: float = 0.2  # seconds
    position_file: str = "position_state.json"
    enable_workspace_limits: bool = True
    enable_safety_checks: bool = True


@dataclass
class CalibrationConfig:
    """Calibration system configuration."""
    calibration_file: str = "calibration_data.json"
    max_calibration_age_days: float = 30.0
    calibration_accuracy_threshold: float = 2.0  # mm
    reference_point_accuracy: float = 1.0  # mm


@dataclass
class LoggingConfig:
    """Logging configuration."""
    level: str = "INFO"
    log_file: str = "cable_system.log"
    max_file_size_mb: float = 10.0
    backup_count: int = 5
    console_output: bool = True
    detailed_format: bool = False


@dataclass
class SafetyConfig:
    """Safety system configuration."""
    emergency_stop_timeout: float = 0.1  # seconds
    workspace_violation_action: str = "stop"  # stop, warn, clamp
    max_cable_tension: float = 100.0  # N (if sensors available)
    collision_detection_enabled: bool = False


class Settings:
    """
    Comprehensive configuration management system.
    
    Handles loading, validation, and management of all system settings
    with support for environment-specific configurations and validation.
    """

    def __init__(self, config_file: str = "config/default_config.yaml"):
        """
        Initialize settings manager.
        
        Args:
            config_file: Path to configuration file
        """
        self.config_file = config_file
        self.logger = logging.getLogger(__name__)
        
        # Configuration sections
        self.serial = SerialConfig()
        self.kinematics = KinematicsConfig()
        self.motor = MotorConfig()
        self.position = PositionConfig()
        self.calibration = CalibrationConfig()
        self.logging = LoggingConfig()
        self.safety = SafetyConfig()
        
        # Custom settings
        self._custom_settings: Dict[str, Any] = {}

    def load_config(self, config_file: str = None) -> bool:
        """
        Load configuration from file.
        
        Args:
            config_file: Configuration file path (optional)
            
        Returns:
            bool: True if loaded successfully
        """
        if config_file:
            self.config_file = config_file
            
        try:
            if not os.path.exists(self.config_file):
                self.logger.warning(f"Config file {self.config_file} not found, using defaults")
                return self._create_default_config()
            
            # Determine file format
            if self.config_file.endswith('.yaml') or self.config_file.endswith('.yml'):
                with open(self.config_file, 'r') as f:
                    config_data = yaml.safe_load(f)
            elif self.config_file.endswith('.json'):
                with open(self.config_file, 'r') as f:
                    config_data = json.load(f)
            else:
                self.logger.error(f"Unsupported config file format: {self.config_file}")
                return False
            
            # Load configuration sections
            self._load_section_config(config_data)
            
            # Validate configuration
            if not self._validate_config():
                return False
            
            self.logger.info(f"Configuration loaded from {self.config_file}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to load configuration: {e}")
            return False

    def save_config(self, config_file: str = None) -> bool:
        """
        Save current configuration to file.
        
        Args:
            config_file: Configuration file path (optional)
            
        Returns:
            bool: True if saved successfully
        """
        if config_file:
            self.config_file = config_file
            
        try:
            # Create directory if needed
            os.makedirs(os.path.dirname(self.config_file), exist_ok=True)
            
            # Prepare configuration data
            config_data = {
                'serial': asdict(self.serial),
                'kinematics': asdict(self.kinematics),
                'motor': asdict(self.motor),
                'position': asdict(self.position),
                'calibration': asdict(self.calibration),
                'logging': asdict(self.logging),
                'safety': asdict(self.safety),
                'custom': self._custom_settings
            }
            
            # Save based on file extension
            if self.config_file.endswith('.yaml') or self.config_file.endswith('.yml'):
                with open(self.config_file, 'w') as f:
                    yaml.dump(config_data, f, default_flow_style=False, indent=2)
            elif self.config_file.endswith('.json'):
                with open(self.config_file, 'w') as f:
                    json.dump(config_data, f, indent=2)
            else:
                self.logger.error(f"Unsupported config file format: {self.config_file}")
                return False
            
            self.logger.info(f"Configuration saved to {self.config_file}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to save configuration: {e}")
            return False

    def get_custom_setting(self, key: str, default: Any = None) -> Any:
        """Get custom setting value."""
        return self._custom_settings.get(key, default)

    def set_custom_setting(self, key: str, value: Any):
        """Set custom setting value."""
        self._custom_settings[key] = value

    def update_from_dict(self, config_dict: Dict[str, Any]):
        """Update configuration from dictionary."""
        self._load_section_config(config_dict)

    def to_dict(self) -> Dict[str, Any]:
        """Convert configuration to dictionary."""
        return {
            'serial': asdict(self.serial),
            'kinematics': asdict(self.kinematics),
            'motor': asdict(self.motor),
            'position': asdict(self.position),
            'calibration': asdict(self.calibration),
            'logging': asdict(self.logging),
            'safety': asdict(self.safety),
            'custom': self._custom_settings
        }

    def load_environment_overrides(self):
        """Load configuration overrides from environment variables."""
        # Serial configuration
        if 'CABLE_SERIAL_PORT' in os.environ:
            self.serial.port = os.environ['CABLE_SERIAL_PORT']
        if 'CABLE_SERIAL_BAUDRATE' in os.environ:
            self.serial.baudrate = int(os.environ['CABLE_SERIAL_BAUDRATE'])
        
        # Logging configuration
        if 'CABLE_LOG_LEVEL' in os.environ:
            self.logging.level = os.environ['CABLE_LOG_LEVEL'].upper()
        if 'CABLE_LOG_FILE' in os.environ:
            self.logging.log_file = os.environ['CABLE_LOG_FILE']
        
        # Safety configuration
        if 'CABLE_SAFETY_ENABLED' in os.environ:
            enabled = os.environ['CABLE_SAFETY_ENABLED'].lower() in ('true', '1', 'yes', 'on')
            self.safety.collision_detection_enabled = enabled
        
        self.logger.info("Environment variable overrides applied")

    def _load_section_config(self, config_data: Dict[str, Any]):
        """Load configuration data into sections."""
        # Serial configuration
        if 'serial' in config_data:
            serial_data = config_data['serial']
            for key, value in serial_data.items():
                if hasattr(self.serial, key):
                    setattr(self.serial, key, value)
        
        # Kinematics configuration
        if 'kinematics' in config_data:
            kinematics_data = config_data['kinematics']
            for key, value in kinematics_data.items():
                if hasattr(self.kinematics, key):
                    setattr(self.kinematics, key, value)
        
        # Motor configuration
        if 'motor' in config_data:
            motor_data = config_data['motor']
            for key, value in motor_data.items():
                if hasattr(self.motor, key):
                    setattr(self.motor, key, value)
        
        # Position configuration
        if 'position' in config_data:
            position_data = config_data['position']
            for key, value in position_data.items():
                if hasattr(self.position, key):
                    setattr(self.position, key, value)
        
        # Calibration configuration
        if 'calibration' in config_data:
            calibration_data = config_data['calibration']
            for key, value in calibration_data.items():
                if hasattr(self.calibration, key):
                    setattr(self.calibration, key, value)
        
        # Logging configuration
        if 'logging' in config_data:
            logging_data = config_data['logging']
            for key, value in logging_data.items():
                if hasattr(self.logging, key):
                    setattr(self.logging, key, value)
        
        # Safety configuration
        if 'safety' in config_data:
            safety_data = config_data['safety']
            for key, value in safety_data.items():
                if hasattr(self.safety, key):
                    setattr(self.safety, key, value)
        
        # Custom settings
        if 'custom' in config_data:
            self._custom_settings.update(config_data['custom'])

    def _validate_config(self) -> bool:
        """Validate configuration values."""
        try:
            # Validate serial config
            if self.serial.baudrate <= 0:
                raise ValueError("Serial baudrate must be positive")
            if self.serial.timeout <= 0:
                raise ValueError("Serial timeout must be positive")
            
            # Validate kinematics config
            if self.kinematics.steps_per_mm <= 0:
                raise ValueError("Steps per mm must be positive")
            if self.kinematics.max_cable_length <= self.kinematics.min_cable_length:
                raise ValueError("Max cable length must be greater than min cable length")
            
            # Validate motor config
            if self.motor.max_speed <= 0:
                raise ValueError("Max motor speed must be positive")
            if self.motor.default_speed > self.motor.max_speed:
                raise ValueError("Default speed cannot exceed max speed")
            
            # Validate position config
            if self.position.max_position_error <= 0:
                raise ValueError("Max position error must be positive")
            
            # Validate workspace bounds
            if (self.kinematics.workspace_x_min >= self.kinematics.workspace_x_max or
                self.kinematics.workspace_y_min >= self.kinematics.workspace_y_max or
                self.kinematics.workspace_z_min >= self.kinematics.workspace_z_max):
                raise ValueError("Invalid workspace bounds")
            
            return True
            
        except ValueError as e:
            self.logger.error(f"Configuration validation failed: {e}")
            return False

    def _create_default_config(self) -> bool:
        """Create default configuration file."""
        try:
            # Use current default values (already initialized)
            self.logger.info("Creating default configuration file")
            return self.save_config()
            
        except Exception as e:
            self.logger.error(f"Failed to create default config: {e}")
            return False 