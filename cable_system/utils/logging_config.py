"""
Logging Configuration Utility

Sets up comprehensive logging for the cable positioning system
with file rotation, multiple handlers, and structured output.
"""

import logging
import logging.handlers
import sys
import os
from typing import Optional


def setup_logging(level: str = "INFO",
                 log_file: str = "cable_system.log",
                 max_file_size_mb: float = 10.0,
                 backup_count: int = 5,
                 console_output: bool = True,
                 detailed_format: bool = False) -> bool:
    """
    Set up comprehensive logging configuration.
    
    Args:
        level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_file: Log file path
        max_file_size_mb: Maximum log file size in MB before rotation
        backup_count: Number of backup files to keep
        console_output: Whether to output to console
        detailed_format: Use detailed log format with more information
        
    Returns:
        bool: True if logging setup successful
    """
    try:
        # Convert level string to logging level
        numeric_level = getattr(logging, level.upper(), logging.INFO)
        
        # Create logger
        logger = logging.getLogger()
        logger.setLevel(numeric_level)
        
        # Clear existing handlers
        logger.handlers.clear()
        
        # Create formatters
        if detailed_format:
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(funcName)s() - %(message)s',
                datefmt='%Y-%m-%d %H:%M:%S'
            )
        else:
            formatter = logging.Formatter(
                '%(asctime)s - %(levelname)s - %(name)s - %(message)s',
                datefmt='%Y-%m-%d %H:%M:%S'
            )
        
        # File handler with rotation
        if log_file:
            # Create log directory if needed
            log_dir = os.path.dirname(log_file)
            if log_dir and not os.path.exists(log_dir):
                os.makedirs(log_dir)
            
            file_handler = logging.handlers.RotatingFileHandler(
                log_file,
                maxBytes=int(max_file_size_mb * 1024 * 1024),
                backupCount=backup_count
            )
            file_handler.setLevel(numeric_level)
            file_handler.setFormatter(formatter)
            logger.addHandler(file_handler)
        
        # Console handler
        if console_output:
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setLevel(numeric_level)
            
            # Use simpler format for console
            console_formatter = logging.Formatter(
                '%(asctime)s - %(levelname)s - %(message)s',
                datefmt='%H:%M:%S'
            )
            console_handler.setFormatter(console_formatter)
            logger.addHandler(console_handler)
        
        # Log initial message
        logger.info(f"Logging initialized - Level: {level}, File: {log_file}")
        return True
        
    except Exception as e:
        print(f"Failed to setup logging: {e}")
        return False


def get_logger(name: str) -> logging.Logger:
    """
    Get a logger with the specified name.
    
    Args:
        name: Logger name (typically __name__)
        
    Returns:
        logging.Logger: Configured logger
    """
    return logging.getLogger(name)


def set_log_level(level: str):
    """
    Change the logging level for all loggers.
    
    Args:
        level: New logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
    """
    try:
        numeric_level = getattr(logging, level.upper(), logging.INFO)
        logger = logging.getLogger()
        logger.setLevel(numeric_level)
        
        # Update all handlers
        for handler in logger.handlers:
            handler.setLevel(numeric_level)
            
        logger.info(f"Log level changed to {level}")
        
    except Exception as e:
        print(f"Failed to set log level: {e}")


def add_file_handler(log_file: str, level: str = "INFO") -> bool:
    """
    Add an additional file handler to the logger.
    
    Args:
        log_file: Path to additional log file
        level: Logging level for this handler
        
    Returns:
        bool: True if handler added successfully
    """
    try:
        logger = logging.getLogger()
        numeric_level = getattr(logging, level.upper(), logging.INFO)
        
        # Create directory if needed
        log_dir = os.path.dirname(log_file)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        # Create file handler
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(numeric_level)
        
        # Use same formatter as root logger
        if logger.handlers:
            formatter = logger.handlers[0].formatter
        else:
            formatter = logging.Formatter(
                '%(asctime)s - %(levelname)s - %(name)s - %(message)s',
                datefmt='%Y-%m-%d %H:%M:%S'
            )
        
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
        
        logger.info(f"Additional file handler added: {log_file}")
        return True
        
    except Exception as e:
        print(f"Failed to add file handler: {e}")
        return False


class ContextFilter(logging.Filter):
    """
    Custom logging filter to add context information.
    """
    
    def __init__(self, context: dict):
        super().__init__()
        self.context = context
    
    def filter(self, record):
        for key, value in self.context.items():
            setattr(record, key, value)
        return True


def add_context_to_logger(context: dict, logger_name: Optional[str] = None):
    """
    Add context information to log messages.
    
    Args:
        context: Dictionary of context information
        logger_name: Specific logger name (None for root logger)
    """
    logger = logging.getLogger(logger_name)
    context_filter = ContextFilter(context)
    logger.addFilter(context_filter) 