"""
Serial Communication Manager for ATmega2560 Motor Controller

Provides robust, thread-safe communication with automatic reconnection,
command queuing, and error handling.
"""

import asyncio
import logging
import json
import time
from typing import Optional, Dict, Any, Callable, List
from dataclasses import dataclass
from enum import Enum
import serial
import serial.tools.list_ports
import threading
from queue import Queue, Empty
from concurrent.futures import ThreadPoolExecutor


class CommandPriority(Enum):
    """Command priority levels for queue management."""
    EMERGENCY = 0
    HIGH = 1
    NORMAL = 2
    LOW = 3


@dataclass
class SerialCommand:
    """Represents a command to send to the motor controller."""
    command: str
    data: Dict[str, Any]
    priority: CommandPriority = CommandPriority.NORMAL
    response_expected: bool = True
    timeout: float = 5.0
    callback: Optional[Callable] = None
    timestamp: float = None

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()

    def to_json(self) -> str:
        """Convert command to JSON string for serial transmission."""
        return json.dumps({
            'command': self.command,
            'data': self.data,
            'timestamp': self.timestamp
        })


class SerialResponse:
    """Represents a response from the motor controller."""
    def __init__(self, success: bool, data: Dict[str, Any] = None, error: str = None):
        self.success = success
        self.data = data or {}
        self.error = error
        self.timestamp = time.time()


class SerialManager:
    """
    Thread-safe serial communication manager for ATmega2560 controller.
    
    Features:
    - Auto-reconnection on connection loss
    - Command queuing with priority handling
    - Response parsing and error detection
    - Thread-safe operation for concurrent access
    """

    def __init__(self, port: str = None, baudrate: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        
        # Connection state
        self._serial: Optional[serial.Serial] = None
        self._connected = False
        self._connection_lock = threading.RLock()
        
        # Command queue with priority
        self._command_queue = Queue()
        self._response_futures: Dict[str, asyncio.Future] = {}
        
        # Worker threads
        self._command_thread: Optional[threading.Thread] = None
        self._read_thread: Optional[threading.Thread] = None
        self._running = False
        
        # Statistics and monitoring
        self._stats = {
            'commands_sent': 0,
            'responses_received': 0,
            'connection_attempts': 0,
            'errors': 0
        }
        
        # Callbacks
        self._status_callbacks: List[Callable] = []
        
        # Logger
        self.logger = logging.getLogger(__name__)

    async def start(self) -> bool:
        """
        Start the serial manager and establish connection.
        
        Returns:
            bool: True if successfully connected, False otherwise
        """
        if self._running:
            self.logger.warning("Serial manager already running")
            return self._connected

        self._running = True
        
        # Start worker threads
        self._command_thread = threading.Thread(target=self._command_worker, daemon=True)
        self._read_thread = threading.Thread(target=self._read_worker, daemon=True)
        
        self._command_thread.start()
        self._read_thread.start()
        
        # Attempt initial connection
        connected = await self._connect()
        if connected:
            self.logger.info("Serial manager started successfully")
        else:
            self.logger.error("Failed to establish initial connection")
            
        return connected

    async def stop(self):
        """Stop the serial manager and close connections."""
        self.logger.info("Stopping serial manager")
        self._running = False
        
        # Close serial connection
        with self._connection_lock:
            if self._serial and self._serial.is_open:
                self._serial.close()
            self._connected = False
        
        # Wait for threads to finish
        if self._command_thread and self._command_thread.is_alive():
            self._command_thread.join(timeout=2.0)
        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=2.0)

    async def send_command(self, command: str, data: Dict[str, Any] = None, 
                          priority: CommandPriority = CommandPriority.NORMAL,
                          timeout: float = 5.0) -> SerialResponse:
        """
        Send a command to the motor controller.
        
        Args:
            command: Command name
            data: Command parameters
            priority: Command priority
            timeout: Response timeout in seconds
            
        Returns:
            SerialResponse: Response from controller or error
        """
        if not self._running:
            return SerialResponse(False, error="Serial manager not running")

        cmd = SerialCommand(
            command=command,
            data=data or {},
            priority=priority,
            timeout=timeout
        )
        
        # Create future for response
        future = asyncio.Future()
        command_id = f"{command}_{time.time()}"
        self._response_futures[command_id] = future
        
        # Add command ID to command data for tracking
        cmd.data['_command_id'] = command_id
        
        # Queue command
        self._command_queue.put((priority.value, cmd))
        
        try:
            # Wait for response
            response = await asyncio.wait_for(future, timeout=timeout)
            return response
        except asyncio.TimeoutError:
            self.logger.error(f"Command {command} timed out after {timeout}s")
            self._response_futures.pop(command_id, None)
            return SerialResponse(False, error=f"Command timeout ({timeout}s)")
        except Exception as e:
            self.logger.error(f"Error waiting for command response: {e}")
            self._response_futures.pop(command_id, None)
            return SerialResponse(False, error=str(e))

    def add_status_callback(self, callback: Callable[[bool], None]):
        """Add callback for connection status changes."""
        self._status_callbacks.append(callback)

    def get_stats(self) -> Dict[str, Any]:
        """Get communication statistics."""
        return self._stats.copy()

    def is_connected(self) -> bool:
        """Check if currently connected to motor controller."""
        return self._connected

    async def _connect(self) -> bool:
        """Establish serial connection to motor controller."""
        if self.port is None:
            self.port = self._auto_detect_port()
            if self.port is None:
                self.logger.error("No suitable serial port found")
                return False

        try:
            with self._connection_lock:
                if self._serial and self._serial.is_open:
                    self._serial.close()

                self._serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                    write_timeout=self.timeout
                )
                
                # Test connection with ping
                await asyncio.sleep(0.1)  # Allow Arduino to reset
                test_response = await self._test_connection()
                
                if test_response:
                    self._connected = True
                    self._stats['connection_attempts'] += 1
                    self._notify_status_change(True)
                    self.logger.info(f"Connected to motor controller on {self.port}")
                    return True
                else:
                    self._serial.close()
                    return False

        except Exception as e:
            self.logger.error(f"Failed to connect to {self.port}: {e}")
            self._stats['errors'] += 1
            return False

    def _auto_detect_port(self) -> Optional[str]:
        """Auto-detect Arduino port by scanning available ports."""
        arduino_vendors = ['2341', '1a86', '0403']  # Common Arduino/USB-Serial vendor IDs
        
        for port in serial.tools.list_ports.comports():
            # Check vendor ID
            if port.vid and f"{port.vid:04x}" in arduino_vendors:
                self.logger.info(f"Found potential Arduino on {port.device}")
                return port.device
            
            # Check description
            if any(keyword in port.description.lower() for keyword in ['arduino', 'ch340', 'ft232']):
                self.logger.info(f"Found potential Arduino on {port.device}")
                return port.device
        
        # If no Arduino found, try first available port
        ports = list(serial.tools.list_ports.comports())
        if ports:
            self.logger.warning(f"No Arduino detected, trying first port: {ports[0].device}")
            return ports[0].device
            
        return None

    async def _test_connection(self) -> bool:
        """Test connection by sending ping command."""
        try:
            ping_cmd = SerialCommand('ping', {}, CommandPriority.HIGH, timeout=2.0)
            self._send_raw_command(ping_cmd)
            
            # Wait for pong response
            start_time = time.time()
            while time.time() - start_time < 2.0:
                if self._serial and self._serial.in_waiting > 0:
                    response = self._read_response()
                    if response and response.get('command') == 'pong':
                        return True
                await asyncio.sleep(0.1)
            
            return False
        except Exception as e:
            self.logger.error(f"Connection test failed: {e}")
            return False

    def _command_worker(self):
        """Worker thread for processing command queue."""
        while self._running:
            try:
                # Get command from priority queue (lower number = higher priority)
                priority, cmd = self._command_queue.get(timeout=0.1)
                
                if not self._connected:
                    # Try to reconnect
                    asyncio.run(self._connect())
                    if not self._connected:
                        self._handle_command_error(cmd, "Not connected")
                        continue
                
                # Send command
                success = self._send_raw_command(cmd)
                if not success:
                    self._handle_command_error(cmd, "Failed to send command")
                
            except Empty:
                continue
            except Exception as e:
                self.logger.error(f"Command worker error: {e}")
                self._stats['errors'] += 1

    def _read_worker(self):
        """Worker thread for reading responses."""
        while self._running:
            try:
                if not self._connected or not self._serial:
                    time.sleep(0.1)
                    continue
                
                if self._serial.in_waiting > 0:
                    response = self._read_response()
                    if response:
                        self._handle_response(response)
                else:
                    time.sleep(0.01)  # Small delay to prevent CPU spinning
                    
            except Exception as e:
                self.logger.error(f"Read worker error: {e}")
                self._connected = False
                self._notify_status_change(False)

    def _send_raw_command(self, cmd: SerialCommand) -> bool:
        """Send raw command to serial port."""
        try:
            with self._connection_lock:
                if not self._serial or not self._serial.is_open:
                    return False
                
                json_data = cmd.to_json() + '\n'
                self._serial.write(json_data.encode('utf-8'))
                self._stats['commands_sent'] += 1
                return True
                
        except Exception as e:
            self.logger.error(f"Failed to send command: {e}")
            self._connected = False
            self._notify_status_change(False)
            return False

    def _read_response(self) -> Optional[Dict[str, Any]]:
        """Read and parse response from serial port."""
        try:
            with self._connection_lock:
                if not self._serial or not self._serial.is_open:
                    return None
                
                line = self._serial.readline().decode('utf-8').strip()
                if line:
                    response = json.loads(line)
                    self._stats['responses_received'] += 1
                    return response
                    
        except json.JSONDecodeError as e:
            self.logger.error(f"Failed to parse JSON response: {e}")
        except Exception as e:
            self.logger.error(f"Failed to read response: {e}")
            self._connected = False
            self._notify_status_change(False)
            
        return None

    def _handle_response(self, response: Dict[str, Any]):
        """Handle incoming response from motor controller."""
        command_id = response.get('_command_id')
        if command_id and command_id in self._response_futures:
            future = self._response_futures.pop(command_id)
            if not future.done():
                if response.get('success', True):
                    serial_response = SerialResponse(True, response.get('data', {}))
                else:
                    serial_response = SerialResponse(False, error=response.get('error'))
                future.set_result(serial_response)

    def _handle_command_error(self, cmd: SerialCommand, error: str):
        """Handle command execution error."""
        command_id = cmd.data.get('_command_id')
        if command_id and command_id in self._response_futures:
            future = self._response_futures.pop(command_id)
            if not future.done():
                future.set_result(SerialResponse(False, error=error))

    def _notify_status_change(self, connected: bool):
        """Notify registered callbacks of connection status change."""
        for callback in self._status_callbacks:
            try:
                callback(connected)
            except Exception as e:
                self.logger.error(f"Status callback error: {e}") 