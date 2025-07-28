"""
Serial Communication Manager for ATmega2560 Motor Controller

Provides robust, thread-safe communication with automatic reconnection,
command queuing, and error handling.
"""

import asyncio
import logging
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

    def to_string(self) -> str:
        """Convert command to plain text string for serial transmission."""
        # Convert to plain text format expected by Arduino
        cmd_upper = self.command.upper()
        
        if cmd_upper == 'PING':
            return 'PING'
        elif cmd_upper == 'STATUS':
            return 'STATUS'
        elif cmd_upper == 'MOVE':
            motor = self.data.get('motor', 0)
            steps = self.data.get('steps', 0)
            return f'MOVE {motor} {steps}'
        elif cmd_upper == 'MOVEMM':
            motor = self.data.get('motor', 0)
            mm = self.data.get('mm', 0)
            return f'MOVEMM {motor} {mm}'
        elif cmd_upper == 'SETPOS':
            motor = self.data.get('motor', 0)
            pos = self.data.get('position', 0)
            return f'SETPOS {motor} {pos}'
        elif cmd_upper == 'SPEED':
            motor = self.data.get('motor', 'ALL')
            speed = self.data.get('speed', 100)
            return f'SPEED {motor} {speed}'
        elif cmd_upper == 'ACCEL':
            motor = self.data.get('motor', 'ALL')
            accel = self.data.get('acceleration', 100)
            return f'ACCEL {motor} {accel}'
        elif cmd_upper == 'STOP':
            motor = self.data.get('motor', '')
            if motor:
                return f'STOP {motor}'
            else:
                return 'STOP'
        elif cmd_upper == 'ENABLE':
            state = self.data.get('state', 1)
            return f'ENABLE {state}'
        elif cmd_upper == 'CALIB':
            if 'motor' in self.data and 'steps_per_mm' in self.data:
                motor = self.data['motor']
                steps_per_mm = self.data['steps_per_mm']
                return f'CALIB {motor} {steps_per_mm}'
            elif 'action' in self.data:
                action = self.data['action'].upper()
                return f'CALIB {action}'
            else:
                return 'CALIB SHOW'
        else:
            # Generic command with parameters
            params = ' '.join(str(v) for v in self.data.values())
            if params:
                return f'{cmd_upper} {params}'
            else:
                return cmd_upper


class SerialResponse:
    """Represents a response from the motor controller."""
    def __init__(self, success: bool, data: Dict[str, Any] = None, error: str = None, raw_response: str = None):
        self.success = success
        self.data = data or {}
        self.error = error
        self.raw_response = raw_response
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

    def __init__(self, port: str = None, baudrate: int = 115200, timeout: float = 2.0):
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
        
        # Response buffer for processing multi-line responses
        self._response_buffer = []
        
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

                self.logger.info(f"Connecting to {self.port} at {self.baudrate} baud...")
                self._serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                    write_timeout=self.timeout
                )
                
                # Allow Arduino to reset and initialize - Arduino typically takes 1-2 seconds
                self.logger.info("Waiting for Arduino to initialize...")
                await asyncio.sleep(3.0)
                
                # Clear any initial bootup messages
                if self._serial.in_waiting > 0:
                    initial_data = self._serial.read_all().decode('utf-8', errors='ignore')
                    self.logger.debug(f"Initial Arduino output: {initial_data}")
                
                # Test connection with ping
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
            self.logger.info("Testing connection with PING command...")
            
            # Send PING command
            with self._connection_lock:
                if not self._serial or not self._serial.is_open:
                    return False
                
                self._serial.write(b'PING\n')
                self._serial.flush()
            
            # Wait for response - Arduino should respond with some acknowledgment
            start_time = time.time()
            response_received = False
            
            while time.time() - start_time < 3.0:  # Increased timeout
                if self._serial and self._serial.in_waiting > 0:
                    line = self._serial.readline().decode('utf-8', errors='ignore').strip()
                    self.logger.info(f"Received response: '{line}'")
                    
                    # Arduino might respond with various messages, just check we got something meaningful
                    if line and not line.startswith('='):  # Skip header lines
                        response_received = True
                        break
                        
                await asyncio.sleep(0.1)
            
            if response_received:
                self.logger.info("Connection test successful")
                return True
            else:
                self.logger.warning("No response to PING command")
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
                    line = self._read_line()
                    if line:
                        self._handle_response_line(line)
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
                
                command_str = cmd.to_string() + '\n'
                self.logger.debug(f"Sending command: {command_str.strip()}")
                self._serial.write(command_str.encode('utf-8'))
                self._serial.flush()
                self._stats['commands_sent'] += 1
                return True
                
        except Exception as e:
            self.logger.error(f"Failed to send command: {e}")
            self._connected = False
            self._notify_status_change(False)
            return False

    def _read_line(self) -> Optional[str]:
        """Read a line from serial port."""
        try:
            with self._connection_lock:
                if not self._serial or not self._serial.is_open:
                    return None
                
                line = self._serial.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self._stats['responses_received'] += 1
                    return line
                    
        except Exception as e:
            self.logger.error(f"Failed to read line: {e}")
            self._connected = False
            self._notify_status_change(False)
            
        return None

    def _handle_response_line(self, line: str):
        """Handle incoming response line from motor controller."""
        self.logger.debug(f"Received: {line}")
        
        # Parse the response and complete any waiting futures
        # For now, just complete the most recent command future
        if self._response_futures:
            # Get the oldest pending future
            command_id = next(iter(self._response_futures))
            future = self._response_futures.pop(command_id)
            
            if not future.done():
                # Parse the response based on content
                success = not any(error_word in line.lower() for error_word in ['error', 'failed', 'invalid'])
                
                response_data = {'message': line}
                
                # Try to extract useful data from specific responses
                if 'status' in line.lower():
                    response_data['type'] = 'status'
                elif 'motor' in line.lower():
                    response_data['type'] = 'motor'
                
                serial_response = SerialResponse(
                    success=success, 
                    data=response_data, 
                    raw_response=line
                )
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