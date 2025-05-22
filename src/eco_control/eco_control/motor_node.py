#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time
import glob
import os

class MotorControlSTM32Node(Node):
    def __init__(self):
        super().__init__('motor_control_stm32_node')

        # Serial setup
        self.serial_port = None
        self.serial_connected = False
        self.system_status = "Initializing"

        # Parameters
        # Target motor speed to send to STM32 (0-2500)
        self.target_motor_speed_stm32 = 0
        self.last_sent_motor_speed_stm32 = -1 # Initialize to a value that ensures first command is sent

        # STM32 connection parameters
        self.potential_ports = self.declare_parameter('potential_ports', ['/dev/ttyACM0', '/dev/ttyUSB0']).value
        self.baudrate = self.declare_parameter('baudrate', 9600).value # Using 9600 as in your VehicleInterfaceNode example
        self.serial_timeout = self.declare_parameter('serial_timeout', 0.5).value
        self.connection_retry_interval = self.declare_parameter('connection_retry_interval', 5.0).value  # How often to try reconnecting (seconds)
        self.update_rate = self.declare_parameter('update_rate', 0.1).value # 10 Hz update rate

        # Logging configuration
        self.verbose_logging = self.declare_parameter('verbose_logging', True).value
        self.connection_attempt_counter = 0
        self.last_connection_log_time = time.time()

        # Try to connect initially
        self.connect_to_stm32()

        # Subscription
        # Subscribes to a Float32 message, assuming a 0.0 to 1.0 range
        self.motor_sub = self.create_subscription(
            Float32,
            '/control/motor_speed', # Topic for motor speed commands
            self.motor_command_callback,
            10)

        # Timers for periodic tasks
        self.connection_timer = self.create_timer(self.connection_retry_interval, self.check_connection)
        self.update_timer = self.create_timer(self.update_rate, self.send_motor_command_periodic)

        self.get_logger().info('Motor Control STM32 Node initialized')
        self.get_logger().info(f'Potential serial ports: {self.potential_ports}')
        self.get_logger().info(f'Baud rate: {self.baudrate}')

    def find_stm32_ports(self):
        """Find potential STM32 ports by checking available serial devices"""
        ports = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/ttyAMA0'] # Default ports for Linux
        
        # Prioritize the potential_ports list
        prioritized_ports = []
        for p in self.potential_ports:
            if p in ports:
                prioritized_ports.append(p)
                ports.remove(p)

        # Add any remaining ports to the end
        prioritized_ports.extend(ports)

        if self.verbose_logging:
            self.get_logger().debug(f'Found potential ports: {prioritized_ports}')
        return prioritized_ports

    def connect_to_stm32(self):
        """Try to connect to STM32 via serial port"""
        if self.serial_connected and self.serial_port is not None:
            return True

        self.connection_attempt_counter += 1
        current_time = time.time()
        time_since_last_log = current_time - self.last_connection_log_time

        # Log connection attempts less frequently after the first few
        should_log_attempt = (self.connection_attempt_counter <= 5 or
                           time_since_last_log > 30) # Log at least every 30 seconds

        if should_log_attempt:
            self.get_logger().info('Attempting to connect to STM32...')
            self.last_connection_log_time = current_time

        ports = self.find_stm32_ports()

        if not ports:
            self.get_logger().warn('No serial ports found')
            self.serial_connected = False
            self.system_status = "No STM32 connection"
            return False

        tried_ports = []

        for port in ports:
            tried_ports.append(port)

            try:
                if self.verbose_logging:
                    self.get_logger().debug(f'Trying port {port}')

                if not os.path.exists(port) and os.name == 'posix': # Check existence on Linux
                     if self.verbose_logging:
                         self.get_logger().debug(f'Port {port} does not exist.')
                     continue # Skip if the file doesn't exist

                serial_port = serial.Serial(
                    port=port,
                    baudrate=self.baudrate,
                    timeout=self.serial_timeout
                )

                # Give the serial port a moment to initialize
                time.sleep(1.0)

                serial_port.reset_input_buffer()
                serial_port.reset_output_buffer()

                # Send a test command (0 speed) to check if the device responds
                try:
                    serial_port.write(b"0000;\n") # Sending a zero speed command
                    # You might want to read a response here if your STM32 sends one
                    # response = serial_port.readline().decode().strip()
                    # if "ACK" in response: # Example check
                    #    pass
                    time.sleep(0.1) # Give STM32 time to process
                except serial.SerialTimeoutException:
                    self.get_logger().debug(f"Timeout during initial communication on {port}")
                    serial_port.close()
                    continue # Try next port
                except Exception as e:
                     self.get_logger().debug(f"Error during initial communication on {port}: {e}")
                     serial_port.close()
                     continue # Try next port


                self.serial_port = serial_port
                self.serial_connected = True
                self.system_status = f"Connected to {port}"
                self.get_logger().info(f'Successfully connected to STM32 on {port}')

                self.connection_attempt_counter = 0
                return True

            except serial.SerialException as e:
                 if self.verbose_logging:
                    self.get_logger().debug(f'Serial error connecting to {port}: {e}')
            except Exception as e:
                if self.verbose_logging:
                    self.get_logger().debug(f'Failed to connect to {port}: {e}')

        self.serial_connected = False
        self.system_status = "No STM32 connection"

        if should_log_attempt:
            if len(tried_ports) > 5:
                port_summary = f"{tried_ports[:3]} and {len(tried_ports)-3} more ports"
            else:
                port_summary = f"{tried_ports}"

            self.get_logger().warn(f'Could not connect to STM32. Tried: {port_summary}')

        return False

    def check_connection(self):
        """Periodically check and attempt to restore STM32 connection if needed"""
        if not self.serial_connected or self.serial_port is None:
            self.connect_to_stm32()
        else:
            try:
                # Send a zero speed command as a heartbeat
                self.send_motor_command_to_stm32(0) # Use the internal sending function
            except Exception as e:
                self.get_logger().error(f'Lost connection to STM32: {e}')
                self.serial_connected = False
                self.system_status = f"Connection lost: {str(e)}"

                try:
                    if self.serial_port is not None:
                        self.serial_port.close()
                except:
                    pass

                self.serial_port = None

    def motor_command_callback(self, msg):
        """Handle motor command (Float32) from ROS topic"""
        start_time = time.time()
        throttle_value = msg.data # Assuming 0.0 to 1.0 range

        # Calculate target motor speed for STM32 (0-2500)
        if throttle_value <= 0.0:
            self.target_motor_speed_stm32 = 0
        else:
            # Scale throttle value (0.0-1.0) to STM32 speed (0-3400)
            # Clamp to 1.0 just in case the input is slightly over
            scaled_speed = int(3400 * min(float(throttle_value), 1.0))
            # Ensure the value is within the valid range [0, 3400]
            self.target_motor_speed_stm32 = max(0, min(3400, scaled_speed))

        elapsed = time.time() - start_time
        if self.verbose_logging:
             self.get_logger().debug(f'Received motor command: throttle={throttle_value}, target_stm32_speed={self.target_motor_speed_stm32}, took {elapsed:.3f}s')

    def send_motor_command_periodic(self):
        """Periodically send the target motor speed to STM32"""
        if not self.serial_connected or self.serial_port is None:
            return

        # Only send if the target speed has changed significantly or it's the first send
        # The tolerance can be adjusted based on how sensitive your STM32 is
        if abs(self.target_motor_speed_stm32 - self.last_sent_motor_speed_stm32) < 5 and self.last_sent_motor_speed_stm32 != -1:
             if self.verbose_logging:
                 self.get_logger().debug(f'Motor speed {self.target_motor_speed_stm32} within tolerance of last sent {self.last_sent_motor_speed_stm32}. Skipping send.')
             return

        self.send_motor_command_to_stm32(self.target_motor_speed_stm32)


    def send_motor_command_to_stm32(self, speed):
        """Send motor speed command to STM32"""
        if not self.serial_connected or self.serial_port is None:
            if self.verbose_logging:
                 self.get_logger().debug(f'Cannot send command {speed} - serial not connected.')
            return

        try:
            # Format the speed as a four-digit integer followed by a semicolon
            formatted_value = f"{speed:04d};"
            start_time = time.time()
            self.serial_port.write(formatted_value.encode('ascii'))
            self.serial_port.flush() # Ensure data is sent immediately
            elapsed = time.time() - start_time

            self.last_sent_motor_speed_stm32 = speed

            if self.verbose_logging:
                self.get_logger().debug(f'Sent to STM32: {formatted_value.strip()}, took {elapsed:.6f}s') # Using strip to remove the potential newline if you added one for testing

            # You might want to add logic here to read a response from the STM32
            # if it sends acknowledgements or sensor data.

        except serial.SerialTimeoutException:
            self.get_logger().error(f"Serial write timeout when sending speed {speed}. Trying to reconnect.")
            self.serial_connected = False
            self.system_status = "Serial write timeout"
            try:
                if self.serial_port is not None:
                    self.serial_port.close()
            except:
                pass
            self.serial_port = None
            self.connect_to_stm32() # Attempt to reconnect immediately

        except Exception as e:
            self.get_logger().error(f"Serial communication error when sending speed {speed}: {e}. Trying to reconnect.")
            self.serial_connected = False
            self.system_status = f"Serial error: {str(e)}"
            try:
                if self.serial_port is not None:
                    self.serial_port.close()
            except:
                pass
            self.serial_port = None
            self.connect_to_stm32() # Attempt to reconnect immediately


    def emergency_stop(self):
        """Stop the motor immediately"""
        self.get_logger().warn('EMERGENCY STOP initiated!')
        self.target_motor_speed_stm32 = 0
        self.last_sent_motor_speed_stm32 = -1 # Ensure the stop command is sent

        if self.serial_connected and self.serial_port is not None:
            try:
                # Send the zero speed command multiple times to be sure
                for _ in range(5):
                    self.send_motor_command_to_stm32(0)
                    time.sleep(0.01) # Short delay between sends
                self.get_logger().info('Emergency stop commands sent to STM32')
            except Exception as e:
                self.get_logger().error(f'Failed to send emergency stop to STM32: {e}')
        else:
            self.get_logger().warn('Emergency stop called but STM32 is not connected')

    def shutdown(self):
        """Clean shutdown procedures"""
        self.get_logger().info('Shutting down Motor Control STM32 Node...')
        # Ensure motor is stopped on shutdown
        self.emergency_stop()

        if self.serial_port is not None:
            try:
                self.serial_port.close()
                self.get_logger().info('Serial port closed')
            except Exception as e:
                self.get_logger().error(f'Error closing serial port: {e}')

def main(args=None):
    rclpy.init(args=args)

    node = MotorControlSTM32Node()

    node.get_logger().info('Motor Control STM32 Node running')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by keyboard interrupt')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
