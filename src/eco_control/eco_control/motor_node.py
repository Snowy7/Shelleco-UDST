#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time
import glob
import os

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        
        # Serial setup
        self.serial_port = None
        self.serial_connected = False
        self.system_status = "Initializing"
        
        # Parameters
        self.target_motor_duty_cycle = 0  # Target motor duty cycle to send
        self.last_sent_motor_duty_cycle = 0
        
        # Arduino connection parameters
        self.connection_retry_interval = 5.0  # How often to try reconnecting (seconds)
        self.potential_ports = ["/dev/ttyACM0"]
        self.baudrate = 230400
        self.serial_timeout = 0.5
        self.update_rate = 0.2  # 5 Hz
        
        # Logging configuration
        self.verbose_logging = self.declare_parameter('verbose_logging', True).value
        self.connection_attempt_counter = 0
        self.last_connection_log_time = time.time()
        
        # Try to connect initially
        self.connect_to_arduino()
        
        # Subscription
        self.motor_sub = self.create_subscription(
            Float32,
            '/control/motor_commands',
            self.motor_command_callback,
            10)
        
        # Timers for periodic tasks
        self.connection_timer = self.create_timer(self.connection_retry_interval, self.check_connection)
        self.update_timer = self.create_timer(self.update_rate, self.send_periodic_command)
        
        self.get_logger().info('Motor Control Node initialized')

    def find_arduino_ports(self):
        """Find potential Arduino ports by checking available serial devices"""
        if os.name == 'posix':
            ports = glob.glob('/dev/tty[A-Za-z]*')
        else:
            ports = ['COM%s' % (i + 1) for i in range(32)]
            
        for default_port in reversed(self.potential_ports):
            if default_port in ports:
                ports.remove(default_port)
            ports.insert(0, default_port)
        
        if self.verbose_logging:
            self.get_logger().debug(f'Found potential ports: {ports}')
        return ports

    def connect_to_arduino(self):
        """Try to connect to Arduino via serial port"""
        if self.serial_connected and self.serial_port is not None:
            return True
        
        self.connection_attempt_counter += 1
        current_time = time.time()
        time_since_last_log = current_time - self.last_connection_log_time
        
        should_log_attempt = (self.connection_attempt_counter == 1 or 
                           time_since_last_log > 60)
        
        if should_log_attempt:
            self.get_logger().info('Attempting to connect to Arduino...')
            self.last_connection_log_time = current_time
            
        ports = self.find_arduino_ports()
        
        if not ports:
            self.get_logger().warn('No serial ports found')
            self.serial_connected = False
            self.system_status = "No Arduino connection"
            return False
        
        tried_ports = []
        
        for port in ports:
            tried_ports.append(port)
            
            try:
                if self.verbose_logging:
                    self.get_logger().debug(f'Trying port {port}')
                
                if not os.path.exists(port):
                    continue
                    
                serial_port = serial.Serial(
                    port=port,
                    baudrate=self.baudrate,
                    timeout=self.serial_timeout
                )
                
                time.sleep(2.0)
                
                serial_port.reset_input_buffer()
                serial_port.reset_output_buffer()
                
                serial_port.write(b"PING\n")
                time.sleep(0.1)
                
                serial_port.readline()
                
                self.serial_port = serial_port
                self.serial_connected = True
                self.system_status = f"Connected to {port}"
                self.get_logger().info(f'Successfully connected to Arduino on {port}')
                
                self.connection_attempt_counter = 0
                return True
                
            except Exception as e:
                if self.verbose_logging:
                    self.get_logger().debug(f'Failed to connect to {port}: {e}')
        
        self.serial_connected = False
        self.system_status = "No Arduino connection"
        
        if should_log_attempt:
            if len(tried_ports) > 5:
                port_summary = f"{tried_ports[:3]} and {len(tried_ports)-3} more ports"
            else:
                port_summary = f"{tried_ports}"
                
            self.get_logger().warn(f'Could not connect to Arduino. Tried: {port_summary}')
        
        return False

    def check_connection(self):
        """Periodically check and attempt to restore Arduino connection if needed"""
        if not self.serial_connected or self.serial_port is None:
            self.connect_to_arduino()
        else:
            try:
                self.serial_port.write(b"PING\n")
                self.serial_port.readline()
            except Exception as e:
                self.get_logger().error(f'Lost connection to Arduino: {e}')
                self.serial_connected = False
                self.system_status = f"Connection lost: {str(e)}"
                
                try:
                    if self.serial_port is not None:
                        self.serial_port.close()
                except:
                    pass
                
                self.serial_port = None

    def motor_command_callback(self, msg):
        """Handle motor command (Float32)"""
        start_time = time.time()
        throttle_value = msg.data
        
        # Calculate target motor duty cycle
        if throttle_value <= 0:
            self.target_motor_duty_cycle = 0
        else:
            # Scale throttle value (0.0-1.0) to duty cycle (0-255)
            self.target_motor_duty_cycle = int(3400 * min(throttle_value, 1.0))
        
        elapsed = time.time() - start_time
        self.get_logger().info(f'Updated motor value: throttle={throttle_value}, duty_cycle={self.target_motor_duty_cycle}, took {elapsed:.3f}s')

    def send_periodic_command(self):
        if not self.serial_connected or self.serial_port is None:
            return

        if abs(self.target_motor_duty_cycle - self.last_sent_motor_duty_cycle) < 2:
            return

        message = f"MOTOR:{self.target_motor_duty_cycle}\n"
        try:
            start_time = time.time()
            self.serial_port.write(message.encode('utf-8'))
            elapsed = time.time() - start_time
            if elapsed > self.serial_timeout:
                self.get_logger().warn(f"Slow serial write: {elapsed:.3f}s")
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
            self.get_logger().debug(f'Sent: {message.strip()}')
            self.last_sent_motor_duty_cycle = self.target_motor_duty_cycle
        except Exception as e:
            self.get_logger().error(f"Serial communication error: {e}")
            self.serial_connected = False
            self.system_status = f"Serial error: {str(e)}"
            try:
                if self.serial_port is not None:
                    self.serial_port.close()
            except:
                pass
            self.serial_port = None
            self.connect_to_arduino()

    def emergency_stop(self):
        """Stop the motor immediately"""
        self.target_motor_duty_cycle = 0
        self.last_sent_motor_duty_cycle = 0
        
        if self.serial_connected and self.serial_port is not None:
            try:
                self.serial_port.write(b"MOTOR:0\n")
                self.get_logger().info('Emergency stop sent')
            except Exception as e:
                self.get_logger().error(f'Failed to send emergency stop: {e}')
        else:
            self.get_logger().warn('Emergency stop called but Arduino is not connected')

    def shutdown(self):
        """Clean shutdown procedures"""
        self.emergency_stop()
        
        if self.serial_port is not None:
            try:
                self.serial_port.close()
                self.get_logger().info('Serial port closed')
            except Exception as e:
                self.get_logger().error(f'Error closing serial port: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    node = MotorControlNode()
    
    node.get_logger().info('Motor Control Node running')
    node.get_logger().info(f'Verbose logging: {node.verbose_logging}')
    node.get_logger().info(f'Connection retry interval: {node.connection_retry_interval} seconds')
    
    port_list = ', '.join(node.potential_ports[:3])
    if len(node.potential_ports) > 3:
        port_list += f" and {len(node.potential_ports) - 3} more"
    node.get_logger().info(f'Default ports to try: {port_list}')
    
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
