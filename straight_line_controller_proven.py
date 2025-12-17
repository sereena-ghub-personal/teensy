#!/usr/bin/env python3
"""
=============================================================================
STRAIGHT LINE CONTROLLER - Using Proven Servo Parameters
=============================================================================
Uses the exact servo values from your working 2m straight line code

Subscribes to: /imu_data (String) - "yaw pitch roll"
Publishes to: /servo_cmd (String) - "left_pwm right_pwm"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

class StraightLineController(Node):
    def __init__(self):
        super().__init__('straight_line_controller')
        
        # ============================================
        # PROVEN SERVO PARAMETERS (from your 2m code)
        # ============================================
        
        # Neutral (STOP) values
        self.PWM_NEUTRAL_RIGHT = 94
        self.PWM_NEUTRAL_LEFT = 94
        
        # Forward base speeds (from your working code!)
        self.BASE_RIGHT_FWD = 88   # Your BASE_RIGHT_FWD
        self.BASE_LEFT_FWD = 97    # Your BASE_LEFT_FWD
        
        # Speed limits (from your SPEED_MIN/SPEED_MAX)
        self.SPEED_MIN = 83
        self.SPEED_MAX = 105
        
        # PID gains (from your 2m code: KP_2M = 0.05)
        # Starting conservative, you can increase if needed
        self.Kp = 0.05   # Your exact value from 2m code
        self.Ki = 0.01   # Small integral term
        self.Kd = 0.02   # Small derivative term
        
        # Maximum error limit (from your MAX_ERROR = 80.0)
        self.MAX_ERROR = 80.0
        
        # ============================================
        # STATE VARIABLES
        # ============================================
        self.initial_yaw = None
        self.current_yaw = 0.0
        self.is_moving = False
        
        # PID state
        self.integral_error = 0.0
        self.last_error = 0.0
        
        # ============================================
        # ROS2 SETUP
        # ============================================
        
        # Subscribe to IMU data
        self.imu_subscription = self.create_subscription(
            String,
            '/imu_data',
            self.imu_callback,
            10)
        
        # Publish servo commands
        self.servo_publisher = self.create_publisher(
            String,
            '/servo_cmd',
            10)
        
        # Services
        self.start_service = self.create_service(
            Trigger,
            '/start_straight',
            self.start_callback)
        
        self.stop_service = self.create_service(
            Trigger,
            '/stop_straight',
            self.stop_callback)
        
        # Control loop timer (125Hz - same as your 8ms delay)
        self.timer = self.create_timer(0.008, self.control_loop)  # 8ms = 125Hz
        
        self.get_logger().info('='*60)
        self.get_logger().info('STRAIGHT LINE CONTROLLER - PROVEN PARAMETERS')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Servo Stop: R={self.PWM_NEUTRAL_RIGHT} L={self.PWM_NEUTRAL_LEFT}')
        self.get_logger().info(f'Forward Base: R={self.BASE_RIGHT_FWD} L={self.BASE_LEFT_FWD}')
        self.get_logger().info(f'PID Gains: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}')
        self.get_logger().info(f'Speed Range: [{self.SPEED_MIN}, {self.SPEED_MAX}]')
        self.get_logger().info('')
        self.get_logger().info('Commands:')
        self.get_logger().info('  Start: ros2 service call /start_straight std_srvs/srv/Trigger')
        self.get_logger().info('  Stop:  ros2 service call /stop_straight std_srvs/srv/Trigger')
        self.get_logger().info('='*60)
    
    def imu_callback(self, msg):
        """Process incoming IMU data"""
        try:
            data = msg.data.split()
            if len(data) >= 1:
                self.current_yaw = float(data[0])
                
                # Set initial yaw on first reading
                if self.initial_yaw is None:
                    self.initial_yaw = self.current_yaw
                    self.get_logger().info(f'Initial yaw recorded: {self.initial_yaw:.2f}°')
        
        except (ValueError, IndexError):
            self.get_logger().warn('Invalid IMU data format')
    
    def normalize_angle(self, angle):
        """Normalize angle to [-180, 180] range"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def control_loop(self):
        """Main control loop - runs at 125Hz (same as your 8ms delay)"""
        if not self.is_moving or self.initial_yaw is None:
            return
        
        # Calculate yaw error (same logic as your code)
        # error = totalDegRight - totalDegLeft
        # Here: error = current_yaw - initial_yaw
        error = self.normalize_angle(self.current_yaw - self.initial_yaw)
        
        # Limit error (from your MAX_ERROR = 80.0)
        if error > self.MAX_ERROR:
            error = self.MAX_ERROR
        elif error < -self.MAX_ERROR:
            error = -self.MAX_ERROR
        
        # PID Controller
        dt = 0.008  # 8ms = 0.008 seconds
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term (accumulated error)
        self.integral_error += error * dt
        # Anti-windup: limit integral
        self.integral_error = max(-50, min(50, self.integral_error))
        I = self.Ki * self.integral_error
        
        # Derivative term (rate of change)
        D = self.Kd * (error - self.last_error) / dt
        
        # Total correction
        correction = P + I + D
        
        # Calculate servo PWM values
        # From your code:
        # speedRight = BASE_RIGHT_FWD + (int)correction
        # speedLeft  = BASE_LEFT_FWD - (int)correction
        
        speed_right = self.BASE_RIGHT_FWD + int(correction)
        speed_left = self.BASE_LEFT_FWD - int(correction)
        
        # Constrain to speed limits (from your code)
        speed_right = max(self.SPEED_MIN, min(self.SPEED_MAX, speed_right))
        speed_left = max(self.SPEED_MIN, min(self.SPEED_MAX, speed_left))
        
        # Publish command
        cmd_msg = String()
        cmd_msg.data = f"{speed_left} {speed_right}"
        self.servo_publisher.publish(cmd_msg)
        
        # Log status (every 1 second)
        if self.get_clock().now().nanoseconds % 1000000000 < 8000000:
            self.get_logger().info(
                f'Yaw: {self.current_yaw:6.2f}° | '
                f'Target: {self.initial_yaw:6.2f}° | '
                f'Error: {error:+5.2f}° | '
                f'PWM: L={speed_left:3d} R={speed_right:3d}'
            )
        
        # Update for next iteration
        self.last_error = error
    
    def start_callback(self, request, response):
        """Start straight line movement"""
        if self.is_moving:
            response.success = False
            response.message = 'Already moving!'
            return response
        
        # Reset PID state
        self.initial_yaw = self.current_yaw
        self.integral_error = 0.0
        self.last_error = 0.0
        self.is_moving = True
        
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info(f'STARTING STRAIGHT LINE FROM YAW: {self.initial_yaw:.2f}°')
        self.get_logger().info('='*60)
        
        response.success = True
        response.message = f'Started from yaw: {self.initial_yaw:.2f}°'
        return response
    
    def stop_callback(self, request, response):
        """Stop movement"""
        if not self.is_moving:
            response.success = False
            response.message = 'Not moving!'
            return response
        
        self.is_moving = False
        
        # Send stop command (neutral values)
        stop_msg = String()
        stop_msg.data = f"{self.PWM_NEUTRAL_LEFT} {self.PWM_NEUTRAL_RIGHT}"
        self.servo_publisher.publish(stop_msg)
        
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('STOPPED')
        self.get_logger().info('='*60)
        
        response.success = True
        response.message = 'Stopped'
        return response

def main(args=None):
    rclpy.init(args=args)
    controller = StraightLineController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Emergency stop on shutdown
        stop_msg = String()
        stop_msg.data = f"{controller.PWM_NEUTRAL_LEFT} {controller.PWM_NEUTRAL_RIGHT}"
        controller.servo_publisher.publish(stop_msg)
        
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
