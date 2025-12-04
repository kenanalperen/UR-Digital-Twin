#!/usr/bin/env python3
"""
Simple Mouse Tracker - Maps mouse position to robot XZ coordinates
Mouse X [815, 3150] -> Robot X [-800, 800] mm
Mouse Y [980, 1960] -> Robot Z [-350, -1000] mm
Robot Y is always 0.0
Publishes directly to /unity_robot/position_command
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from pynput import mouse
import threading
import time

class MouseToRobotController(Node):
    def __init__(self):
        super().__init__('mouse_to_robot_controller')
        
        # Create publisher for robot position command
        self.robot_pub = self.create_publisher(
            Vector3, 
            '/unity_robot/position_command', 
            10
        )
        
        # Current mouse position in pixels
        self.mouse_x = 0
        self.mouse_y = 0
        
        # Mouse position limits (pixels) - from your specification
        self.mouse_x_min = 815   # Minimum mouse X pixel
        self.mouse_x_max = 3150  # Maximum mouse X pixel
        self.mouse_y_min = 980   # Minimum mouse Y pixel (maps to Z=-350mm)
        self.mouse_y_max = 1960  # Maximum mouse Y pixel (maps to Z=-1000mm)
        
        # Robot position limits (meters)
        self.robot_x_min = -0.800  # -800 mm
        self.robot_x_max = 0.800   #  800 mm
        self.robot_z_min = -0.350  # -350 mm (corresponds to mouse_y_min)
        self.robot_z_max = -1.000  # -1000 mm (corresponds to mouse_y_max)
        self.robot_y_fixed = 0.0   # Always 0.0 as requested
        
        # Mouse controller
        self.mouse_controller = mouse.Controller()
        
        # Thread control
        self.running = True
        
        # Current robot position (for smoothing)
        self.current_robot_x = 0.0
        self.current_robot_z = -0.600  # Start in middle of Z range
        
        # Smoothing factor (0.0 = no smoothing, 1.0 = full smoothing)
        self.smoothing = 0.3
        
        print("=" * 60)
        print("🖱️→🤖 Mouse to Robot Controller")
        print("=" * 60)
        print("Mouse to Robot Mapping:")
        print(f"  Mouse X [{self.mouse_x_min}, {self.mouse_x_max}] → Robot X [{self.robot_x_min*1000:.0f}, {self.robot_x_max*1000:.0f}] mm")
        print(f"  Mouse Y [{self.mouse_y_min}, {self.mouse_y_max}] → Robot Z [{self.robot_z_min*1000:.0f}, {self.robot_z_max*1000:.0f}] mm")
        print(f"  Robot Y: Always {self.robot_y_fixed*1000:.0f} mm")
        print("=" * 60)
        print("Publishing directly to /unity_robot/position_command")
        print("Press Ctrl+C to exit")
        print("=" * 60)
        
        # Start mouse tracking thread
        self.tracking_thread = threading.Thread(target=self.track_mouse)
        self.tracking_thread.start()
        
        # Start publishing timer (20 Hz)
        self.publish_timer = self.create_timer(0.05, self.publish_robot_position)
    
    def track_mouse(self):
        """Continuously track mouse position"""
        while self.running:
            try:
                x, y = self.mouse_controller.position
                self.mouse_x = x
                self.mouse_y = y
                time.sleep(0.01)  # 100 Hz tracking
            except Exception as e:
                self.get_logger().error(f"Error tracking mouse: {e}")
    
    def map_value(self, value, in_min, in_max, out_min, out_max, clamp=True):
        """
        Map a value from one range to another
        If clamp=True, clamp the value to the input range
        """
        if clamp:
            value = max(in_min, min(in_max, value))
        
        # Map the value
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def publish_robot_position(self):
        """Map mouse position to robot position and publish"""
        # Map mouse X to robot X
        robot_x = self.map_value(
            self.mouse_x,
            self.mouse_x_min, self.mouse_x_max,
            self.robot_x_min, self.robot_x_max,
            clamp=True
        )
        
        # Map mouse Y to robot Z (note: Y increases downward on screen, 
        # so higher Y = more negative Z = further back)
        robot_z = self.map_value(
            self.mouse_y,
            self.mouse_y_min, self.mouse_y_max,
            self.robot_z_min, self.robot_z_max,  # This handles the inversion
            clamp=True
        )
        
        # Apply smoothing
        self.current_robot_x = (self.current_robot_x * (1 - self.smoothing) + 
                               robot_x * self.smoothing)
        self.current_robot_z = (self.current_robot_z * (1 - self.smoothing) + 
                               robot_z * self.smoothing)
        
        # Create and publish robot position message
        msg = Vector3()
        msg.x = float(self.current_robot_x)
        msg.y = float(self.robot_y_fixed)  # Always 0.0 as requested
        msg.z = float(self.current_robot_z)
        
        self.robot_pub.publish(msg)
        
        # Print position occasionally (every 0.5 seconds)
        if hasattr(self, 'last_print_time'):
            if time.time() - self.last_print_time > 0.5:
                print(f"🖱️ Mouse: ({self.mouse_x}, {self.mouse_y}) px → "
                      f"🤖 Robot: X={self.current_robot_x*1000:.1f}mm, "
                      f"Z={self.current_robot_z*1000:.1f}mm, "
                      f"Y={self.robot_y_fixed*1000:.0f}mm")
                self.last_print_time = time.time()
        else:
            self.last_print_time = time.time()
    
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        if hasattr(self, 'tracking_thread'):
            self.tracking_thread.join(timeout=1.0)
        self.destroy_node()


def main():
    rclpy.init()
    
    controller = MouseToRobotController()
    
    print("\n✅ Mouse to Robot Controller started!")
    print("Move your mouse to control the robot.")
    print("Mouse X controls robot X position")
    print("Mouse Y controls robot Z position (forward/backward)")
    print("Robot Y is always 0.0")
    print("Press Ctrl+C to exit.\n")
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\n🛑 Stopping controller...")
    finally:
        controller.cleanup()
        rclpy.shutdown()
        print("👋 Goodbye!")


if __name__ == '__main__':
    main()
