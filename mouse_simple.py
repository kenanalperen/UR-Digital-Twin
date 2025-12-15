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
import threading
import time
import math
import sys

# Try multiple mouse tracking methods
try:
    import pyautogui
    MOUSE_METHOD = "pyautogui"
    print("✅ Using pyautogui for mouse tracking")
except ImportError:
    try:
        import pynput.mouse
        MOUSE_METHOD = "pynput"
        print("✅ Using pynput for mouse tracking")
    except ImportError:
        try:
            # Try tkinter for basic mouse position
            import tkinter as tk
            MOUSE_METHOD = "tkinter"
            print("✅ Using tkinter for mouse tracking")
        except ImportError:
            MOUSE_METHOD = "manual"
            print("⚠️ No mouse tracking library found. Will use manual input.")

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
        
        # Screen dimensions (will be auto-detected or set manually)
        self.screen_width = 0
        self.screen_height = 0
        
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
        
        # Thread control
        self.running = True
        
        # Current robot position
        self.current_robot_x = 0.0
        self.current_robot_z = -0.600  # Start in middle of Z range
        
        # Previous position for velocity calculation
        self.prev_robot_x = 0.0
        self.prev_robot_z = -0.600
        
        # Target robot position (from direct mapping)
        self.target_robot_x = 0.0
        self.target_robot_z = -0.600
        
        # Time tracking for velocity calculation
        self.last_update_time = time.time()
        
        # Debug flag
        self.debug_mouse_raw = True  # Set to True to see raw mouse coordinates
        
        # ==================== VELOCITY LIMITING SETTINGS ====================
        # Maximum speed in m/s (adjust based on your robot's capabilities)
        self.max_speed_mps = 0.4  # 500 mm/s maximum speed
        
        # Maximum speed in mm/s (for display)
        self.max_speed_mmps = self.max_speed_mps * 1000
        
        # Deadzone to prevent jitter from tiny movements (in meters)
        self.deadzone_m = 0.001  # 1 mm deadzone
        
        # Optional: Separate speeds for X and Z axes
        self.use_separate_speeds = False
        self.max_speed_x_mps = 0.5  # X axis max speed
        self.max_speed_z_mps = 0.5  # Z axis max speed
        
        # Smoothing factor for velocity calculation
        self.velocity_smoothing = 0.1
        
        # Current velocity (for monitoring)
        self.current_velocity_x = 0.0
        self.current_velocity_z = 0.0
        self.current_speed = 0.0
        # ===================================================================
        
        # Increase publishing rate for smooth velocity control
        self.publish_rate_hz = 30  # 30 Hz should be smooth
        self.dt = 1.0 / self.publish_rate_hz  # Time step
        
        # Initialize mouse tracking
        self.init_mouse_tracking()
        
        print("=" * 70)
        print("🖱️→🤖 Mouse to Robot Controller")
        print("=" * 70)
        print(f"Mouse tracking method: {MOUSE_METHOD}")
        print(f"Screen size: {self.screen_width} x {self.screen_height}")
        print("\nMouse to Robot Mapping:")
        print(f"  Mouse X [{self.mouse_x_min}, {self.mouse_x_max}] → Robot X [{self.robot_x_min*1000:.0f}, {self.robot_x_max*1000:.0f}] mm")
        print(f"  Mouse Y [{self.mouse_y_min}, {self.mouse_y_max}] → Robot Z [{self.robot_z_min*1000:.0f}, {self.robot_z_max*1000:.0f}] mm")
        print(f"  Robot Y: Always {self.robot_y_fixed*1000:.0f} mm")
        print("=" * 70)
        print(f"Maximum Speed: {self.max_speed_mps*1000:.0f} mm/s ({self.max_speed_mps:.1f} m/s)")
        print(f"Deadzone: {self.deadzone_m*1000:.1f} mm")
        print(f"Publishing rate: {self.publish_rate_hz} Hz")
        print("Publishing directly to /unity_robot/position_command")
        print("Press Ctrl+C to exit")
        print("=" * 70)
        
        # Start mouse tracking thread
        self.tracking_thread = threading.Thread(target=self.track_mouse)
        self.tracking_thread.daemon = True
        self.tracking_thread.start()
        
        # Start publishing timer
        publish_interval = 1.0 / self.publish_rate_hz
        self.publish_timer = self.create_timer(publish_interval, self.publish_robot_position)
        
        # For print timing
        self.last_print_time = time.time()
        
        # Track mouse position validity
        self.mouse_position_valid = False
    
    def init_mouse_tracking(self):
        """Initialize mouse tracking based on available method"""
        if MOUSE_METHOD == "pyautogui":
            # Get screen size
            self.screen_width, self.screen_height = pyautogui.size()
            
        elif MOUSE_METHOD == "pynput":
            self.mouse_controller = pynput.mouse.Controller()
            # Try to get screen size (this is a bit hacky for pynput)
            # You might need to adjust this based on your system
            import ctypes
            user32 = ctypes.windll.user32 if sys.platform == "win32" else None
            if user32:
                self.screen_width = user32.GetSystemMetrics(0)
                self.screen_height = user32.GetSystemMetrics(1)
            else:
                # Default to your expected range
                self.screen_width = 3840  # Common 4K width
                self.screen_height = 2160  # Common 4K height
                
        elif MOUSE_METHOD == "tkinter":
            # Create a hidden window to get screen info
            root = tk.Tk()
            self.screen_width = root.winfo_screenwidth()
            self.screen_height = root.winfo_screenheight()
            root.destroy()
            
        else:
            # Manual mode - set screen size manually
            self.screen_width = 3840
            self.screen_height = 2160
            print("⚠️ Manual mode: Using default screen size 3840x2160")
        
        # Update mouse limits based on screen size
        print(f"Detected screen: {self.screen_width} x {self.screen_height}")
        
        # Check if your specified ranges are within screen bounds
        if (self.mouse_x_max > self.screen_width or 
            self.mouse_y_max > self.screen_height):
            print(f"⚠️ Warning: Your mouse range [{self.mouse_x_min}-{self.mouse_x_max}, "
                  f"{self.mouse_y_min}-{self.mouse_y_max}] exceeds screen size!")
            print("Consider adjusting mouse limits or screen resolution.")
    
    def track_mouse(self):
        """Continuously track mouse position"""
        print("Starting mouse tracking...")
        
        while self.running:
            try:
                if MOUSE_METHOD == "pyautogui":
                    x, y = pyautogui.position()
                    
                elif MOUSE_METHOD == "pynput":
                    x, y = self.mouse_controller.position
                    
                elif MOUSE_METHOD == "tkinter":
                    # Tkinter needs a window for continuous tracking
                    # We'll just use pyautogui or pynput for continuous
                    time.sleep(0.01)
                    continue
                    
                else:  # Manual mode
                    # For testing without mouse
                    x = self.screen_width // 2
                    y = self.screen_height // 2
                
                # Store mouse position
                self.mouse_x = x
                self.mouse_y = y
                self.mouse_position_valid = True
                
                # DEBUG: Print raw mouse position occasionally
                if self.debug_mouse_raw and time.time() - getattr(self, 'last_raw_print', 0) > 2.0:
                    print(f"[DEBUG] Raw mouse: ({x}, {y})")
                    self.last_raw_print = time.time()
                
                time.sleep(0.005)  # Fast tracking
                
            except Exception as e:
                self.get_logger().error(f"Error tracking mouse: {e}")
                self.mouse_position_valid = False
                time.sleep(0.1)
    
    def map_value(self, value, in_min, in_max, out_min, out_max, clamp=True):
        """
        Map a value from one range to another
        If clamp=True, clamp the value to the input range
        """
        if clamp:
            value = max(in_min, min(in_max, value))
        
        # Map the value
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def limit_velocity(self, target_x, target_z):
        """
        Move towards target position while respecting maximum speed limits
        Returns: (new_x, new_z) position after applying velocity limits
        """
        # Calculate desired movement vector
        dx = target_x - self.current_robot_x
        dz = target_z - self.current_robot_z
        
        # Calculate distance to target
        distance = math.sqrt(dx*dx + dz*dz)
        
        # Check if within deadzone
        if distance < self.deadzone_m:
            return self.current_robot_x, self.current_robot_z
        
        # Calculate maximum allowed movement this time step
        max_movement = self.max_speed_mps * self.dt
        
        if distance <= max_movement:
            # Can reach target in one step
            new_x = target_x
            new_z = target_z
        else:
            # Move towards target at maximum speed
            ratio = max_movement / distance
            new_x = self.current_robot_x + dx * ratio
            new_z = self.current_robot_z + dz * ratio
        
        return new_x, new_z
    
    def calculate_velocity(self, new_x, new_z):
        """Calculate current velocity based on position change"""
        current_time = time.time()
        dt_actual = current_time - self.last_update_time
        
        if dt_actual > 0:
            # Calculate instantaneous velocity
            vx_inst = (new_x - self.prev_robot_x) / dt_actual
            vz_inst = (new_z - self.prev_robot_z) / dt_actual
            
            # Apply smoothing to velocity readings
            self.current_velocity_x = (self.current_velocity_x * (1 - self.velocity_smoothing) + 
                                      vx_inst * self.velocity_smoothing)
            self.current_velocity_z = (self.current_velocity_z * (1 - self.velocity_smoothing) + 
                                      vz_inst * self.velocity_smoothing)
            
            # Calculate speed magnitude
            self.current_speed = math.sqrt(self.current_velocity_x**2 + self.current_velocity_z**2)
        
        # Update previous positions and time
        self.prev_robot_x = new_x
        self.prev_robot_z = new_z
        self.last_update_time = current_time
    
    def publish_robot_position(self):
        """Map mouse position to robot position and publish"""
        # Check if mouse position is valid
        if not self.mouse_position_valid:
            print("⚠️ Mouse position not available. Using last known position.")
            return
        
        # Direct map mouse to target robot positions
        target_x = self.map_value(
            self.mouse_x,
            self.mouse_x_min, self.mouse_x_max,
            self.robot_x_min, self.robot_x_max,
            clamp=True
        )
        
        target_z = self.map_value(
            self.mouse_y,
            self.mouse_y_min, self.mouse_y_max,
            self.robot_z_min, self.robot_z_max,
            clamp=True
        )
        
        # Store target positions
        self.target_robot_x = target_x
        self.target_robot_z = target_z
        
        # Apply velocity limiting
        new_x, new_z = self.limit_velocity(target_x, target_z)
        
        # Update current position
        self.current_robot_x = new_x
        self.current_robot_z = new_z
        
        # Calculate velocity
        self.calculate_velocity(new_x, new_z)
        
        # Create and publish robot position message
        msg = Vector3()
        msg.x = float(self.current_robot_x)
        msg.y = float(self.robot_y_fixed)  # Always 0.0 as requested
        msg.z = float(self.current_robot_z)
        
        self.robot_pub.publish(msg)
        
        # Print position and velocity occasionally
        current_time = time.time()
        if current_time - self.last_print_time > 0.3:  # Every 0.3 seconds
            # Check if mouse is at (0,0) - this is usually wrong
            if self.mouse_x == 0 and self.mouse_y == 0:
                print("⚠️ WARNING: Mouse is at (0,0)! This might indicate:")
                print("  1. Mouse tracking isn't working")
                print("  2. Mouse is actually at top-left corner")
                print("  3. Screen coordinates are different than expected")
                print(f"  Screen size: {self.screen_width}x{self.screen_height}")
            
            # Calculate remaining distance to target
            dx_remaining = target_x - self.current_robot_x
            dz_remaining = target_z - self.current_robot_z
            distance_remaining = math.sqrt(dx_remaining**2 + dz_remaining**2) * 1000  # in mm
            
            print(f"🖱️ Mouse: ({self.mouse_x:4d}, {self.mouse_y:4d}) px")
            print(f"  Screen %: X={self.mouse_x/self.screen_width*100:.1f}%, Y={self.mouse_y/self.screen_height*100:.1f}%")
            print(f"  Target:  X={target_x*1000:6.1f}mm, Z={target_z*1000:6.1f}mm")
            print(f"  Current: X={self.current_robot_x*1000:6.1f}mm, Z={self.current_robot_z*1000:6.1f}mm")
            print(f"  Remaining: {distance_remaining:5.1f}mm")
            print(f"  Velocity: X={self.current_velocity_x*1000:5.0f}mm/s, Z={self.current_velocity_z*1000:5.0f}mm/s")
            print(f"  Speed: {self.current_speed*1000:5.0f}mm/s (Max: {self.max_speed_mmps:.0f}mm/s)")
            print("-" * 60)
            self.last_print_time = current_time
    
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
    print(f"Maximum speed: {controller.max_speed_mps*1000:.0f} mm/s")
    print("Robot Y is always 0.0")
    print("Press Ctrl+C to exit.\n")
    
    # Quick test of mouse position
    print("Quick mouse position test:")
    print(f"Current mouse: ({controller.mouse_x}, {controller.mouse_y})")
    print(f"Screen size: {controller.screen_width}x{controller.screen_height}")
    print("\n")
    
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
