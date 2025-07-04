#!/usr/bin/env python3
"""
Steering Trim Utility for RACECAR NEO

This script allows real-time adjustment of steering PWM values using a gamepad controller.
The adjusted values are saved back to the pwm.py file when the Y button is pressed.

Controls:
- X Button: Trim LEFT (decrease center value and extend left range)
- B Button: Trim RIGHT (increase center value and extend right range)  
- A Button: Print current PWM values or Toggle auto-restart
- Y Button: Save current values to pwm.py
- LEFT TRIGGER: Toggle steering inversion (swap min/max ranges)

The script reads current values from pwm.py and allows fine-tuning with immediate feedback.
"""

import sys
import os
import re
import time
import subprocess
import signal

sys.path.insert(0, "../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Current PWM values (start with defaults from pwm.py)
current_min_range = 3000      # Left edge
current_max_range = 9000      # Right edge  
current_center = 6000         # Center/straight

# Control settings
steering_inverted = False     # Track if steering is inverted
auto_restart_teleop = True    # Whether to automatically restart teleop on changes (always on now)

# State machine for range adjustment
range_mode = "DECREASE"       # "DECREASE" or "INCREASE" - controls what X/B buttons do

# Adjustment increments
RANGE_INCREMENT = 100         # How much to adjust range per button press

# Path to pwm.py file (adjust this path as needed)
PWM_FILE_PATH = "/home/racecar/racecar_ws/src/racecar_neo/racecar_neo/pwm.py"

# Button press tracking to avoid rapid repeated adjustments
last_x_press = 0
last_b_press = 0
last_y_press = 0
last_LT_press = 0
last_a_press = 0
BUTTON_DELAY = 0.2  # Minimum time between button presses

# Live feedback tracking
last_live_feedback = 0
LIVE_FEEDBACK_DELAY = 0.1  # Show live feedback every 100ms max

# Global variables for process management
teleop_process = None
teleop_pid = None

########################################################################################
# Functions
########################################################################################

def start():
    """
    This function is run once every time the start button is pressed
    """
    global current_min_range, current_max_range, current_center, steering_inverted, range_mode
    
    # Stop the car initially
    rc.drive.stop()
    
    # Reset states
    steering_inverted = False
    range_mode = "DECREASE"
    
    # Try to read current values from pwm.py
    read_current_pwm_values()
    
    print("=" * 60)
    print("STEERING TRIM UTILITY - RANGE ADJUSTMENT MODE")
    print("=" * 60)
    print()
    print("Reading current values from pwm.py...")
    print()
    print("CONTROLS:")
    print("  X Button: Adjust LEFT range")
    print("  B Button: Adjust RIGHT range") 
    print("  A Button: Toggle DECREASE/INCREASE mode")
    print("  Y Button: Save to pwm.py")
    print("  LEFT TRIGGER: Toggle steering inversion")
    print()
    print(f" Current Mode: {range_mode} RANGE")
    if range_mode == "DECREASE":
        print("   X = Decrease left range (make narrower)")
        print("   B = Decrease right range (make narrower)")
    else:
        print("   X = Increase left range (make wider)")
        print("   B = Increase right range (make wider)")
    print()
    print(" Live steering feedback enabled!")
    print("   Move RIGHT stick to see live PWM values")
    print()
    print(" Auto-restart teleop: ON (always enabled)")
    print()
    print("Press CTRL+C to exit without saving")
    print("=" * 60)
    print()

def update():
    """
    Main update loop - check for button presses and adjust values
    """
    global current_min_range, current_max_range, current_center, steering_inverted, range_mode
    global last_x_press, last_b_press, last_y_press, last_LT_press, last_live_feedback, last_a_press
    
    current_time = time.time()
    
    # Show live steering feedback using controller input
    show_live_steering_feedback(current_time)
    
    # LEFT Trigger: Toggle steering inversion
    if rc.controller.get_trigger(rc.controller.Trigger.LEFT) > 0.5 and (current_time - last_LT_press) > BUTTON_DELAY:
        last_LT_press = current_time
        toggle_steering_inversion()
    
    # X Button: Adjust left range based on current mode
    if rc.controller.is_down(rc.controller.Button.X) and (current_time - last_x_press) > BUTTON_DELAY:
        last_x_press = current_time
        adjust_left_range()
    
    # B Button: Adjust right range based on current mode
    if rc.controller.is_down(rc.controller.Button.B) and (current_time - last_b_press) > BUTTON_DELAY:
        last_b_press = current_time
        adjust_right_range()
    
    # A Button: Toggle range adjustment mode
    if rc.controller.is_down(rc.controller.Button.A) and (current_time - last_a_press) > BUTTON_DELAY:
        last_a_press = current_time
        toggle_range_mode()
    
    # Y Button: Save to pwm.py
    if rc.controller.is_down(rc.controller.Button.Y) and (current_time - last_y_press) > BUTTON_DELAY:
        last_y_press = current_time
        save_to_pwm_file()
    
    # Keep the car stationary but apply steering for testing
    rc.drive.set_speed_angle(0, 0)

def show_live_steering_feedback(current_time):
    """Show live steering feedback using controller joystick"""
    global last_live_feedback
    
    # Only show feedback periodically to avoid spam
    if current_time - last_live_feedback < LIVE_FEEDBACK_DELAY:
        return
    
    # Get steering input from left joystick X-axis
    steering_input, _ = rc.controller.get_joystick(rc.controller.Joystick.RIGHT)
    
    # Only show if there's significant steering input
    if abs(steering_input) > 0.1:
        last_live_feedback = current_time
        
        # Convert to PWM value based on current range
        # Note: The inversion is now handled in pwm.py mapping, so we always use normal calculation here
        if steering_input < 0:  # Left input
            pwm_value = current_center - abs(steering_input) * (current_center - current_min_range)
        else:  # Right input
            pwm_value = current_center + steering_input * (current_max_range - current_center)
        
        direction = "LEFT" if steering_input < 0 else "RIGHT"
        invert_status = " [INVERTED]" if steering_inverted else ""
        print(f"Live Steering: {direction} Input={steering_input:.2f} -> PWM={pwm_value:.0f}{invert_status}")

def toggle_steering_inversion():
    """Toggle steering inversion by modifying the map_val line in pwm.py"""
    global steering_inverted
    
    steering_inverted = not steering_inverted
    
    # Apply the inversion change to pwm.py immediately
    apply_inversion_to_pwm_file()
    
    print("\n" + "=" * 50)
    if steering_inverted:
        print("STEERING INVERTED!")
        print("  Changed mapping: map_val(..., 4000, 8000)")
        print("  LEFT input now goes RIGHT")
        print("  RIGHT input now goes LEFT")
    else:
        print("STEERING NORMAL")
        print("  Restored mapping: map_val(..., 8000, 4000)")
        print("  LEFT input goes LEFT")
        print("  RIGHT input goes RIGHT")
    print("=" * 50 + "\n")
    
    # Restart teleop if auto-restart is enabled
    if auto_restart_teleop:
        restart_teleop()

def apply_inversion_to_pwm_file():
    """Apply the current inversion state to the pwm.py file"""
    if not os.path.exists(PWM_FILE_PATH):
        print(f"Error: Could not find {PWM_FILE_PATH}")
        return
    
    try:
        # Read the current file
        with open(PWM_FILE_PATH, 'r') as f:
            content = f.read()
        
        if steering_inverted:
            # Change from 8000,4000 to 4000,8000 (inverted)
            content = re.sub(
                r'map_val\(msg\.drive\.steering_angle,\s*-CAR_MAX_TURN,\s*CAR_MAX_TURN,\s*8000,\s*4000\)',
                'map_val(msg.drive.steering_angle, -CAR_MAX_TURN, CAR_MAX_TURN, 4000, 8000)',
                content
            )
        else:
            # Change from 4000,8000 to 8000,4000 (normal)
            content = re.sub(
                r'map_val\(msg\.drive\.steering_angle,\s*-CAR_MAX_TURN,\s*CAR_MAX_TURN,\s*4000,\s*8000\)',
                'map_val(msg.drive.steering_angle, -CAR_MAX_TURN, CAR_MAX_TURN, 8000, 4000)',
                content
            )
        
        # Write back to file
        with open(PWM_FILE_PATH, 'w') as f:
            f.write(content)
        
        print(f"Inversion applied to {PWM_FILE_PATH}")
        
    except Exception as e:
        print(f"Error applying inversion to {PWM_FILE_PATH}: {e}")

def adjust_left_range():
    """Adjust left range based on current mode"""
    global current_min_range, range_mode
    
    if range_mode == "DECREASE":
        # Decrease left range (make narrower - increase min value)
        current_min_range += RANGE_INCREMENT
        current_min_range = min(current_min_range, current_center - 500)  # Keep away from center
        action = "DECREASED"
        direction = "narrower"
    else:  # INCREASE mode
        # Increase left range (make wider - decrease min value)
        current_min_range -= RANGE_INCREMENT
        current_min_range = max(current_min_range, 1000)  # Keep reasonable minimum
        action = "INCREASED"
        direction = "wider"
    
    invert_status = " [INVERTED]" if steering_inverted else ""
    print(f"[X] {action} LEFT RANGE{invert_status}: {current_min_range} ({direction})")
    print(f"    Full range now: {current_min_range} to {current_max_range}")

def adjust_right_range():
    """Adjust right range based on current mode"""
    global current_max_range, range_mode
    
    if range_mode == "DECREASE":
        # Decrease right range (make narrower - decrease max value)
        current_max_range -= RANGE_INCREMENT
        current_max_range = max(current_max_range, current_center + 500)  # Keep away from center
        action = "DECREASED"
        direction = "narrower"
    else:  # INCREASE mode
        # Increase right range (make wider - increase max value)
        current_max_range += RANGE_INCREMENT
        current_max_range = min(current_max_range, 10000)  # Keep reasonable maximum
        action = "INCREASED"
        direction = "wider"
    
    invert_status = " [INVERTED]" if steering_inverted else ""
    print(f"[B] {action} RIGHT RANGE{invert_status}: {current_max_range} ({direction})")
    print(f"    Full range now: {current_min_range} to {current_max_range}")

def toggle_range_mode():
    """Toggle between DECREASE and INCREASE range modes"""
    global range_mode
    
    range_mode = "INCREASE" if range_mode == "DECREASE" else "DECREASE"
    
    print(f"\nRANGE MODE: {range_mode}")
    if range_mode == "DECREASE":
        print("   X Button: Decrease left range (make narrower)")
        print("   B Button: Decrease right range (make narrower)")
    else:
        print("   X Button: Increase left range (make wider)")
        print("   B Button: Increase right range (make wider)")
    print()

def print_current_values():
    """Print current PWM values"""
    print("\n" + "=" * 50)
    print("CURRENT PWM VALUES:")
    print(f"  Center (straight): {current_center}")
    print(f"  Range: setRange(1,{current_min_range},{current_max_range})")
    print(f"  Total Range Width: {abs(current_max_range - current_min_range)}")
    print(f"  Left Range: {current_center - current_min_range}")
    print(f"  Right Range: {current_max_range - current_center}")
    print(f"  Steering Inverted: {steering_inverted}")
    if steering_inverted:
        print("  LEFT input -> RIGHT turn")
        print("  RIGHT input -> LEFT turn")
    else:
        print("  LEFT input -> LEFT turn")
        print("  RIGHT input -> RIGHT turn")
    print(f"  Range Mode: {range_mode}")
    if range_mode == "DECREASE":
        print("    X/B buttons will make ranges narrower")
    else:
        print("    X/B buttons will make ranges wider")
    print(f"  Teleop status: {check_teleop_status()}")
    print()
    print("PROCESS MANAGEMENT:")
    print("  To kill teleop manually:")
    if teleop_pid:
        print(f"    kill {teleop_pid}")
    print("    pkill -f 'ros2 launch'")
    print("    pkill -f teleop")
    print("    killall -9 python3  # (nuclear option)")
    print("  To restart teleop manually:")
    print("    ros2 launch racecar_neo teleop.launch.py")
    print("=" * 50 + "\n")
    
    # Verify the changes are actually in the file
    verify_pwm_changes()

def read_current_pwm_values():
    """Read current values from pwm.py file"""
    global current_min_range, current_max_range, current_center
    
    if not os.path.exists(PWM_FILE_PATH):
        print(f"Warning: Could not find {PWM_FILE_PATH}")
        print("Using default values")
        return
    
    try:
        with open(PWM_FILE_PATH, 'r') as f:
            content = f.read()
        
        # Extract current values using regex
        range_match = re.search(r'controller\.setRange\(1,(\d+),(\d+)\)', content)
        target_match = re.search(r'controller\.setTarget\(1,(\d+)\)', content)
        
        if range_match:
            current_min_range = int(range_match.group(1))
            current_max_range = int(range_match.group(2))
        
        if target_match:
            current_center = int(target_match.group(1))
        
        print(f"Read from {PWM_FILE_PATH}: Center={current_center}, Range=({current_min_range}, {current_max_range})")
        
    except Exception as e:
        print(f"Error reading {PWM_FILE_PATH}: {e}")
        print("Using default values")

def save_to_pwm_file():
    """Save current values back to pwm.py file"""
    if not os.path.exists(PWM_FILE_PATH):
        print(f"Error: Could not find {PWM_FILE_PATH}")
        return
    
    try:
        # Read the current file
        with open(PWM_FILE_PATH, 'r') as f:
            content = f.read()
        
        # Replace the setRange line (steering is channel 1)
        content = re.sub(
            r'controller\.setRange\(1,\d+,\d+\)',
            f'controller.setRange(1,{current_min_range},{current_max_range})',
            content
        )
        
        # Write back to file
        with open(PWM_FILE_PATH, 'w') as f:
            f.write(content)
        
        print("\n" + "=" * 50)
        print("SUCCESS: Range values saved to pwm.py!")
        print(f"  setRange(1,{current_min_range},{current_max_range})")
        print(f"  Range Width: {abs(current_max_range - current_min_range)}")
        print(f"  Left Range: {current_center - current_min_range}")
        print(f"  Right Range: {current_max_range - current_center}")
        invert_status = " [INVERTED]" if steering_inverted else ""
        print(f"  Steering{invert_status}")
        if steering_inverted:
            print("  Mapping: map_val(..., 4000, 8000)")
        else:
            print("  Mapping: map_val(..., 8000, 4000)")
        print("=" * 50 + "\n")
        
        # Always restart teleop (auto-restart is always on)
        restart_teleop()
        
    except Exception as e:
        print(f"Error saving to {PWM_FILE_PATH}: {e}")

def restart_teleop():
    """Restart the teleop launch file to apply pwm.py changes"""
    global teleop_process, teleop_pid
    
    print("\nRestarting teleop to apply changes...")
    
    try:
        # Kill existing teleop processes (both our spawned one and any others)
        kill_teleop_processes()
        
        # Extra delay to ensure all processes are fully dead
        print("   Waiting for complete shutdown...")
        time.sleep(2)
        
        # Kill any remaining ROS2 nodes that might be holding onto the old pwm.py
        print("   Cleaning up any remaining ROS2 processes...")
        subprocess.run(["pkill", "-f", "ros2"], capture_output=True, check=False)
        subprocess.run(["pkill", "-f", "racecar_neo"], capture_output=True, check=False)
        
        # Another delay to let everything fully die
        time.sleep(1)
        
        # Start teleop in background
        print("   Starting new teleop process...")
        teleop_process = subprocess.Popen(
            ["bash", "-c", "ros2 launch racecar_neo teleop.launch.py"], 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )
        
        teleop_pid = teleop_process.pid
        
        # Give it time to start up
        time.sleep(3)
        
        print("Teleop restarted successfully!")
        print(f"   Process ID: {teleop_pid}")
        print("   PWM changes should now be active")
        print(f"   To kill manually: kill {teleop_pid}")
        print("   Or use: pkill -f 'ros2 launch'")
        print("   Or use: pkill -f teleop")
        
        # Verify the process is still running
        if teleop_process.poll() is None:
            print("   Teleop process confirmed running")
        else:
            print("   Warning: Teleop process may have failed to start")
            print("   Check manually with: teleop")
        
    except Exception as e:
        print(f"Error restarting teleop: {e}")
        print("   You may need to restart manually:")
        print("   ros2 launch racecar_neo teleop.launch.py")

def kill_teleop_processes():
    """Kill all existing teleop processes and related ROS2 nodes"""
    global teleop_process, teleop_pid
    
    try:
        # Kill our spawned process first
        if teleop_process and teleop_process.poll() is None:
            print(f"   Killing our teleop process (PID: {teleop_pid})...")
            try:
                os.killpg(os.getpgid(teleop_process.pid), signal.SIGTERM)
                time.sleep(1)
                
                # Force kill if still running
                if teleop_process.poll() is None:
                    print("   Force killing stubborn process...")
                    os.killpg(os.getpgid(teleop_process.pid), signal.SIGKILL)
                    time.sleep(1)
            except ProcessLookupError:
                pass  # Process already dead
        
        # Kill any other teleop/launch processes
        kill_commands = [
            ["pkill", "-f", "ros2 launch racecar_neo teleop"],
            ["pkill", "-f", "teleop.launch.py"],
            ["pkill", "-f", "pwm_node"],
            ["pkill", "-f", "mux_node"],
            ["pkill", "-f", "gamepad_node"],
            ["pkill", "-f", "throttle_node"]
        ]
        
        for cmd in kill_commands:
            result = subprocess.run(cmd, capture_output=True, text=True, check=False)
            if result.returncode == 0:
                print(f"   Killed processes: {' '.join(cmd[2:])}")
        
        # Final cleanup - kill any remaining ROS2 launch processes
        subprocess.run(["pkill", "-f", "launch"], capture_output=True, check=False)
        
        # Wait for everything to die
        time.sleep(1)
        
        # Reset our tracking variables
        teleop_process = None
        teleop_pid = None
        
        print("   All teleop processes killed")
        
    except Exception as e:
        print(f"   Warning: Error killing teleop processes: {e}")

def verify_pwm_changes():
    """Verify that the current pwm.py file has our expected changes"""
    try:
        with open(PWM_FILE_PATH, 'r') as f:
            content = f.read()
        
        # Check if our range values are in the file
        if f"setRange(1,{current_min_range},{current_max_range})" in content:
            print(f"   Range values confirmed in pwm.py: {current_min_range}-{current_max_range}")
        else:
            print(f"   Range values may not be saved correctly")
        
        # Check if our center value is in the file  
        if f"setTarget(1,{current_center})" in content:
            print(f"   Center value confirmed in pwm.py: {current_center}")
        else:
            print(f"   Center value may not be saved correctly")
            
        # Check inversion state
        if steering_inverted:
            if "map_val(msg.drive.steering_angle, -CAR_MAX_TURN, CAR_MAX_TURN, 4000, 8000)" in content:
                print("   Steering inversion confirmed: 4000,8000 (inverted)")
            else:
                print("   Steering inversion may not be applied correctly")
        else:
            if "map_val(msg.drive.steering_angle, -CAR_MAX_TURN, CAR_MAX_TURN, 8000, 4000)" in content:
                print("   Steering normal confirmed: 8000,4000 (normal)")
            else:
                print("   Steering normal may not be applied correctly")
                
    except Exception as e:
        print(f"   Could not verify pwm.py changes: {e}")

def check_teleop_status():
    """Check if our teleop process is still running"""
    global teleop_process, teleop_pid
    
    if teleop_process is None:
        return "No teleop process tracked"
    
    if teleop_process.poll() is None:
        return f"Teleop running (PID: {teleop_pid})"
    else:
        return f"Teleop process died (was PID: {teleop_pid})"

# Add cleanup function for when script exits
def cleanup_on_exit():
    """Clean up spawned processes when script exits"""
    global teleop_process
    
    print("\nCleaning up spawned processes...")
    
    if teleop_process and teleop_process.poll() is None:
        print(f"   Keeping teleop running (PID: {teleop_pid})")
        print(f"   To kill later: kill {teleop_pid} or pkill -f teleop")
    
    print("   Steering trim script exiting...")

# Register cleanup function
import atexit
atexit.register(cleanup_on_exit)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go() 