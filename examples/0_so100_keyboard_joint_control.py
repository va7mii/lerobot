#!/usr/bin/env python3
"""
Simplified keyboard control for SO100/SO101 robot
Fixed action format conversion issues
Uses P control, keyboard only changes target joint angles
Now uses `evdev` for Linux direct keyboard event reading
"""

import time
import logging
import traceback
import select
import evdev
from evdev import categorize, ecodes

# Replace 'event4' with the event number you found above
device_path = '/dev/input/event4'

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Joint calibration coefficients - manually edit
# Format: [joint_name, zero_position_offset(degrees), scale_factor]
JOINT_CALIBRATION = [
    ['shoulder_pan', 6.0, 1.0],      # Joint1
    ['shoulder_lift', 2.0, 0.97],    # Joint2
    ['elbow_flex', 0.0, 1.05],       # Joint3
    ['wrist_flex', 0.0, 0.94],       # Joint4
    ['wrist_roll', 0.0, 0.5],        # Joint5
    ['gripper', 0.0, 1.0],           # Joint6
]

def apply_joint_calibration(joint_name, raw_position):
    """Apply joint calibration coefficients"""
    for joint_cal in JOINT_CALIBRATION:
        if joint_cal[0] == joint_name:
            offset = joint_cal[1]  
            scale = joint_cal[2]   
            return (raw_position - offset) * scale
    return raw_position

def move_to_ready_position(robot, duration=3.0, kp=0.5):
    """
    Use P control to slowly move robot to a safe, middle-range ready position
    """
    print("Using P control to slowly move robot to a safe ready position...")

    # Safe middle-range targets (avoids hitting the table)
    ready_positions = {
        'shoulder_pan': 0.0,
        'shoulder_lift': -45.0,
        'elbow_flex': 90.0,
        'wrist_flex': -45.0,
        'wrist_roll': 0.0,
        'gripper': 50.0
    }

    control_freq = 50  
    total_steps = int(duration * control_freq)
    step_time = 1.0 / control_freq

    print(f"Moving to ready position in {duration}s using P control (Freq: {control_freq}Hz, Kp: {kp})")

    for step in range(total_steps):
        current_obs = robot.get_observation()
        current_positions = {}
        for key, value in current_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                current_positions[motor_name] = apply_joint_calibration(motor_name, value)

        robot_action = {}
        for joint_name, target_pos in ready_positions.items():
            if joint_name in current_positions:
                current_pos = current_positions[joint_name]
                error = target_pos - current_pos
                robot_action[f"{joint_name}.pos"] = current_pos + (kp * error)

        if robot_action:
            robot.send_action(robot_action)

        if step % (control_freq // 2) == 0:  
            progress = (step / total_steps) * 100
            print(f"Moving to ready position progress: {progress:.1f}%")

        time.sleep(step_time)

    print("Robot is now in the ready position.")

def return_to_start_position(robot, start_positions, kp=0.5, control_freq=50):
    """Use P control to return to the physical position the arm started in"""
    print("Returning to start position...")
    control_period = 1.0 / control_freq
    max_steps = int(5.0 * control_freq)  

    for step in range(max_steps):
        current_obs = robot.get_observation()
        current_positions = {}
        for key, value in current_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                current_positions[motor_name] = value  

        robot_action = {}
        total_error = 0
        for joint_name, target_pos in start_positions.items():
            if joint_name in current_positions:
                current_pos = current_positions[joint_name]
                error = target_pos - current_pos
                total_error += abs(error)
                robot_action[f"{joint_name}.pos"] = current_pos + (kp * error)

        if robot_action:
            robot.send_action(robot_action)

        if total_error < 2.0:  
            print("Returned to start position")
            break

        time.sleep(control_period)

def p_control_loop(robot, keyboard_dev, target_positions, start_positions, kp=0.5, control_freq=50):
    """P control loop using evdev non-blocking reads"""
    control_period = 1.0 / control_freq
    print(f"Starting P control loop (Freq: {control_freq}Hz, Kp: {kp})")

    active_keys = set()

    joint_controls = {
        ecodes.KEY_Q: ('shoulder_pan', -1),     
        ecodes.KEY_A: ('shoulder_pan', 1),      
        ecodes.KEY_W: ('shoulder_lift', -1),    
        ecodes.KEY_S: ('shoulder_lift', 1),     
        ecodes.KEY_E: ('elbow_flex', -1),       
        ecodes.KEY_D: ('elbow_flex', 1),        
        ecodes.KEY_R: ('wrist_flex', -1),       
        ecodes.KEY_F: ('wrist_flex', 1),        
        ecodes.KEY_T: ('wrist_roll', -1),       
        ecodes.KEY_G: ('wrist_roll', 1),        
        ecodes.KEY_Y: ('gripper', -1),          
        ecodes.KEY_H: ('gripper', 1),           
    }

    while True:
        try:
            # Non-blocking read of keyboard events
            r, w, x = select.select([keyboard_dev.fd], [], [], 0.0)
            if r:
                for event in keyboard_dev.read():
                    if event.type == ecodes.EV_KEY:
                        if event.value == 1:  # Key down
                            active_keys.add(event.code)
                        elif event.value == 0:  # Key up
                            active_keys.discard(event.code)

            if active_keys:
                if ecodes.KEY_X in active_keys or ecodes.KEY_ESC in active_keys:
                    print("Exit command detected, returning to start position...")
                    return_to_start_position(robot, start_positions, 0.2, control_freq)
                    return

                for key_code in active_keys:
                    if key_code in joint_controls:
                        joint_name, delta = joint_controls[key_code]
                        if joint_name in target_positions:
                            current_target = target_positions[joint_name]

                            if joint_name == 'gripper':
                                step = delta * 5.0
                                new_target = max(0.0, min(100.0, current_target + step))
                                if current_target != new_target:
                                    print(f"Updated Gripper: {current_target:.1f}% -> {new_target:.1f}%")
                            else:
                                new_target = int(current_target + delta)
                                print(f"Updated joint {joint_name}: {current_target} -> {new_target}°")

                            target_positions[joint_name] = new_target

            # Execute P Control
            current_obs = robot.get_observation()
            current_positions = {}
            for key, value in current_obs.items():
                if key.endswith('.pos'):
                    motor_name = key.removesuffix('.pos')
                    current_positions[motor_name] = apply_joint_calibration(motor_name, value)

            robot_action = {}
            for joint_name, target_pos in target_positions.items():
                if joint_name in current_positions:
                    current_pos = current_positions[joint_name]
                    error = target_pos - current_pos
                    robot_action[f"{joint_name}.pos"] = current_pos + (kp * error)

            if robot_action:
                robot.send_action(robot_action)

            time.sleep(control_period)

        except KeyboardInterrupt:
            print("User interrupted program")
            break
        except Exception as e:
            print(f"P control loop error: {e}")
            traceback.print_exc()
            break

def main():
    print("LeRobot Simplified Keyboard Control Example (P Control + Evdev)")
    print("="*50)
    
    # 1. Initialize the keyboard via evdev
    try:
        keyboard_dev = evdev.InputDevice(device_path)
        print(f"Listening for inputs on: {keyboard_dev.name} ({device_path})...")
    except PermissionError:
        print(f"Permission denied to read {device_path}.")
        print("Run with sudo, or add your user to the 'input' group.")
        return
    except FileNotFoundError:
        print(f"Error: {device_path} not found. Is the keyboard plugged in?")
        return

    try:
        from lerobot.robots.so_follower.so_follower import SO100Follower
        from lerobot.robots.so_follower.config_so_follower import SO100FollowerConfig

        port = input("Please enter SO100 robot USB port (e.g.: /dev/ttyACM0): ").strip()
        if not port:
            port = "/dev/ttyACM0"
            print(f"Using default port: {port}")
        else:
            print(f"Connecting to port: {port}")

        robot_config = SO100FollowerConfig(port=port)
        robot = SO100Follower(robot_config)

        robot.connect()
        print("Robot connected successfully!")

        while True:
            calibrate_choice = input("Do you want to recalibrate the robot? (y/n): ").strip().lower()
            if calibrate_choice in ['y', 'yes']:
                print("Starting recalibration...")
                robot.calibrate()
                print("Calibration completed!")
                break
            elif calibrate_choice in ['n', 'no']:
                print("Using previous calibration file")
                break
            else:
                print("Please enter y or n")

        print("Reading starting joint angles...")
        start_obs = robot.get_observation()
        start_positions = {}
        for key, value in start_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                start_positions[motor_name] = int(value) 

        print("Starting joint angles:")
        for joint_name, position in start_positions.items():
            print(f"  {joint_name}: {position}°")

        # Move to SAFE ready position instead of zero
        move_to_ready_position(robot, duration=3.0)

        # Initialize targets to match the safe ready position
        target_positions = {
            'shoulder_pan': 0.0,
            'shoulder_lift': -45.0,
            'elbow_flex': 90.0,
            'wrist_flex': -45.0,
            'wrist_roll': 0.0,
            'gripper': 50.0
        }

        print("Keyboard control instructions:")
        print("- Q/A: Joint1 (shoulder_pan) decrease/increase")
        print("- W/S: Joint2 (shoulder_lift) decrease/increase")
        print("- E/D: Joint3 (elbow_flex) decrease/increase")
        print("- R/F: Joint4 (wrist_flex) decrease/increase")
        print("- T/G: Joint5 (wrist_roll) decrease/increase")
        print("- Y/H: Joint6 (gripper) decrease/increase")
        print("- X/ESC: Exit program (first return to start position)")
        print("="*50)
        
        p_control_loop(robot, keyboard_dev, target_positions, start_positions, kp=0.5, control_freq=50)

        robot.disconnect()
        print("Program ended")

    except Exception as e:
        print(f"Program execution failed: {e}")
        traceback.print_exc()

if __name__ == "__main__":
    main()