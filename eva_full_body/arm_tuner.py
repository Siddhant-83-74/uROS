#!/usr/bin/env python3
"""
arm_tuner.py — ROS2 keyboard tuning node for EVA arm gestures

Usage (on Jetson, with micro-ROS agent running):
    python3 arm_tuner.py

Keyboard controls:
    q/a  = Joint 0  (L shoulder pitch)  up/down
    w/s  = Joint 1  (L shoulder roll)   up/down
    e/d  = Joint 2  (L bicep)           up/down
    r/f  = Joint 3  (L elbow)           up/down
    t/g  = Joint 4  (L forearm RMCS)    up/down
    y/h  = Joint 5  (L wrist servo)     up/down
    u/j  = Joint 6  (R shoulder pitch)  up/     
    i/k  = Joint 7  (R shoulder roll)   up/down
    o/l  = Joint 8  (R bicep)           up/down
    p/;  = Joint 9  (R elbow)           up/down
    [/'  = Joint 10 (R forearm RMCS)    up/down
    ]/\\ = Joint 11 (R wrist servo)     up/down

    +/-     = Increase/decrease step size (default 5°)
    SPACE   = Print current pose (C++ array format, ready to paste)
    1-9     = Save current pose as waypoint N
    0       = Print all saved waypoints as a C++ Gesture struct
    z       = Zero all joints
    ESC/x   = Exit
"""

import sys
import curses
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

NUM_JOINTS = 12

JOINT_NAMES = [
    "L ShoulderPitch",  # 0
    "L ShoulderRoll",   # 1
    "L Bicep",          # 2
    "L Elbow",          # 3
    "L Forearm(RMCS)",  # 4
    "L Wrist(Servo)",   # 5
    "R ShoulderPitch",  # 6
    "R ShoulderRoll",   # 7
    "R Bicep",          # 8
    "R Elbow",          # 9
    "R Forearm(RMCS)",  # 10
    "R Wrist(Servo)",   # 11
]

# Key mappings: (increment_key, decrement_key) for each joint
KEY_MAP_INC = "qwertyuiop[]["
KEY_MAP_DEC = "asdfghjkl;'\\"


class ArmTuner(Node):
    def __init__(self):
        super().__init__("arm_tuner")
        self.publisher = self.create_publisher(Float32MultiArray, "joint_targets", 10)
        self.joints = [0.0] * NUM_JOINTS
        self.step_size = 5.0
        self.saved_waypoints = {}  # key: int 1-9, value: list of floats

    def publish_joints(self):
        msg = Float32MultiArray()
        msg.data = self.joints[:]
        self.publisher.publish(msg)

    def format_pose_cpp(self, joints, duration_ms=800):
        vals = ", ".join(f"{v:.1f}" for v in joints)
        return f"{{ .joints = {{{vals}}}, .duration_ms = {duration_ms} }},"


def main(stdscr):
    rclpy.init()
    node = ArmTuner()

    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(50)

    try:
        while True:
            # Spin ROS once
            rclpy.spin_once(node, timeout_sec=0)

            # Draw UI
            stdscr.clear()
            stdscr.addstr(0, 0, "═══ EVA Arm Tuner ═══", curses.A_BOLD)
            stdscr.addstr(1, 0, f"Step size: {node.step_size:.1f}°  (+/- to change)")
            stdscr.addstr(2, 0, "")

            # Joint table header
            stdscr.addstr(3, 0, f" {'#':>2}  {'Joint':<18} {'Angle':>8}  {'Keys':>6}")
            stdscr.addstr(4, 0, "─" * 42)

            for i in range(NUM_JOINTS):
                inc_key = KEY_MAP_INC[i] if i < len(KEY_MAP_INC) else "?"
                dec_key = KEY_MAP_DEC[i] if i < len(KEY_MAP_DEC) else "?"
                line = f" {i:>2}  {JOINT_NAMES[i]:<18} {node.joints[i]:>8.1f}°  {inc_key}/{dec_key}"
                stdscr.addstr(5 + i, 0, line)

            stdscr.addstr(18, 0, "─" * 42)
            stdscr.addstr(19, 0, "SPACE=print pose  1-9=save waypoint  0=print all")
            stdscr.addstr(20, 0, "z=zero all  ESC/x=exit")

            # Saved waypoints indicator
            if node.saved_waypoints:
                wp_str = "Saved: " + ", ".join(str(k) for k in sorted(node.saved_waypoints.keys()))
                stdscr.addstr(22, 0, wp_str)

            stdscr.refresh()

            # Read key
            try:
                key = stdscr.getch()
            except curses.error:
                continue

            if key == -1:
                continue

            ch = chr(key) if 0 <= key < 256 else ""

            # Exit
            if key == 27 or ch == "x":
                break

            # Step size
            if ch == "+" or ch == "=":
                node.step_size = min(node.step_size + 1.0, 45.0)
                continue
            if ch == "-" or ch == "_":
                node.step_size = max(node.step_size - 1.0, 0.5)
                continue

            # Zero all
            if ch == "z":
                node.joints = [0.0] * NUM_JOINTS
                node.publish_joints()
                continue

            # Print current pose
            if ch == " ":
                stdscr.addstr(24, 0, node.format_pose_cpp(node.joints))
                stdscr.refresh()
                curses.napms(2000)
                continue

            # Save waypoint (1-9)
            if ch.isdigit() and ch != "0":
                idx = int(ch)
                node.saved_waypoints[idx] = node.joints[:]
                stdscr.addstr(24, 0, f"Waypoint {idx} saved!")
                stdscr.refresh()
                curses.napms(500)
                continue

            # Print all saved waypoints as C++ gesture
            if ch == "0":
                stdscr.clear()
                stdscr.addstr(0, 0, "═══ Saved Gesture (copy to firmware) ═══", curses.A_BOLD)
                stdscr.addstr(2, 0, "const Gesture MY_GESTURE = {")
                stdscr.addstr(3, 0, "  .frames = {")
                row = 4
                for k in sorted(node.saved_waypoints.keys()):
                    wp = node.saved_waypoints[k]
                    line = "    " + node.format_pose_cpp(wp)
                    if row < curses.LINES - 2:
                        stdscr.addstr(row, 0, line)
                    row += 1
                if row < curses.LINES - 2:
                    stdscr.addstr(row, 0, "  },")
                    stdscr.addstr(row + 1, 0, f"  .count = {len(node.saved_waypoints)},")
                    stdscr.addstr(row + 2, 0, "};")
                    stdscr.addstr(row + 4, 0, "Press any key to return...")
                stdscr.refresh()
                stdscr.nodelay(False)
                stdscr.getch()
                stdscr.nodelay(True)
                continue

            # Joint increment/decrement
            if ch in KEY_MAP_INC:
                idx = KEY_MAP_INC.index(ch)
                if idx < NUM_JOINTS:
                    node.joints[idx] += node.step_size
                    node.publish_joints()
                continue

            if ch in KEY_MAP_DEC:
                idx = KEY_MAP_DEC.index(ch)
                if idx < NUM_JOINTS:
                    node.joints[idx] -= node.step_size
                    node.publish_joints()
                continue

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    curses.wrapper(main)
