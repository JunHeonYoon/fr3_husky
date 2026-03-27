#!/usr/bin/env python3
"""
ROS 2 node: global keyboard -> /cmd_vel (Twist)

Logic:
- Each axis is computed from key states only.
- Pressed key contributes +1 or -1.
- Final command = multiplier * axis_state
- Releasing a key immediately updates the command to 0 if no opposite key is pressed.
"""

from typing import Set
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from pynput import keyboard
from pynput.keyboard import Key, KeyCode


class HuskyKeyboardTeleop(Node):
    """
    Publish Twist commands from global keyboard input.

    Input:
        Global keyboard events

    Output:
        geometry_msgs.msg.Twist on /cmd_vel
    """

    def __init__(self) -> None:
        """
        Initialize ROS node, publisher, keyboard listener, and timer.
        """
        super().__init__('husky_keyboard_teleop')

        # Publisher for velocity command
        self.pub = self.create_publisher(Twist, 'fr3_husky/cmd_vel', 10)

        # Command multipliers
        self.vx_multiplier: float = 0.5
        self.w_multiplier: float = 1.0

        # Currently pressed character keys
        self.pressed_keys: Set[str] = set()

        # Current command values
        self.vx: float = 0.0
        self.w: float = 0.0

        # Lock because keyboard callbacks run in another thread
        self.lock = threading.Lock()

        # Publish at fixed rate
        self.timer = self.create_timer(0.02, self.publish_cmd)  # 50 Hz

        # Global keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release,
        )
        self.listener.start()

        self.get_logger().info(
            'Started keyboard teleop: w/s -> vx, a/d -> vy, q/e -> w, esc -> quit'
        )

    def on_press(self, key) -> None:
        """
        Handle key press event.

        Input:
            key: pynput keyboard key event

        Output:
            None
        """
        with self.lock:
            if key == Key.esc:
                self.get_logger().info('ESC pressed. Shutting down.')
                rclpy.shutdown()
                return False

            if isinstance(key, KeyCode) and key.char is not None:
                self.pressed_keys.add(key.char.lower())
                self.update_cmd()

    def on_release(self, key) -> None:
        """
        Handle key release event.

        Input:
            key: pynput keyboard key event

        Output:
            None
        """
        with self.lock:
            if isinstance(key, KeyCode) and key.char is not None:
                self.pressed_keys.discard(key.char.lower())
                self.update_cmd()

    @staticmethod
    def key_to_axis(pos_pressed: bool, neg_pressed: bool) -> int:
        """
        Convert pressed state of a positive/negative key pair into axis state.

        Input:
            pos_pressed (bool): True if positive-direction key is pressed
            neg_pressed (bool): True if negative-direction key is pressed

        Output:
            (int): +1, -1, or 0
        """
        return int(pos_pressed) - int(neg_pressed)

    def update_cmd(self) -> None:
        """
        Update vx, vy, w from current keyboard states.

        Input:
            None

        Output:
            None
        """
        # Axis states: pressed -> 1, released -> 0
        x_state = self.key_to_axis('w' in self.pressed_keys, 's' in self.pressed_keys)
        w_state = self.key_to_axis('a' in self.pressed_keys, 'd' in self.pressed_keys)

        # Final command = multiplier * state
        self.vx = self.vx_multiplier * x_state
        self.w = self.w_multiplier * w_state

    def publish_cmd(self) -> None:
        """
        Publish current Twist command.

        Input:
            None

        Output:
            None
        """
        with self.lock:
            msg = Twist()
            msg.linear.x = self.vx
            msg.angular.z = self.w
            self.pub.publish(msg)


def main() -> None:
    """
    Main entry point.

    Input:
        None

    Output:
        None
    """
    rclpy.init()
    node = HuskyKeyboardTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()