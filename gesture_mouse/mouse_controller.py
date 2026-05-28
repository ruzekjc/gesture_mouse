#!/usr/bin/env python3

import pyautogui
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool

pyautogui.FAILSAFE = False

DRAG_THRESHOLD = 18


class MouseController(Node):
    def __init__(self):
        super().__init__('mouse_controller')

        self.screen_w, self.screen_h = pyautogui.size()
        self.get_logger().info(
            'Screen resolution detected: %dx%d' % (self.screen_w, self.screen_h)
        )

        self.position_sub = self.create_subscription(
            Point, '/hand_position', self.position_callback, 10
        )
        self.left_sub = self.create_subscription(
            Bool, '/hand_left_button', self.left_button_callback, 10
        )
        self.right_sub = self.create_subscription(
            Bool, '/hand_right_click', self.right_click_callback, 10
        )

        # Cursor / drag state
        pos = pyautogui.position()
        self.target = (int(pos[0]), int(pos[1]))  # latest mapped screen target
        self.anchor = self.target                  # where the button went down
        self.left_held = False
        self.dragging = False

    def position_callback(self, msg: Point):
        nx = max(0.0, min(1.0, msg.x))
        ny = max(0.0, min(1.0, msg.y))
        px = int(nx * (self.screen_w - 1))
        py = int((1.0 - ny) * (self.screen_h - 1))  # vertical flip
        self.target = (px, py)

        if not self.left_held:
            # Free movement
            pyautogui.moveTo(px, py, _pause=False)
        elif self.dragging:
            # Drag committed: follow the hand
            pyautogui.moveTo(px, py, _pause=False)
        else:
            dx = px - self.anchor[0]
            dy = py - self.anchor[1]
            if (dx * dx + dy * dy) ** 0.5 > DRAG_THRESHOLD:
                self.dragging = True
                pyautogui.moveTo(px, py, _pause=False)
            # else: do nothing -> cursor stays put

    def left_button_callback(self, msg: Bool):
        if msg.data and not self.left_held:
            self.anchor = self.target
            pyautogui.mouseDown(button='left')
            self.left_held = True
            self.dragging = False
            self.get_logger().info('Left button DOWN')
        elif not msg.data and self.left_held:
            was_drag = self.dragging
            pyautogui.mouseUp(button='left')
            self.left_held = False
            self.dragging = False
            self.get_logger().info(
                'Left button UP (%s)' % ('drag' if was_drag else 'click')
            )

    def right_click_callback(self, msg: Bool):
        if msg.data:
            pyautogui.click(button='right')
            self.get_logger().info('Right click executed')


def main(args=None):
    rclpy.init(args=args)
    node = MouseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()