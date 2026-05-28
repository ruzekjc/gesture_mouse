#!/usr/bin/env python3

import math
import time

import cv2
import mediapipe.python.solutions.drawing_utils as mp_draw
import mediapipe.python.solutions.hands as mp_hands
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool

SHOW_PREVIEW = True

MIN_CUTOFF = 0.9            # Lower => smoother (less jitter) at rest, more lag
BETA = 0.5                  # Higher => snappier (less lag) during fast motion
D_CUTOFF = 1.0

SENSITIVITY = 0.3
PUBLISH_RATE_HZ = 30.0

CLICK_COOLDOWN = 1.0
EXTENSION_FACTOR = 1.15
GESTURE_STABILITY_FRAMES = 3

GESTURE_LABELS = {
    'neutral': 'MOVE',
    'left': 'LEFT  (peace = click / hold to drag)',
    'right': 'RIGHT (open hand)',
}
LABEL_COLORS = {
    'neutral': (0, 255, 0),
    'left': (0, 200, 255),
    'right': (255, 120, 0),
}


class OneEuroFilter:
    """Adaptive low-pass filter for noisy pointer signals."""

    def __init__(self, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x, t):
        if self.t_prev is None:
            self.t_prev = t
            self.x_prev = x
            return x
        dt = t - self.t_prev
        if dt <= 0.0:
            dt = 1e-3
        dx = (x - self.x_prev) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        dx_hat = a_d * dx + (1.0 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1.0 - a) * self.x_prev
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t
        return x_hat


def _dist_from_wrist(p, wrist):
    dx = p.x - wrist.x
    dy = p.y - wrist.y
    dz = p.z - wrist.z
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def classify_gesture(lm):
    """Return 'left' (peace), 'right' (open hand), or 'neutral'."""
    wrist = lm[0]

    def extended(tip_idx, pip_idx):
        return (_dist_from_wrist(lm[tip_idx], wrist) >
                _dist_from_wrist(lm[pip_idx], wrist) * EXTENSION_FACTOR)

    middle = extended(12, 10)
    ring = extended(16, 14)
    pinky = extended(20, 18)

    if middle and ring and pinky:
        return 'right'
    if middle and not ring and not pinky:
        return 'left'
    return 'neutral'


class GesturePublisher(Node):
    def __init__(self):
        super().__init__('gesture_publisher')

        self.position_pub = self.create_publisher(Point, '/hand_position', 10)
        self.left_button_pub = self.create_publisher(Bool, '/hand_left_button', 10)
        self.right_click_pub = self.create_publisher(Bool, '/hand_right_click', 10)

        self.detector = mp_hands.Hands(static_image_mode=False, max_num_hands=1)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Could not open webcam (index 0).')
            raise RuntimeError('Webcam unavailable')

        self.filter_x = OneEuroFilter(MIN_CUTOFF, BETA, D_CUTOFF)
        self.filter_y = OneEuroFilter(MIN_CUTOFF, BETA, D_CUTOFF)
        self.smooth_x = 0.5
        self.smooth_y = 0.5

        self.confirmed_gesture = 'neutral'
        self.pending_gesture = 'neutral'
        self.pending_count = 0
        self.last_click_time = 0.0

        period = 1.0 / PUBLISH_RATE_HZ
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(
            'gesture_publisher started at %.1f Hz target. '
            'Left=peace (hold to drag), Right=open hand. Preview=%s'
            % (PUBLISH_RATE_HZ, SHOW_PREVIEW)
        )

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)
        results = self.detector.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        raw_gesture = 'neutral'

        if results.multi_hand_landmarks:
            lm = results.multi_hand_landmarks[0].landmark
            raw_x = lm[8].x
            raw_y = lm[8].y

            margin = (1.0 - SENSITIVITY) / 2.0
            raw_x = (raw_x - margin) / SENSITIVITY
            raw_y = (raw_y - margin) / SENSITIVITY
            raw_x = max(0.0, min(1.0, raw_x))
            raw_y = max(0.0, min(1.0, raw_y))

            t = time.time()
            self.smooth_x = self.filter_x(raw_x, t)
            self.smooth_y = self.filter_y(raw_y, t)

            raw_gesture = classify_gesture(lm)
        if raw_gesture == self.pending_gesture:
            self.pending_count += 1
        else:
            self.pending_gesture = raw_gesture
            self.pending_count = 1

        new_confirmed = self.confirmed_gesture
        if self.pending_count >= GESTURE_STABILITY_FRAMES:
            new_confirmed = self.pending_gesture

        left_button_down = (new_confirmed == 'left')

        right_event = False
        now = time.time()
        if (self.confirmed_gesture == 'neutral'
                and new_confirmed == 'right'
                and (now - self.last_click_time) > CLICK_COOLDOWN):
            right_event = True
            self.last_click_time = now

        self.confirmed_gesture = new_confirmed

        position_msg = Point()
        position_msg.x = float(self.smooth_x)
        position_msg.y = float(self.smooth_y)
        position_msg.z = 0.0
        self.position_pub.publish(position_msg)

        left_msg = Bool()
        left_msg.data = bool(left_button_down)
        self.left_button_pub.publish(left_msg)

        right_msg = Bool()
        right_msg.data = bool(right_event)
        self.right_click_pub.publish(right_msg)

        if right_event:
            self.get_logger().info('Right click published (open hand)')

        if SHOW_PREVIEW:
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_draw.draw_landmarks(
                        frame, hand_landmarks, mp_hands.HAND_CONNECTIONS
                    )
            label = GESTURE_LABELS[self.confirmed_gesture]
            color = LABEL_COLORS[self.confirmed_gesture]
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 50), (0, 0, 0), -1)
            cv2.putText(frame, label, (10, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.imshow('Gesture Mouse - hand tracking', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        try:
            self.cap.release()
            self.detector.close()
            if SHOW_PREVIEW:
                cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GesturePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()