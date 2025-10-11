#!/usr/bin/env python3
# rps_gesture_node.py
# ROS2 node that publishes Rock/Paper/Scissors gestures using MediaPipe.
# Topic: /rps_gesture (std_msgs/String)

import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

mp_hands = mp.solutions.hands
FINGER_TIPS = [4, 8, 12, 16, 20]
FINGER_PIPS = [3, 6, 10, 14, 18]

def count_fingers(hand, handedness_label, w, h, mirrored=True):
    lm = hand.landmark
    def px(i): return int(lm[i].x * w), int(lm[i].y * h)
    fingers = [False]*5

    # Thumb (x-direction heuristic; mirrored selfie view)
    tipx, _ = px(4)
    ipx, _  = px(3)
    if handedness_label == "Right":
        fingers[0] = tipx < ipx if mirrored else tipx > ipx
    else:
        fingers[0] = tipx > ipx if mirrored else tipx < ipx

    # Other fingers: tip above PIP => extended
    for k, (tip_i, pip_i) in enumerate(zip(FINGER_TIPS[1:], FINGER_PIPS[1:]), start=1):
        _, tipy = px(tip_i)
        _, pipy = px(pip_i)
        fingers[k] = tipy < pipy

    return fingers  # [thumb, index, middle, ring, pinky]

def rps_gesture(fingers):
    s = sum(fingers)
    if s == 0:
        return "rock"
    if s == 5:
        return "paper"
    if fingers[1] and fingers[2] and not fingers[0] and not fingers[3] and not fingers[4]:
        return "scissors"
    return "none"

# ADDED: compute pixel-space bounding box area for a hand
def hand_bbox_area(hand_landmarks, w, h):
    xs = [int(pt.x * w) for pt in hand_landmarks.landmark]
    ys = [int(pt.y * h) for pt in hand_landmarks.landmark]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    return max(1, xmax - xmin) * max(1, ymax - ymin)

class RPSGestureNode(Node):
    def __init__(self):
        super().__init__("rps_gesture_node")

        # Declare/get parameters
        self.declare_parameter("device_id", 4)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("min_detection_confidence", 0.6)
        self.declare_parameter("min_tracking_confidence", 0.6)

        self.dev  = int(self.get_parameter("device_id").value)
        self.w    = int(self.get_parameter("width").value)
        self.h    = int(self.get_parameter("height").value)
        self.fps  = int(self.get_parameter("fps").value)
        self.mdc  = float(self.get_parameter("min_detection_confidence").value)
        self.mtc  = float(self.get_parameter("min_tracking_confidence").value)

        # Publisher (publish only on change, but you can republish every frame if desired)
        self.pub = self.create_publisher(String, "/rps_gesture", 10)
        self.last_gesture = None

        # Open camera
        self.cap = cv2.VideoCapture(self.dev, cv2.CAP_V4L2)
        # Prefer MJPG on USB3; drop to YUYV if you must
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        self.cap.set(cv2.CAP_PROP_FPS,          self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

        ok, _ = self.cap.read()
        if not ok:
            self.get_logger().warn(f"Failed to read from /dev/video{self.dev}. "
                                   "Try lowering resolution or switching FOURCC to YUYV.")
        else:
            self.get_logger().info(f"Camera /dev/video{self.dev} opened at {self.w}x{self.h}@{self.fps}")

        # MediaPipe Hands
        self.hands = mp_hands.Hands(
            max_num_hands=2,             # CHANGED: allow multiple hands
            model_complexity=0,
            min_detection_confidence=self.mdc,
            min_tracking_confidence=self.mtc
        )

        # Timer at ~camera FPS
        period = 1.0 / max(self.fps, 1)
        self.timer = self.create_timer(period, self.loop)

    def loop(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Frame grab failed")
            return

        # Selfie view for consistent thumb logic
        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)

        gesture = "none"
        if res.multi_hand_landmarks and res.multi_handedness:
            # ADDED: pick the largest hand by bbox area
            areas = [hand_bbox_area(hand, w, h) for hand in res.multi_hand_landmarks]
            best_idx = max(range(len(areas)), key=lambda i: areas[i])

            hand = res.multi_hand_landmarks[best_idx]
            handed_label = res.multi_handedness[best_idx].classification[0].label  # "Left"/"Right"

            fingers = count_fingers(hand, handed_label, w, h, mirrored=True)
            gesture = rps_gesture(fingers)

        # Publish only on change to reduce chatter
        if (gesture != self.last_gesture) and (gesture != "none"):  # CHANGED: use logical and
            msg = String()
            msg.data = gesture
            self.pub.publish(msg)
            self.last_gesture = gesture

    def destroy_node(self):
        try:
            self.hands.close()
        except Exception:
            pass
        if self.cap:
            self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    node = RPSGestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
