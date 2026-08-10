#!/usr/bin/env python3
"""
aruco_face_detector.py
=======================
訂閱車上兩個 RGB 相機 (/camera/color/image_raw, /camera_right/image_raw)，
用 OpenCV 的 ArUco 偵測找豬臉 marker，並且：
    1. 發布畫好偵測框/ID的除錯影像  -> <camera_topic>/aruco_debug
    2. 發布偵測結果 (JSON字串)      -> /aruco_detections

不需要訓練模型，ArUco 是傳統幾何角點偵測演算法，OpenCV 內建即可用。
字典預設用 DICT_4X4_50 (跟我們產生 aruco_marker.png 時用的字典一致)。

用法:
    ros2 run <你的package> aruco_face_detector.py
    或直接:
    python3 aruco_face_detector.py --ros-args -p cameras:="['/camera/color/image_raw','/camera_right/image_raw']"

可調整參數 (ros2 param)：
    cameras        : 要訂閱的相機 topic 列表 (預設前方+右側兩個RGB相機)
    dictionary     : ArUco 字典名稱 (預設 DICT_4X4_50)
    publish_debug  : 是否發布疊框除錯影像 (預設 true)
    log_interval   : 每隔幾秒印一次統計資訊 (預設 5.0)
"""

import json
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

# 字典名稱字串 -> cv2常數 的對照表
ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


class ArucoFaceDetector(Node):
    def __init__(self):
        super().__init__("aruco_face_detector")

        self.declare_parameter("cameras", ["/camera/color/image_raw", "/camera_right/image_raw"])
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("log_interval", 5.0)

        camera_topics = self.get_parameter("cameras").get_parameter_value().string_array_value
        dict_name = self.get_parameter("dictionary").get_parameter_value().string_value
        self.publish_debug = self.get_parameter("publish_debug").get_parameter_value().bool_value
        self.log_interval = self.get_parameter("log_interval").get_parameter_value().double_value

        if dict_name not in ARUCO_DICTS:
            self.get_logger().warn(f"未知字典 '{dict_name}'，改用預設 DICT_4X4_50")
            dict_name = "DICT_4X4_50"

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])

        # OpenCV 4.7+ 用新式的 ArucoDetector 類別；
        # 舊版 (例如 4.5.x) 只有 detectMarkers() 函式 + DetectorParameters_create()。
        # 這裡自動判斷、兩種版本都相容。
        if hasattr(cv2.aruco, "ArucoDetector"):
            self._api_style = "new"
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self._api_style = "old"
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.detector = None
        self.get_logger().info(f"OpenCV ArUco API 版本: {self._api_style} (cv2 {cv2.__version__})")

        self.bridge = CvBridge()

        # 每個相機各自的統計數字 (幾張frame、幾張偵測到marker)
        self.stats = {topic: {"frames": 0, "hits": 0} for topic in camera_topics}

        self.detection_pub = self.create_publisher(String, "/aruco_detections", 10)

        self.debug_pubs = {}
        for topic in camera_topics:
            self.create_subscription(
                Image, topic, self._make_callback(topic), 10
            )
            if self.publish_debug:
                debug_topic = topic.rstrip("/") + "/aruco_debug"
                self.debug_pubs[topic] = self.create_publisher(Image, debug_topic, 10)
            self.get_logger().info(f"訂閱相機: {topic}")

        self._last_log_time = time.time()

        if self.log_interval > 0:
            self.create_timer(self.log_interval, self._log_stats)

    def _make_callback(self, camera_topic: str):
        def callback(msg: Image):
            self._on_image(camera_topic, msg)
        return callback

    def _on_image(self, camera_topic: str, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"[{camera_topic}] 影像轉換失敗: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._api_style == "new":
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

        self.stats[camera_topic]["frames"] += 1

        if ids is not None and len(ids) > 0:
            self.stats[camera_topic]["hits"] += 1

            detections = []
            for i, marker_id in enumerate(ids.flatten()):
                pts = corners[i][0]  # 4x2
                center = pts.mean(axis=0)
                detections.append(
                    {
                        "id": int(marker_id),
                        "center_px": [float(center[0]), float(center[1])],
                        "corners_px": pts.tolist(),
                    }
                )

            result = {
                "camera": camera_topic,
                "stamp_sec": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                "detections": detections,
            }
            out_msg = String()
            out_msg.data = json.dumps(result)
            self.detection_pub.publish(out_msg)

        if self.publish_debug:
            debug_frame = frame.copy()
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(debug_frame, corners, ids)
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
                debug_msg.header = msg.header
                self.debug_pubs[camera_topic].publish(debug_msg)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"[{camera_topic}] 除錯影像發布失敗: {e}")

    def _log_stats(self):
        parts = []
        for topic, s in self.stats.items():
            frames = s["frames"]
            hits = s["hits"]
            rate = (hits / frames * 100.0) if frames > 0 else 0.0
            parts.append(f"{topic}: {hits}/{frames} ({rate:.1f}%)")
        self.get_logger().info("ArUco 偵測率  " + "  |  ".join(parts))


def main():
    rclpy.init()
    node = ArucoFaceDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()