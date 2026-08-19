#!/usr/bin/env python3

"""Collect manually clicked map landmarks without modifying the semantic graph."""

from __future__ import annotations

import math
import os
from pathlib import Path
import re
import signal
import sys
import tempfile

import yaml

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QVBoxLayout,
)

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from visualization_msgs.msg import Marker, MarkerArray


SCHEMA_VERSION = 1
MAXIMUM_TEXT_LENGTH = 64
IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+$")
MANUAL_ID_PATTERN = re.compile(r"^landmark_manual_(\d+)$")


class ManualLandmarkError(ValueError):
    """Raised when a draft landmark file or user entry is invalid."""


def normalized_identifier(value: str, description: str) -> str:
    result = str(value).strip()
    if not result or not IDENTIFIER_PATTERN.fullmatch(result):
        raise ManualLandmarkError(
            f"{description}只能包含字母、数字、下划线、短横线和点"
        )
    return result


def normalized_text(value: str, description: str) -> str:
    result = " ".join(str(value).strip().split())
    if not result:
        raise ManualLandmarkError(f"{description}不能为空")
    if len(result) > MAXIMUM_TEXT_LENGTH:
        raise ManualLandmarkError(
            f"{description}不能超过{MAXIMUM_TEXT_LENGTH}个字符"
        )
    if any(ord(character) < 32 for character in result):
        raise ManualLandmarkError(f"{description}包含控制字符")
    return result


def finite_coordinate(value, description: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ManualLandmarkError(f"{description}不是有效坐标") from error
    if not math.isfinite(result) or abs(result) > 100000.0:
        raise ManualLandmarkError(f"{description}不是有效坐标")
    return result


class ManualLandmarkStore:
    """Atomically stores a map-specific manual landmark draft."""

    def __init__(self, path: Path, map_id: str, frame_id: str = "map"):
        self.path = Path(path).expanduser().resolve()
        if self.path.suffix.lower() not in (".yaml", ".yml"):
            raise ManualLandmarkError("手工地标文件必须使用.yaml或.yml后缀")
        self.map_id = normalized_identifier(map_id, "地图ID")
        self.frame_id = normalized_identifier(frame_id.lstrip("/"), "坐标系")

    def _empty_document(self) -> dict:
        return {
            "schema_version": SCHEMA_VERSION,
            "map_id": self.map_id,
            "frame_id": self.frame_id,
            "landmarks": [],
        }

    def _validate_landmark(self, item, index: int) -> dict:
        if not isinstance(item, dict):
            raise ManualLandmarkError(f"第{index + 1}个地标不是对象")
        allowed = {"id", "name", "category", "position", "source"}
        unexpected = sorted(set(item) - allowed)
        if unexpected:
            raise ManualLandmarkError(
                "手工地标包含未支持字段: {}".format(", ".join(unexpected))
            )
        landmark_id = str(item.get("id", ""))
        if not MANUAL_ID_PATTERN.fullmatch(landmark_id):
            raise ManualLandmarkError(f"手工地标ID无效: {landmark_id}")
        if item.get("source") != "manual":
            raise ManualLandmarkError(f"{landmark_id}的来源必须是manual")
        position = item.get("position")
        if not isinstance(position, dict) or set(position) != {"x", "y", "z"}:
            raise ManualLandmarkError(f"{landmark_id}的坐标格式无效")
        return {
            "id": landmark_id,
            "name": normalized_text(item.get("name", ""), "地标名称"),
            "category": normalized_text(item.get("category", ""), "地标类别"),
            "position": {
                "x": finite_coordinate(position.get("x"), "X坐标"),
                "y": finite_coordinate(position.get("y"), "Y坐标"),
                "z": finite_coordinate(position.get("z"), "Z坐标"),
            },
            "source": "manual",
        }

    def load_document(self) -> dict:
        if not self.path.exists():
            return self._empty_document()
        try:
            document = yaml.safe_load(self.path.read_text(encoding="utf-8"))
        except (OSError, UnicodeDecodeError, yaml.YAMLError) as error:
            raise ManualLandmarkError(f"无法读取手工地标文件: {error}") from error
        if not isinstance(document, dict):
            raise ManualLandmarkError("手工地标文件必须是YAML对象")
        allowed = {"schema_version", "map_id", "frame_id", "landmarks"}
        unexpected = sorted(set(document) - allowed)
        if unexpected:
            raise ManualLandmarkError(
                "手工地标文件包含未支持字段: {}".format(", ".join(unexpected))
            )
        if document.get("schema_version") != SCHEMA_VERSION:
            raise ManualLandmarkError("手工地标文件版本不受支持")
        if document.get("map_id") != self.map_id:
            raise ManualLandmarkError("手工地标文件属于另一张地图")
        if str(document.get("frame_id", "")).lstrip("/") != self.frame_id:
            raise ManualLandmarkError("手工地标文件使用了不同坐标系")
        raw_landmarks = document.get("landmarks")
        if not isinstance(raw_landmarks, list):
            raise ManualLandmarkError("landmarks必须是列表")
        landmarks = [
            self._validate_landmark(item, index)
            for index, item in enumerate(raw_landmarks)
        ]
        identifiers = [item["id"] for item in landmarks]
        if len(identifiers) != len(set(identifiers)):
            raise ManualLandmarkError("手工地标ID重复")
        return {
            "schema_version": SCHEMA_VERSION,
            "map_id": self.map_id,
            "frame_id": self.frame_id,
            "landmarks": landmarks,
        }

    def landmarks(self) -> list[dict]:
        return self.load_document()["landmarks"]

    @staticmethod
    def _next_identifier(landmarks: list[dict]) -> str:
        maximum = 0
        for item in landmarks:
            match = MANUAL_ID_PATTERN.fullmatch(item["id"])
            if match:
                maximum = max(maximum, int(match.group(1)))
        return f"landmark_manual_{maximum + 1:04d}"

    def _atomic_write(self, document: dict) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        serialized = yaml.safe_dump(
            document,
            allow_unicode=True,
            sort_keys=False,
            default_flow_style=False,
        )
        descriptor, temporary_name = tempfile.mkstemp(
            dir=str(self.path.parent), prefix=f".{self.path.name}.", suffix=".tmp"
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                stream.write(serialized)
                stream.flush()
                os.fsync(stream.fileno())
            os.chmod(temporary_name, 0o644)
            os.replace(temporary_name, self.path)
            directory = os.open(str(self.path.parent), os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            if os.path.exists(temporary_name):
                os.unlink(temporary_name)

    def add(self, name: str, category: str, x, y, z=0.0) -> dict:
        document = self.load_document()
        normalized_name = normalized_text(name, "地标名称")
        normalized_category = normalized_text(category, "地标类别")
        position = {
            "x": finite_coordinate(x, "X坐标"),
            "y": finite_coordinate(y, "Y坐标"),
            "z": finite_coordinate(z, "Z坐标"),
        }
        for item in document["landmarks"]:
            distance = math.hypot(
                item["position"]["x"] - position["x"],
                item["position"]["y"] - position["y"],
            )
            if item["name"] == normalized_name and distance < 0.05:
                raise ManualLandmarkError("同名地标已保存在相同位置附近")
        landmark = {
            "id": self._next_identifier(document["landmarks"]),
            "name": normalized_name,
            "category": normalized_category,
            "position": position,
            "source": "manual",
        }
        document["landmarks"].append(landmark)
        self._atomic_write(document)
        return landmark


class LandmarkDialog(QDialog):
    def __init__(self, x: float, y: float, z: float, parent=None):
        super().__init__(parent)
        self.setWindowTitle("添加地图地标")
        self.setModal(True)
        self.setWindowFlag(Qt.WindowStaysOnTopHint, True)
        self.setMinimumWidth(420)

        self.name_edit = QLineEdit(self)
        self.name_edit.setPlaceholderText("例如：东侧充电站入口")
        self.name_edit.setMaxLength(MAXIMUM_TEXT_LENGTH)
        self.category_edit = QLineEdit(self)
        self.category_edit.setPlaceholderText("例如：充电设施")
        self.category_edit.setMaxLength(MAXIMUM_TEXT_LENGTH)

        form = QFormLayout()
        form.addRow("地标名称：", self.name_edit)
        form.addRow("地标类别：", self.category_edit)
        form.addRow("地图坐标：", QLabel(f"X {x:.3f} m，Y {y:.3f} m，Z {z:.3f} m"))

        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        buttons.button(QDialogButtonBox.Save).setText("保存")
        buttons.button(QDialogButtonBox.Cancel).setText("取消")
        buttons.accepted.connect(self._accept_if_valid)
        buttons.rejected.connect(self.reject)

        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(buttons)
        self.name_edit.setFocus()

    def _accept_if_valid(self):
        try:
            normalized_text(self.name_edit.text(), "地标名称")
            normalized_text(self.category_edit.text(), "地标类别")
        except ManualLandmarkError as error:
            QMessageBox.warning(self, "无法保存", str(error))
            return
        self.accept()


class ManualLandmarkEditor(Node):
    def __init__(self):
        super().__init__("manual_landmark_editor")
        self.declare_parameter("map_id", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("output_file", "")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("marker_topic", "/manual_landmarks/markers")

        map_id = self.get_parameter("map_id").value
        self.frame_id = str(self.get_parameter("frame_id").value).lstrip("/")
        output_file = self.get_parameter("output_file").value
        if not output_file:
            raise ManualLandmarkError("必须配置output_file")
        self.store = ManualLandmarkStore(Path(output_file), map_id, self.frame_id)
        self.store.load_document()

        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            marker_qos,
        )
        self.point_subscription = self.create_subscription(
            PointStamped,
            str(self.get_parameter("clicked_point_topic").value),
            self._on_clicked_point,
            10,
        )
        self.active_dialog = None
        self.publish_markers()
        self.get_logger().info(
            "手工地标编辑器已就绪：在RViz选择Publish Point并点击地图；草稿保存到%s"
            % self.store.path
        )

    def _on_clicked_point(self, message: PointStamped):
        clicked_frame = str(message.header.frame_id).lstrip("/")
        if clicked_frame != self.frame_id:
            QMessageBox.warning(
                None,
                "坐标系不匹配",
                f"点击坐标系是{clicked_frame or '空'}，必须使用{self.frame_id}",
            )
            return
        if self.active_dialog is not None:
            self.get_logger().warning("已有地标编辑窗口，忽略重复点击")
            self.active_dialog.showNormal()
            self.active_dialog.raise_()
            self.active_dialog.activateWindow()
            return
        point = message.point
        dialog = LandmarkDialog(point.x, point.y, point.z)
        dialog.accepted.connect(
            lambda: self._save_landmark(dialog, point.x, point.y, point.z)
        )
        dialog.finished.connect(self._close_dialog)
        self.active_dialog = dialog
        dialog.open()
        dialog.raise_()
        dialog.activateWindow()

    def _save_landmark(self, dialog: LandmarkDialog, x: float, y: float, z: float):
        try:
            landmark = self.store.add(
                dialog.name_edit.text(),
                dialog.category_edit.text(),
                x,
                y,
                z,
            )
            self.publish_markers()
            self.get_logger().info(
                "已保存%s：%s（%s），坐标(%.3f, %.3f, %.3f)"
                % (
                    landmark["id"],
                    landmark["name"],
                    landmark["category"],
                    landmark["position"]["x"],
                    landmark["position"]["y"],
                    landmark["position"]["z"],
                )
            )
        except ManualLandmarkError as error:
            QMessageBox.critical(None, "保存失败", str(error))

    def _close_dialog(self, _result):
        dialog = self.active_dialog
        self.active_dialog = None
        if dialog is not None:
            dialog.deleteLater()

    @staticmethod
    def _base_marker(frame_id, stamp, marker_id, namespace, marker_type):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        delete_all = Marker()
        delete_all.header.frame_id = self.frame_id
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        markers = [delete_all]
        for index, item in enumerate(self.store.landmarks()):
            position = item["position"]
            point = self._base_marker(
                self.frame_id,
                stamp,
                index * 2,
                "manual_landmark_points",
                Marker.SPHERE,
            )
            point.pose.position.x = position["x"]
            point.pose.position.y = position["y"]
            point.pose.position.z = max(position["z"], 0.0) + 0.18
            point.scale.x = 0.35
            point.scale.y = 0.35
            point.scale.z = 0.35
            point.color.r = 0.0
            point.color.g = 0.85
            point.color.b = 0.95
            point.color.a = 1.0

            label = self._base_marker(
                self.frame_id,
                stamp,
                index * 2 + 1,
                "manual_landmark_labels",
                Marker.TEXT_VIEW_FACING,
            )
            label.pose.position.x = position["x"]
            label.pose.position.y = position["y"]
            label.pose.position.z = max(position["z"], 0.0) + 0.65
            label.scale.z = 0.42
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"{item['name']} [{item['category']}]"
            markers.extend((point, label))
        self.marker_publisher.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    application = QApplication.instance() or QApplication([sys.argv[0]])
    application.setApplicationName("农机地图地标编辑器")
    application.setQuitOnLastWindowClosed(False)
    node = None
    timer = QTimer()

    def request_shutdown(_signal_number, _frame):
        timer.stop()
        application.closeAllWindows()
        application.quit()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    try:
        node = ManualLandmarkEditor()

        def spin_ros_once():
            if not rclpy.ok():
                timer.stop()
                application.quit()
                return
            try:
                rclpy.spin_once(node, timeout_sec=0.0)
            except KeyboardInterrupt:
                timer.stop()
                application.quit()
            except Exception:
                if not rclpy.ok():
                    timer.stop()
                    application.quit()
                    return
                raise

        timer.timeout.connect(spin_ros_once)
        timer.start(10)
        return application.exec_()
    finally:
        timer.stop()
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                if rclpy.ok():
                    raise
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
