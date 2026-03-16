from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QComboBox
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QImage, QPixmap
import cv2
import time


class CameraWidget(QWidget):

    def __init__(self, name):

        super().__init__()

        self.name = name
        self.last_time = time.time()

        layout = QVBoxLayout()

        # -------------------------
        # TOP CONTROL BAR
        # -------------------------

        bar = QHBoxLayout()

        self.camera_select = QComboBox()
        self.camera_select.addItems(["No Camera", name])

        self.filter_select = QComboBox()
        self.filter_select.addItems(["No Filter", "Edge", "Gray"])

        self.res_select = QComboBox()
        self.res_select.addItems(["1280x720", "640x480"])

        bar.addWidget(self.camera_select)
        bar.addWidget(self.filter_select)
        bar.addWidget(self.res_select)

        # -------------------------
        # VIDEO
        # -------------------------

        self.video = QLabel()
        self.video.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video.setStyleSheet("""
            background-color:black;
            border:2px solid #444;
        """)

        layout.addLayout(bar)
        layout.addWidget(self.video)

        self.setLayout(layout)

    def update_frame(self, frame):

        # FPS
        now = time.time()
        fps = 1 / (now - self.last_time)
        self.last_time = now

        # filters
        filter_mode = self.filter_select.currentText()

        if filter_mode == "Gray":
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        if filter_mode == "Edge":
            edges = cv2.Canny(frame, 50, 150)
            frame = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        cv2.putText(
            frame,
            f"{self.name} | {fps:.1f} FPS",
            (10,30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255,255,255),
            2
        )

        height, width, channel = frame.shape
        bytes_per_line = 3 * width

        image = QImage(
            frame.data,
            width,
            height,
            bytes_per_line,
            QImage.Format.Format_BGR888
        )

        pixmap = QPixmap.fromImage(image)

        self.video.setPixmap(pixmap)