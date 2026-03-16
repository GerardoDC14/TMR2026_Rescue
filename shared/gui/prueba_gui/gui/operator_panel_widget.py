from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSlider, QFrame
)
from PyQt6.QtCore import Qt


class OperatorPanelWidget(QWidget):

    def __init__(self):

        super().__init__()

        main_layout = QVBoxLayout()

        track_frame = self.create_box("Track velocity")
        track_layout = QHBoxLayout()

        self.left_track = QLabel("Left: 0.00")
        self.right_track = QLabel("Right: 0.00")

        track_layout.addWidget(self.left_track)
        track_layout.addWidget(self.right_track)

        track_frame.layout().addLayout(track_layout)

        speech_frame = self.create_box("Speech")
        speech_frame.layout().addWidget(QLabel("No data"))

        mag_frame = self.create_box("Magnetometer")

        mag_frame.layout().addWidget(QLabel("X: 0.00"))
        mag_frame.layout().addWidget(QLabel("Y: 0.00"))
        mag_frame.layout().addWidget(QLabel("Z: 0.00"))

        button_layout = QHBoxLayout()

        audio = QPushButton("Toggle audio")
        audio.setStyleSheet("background-color:yellow")

        clear = QPushButton("Clear data")

        estop = QPushButton("E-STOP")
        estop.setStyleSheet("""
            background-color:red;
            color:white;
            font-weight:bold;
            font-size:18px;
        """)

        button_layout.addWidget(audio)
        button_layout.addWidget(clear)
        button_layout.addWidget(estop)

        speed_label = QLabel("Speed")
        speed_slider = QSlider(Qt.Orientation.Horizontal)

        setting_label = QLabel("Setting")
        setting_slider = QSlider(Qt.Orientation.Horizontal)

        main_layout.addWidget(track_frame)
        main_layout.addWidget(speech_frame)
        main_layout.addWidget(mag_frame)
        main_layout.addLayout(button_layout)
        main_layout.addWidget(speed_label)
        main_layout.addWidget(speed_slider)
        main_layout.addWidget(setting_label)
        main_layout.addWidget(setting_slider)

        main_layout.addStretch()

        self.setLayout(main_layout)

    def create_box(self, title):

        frame = QFrame()
        frame.setFrameShape(QFrame.Shape.Box)

        layout = QVBoxLayout()

        label = QLabel(title)
        layout.addWidget(label)

        frame.setLayout(layout)

        return frame