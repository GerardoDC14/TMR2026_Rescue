from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSlider, QFrame
)
from PyQt6.QtCore import Qt


class OperatorPanelWidget(QWidget):

    def __init__(self):

        super().__init__()

        main_layout = QVBoxLayout()


        #track velocity

        track_frame = self.create_box("Track velocity")
        track_layout = QHBoxLayout()

        self.left_track = QLabel("Left: 0.00")
        self.right_track = QLabel("Right: 0.00")

        track_layout.addWidget(self.left_track)
        track_layout.addWidget(self.right_track)

        track_frame.layout().addLayout(track_layout)


        speech_frame = self.create_box("Speech")
        self.speech_label = QLabel("No data")
        speech_frame.layout().addWidget(self.speech_label)


        mag_frame = self.create_box("Magnetometer")

        self.mag_x = QLabel("X: 0.00")
        self.mag_y = QLabel("Y: 0.00")
        self.mag_z = QLabel("Z: 0.00")

        mag_frame.layout().addWidget(self.mag_x)
        mag_frame.layout().addWidget(self.mag_y)
        mag_frame.layout().addWidget(self.mag_z)

        #buttons

        button_layout = QHBoxLayout()

        self.audio_button = QPushButton("Toggle audio")
        self.audio_button.setStyleSheet("background-color: yellow")

        self.clear_button = QPushButton("Clear data")

        self.estop_button = QPushButton("E-STOP")
        self.estop_button.setStyleSheet("background-color: red; color:white")

        button_layout.addWidget(self.audio_button)
        button_layout.addWidget(self.clear_button)
        button_layout.addWidget(self.estop_button)

        #controller buttons

        control_layout = QHBoxLayout()

        self.controller_button = QPushButton("Controller")
        self.restart_button = QPushButton("Restart")
        self.restart_button.setStyleSheet("background-color: red; color:white")

        control_layout.addWidget(self.controller_button)
        control_layout.addWidget(self.restart_button)

        # speed slider

        speed_layout = QVBoxLayout()

        speed_label = QLabel("Speed")

        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(100)

        speed_layout.addWidget(speed_label)
        speed_layout.addWidget(self.speed_slider)

        setting_layout = QVBoxLayout()

        setting_label = QLabel("Setting")

        self.setting_slider = QSlider(Qt.Orientation.Horizontal)
        self.setting_slider.setMinimum(0)
        self.setting_slider.setMaximum(100)

        setting_layout.addWidget(setting_label)
        setting_layout.addWidget(self.setting_slider)
        #add to main


        main_layout.addWidget(track_frame)
        main_layout.addWidget(speech_frame)
        main_layout.addWidget(mag_frame)

        main_layout.addLayout(button_layout)
        main_layout.addLayout(control_layout)

        main_layout.addLayout(speed_layout)
        main_layout.addLayout(setting_layout)

        main_layout.addStretch()

        self.setLayout(main_layout)

    def create_box(self, title):

        frame = QFrame()
        frame.setFrameShape(QFrame.Shape.Box)

        layout = QVBoxLayout()

        label = QLabel(title)
        label.setStyleSheet("font-weight: bold")

        layout.addWidget(label)

        frame.setLayout(layout)

        return frame