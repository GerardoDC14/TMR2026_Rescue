from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QPushButton, QLabel
)


class ArmControlWidget(QWidget):

    def __init__(self):

        super().__init__()

        layout = QVBoxLayout()

        layout.addWidget(QLabel("Arm Control"))

        layout.addWidget(QPushButton("X+"))
        layout.addWidget(QPushButton("X-"))

        layout.addWidget(QPushButton("Y+"))
        layout.addWidget(QPushButton("Y-"))

        layout.addWidget(QPushButton("Z+"))
        layout.addWidget(QPushButton("Z-"))

        layout.addWidget(QPushButton("Plan"))
        layout.addWidget(QPushButton("Execute"))
        layout.addWidget(QPushButton("Stop"))

        self.setLayout(layout)