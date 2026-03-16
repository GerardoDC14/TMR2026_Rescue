from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel


class TelemetryWidget(QWidget):

    def __init__(self):

        super().__init__()

        layout = QVBoxLayout()

        layout.addWidget(QLabel("Arm Telemetry"))

        self.labels = []

        for i in range(6):

            label = QLabel(f"Joint {i+1}: 0.0")
            layout.addWidget(label)

            self.labels.append(label)

        self.setLayout(layout)

    def update_joints(self, positions):

        for i, value in enumerate(positions):

            self.labels[i].setText(
                f"Joint {i+1}: {value:.3f}"
            )