from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QProgressBar


class TelemetryWidget(QWidget):

    def __init__(self):

        super().__init__()

        layout = QVBoxLayout()

        layout.addWidget(QLabel("Arm Telemetry"))

        self.bars = []

        for i in range(6):

            label = QLabel(f"Joint {i+1}")

            bar = QProgressBar()
            bar.setMinimum(-314)
            bar.setMaximum(314)

            layout.addWidget(label)
            layout.addWidget(bar)

            self.bars.append(bar)

        self.setLayout(layout)

    def update_joints(self, positions):

        for i, value in enumerate(positions):

            self.bars[i].setValue(int(value * 100))