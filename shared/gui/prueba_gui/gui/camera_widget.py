from PyQt6.QtWidgets import QLabel
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap


class CameraWidget(QLabel):

    clicked = pyqtSignal(object)

    def __init__(self, name):

        super().__init__()

        self.name = name

        self.setText(name)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.setStyleSheet("""
            background-color: black;
            color: white;
            border: 2px solid gray;
        """)

        # ESTA LINEA ES CLAVE
        self.setScaledContents(True)

    def mousePressEvent(self, event):

        if event.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit(self)

    def update_frame(self, frame):

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

        # SIN scaled()
        self.setPixmap(pixmap)