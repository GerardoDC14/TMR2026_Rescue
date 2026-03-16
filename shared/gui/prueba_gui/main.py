import sys
from PyQt6.QtWidgets import QApplication
from gui.main_window import MainWindow

def main():

    app = QApplication(sys.argv)
    app.setStyleSheet("""
        QWidget {
            background-color: #111111;
            color: white;
        }

        QFrame {
            background-color: #1a1a1a;
            border: 1px solid #444;
        }

        QPushButton {
            background-color: #2a2a2a;
            border: 1px solid #555;
            padding: 5px;
        }

        QPushButton:hover {
            background-color: #3a3a3a;
        }

        QSlider::groove:horizontal {
            height: 6px;
            background: #333;
        }

        QSlider::handle:horizontal {
            background: #ff6600;
            width: 14px;
        }
        """)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()