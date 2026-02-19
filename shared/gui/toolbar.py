import sys
from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QPushButton, QGroupBox, QLabel)
import random


global sen_1, sen_2, sen_3
sen_1 = 0
sen_2 = 0
sen_3 = 0
class SideToolBar(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Toolbar")
        self.resize(300, 1000)
        # Declaracion de contenidos
        main        = QVBoxLayout()

        robot_box   = QGroupBox("Robot")
        robot       = QVBoxLayout()
        robotisa    = QLabel("Aqui va lo del robot we")
        
        sensores_box= QGroupBox("Sensores")
        sensores    = QVBoxLayout()
        self.sensor_1    = QLabel("Sin datos, dale al boton uno we")
        self.sensor_2    = QLabel("Sin datos, dale al boton uno we")
        self.sensor_3    = QLabel("Sin datos, dale al boton uno we")


        buttons_box = QGroupBox("Botones")
        buttons     = QVBoxLayout()
        
        buttons_label   = QLabel("Aqui van los botones we")
        self.button1         = QPushButton("funcion 1")
        self.button1.clicked.connect(self.funcionRandom)
        self.button2         = QPushButton("funcion 2")

        #Agrupacion en contenedores        
        robot.addWidget(robotisa)
        buttons.addWidget(buttons_label)
        buttons.addWidget(self.button1)
        buttons.addWidget(self.button2)
        
        sensores.addWidget(self.sensor_1)
        sensores.addWidget(self.sensor_2)
        sensores.addWidget(self.sensor_3)


        robot_box.setLayout(robot)
        sensores_box.setLayout(sensores)
        buttons_box.setLayout(buttons)

        main.addWidget(robot_box)
        main.addWidget(sensores_box)
        main.addWidget(buttons_box)

        self.setLayout(main)
    def funcionRandom(self):
        sen_1 = random.randint(0,255)
        format_text = f"Sensor 1: {sen_1}"
        self.sensor_1.setText(format_text)
        sen_2 = random.randint(0,255)
        format_text = f"Sensor 2: {sen_2}"
        self.sensor_2.setText(format_text)
        sen_3 = random.randint(0,255)
        format_text = f"Sensor 3: {sen_3}"
        self.sensor_3.setText(format_text)


app = QApplication(sys.argv)
ventanawe = SideToolBar()
ventanawe.show()
sys.exit(app.exec())