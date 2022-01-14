import sys
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QGridLayout, QWidget, QPushButton, QSlider, QLabel
from PyQt5.QtCore import QSize

import rospy
from stm32_ros_msgs.srv import *


class RosDashboard(QMainWindow):
    def __init__(self):
        rospy.init_node('ros_dashboard')
        rospy.wait_for_service('stm32/emergency_stop')
        rospy.wait_for_service('stm32/brakes')
        rospy.wait_for_service('stm32/motor')
        rospy.wait_for_service('stm32/steering')

        self.emergency_stop_srv = rospy.ServiceProxy('stm32/emergency_stop', emergency_stop)
        self.brakes_srv = rospy.ServiceProxy('stm32/brakes', brakes)
        self.motor_srv = rospy.ServiceProxy('stm32/motor', motor)
        self.steering_srv = rospy.ServiceProxy('stm32/steering', steering)

        QMainWindow.__init__(self)

        self.setMinimumSize(QSize(640, 480))
        self.setWindowTitle("Tricycle dashboard")

        root = QWidget(self)
        self.setCentralWidget(root)

        grid = QGridLayout(self)
        root.setLayout(grid)

        emergency_stop_container = QWidget(root)
        emergency_stop_grid = QGridLayout(emergency_stop_container)
        emergency_stop_container.setLayout(emergency_stop_grid)
        grid.addWidget(emergency_stop_container, 0, 0)

        emergency_stop_on_btn = QPushButton("Activate emergency stop")
        emergency_stop_on_btn.clicked.connect(lambda: self.send_emergency_stop(True))
        emergency_stop_grid.addWidget(emergency_stop_on_btn, 0, 0)

        emergency_stop_off_btn = QPushButton("Deactivate emergency stop")
        emergency_stop_off_btn.clicked.connect(lambda: self.send_emergency_stop(False))
        emergency_stop_grid.addWidget(emergency_stop_off_btn, 0, 1)

        brakes_container = QWidget(root)
        brakes_grid = QGridLayout(brakes_container)
        brakes_container.setLayout(brakes_grid)
        grid.addWidget(brakes_container, 1, 0)

        brakes_on_btn = QPushButton("Set brakes")
        brakes_on_btn.clicked.connect(lambda: self.send_brakes(True))
        brakes_grid.addWidget(brakes_on_btn, 0, 0)

        brakes_off_btn = QPushButton("Release brakes")
        brakes_off_btn.clicked.connect(lambda: self.send_brakes(False))
        brakes_grid.addWidget(brakes_off_btn, 0, 1)

        motor_label = QLabel("Motor power")
        grid.addWidget(motor_label, 2, 0)

        motor_slider = QSlider(QtCore.Qt.Orientation.Horizontal)
        motor_slider.setRange(0, 100)
        motor_slider.valueChanged.connect(lambda val: self.send_motor(val))
        grid.addWidget(motor_slider, 3, 0)

        motor_label = QLabel("Steering angle")
        grid.addWidget(motor_label, 4, 0)

        steering_slider = QSlider(QtCore.Qt.Orientation.Horizontal)
        steering_slider.setRange(0, 120)
        steering_slider.valueChanged.connect(lambda val: self.send_steering(val))
        grid.addWidget(steering_slider, 5, 0)

    def send_emergency_stop(self, stop: bool):
        print(f"setting emergency stop {stop}")
        try:
            resp = self.emergency_stop_srv(stop)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def send_brakes(self, brakes_on: bool):
        print(f"setting brakes {brakes_on}")
        try:
            resp = self.brakes_srv(brakes_on)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def send_motor(self, power: int):
        print(f"setting motor at {power}%")
        try:
            resp = self.motor_srv(power)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def send_steering(self, angle: int):
        print(f"setting steering at {angle}")
        try:
            resp = self.steering_srv(angle)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    mainWin = RosDashboard()
    mainWin.show()
    sys.exit(app.exec_())
