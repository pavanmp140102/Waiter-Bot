# -*- coding: utf-8 -*-
from PyQt5 import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

class ROS2Node(Node):
    """ROS 2 Node to handle communication with the WaiterBot."""
    def __init__(self):
        super().__init__('waiter_bot_gui_node')
        self.command_pub = self.create_publisher(String, '/waiter_bot/command', 10)
        self.battery_sub = self.create_subscription(Float32, '/waiter_bot/battery_level', self.battery_callback, 10)
        self.battery_level = 0.0  # Store battery level value

    def publish_command(self, command: str):
        """Publish commands to the robot."""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f'Published: {command}')

    def battery_callback(self, msg: Float32):
        """Callback to handle battery level updates."""
        self.battery_level = msg.data

class Ui_waiter_bot(object):
    """UI setup for WaiterBot Control Panel."""
    def setupUi(self, waiter_bot, ros2_node):
        self.ros2_node = ros2_node  # Link ROS 2 Node
        waiter_bot.setObjectName("waiter_bot")
        waiter_bot.resize(669, 796)

        # Main Label
        self.label = QtWidgets.QLabel(waiter_bot)
        self.label.setGeometry(QtCore.QRect(0, 10, 651, 41))
        font = QtGui.QFont()
        font.setFamily("Tibetan Machine Uni")
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")

        # Battery Status
        self.verticalLayoutWidget_3 = QtWidgets.QWidget(waiter_bot)
        self.verticalLayoutWidget_3.setGeometry(QtCore.QRect(10, 60, 171, 181))
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_3)
        self.verticalLayout_3.setContentsMargins(0, 9, 0, 0)

        self.label_2 = QtWidgets.QLabel(self.verticalLayoutWidget_3)
        font.setPointSize(18)
        self.label_2.setFont(font)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.verticalLayout_3.addWidget(self.label_2)

        self.label_12 = QtWidgets.QLabel(self.verticalLayoutWidget_3)
        font.setPointSize(16)
        self.label_12.setFont(font)
        self.label_12.setAlignment(QtCore.Qt.AlignCenter)
        self.label_12.setObjectName("label_12")
        self.verticalLayout_3.addWidget(self.label_12)

        self.battery_low = QtWidgets.QPushButton(self.verticalLayoutWidget_3)
        self.battery_low.setObjectName("battery_low")
        self.battery_low.clicked.connect(self.on_battery_low_clicked)
        self.verticalLayout_3.addWidget(self.battery_low)

        # Tab Widget
        self.tabWidget = QtWidgets.QTabWidget(waiter_bot)
        self.tabWidget.setGeometry(QtCore.QRect(190, 60, 471, 731))

        # Table Navigation Tab
        self.table_navigation = QtWidgets.QWidget()
        self.gridLayoutWidget_2 = QtWidgets.QWidget(self.table_navigation)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(10, 0, 461, 690))
        self.gridLayout_2 = QtWidgets.QGridLayout(self.gridLayoutWidget_2)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)

        # Table Buttons
        self.table_buttons = {}
        for i in range(1, 10):
            btn = QtWidgets.QPushButton(self.gridLayoutWidget_2)
            btn.setObjectName(f"table{i}")
            btn.clicked.connect(lambda _, i=i: self.on_table_clicked(i))
            self.table_buttons[f"table{i}"] = btn
            row = (i - 1) // 3
            col = (i - 1) % 3
            self.gridLayout_2.addWidget(btn, row, col)

        self.tabWidget.addTab(self.table_navigation, "")

        # Manual Control Tab
        self.manual_control = QtWidgets.QWidget()
        self.gridLayoutWidget = QtWidgets.QWidget(self.manual_control)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(0, 70, 461, 471))
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)

        # Manual Control Buttons
        directions = ["F", "B", "L", "R", "LF", "RF", "LB", "RB", "STOP"]
        self.control_buttons = {}
        for i, direction in enumerate(directions):
            btn = QtWidgets.QPushButton(self.gridLayoutWidget)
            btn.setObjectName(direction)
            btn.clicked.connect(lambda _, dir=direction: self.on_manual_control_clicked(dir))
            self.control_buttons[direction] = btn
            row = i // 3
            col = i % 3
            self.gridLayout.addWidget(btn, row, col)

        self.tabWidget.addTab(self.manual_control, "")

        # Mapping Tab
        self.mapping = QtWidgets.QWidget()
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.mapping)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(100, 50, 291, 381))
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)

        self.manual_map = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.manual_map.setObjectName("manual_map")
        self.manual_map.clicked.connect(lambda: self.on_mapping_clicked("manual"))
        self.verticalLayout_2.addWidget(self.manual_map)

        self.autonomous_map = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.autonomous_map.setObjectName("autonomous_map")
        self.autonomous_map.clicked.connect(lambda: self.on_mapping_clicked("autonomous"))
        self.verticalLayout_2.addWidget(self.autonomous_map)

        self.map_view = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.map_view.setObjectName("map_view")
        self.map_view.clicked.connect(lambda: self.on_mapping_clicked("view"))
        self.verticalLayout_2.addWidget(self.map_view)

        self.tabWidget.addTab(self.mapping, "")

        # Emergency Stop
        self.emergency_stop = QtWidgets.QPushButton(waiter_bot)
        self.emergency_stop.setGeometry(QtCore.QRect(20, 300, 141, 100))
        self.emergency_stop.setObjectName("emergency_stop")
        self.emergency_stop.clicked.connect(self.on_emergency_stop_clicked)

        self.retranslateUi(waiter_bot)
        self.tabWidget.setCurrentIndex(0)

        # Timer for Dynamic Updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_battery_level)
        self.timer.start(100)

    def retranslateUi(self, waiter_bot):
        _translate = QtCore.QCoreApplication.translate
        waiter_bot.setWindowTitle(_translate("waiter_bot", "Control Panel"))
        self.label.setText(_translate("waiter_bot", "RMP Waiter Bot Control"))
        self.label_2.setText(_translate("waiter_bot", "Battery Level"))
        self.label_12.setText(_translate("waiter_bot", "0%"))
        self.battery_low.setText(_translate("waiter_bot", "Battery Low"))
        for i in range(1, 10):
            self.table_buttons[f"table{i}"].setText(_translate("waiter_bot", f"Table {i}"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.table_navigation), _translate("waiter_bot", "Table Navigation"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.manual_control), _translate("waiter_bot", "Manual Control"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.mapping), _translate("waiter_bot", "Mapping"))
        self.manual_map.setText(_translate("waiter_bot", "Manual"))
        self.autonomous_map.setText(_translate("waiter_bot", "Autonomous"))
        self.map_view.setText(_translate("waiter_bot", "Map View"))
        self.emergency_stop.setText(_translate("waiter_bot", "Emergency Stop"))

    # Callback Handlers
    def update_battery_level(self):
        self.label_12.setText(f"{self.ros2_node.battery_level:.1f}%")

    def on_battery_low_clicked(self):
        self.ros2_node.publish_command("Battery Low")

    def on_table_clicked(self, table_number):
        self.ros2_node.publish_command(f"Navigate to Table {table_number}")

    def on_manual_control_clicked(self, direction):
        self.ros2_node.publish_command(f"Move {direction}")

    def on_mapping_clicked(self, mode):
        self.ros2_node.publish_command(f"Mapping Mode: {mode}")

    def on_emergency_stop_clicked(self):
        self.ros2_node.publish_command("EMERGENCY STOP")


if __name__ == "__main__":
    rclpy.init()
    ros2_node = ROS2Node()

    app = QtWidgets.QApplication([])
    waiter_bot = QtWidgets.QWidget()
    ui = Ui_waiter_bot()
    ui.setupUi(waiter_bot, ros2_node)

    waiter_bot.show()

    try:
        app.exec_()
    finally:
        rclpy.shutdown()

