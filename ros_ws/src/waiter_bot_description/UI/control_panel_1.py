from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_waiter_bot(object):
    def setupUi(self, waiter_bot):
        waiter_bot.setObjectName("waiter_bot")
        waiter_bot.resize(400, 600)

        self.tabWidget = QtWidgets.QTabWidget(waiter_bot)
        self.tabWidget.setGeometry(QtCore.QRect(20, 20, 361, 511))
        self.tabWidget.setObjectName("tabWidget")

        # Table navigation tab
        self.table_navigation = QtWidgets.QWidget()
        self.table_navigation.setObjectName("table_navigation")
        
        self.gridLayoutWidget = QtWidgets.QWidget(self.table_navigation)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(10, 60, 271, 181))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")

        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")

        # Direction buttons for navigation
        self.LF = self.create_direction_button("left up.png", "LF")
        self.gridLayout.addWidget(self.LF, 0, 0, 1, 1)

        self.stop = self.create_direction_button("stop.png", "stop")
        self.gridLayout.addWidget(self.stop, 1, 1, 1, 1)

        self.B = self.create_direction_button("down arrow.png", "B")
        self.gridLayout.addWidget(self.B, 2, 1, 1, 1)

        self.RF = self.create_direction_button("right up.png", "RF")
        self.gridLayout.addWidget(self.RF, 0, 2, 1, 1)

        self.LB = self.create_direction_button("right down.png", "LB")
        self.gridLayout.addWidget(self.LB, 2, 0, 1, 1)

        self.RB = self.create_direction_button("left down.png", "RB")
        self.gridLayout.addWidget(self.RB, 2, 2, 1, 1)

        self.L = self.create_direction_button("left arrow.png", "L")
        self.gridLayout.addWidget(self.L, 1, 0, 1, 1)

        self.R = self.create_direction_button("right arrow.png", "R")
        self.gridLayout.addWidget(self.R, 1, 2, 1, 1)

        self.tabWidget.addTab(self.table_navigation, "Table Navigation")

        # Manual control tab
        self.manual_control = QtWidgets.QWidget()
        self.manual_control.setObjectName("manual_control")
        self.start_auto_nav = QtWidgets.QPushButton(self.manual_control)
        self.start_auto_nav.setGeometry(QtCore.QRect(10, 450, 160, 60))
        self.start_auto_nav.setObjectName("start_auto_nav")

        self.tabWidget.addTab(self.manual_control, "Manual Control")

        # Mapping tab
        self.mapping = QtWidgets.QWidget()
        self.mapping.setObjectName("mapping")

        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.mapping)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(100, 50, 291, 381))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")

        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")

        self.manual_map = self.create_map_button("Manual")
        self.verticalLayout_2.addWidget(self.manual_map)

        self.autonomous_map = self.create_map_button("Autonomous")
        self.verticalLayout_2.addWidget(self.autonomous_map)

        self.map_view = self.create_map_button("Map View / Save")
        self.verticalLayout_2.addWidget(self.map_view)

        self.tabWidget.addTab(self.mapping, "Mapping")

        # Emergency stop button
        self.emergency_stop = QtWidgets.QPushButton(waiter_bot)
        self.emergency_stop.setGeometry(QtCore.QRect(20, 300, 141, 100))
        self.emergency_stop.setMinimumSize(QtCore.QSize(0, 100))
        self.emergency_stop.setText("")
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap("emergency_stop.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.emergency_stop.setIcon(icon4)
        self.emergency_stop.setIconSize(QtCore.QSize(65, 65))
        self.emergency_stop.setObjectName("emergency_stop")

        # Camera view buttons
        self.verticalLayoutWidget = QtWidgets.QWidget(waiter_bot)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 450, 160, 211))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")

        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")

        self.label_13 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_13.setMinimumSize(QtCore.QSize(0, 50))
        self.label_13.setMaximumSize(QtCore.QSize(16777215, 50))
        font = QtGui.QFont()
        font.setFamily("URW Bookman L")
        font.setPointSize(15)
        self.label_13.setFont(font)
        self.label_13.setAlignment(QtCore.Qt.AlignCenter)
        self.label_13.setObjectName("label_13")
        self.verticalLayout.addWidget(self.label_13)

        self.bottom_front_cam = self.create_camera_button("Bottom Front")
        self.verticalLayout.addWidget(self.bottom_front_cam)

        self.up_front_cam = self.create_camera_button("Top Front")
        self.verticalLayout.addWidget(self.up_front_cam)

        self.retranslateUi(waiter_bot)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(waiter_bot)

    def create_direction_button(self, icon_path, object_name):
        button = QtWidgets.QPushButton()
        button.setMinimumSize(QtCore.QSize(0, 100))
        button.setMaximumSize(QtCore.QSize(102, 100))
        button.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(icon_path), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        button.setIcon(icon)
        button.setIconSize(QtCore.QSize(60, 60))
        button.setObjectName(object_name)
        return button

    def create_map_button(self, text):
        button = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        button.setMinimumSize(QtCore.QSize(0, 100))
        button.setObjectName(text.lower().replace(" ", "_"))
        button.setText(text)
        return button

    def create_camera_button(self, text):
        button = QtWidgets.QPushButton(self.verticalLayoutWidget)
        button.setMinimumSize(QtCore.QSize(0, 62))
        button.setMaximumSize(QtCore.QSize(16777215, 61))
        button.setObjectName(text.lower().replace(" ", "_"))
        button.setText(text)
        return button

    def retranslateUi(self, waiter_bot):
        _translate = QtCore.QCoreApplication.translate
        waiter_bot.setWindowTitle(_translate("waiter_bot", "Control Panel"))

        # Setting up the labels and buttons in English
        self.label_13.setText(_translate("waiter_bot", "Camera View"))
        self.bottom_front_cam.setText(_translate("waiter_bot", "Bottom Front"))
        self.up_front_cam.setText(_translate("waiter_bot", "Top Front"))
        self.manual_map.setText(_translate("waiter_bot", "Manual"))
        self.autonomous_map.setText(_translate("waiter_bot", "Autonomous"))
        self.map_view.setText(_translate("waiter_bot", "Map View / Save"))
        self.start_auto_nav.setText(_translate("waiter_bot", "Start"))

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    waiter_bot = QtWidgets.QWidget()
    ui = Ui_waiter_bot()
    ui.setupUi(waiter_bot)
    waiter_bot.show()
    sys.exit(app.exec_())

