#!/usr/bin/env python3

# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'control.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import time
from threading import Thread
import sys
import os
import cv2
from subprocess import Popen

move_cmd = Twist()
t1 = ["t1", 1.47, 5.98]
t2 = ["t2", -2.28, 6.02]
t3 = ["t3", -4.86, 6.15]
t4 = ["t4", 1.68, 9.27]
t5 = ["t5", -2.31, 9.46]
t6 = ["t6", -4.76, 9.46]
t7 = ["t7", 1.68, 13.03]
t8 = ["t8", -2.08, 13.10]
t9 = ["t9", -4.86, 13.16]
charge = [-4.80, -0.75]
kitchen = [0.10, -0.36]

table_cor = [t1, t2, t3, t4, t5, t6, t7, t8, t9]
table_num = [[]]

battery = 100

class Ui_waiter_bot(object):
    def __init__(self, node):
        super(Ui_waiter_bot, self).__init__()
        self.node = node
        self.stop_flag = False
        self.map_flag = True
        self.move_pub = node.create_publisher(Twist, 'cmd_vel', 10)
        self.action_client = ActionClient(node, MoveBaseAction, 'move_base')

    def movebase_client(self, x_corr, y_corr):
        xGoal = x_corr
        yGoal = y_corr
        goal = MoveBaseAction.Goal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.target_pose.pose.position = Point(x=xGoal, y=yGoal, z=0.0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.701
        goal.target_pose.pose.orientation.w = 0.712

        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()

        if not result:
            self.node.get_logger().error("Action server not available!")
            rclpy.shutdown()
        else:
            return result

    def button_released(self):
        global move_cmd
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.move_pub.publish(move_cmd)

    def fl_button_pressed(self):
        global move_cmd
        move_cmd.linear.x = 1.0
        move_cmd.angular.z = 2.0
        self.move_pub.publish(move_cmd)

    def f_button_pressed(self):
        global move_cmd
        move_cmd.linear.x = 1.0
        move_cmd.angular.z = 0.0
        self.move_pub.publish(move_cmd)

    def fr_button_pressed(self):
        global move_cmd
        move_cmd.linear.x = 1.0
        move_cmd.angular.z = -2.0
        self.move_pub.publish(move_cmd)

    def stop_button_pressed(self):
        global move_cmd
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.move_pub.publish(move_cmd)

    # Other button press methods remain the same...

    def emergency_stop_button(self):
        self.stop_flag = True
        global move_cmd
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.move_pub.publish(move_cmd)
        Popen('ros2 action send_goal /move_base/cancel action_msgs/GoalID -- {}', shell=True)

    # Continue with similar updates for all other methods...

if __name__ == "__main__":
    rclpy.init()
    node = Node('waiter_bot_controller')

    app = QtWidgets.QApplication(sys.argv)
    waiter_bot = QtWidgets.QWidget()
    ui = Ui_waiter_bot(node)
    ui.setupUi(waiter_bot)
    waiter_bot.show()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(app.exec_())

