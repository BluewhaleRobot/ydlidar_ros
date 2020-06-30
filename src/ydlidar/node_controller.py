#!/usr/bin/env python
# encoding=utf-8

"""
ydlidar节点管理程序。在ydlidar节点运行不正常的时候重启ydlidar节点
"""

import rospy
import roslaunch
import time
from galileo_serial_server.msg import GalileoStatus
import rosservice
import subprocess
import os

LAST_UPDATE_TIME = 0
VISUAL_FLAG = False


def get_scan(scan):
    global LAST_UPDATE_TIME
    LAST_UPDATE_TIME = int(time.time())


def get_galileo_status(status):
    global VISUAL_FLAG
    if status.visualStatus == -1:
        VISUAL_FLAG = False
    else:
        if not VISUAL_FLAG:
            rospy.set_param("/rplidar_node_manager/keep_running", True)
        VISUAL_FLAG = True


if __name__ == "__main__":
    rospy.init_node("ydlidar_manager")
    rospy.Subscriber("/scan", rospy.AnyMsg, get_scan)
    rospy.Subscriber("/galileo/status", GalileoStatus, get_galileo_status)
    rospy.set_param("/rplidar_node_manager/keep_running", True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    ydlidar_launch = roslaunch.parent.ROSLaunchParent(
        uuid, ["/home/xiaoqiang/Documents/ros/src/ydlidar_ros/launch/ydlidar.launch"])
    ydlidar_launch.start()
    rospy.loginfo("ydlidar started")
    while not rospy.is_shutdown():
        time.sleep(5)
        keep_running_flag = rospy.get_param("/rplidar_node_manager/keep_running", True)
        # 处于导航状态下且无法收到雷达数据
        if int(time.time()) - LAST_UPDATE_TIME > 5 and VISUAL_FLAG and keep_running_flag:
            rospy.logerr("restart ydlidar node")
            ydlidar_launch.shutdown()
            roslaunch.configure_logging(uuid)
            ydlidar_launch = roslaunch.parent.ROSLaunchParent(
                uuid, ["/home/xiaoqiang/Documents/ros/src/ydlidar_ros/launch/ydlidar.launch"])
            ydlidar_launch.start()
        # 在停用状态下却有雷达数据
        if int(time.time()) - LAST_UPDATE_TIME < 5 and (not VISUAL_FLAG or not keep_running_flag):
            if rosservice.get_service_node("/stop_motor") is not None:
                cmd = "rosservice call /stop_motor"
                new_env = os.environ.copy()
                subprocess.Popen(
                    cmd, shell=True, env=new_env)
                time.sleep(5)
            ydlidar_launch.shutdown()
