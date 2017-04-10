#!/usr/bin/env python
# -*_ coding:utf-8 _*_

'''
publish:
    /AIUI/nav_cmd topic: output cmd for navigation
    /AIUI/angle topic: out put wakeup angle
    /AIUI/
subscribe:
    location
'''
import rospy
from std_msgs.msg import String
import roslib
import json
from serial import Serial
import thread
import time
import array
import gzip
from cStringIO import StringIO
import traceback


__author__ = 'Yuxiang Gao'
__copyright__ = 'Copyright (C) 2017 Yuxiang Gao'
__email__ = 'gaoyuxiang@stu.xjtu.edu.cn'
__license__ = 'GPL'
__version__ = '0.1'

AIUIAppid = '583c10e6'
AIUIKey = '2d8c2fa8a465b0dcbaca063e9493a2d9'
AIUIScene = 'main'


class AIUI_ROS:
    def __init__(self):
        rospy.init_node('AIUI', log_level=rospy.DEBUG)

        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)

        # start serial port
        self.ser = Serial(port='/dev/xunfei',
                          baudrate=115200,
                          timeout=0.5)
        if self.ser is None:
            rospy.logerr('AIUI serial port init failed')

        # Overall loop rate
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)

        # start the message publisher
        self.aiuiPub = rospy.Publisher('aiuiMsg', String, queue_size=10)
        rospy.loginfo('Started AIUI Listener')

        # Reserve a thread lock
        mutex = thread.allocate_lock()

        # Start receiving aiui serial
        while not rospy.is_shutdown():








