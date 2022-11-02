#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

import socket
import sys

# 直连模式下，机器人默认 IP 地址为 192.168.2.1, 控制命令端口号为 40923
host = "192.168.42.2"
port = 40923
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %f %f %f', data.linear.x, data.linear.y, data.angular.z)

    data.linear.x = max(-1.0, data.linear.x)
    data.linear.x = min(1.0, data.linear.x)
    data.linear.y = max(-1.0, data.linear.y)
    data.linear.y = min(1.0, data.linear.y)
    data.angular.z = max(-1.0, data.angular.z)
    data.angular.z = min(1.0, data.angular.z)
    
    cmd = 'chassis speed x %f y %f z %f;' %(data.linear.x, -data.linear.y, -data.angular.z*57.2)


    # 发送控制命令给机器人
    s.send(cmd.encode('utf-8'))
    print(cmd)

def callback_position(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %f %f %f', data.linear.x, data.linear.y, data.angular.z)

    data.linear.x = max(-0.5, data.linear.x)
    data.linear.x = min(0.5, data.linear.x)
    data.linear.y = max(-0.5, data.linear.y)
    data.linear.y = min(0.5, data.linear.y)
    data.angular.z = max(-900.0, data.angular.z)
    data.angular.z = min(900.0, data.angular.z)
    
    cmd = 'chassis move x %f y %f z %f;' %(data.linear.x, -data.linear.y, -data.angular.z)


    # 发送控制命令给机器人
    s.send(cmd.encode('utf-8'))
    print(cmd)

def callback_arm(data):

    rospy.loginfo(rospy.get_caller_id() + 'I heard %f %f', data.position.x, data.position.y)

    #data.position.x = max(90, data.position.x)
    #data.position.x = min(250, data.position.x)
    #data.position.y = max(40, data.position.y)
    #data.position.y = min(-100, data.position.y)

    if abs(data.position.x) < 1e-10 and abs(data.position.y) < 1e-10:
        cmd = 'robotic_arm recenter;'
    else:
        cmd = 'robotic_arm moveto x %f y %f;' %(data.position.x, data.position.y)
        # cmd = 'robotic_arm move x %f y %f;' %(data.position.x, data.position.y)


    # 发送控制命令给机器人
    s.send(cmd.encode('utf-8'))
    print(cmd)

def callback_gripper(data):

    rospy.loginfo(rospy.get_caller_id() + 'I heard %f', data.x)

    #data.position.x = max(90, data.position.x)
    #data.position.x = min(250, data.position.x)
    #data.position.y = max(40, data.position.y)
    #data.position.y = min(-100, data.position.y)

    if abs(data.x - 1.0) < 1e-10:
        cmd = 'robotic_gripper close 1;'
        
    if abs(data.x) < 1e-10:
        cmd = 'robotic_gripper open 1;'


    # 发送控制命令给机器人
    s.send(cmd.encode('utf-8'))
    print(cmd)

def listener():

    address = (host, int(port))

    # 与机器人控制命令端口建立 TCP 连接

    print("Connecting...")

    s.connect(address)

    print("Connected!")

    msg = 'command;'

    # 发送控制命令给机器人
    s.send(msg.encode('utf-8'))
    cmd = "chassis push position on attitude on;"
    s.send(cmd.encode('utf-8'))
    print(cmd)
    try:
        # 等待机器人返回执行结果
        buf = s.recv(1024)

        print(buf.decode('utf-8'))
    except socket.error as e:
        print("Error receiving :", e)
        sys.exit(1)

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('cmd_vel', Twist, callback)
    rospy.Subscriber('cmd_position', Twist, callback_position)
    rospy.Subscriber('arm_position', Pose, callback_arm)
    rospy.Subscriber('arm_gripper', Point, callback_gripper)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def turn_off_connect():
    msg = 'quit;'

    # 发送控制命令给机器人
    s.send(msg.encode('utf-8'))
    try:
        # 等待机器人返回执行结果
        buf = s.recv(1024)

        if buf.decode('utf-8') == 'ok;':
            print("")
            print("Disable SDK")
    except socket.error as e:
        print("Error receiving :", e)
        sys.exit(1)
    # 关闭端口连接
    s.shutdown(socket.SHUT_WR)
    s.close()

if __name__ == '__main__':
    listener()
    turn_off_connect()

