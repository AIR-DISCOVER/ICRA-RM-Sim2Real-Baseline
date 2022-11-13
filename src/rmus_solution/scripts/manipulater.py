#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import numpy as np
from scipy.spatial.transform import Rotation as sciR

import rospy
from geometry_msgs.msg import Twist, Pose, Point
from rmus_solution.srv import graspsignal, graspsignalResponse

import tf2_ros
import tf2_geometry_msgs


class manipulater:
    def __init__(self) -> None:
        self.arm_gripper_pub = rospy.Publisher("arm_gripper", Point, queue_size=2)
        self.arm_position_pub = rospy.Publisher("arm_position", Pose, queue_size=2)
        self.cmd_vel_puber = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/get_blockinfo", Pose, self.markerPoseCb, queue_size=1)

        self.current_marker_poses = None
        self.image_time_now = 0
        self.observation = np.array([0.0, 0.0])

        self.ros_rate = 30
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        self.service = rospy.Service(
            "/let_manipulater_work", graspsignal, self.trimerworkCallback
        )

    def markerPoseCb(self, msg):
        self.current_marker_poses = msg
        self.image_time_now = rospy.get_time()
        self.observation = [msg.position.x, msg.position.z]

    def getTargetPosAndAngleInBaseLinkFrame(self, pose_in_cam):
        if not self.tfBuffer.can_transform(
            "base_link", "camera_aligned_depth_to_color_frame_correct", rospy.Time.now()
        ):
            rospy.logerr(
                "pick_and_place: cannot find transform between base_link and camera_aligned_depth_to_color_frame_correct"
            )
            return None, None
        posestamped_in_cam = tf2_geometry_msgs.PoseStamped()
        posestamped_in_cam.header.stamp = rospy.Time.now()
        posestamped_in_cam.header.frame_id = (
            "camera_aligned_depth_to_color_frame_correct"
        )
        posestamped_in_cam.pose = pose_in_cam
        posestamped_in_base = self.tfBuffer.transform(posestamped_in_cam, "base_link")
        pose_in_base = posestamped_in_base.pose
        pos = np.array(
            [pose_in_base.position.x, pose_in_base.position.y, pose_in_base.position.z]
        )
        quat = np.array(
            [
                pose_in_cam.orientation.x,
                pose_in_cam.orientation.y,
                pose_in_cam.orientation.z,
                pose_in_cam.orientation.w,
            ]
        )
        angle = sciR.from_quat(quat).as_euler("YXZ")[0]

        return pos, angle

    def sendBaseVel(self, vel):
        twist = Twist()
        twist.linear.z = 0.0
        twist.linear.x = vel[0]
        twist.linear.y = vel[1]
        twist.angular.z = vel[2]
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        self.cmd_vel_puber.publish(twist)

    def trimerworkCallback(self, req):
        if req.mode == 0:
            self.reset_arm()
            rospy.sleep(0.2)
            self.open_gripper()
            resp = graspsignalResponse()
            resp.res = True
            resp.response = "reset arm position and open gripper"
            return resp

        initial_time = rospy.get_time()
        while rospy.get_time() - self.image_time_now > 0.1:
            rospy.loginfo("latency detected!")
            if rospy.get_time() - initial_time > 3.0:
                self.sendBaseVel([-0.2, 0.0, 0.0])
                rospy.sleep(0.5)
                self.sendBaseVel([-0.2, 0.0, 0.0])
            if rospy.get_time() - initial_time > 6.0:
                resp = graspsignalResponse()
                resp.res = True
                resp.response = "Successfully Grasp fake"
                return resp
            rospy.sleep(0.1)

        rate = rospy.Rate(self.ros_rate)

        resp = graspsignalResponse()

        if req.mode == 1:
            rospy.loginfo("First trim then grasp")
            rospy.loginfo("Trim to the right place")

            self.open_gripper()
            rospy.sleep(0.1)
            adjust_time = 0.1

            flag = 0
            y_threshold_p = 0.018
            y_threshold_n = 0.018
            x_dis_tar = 0.385
            while not rospy.is_shutdown():
                target_marker_pose = self.current_marker_poses
                if target_marker_pose is None:
                    continue

                target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                    target_marker_pose
                )
                cmd_vel = [0.0, 0.0, 0.0]

                if (target_pos[0] - x_dis_tar) > 0.13:
                    cmd_vel[0] = 0.3
                elif (target_pos[0] - x_dis_tar) > 0.08:
                    cmd_vel[0] = 0.2
                elif (target_pos[0] - x_dis_tar) < -0.08:
                    cmd_vel[0] = -0.15
                elif (target_pos[0] - x_dis_tar) > 0.07:
                    cmd_vel[0] = 0.15
                elif (target_pos[0] - x_dis_tar) < -0.07:
                    cmd_vel[0] = -0.15
                elif (target_pos[0] - x_dis_tar) > 0.02:
                    cmd_vel[0] = 0.12
                elif (target_pos[0] - x_dis_tar) < -0.02:
                    cmd_vel[0] = -0.12

                if (target_pos[1] - 0.0) > 0.035 or (target_pos[1] - 0.0) < -0.035:
                    cmd_vel[0] = 0.0

                if (target_pos[1] - 0.0) > y_threshold_p:
                    if flag == 0:
                        flag = 1
                        y_threshold_p += 0.01
                    cmd_vel[1] = 0.11
                elif (target_pos[1] - 0.0) < -y_threshold_n:
                    if flag == 0:
                        flag = 1
                        y_threshold_n += 0.01
                    cmd_vel[1] = -0.11

                flag = 1

                cmd_vel[2] = 0
                if np.abs(target_pos[0] - x_dis_tar) <= 0.02 and (
                    (target_pos[1] - 0.0) <= y_threshold_p
                    and (0.0 - target_pos[1]) <= y_threshold_n
                ):
                    cmd_vel = [0.0, 0.0, 0.0]
                self.sendBaseVel(cmd_vel)
                if np.abs(target_pos[0] - x_dis_tar) <= 0.02 and (
                    (target_pos[1] - 0.0) <= y_threshold_p
                    and (0.0 - target_pos[1]) <= y_threshold_n
                ):
                    pose = Pose()
                    pose.position.x = 0.19
                    pose.position.y = -0.08
                    self.sendBaseVel([0.25, 0.0, 0.0])
                    rospy.sleep(0.3)
                    self.sendBaseVel([0.0, 0.0, 0.0])
                    rospy.sleep(1.0)
                    self.arm_position_pub.publish(pose)
                    rospy.sleep(1.0)
                    rospy.loginfo("Place: reach the goal for placing.")
                    break
                adjust_time += 0.0333
                rate.sleep()

            target_marker_pose = self.current_marker_poses
            self.close_gripper()
            rospy.sleep(1.0)

            self.close_gripper()
            rospy.sleep(1.0)
            self.reset_arm()

            self.sendBaseVel([-0.3, 0.0, 0.0])
            rospy.sleep(0.4)
            self.sendBaseVel([0.0, 0.0, 0.0])

            resp.res = True
            resp.response = str(target_angle)
            return resp

        elif req.mode == 2:
            rospy.loginfo("First trim then place")

            self.pre()
            flag = 0
            y_threshold_p = 0.018
            y_threshold_n = 0.018
            x_dis_tar = 0.385

            while not rospy.is_shutdown():
                target_marker_pose = self.current_marker_poses
                if target_marker_pose is None:
                    continue

                target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                    target_marker_pose
                )
                cmd_vel = [0.0, 0.0, 0.0]

                if (target_pos[0] - x_dis_tar) > 0.1:
                    cmd_vel[0] = 0.14
                elif (target_pos[0] - x_dis_tar) < -0.1:
                    cmd_vel[0] = -0.14
                elif (target_pos[0] - x_dis_tar) > 0.02:
                    cmd_vel[0] = 0.12
                elif (target_pos[0] - x_dis_tar) < -0.02:
                    cmd_vel[0] = -0.12

                if (target_pos[1] - 0.0) > 0.05 or (target_pos[1] - 0.0) < -0.05:
                    cmd_vel[0] = 0.0

                if (target_pos[1] - 0.0) > y_threshold_p:
                    if flag == 0:
                        flag = 1
                        y_threshold_p += 0.01
                    cmd_vel[1] = 0.11
                elif (target_pos[1] - 0.0) < -y_threshold_n:
                    if flag == 0:
                        flag = 1
                        y_threshold_n += 0.01
                    cmd_vel[1] = -0.11
                flag = 1

                cmd_vel[2] = 0
                self.sendBaseVel(cmd_vel)
                if np.abs(target_pos[0] - x_dis_tar) <= 0.02 and (
                    (target_pos[1] - 0.0) <= y_threshold_p
                    and (0.0 - target_pos[1]) <= y_threshold_n
                ):
                    rospy.loginfo("Trim well in the all dimention, going open loop")
                    self.sendBaseVel([0.0, 0.0, 0.0])
                    rospy.sleep(1.0)
                    self.sendBaseVel([0.25, 0.0, 0.0])
                    rospy.sleep(0.3)
                    self.sendBaseVel([0.25, 0.0, 0.0])
                    rospy.sleep(0.3)
                    self.sendBaseVel([0.0, 0.0, 0.0])
                    rospy.loginfo("Place: reach the goal for placing.")
                    break
                rate.sleep()

            rospy.loginfo("Trim well in the horizon dimention")

            target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                self.current_marker_poses
            )
            self.sendBaseVel([0.0, 0.0, 0.0])
            rospy.sleep(1.0)
            self.open_gripper()
            rospy.sleep(1.0)
            reset_thread = threading.Thread(target=self.reset_arm)
            reset_thread.start()

            self.sendBaseVel([-0.3, 0.0, 0.0])
            rospy.sleep(0.6)
            self.sendBaseVel([0.0, 0.0, 0.0])

            resp.res = True
            resp.response = "Successfully Place"

        return resp

    def open_gripper(self):
        open_gripper_msg = Point()
        open_gripper_msg.x = 0.0
        open_gripper_msg.y = 0.0
        open_gripper_msg.z = 0.0
        rospy.loginfo("open the gripper")
        self.arm_gripper_pub.publish(open_gripper_msg)

    def close_gripper(self):
        close_gripper_msg = Point()
        close_gripper_msg.x = 1.0
        close_gripper_msg.y = 0.0
        close_gripper_msg.z = 0.0
        rospy.loginfo("close the gripper")
        self.arm_gripper_pub.publish(close_gripper_msg)

    def reset_arm(self):
        reset_arm_msg = Pose()
        reset_arm_msg.position.x = 0.1
        reset_arm_msg.position.y = 0.12
        reset_arm_msg.position.z = 0.0
        reset_arm_msg.orientation.x = 0.0
        reset_arm_msg.orientation.y = 0.0
        reset_arm_msg.orientation.z = 0.0
        reset_arm_msg.orientation.w = 0.0
        rospy.loginfo("reset the arm")
        self.arm_position_pub.publish(reset_arm_msg)

    def pre(self):
        rospy.loginfo("<manipulater>: now prepare to grip")
        pose = Pose()
        pose.position.x = 0.21
        pose.position.y = 0.0
        self.arm_position_pub.publish(pose)


if __name__ == "__main__":
    rospy.init_node("manipulater_node", anonymous=True)
    rter = manipulater()
    rospy.spin()

