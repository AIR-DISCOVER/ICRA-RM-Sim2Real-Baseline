#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from math import pi

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from rmus_solution.srv import setgoal, setgoalResponse

import tf2_ros
from tf_conversions import transformations

class router:
    """
    brief
    ----
    发布move_base目标
    """

    def __init__(self) -> None:
        self.M_reach_goal = False

        # 所有观察点的索引、名称、位置(posi_x,pose_y,yaw)、误差容限(posi,angle)
        self.mission_point = {
            0: ("home", [0.00, 0.00, 0.00], [0.02, 0.05]),
            1: ("cube1", [0.10, 2.75, pi / 2], [0.05, 0.1]),  # 从起点来的
            10: ("cube1", [0.60, 3.20, pi], [0.05, 0.1]),  # 从交换站来的
            2: ("cube2", [-0.03, 2.35, 0.00], [0.025, 0.05]),
            3: ("cube3", [2.05, 2.68, 0.00], [0.075, 0.1]),
            4: ("cube4", [2.13, 0.21, -pi], [0.05, 0.1]),
            5: ("cube5", [2.76, -0.80, 0.00], [0.05, 0.1]),
            6: ("station1", [1.18, 1.91, 0.00], [0.075, 0.1]),
            7: ("station2", [1.18, 1.80, 0.00], [0.075, 0.1]),
            8: ("station3", [1.18, 1.65, 0.00], [0.075, 0.1]),
            9: ("noticeboard", [0.00, 1.60, 0.00], [0.05, 0.1]),
        }


        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.cmd_vel_puber = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        while not rospy.is_shutdown():
            try:
                self.tfBuffer.lookup_transform(
                    "map", "base_link", rospy.Time(), timeout=rospy.Duration(2)
                )
                rospy.loginfo("Get tf from map to base_link")
                break
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                rospy.logwarn("Waiting for tf from map to base_link")
            rospy.sleep(0.5)

        rospy.Subscriber(
            "/move_base/result", MoveBaseActionResult, self.MoveBaseResultCallback
        )
        self.goal_puber = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )
        self.cancel_goal_puber = rospy.Publisher(
            "/move_base/cancel", GoalID, queue_size=10
        )
        self.get_trapped_cnt = 0  # 单点被困计数,任务完成后或者current point变换后都会清零

        self.service = rospy.Service(
            "/set_navigation_goal", setgoal, self.setgoalCallback
        )
        self.mission = -1  # 当前的任务 int valid:{0,1,2,3,4,5}
        self.last_mission = -1

    def MoveBaseResultCallback(self, msg: MoveBaseActionResult):
        if msg.status.status == 3:
            self.M_reach_goal = True

    def pubMovebaseMissionGoal(self):
        simple_goal = PoseStamped()
        simple_goal.header.stamp = rospy.Time.now()

        simple_goal.header.frame_id = "map"
        simple_goal.pose.position.x = self.mission_point[self.mission][1][0]
        simple_goal.pose.position.y = self.mission_point[self.mission][1][1]
        simple_goal.pose.position.z = 0.0
        quat = transformations.quaternion_from_euler(
            0.0, 0.0, self.mission_point[self.mission][1][2]
        )
        simple_goal.pose.orientation.x = quat[0]
        simple_goal.pose.orientation.y = quat[1]
        simple_goal.pose.orientation.z = quat[2]
        simple_goal.pose.orientation.w = quat[3]
        self.goal_puber.publish(simple_goal)

    def setgoalCallback(self, req):
        resp = setgoalResponse()
        rospy.loginfo(">>>>>>>>>>>>>>>>>>>>>>>>>")
        rospy.loginfo("req: call = {} point = {}".format(req.call, req.point))
        rospy.loginfo("last mission = {}".format(self.last_mission))

        if 0 <= req.point <= len(self.mission_point):
            if req.point == 1 and (5 < self.last_mission < 9):
                self.mission = 10
            else:
                self.mission = req.point
            self.pubMovebaseMissionGoal()

            self.M_reach_goal = False

            r = rospy.Rate(10)
            while not rospy.is_shutdown():
                if self.M_reach_goal:
                    rospy.loginfo(
                        "Reach Goal {}!".format(self.mission_point[req.point][0])
                    )
                    resp.res = True
                    resp.response = "Accomplish!"

                    self.last_mission = self.mission
                    self.mission = -1
                    self.get_trapped_cnt = 0
                    break

                r.sleep()

        else:
            rospy.loginfo("Invalid request!")
            resp.res = False
            resp.response = "Invalid request!"

        return resp


if __name__ == "__main__":
    rospy.init_node("router", anonymous=True)
    rospy.loginfo(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    rter = router()
    rospy.spin()
