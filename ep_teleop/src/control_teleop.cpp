#include "control_teleop.h"
using namespace CONTROLTELEOP;

ControlTeleop::ControlTeleop(bool publish_vel, double max_linear_velocity,
                             double max_angular_velocity, bool diff_drive) {
  control_trigger_ = Default;
  publish_vel_ = publish_vel;
  diff_drive_ = diff_drive;
  Joy_sub_ = nh_.subscribe("joy", 100, &ControlTeleop::JoyCallback, this);
  if (publish_vel) {
    max_angular_velocity_ = max_angular_velocity;
    max_linear_velocity_ = max_linear_velocity;
    joy_vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    arm_position_pub = nh_.advertise<geometry_msgs::Pose>("arm_position",1);
    arm_gripper_pub = nh_.advertise<geometry_msgs::Point>("arm_gripper",1);
    cancel_goal_pub = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel",1);
    ROS_INFO_STREAM(
        "Enable joy control, velocity commands are published in topic: "
        << joy_vel_pub.getTopic());
    ROS_INFO_STREAM("Max linear vel:  " << max_linear_velocity_);
    ROS_INFO_STREAM("Max angular vel: " << max_angular_velocity_);
    if (diff_drive_)
      ROS_INFO_STREAM("Using differential drive.");
    else
      ROS_INFO_STREAM("Using omnidirectional drive.");
  }
  joy_watchdog_timer_ = nh_.createTimer(
      ros::Duration(WATCHDOG_PERIOD_), &ControlTeleop::Joywatchdog, this, true);
  joy_watchdog_timer_.start();
}

void ControlTeleop::Joywatchdog(const ros::TimerEvent &e) {
  // ROS_WARN("joy not received for %f seconds, is the joy node drop?",
  //          WATCHDOG_PERIOD_);
  this->joy_alive_ = false;
}
void ControlTeleop::JoyCallback(const sensor_msgs::JoyConstPtr &msg) {
  joy_watchdog_timer_.stop();
  joy_watchdog_timer_.start();
  boost::unique_lock<boost::shared_mutex> writLock(control_trigger_mutex_);
  geometry_msgs::Pose arm_position;
  geometry_msgs::Point arm_gripper;
  if (msg->axes[2] == -1 && msg->buttons[0])  // A
  {
  } else if (msg->axes[2] == -1 && msg->buttons[1])  // B
  {
  } else if (msg->axes[2] == -1 && msg->buttons[2])  // X
  {
  } else if (msg->axes[2] == -1 && msg->buttons[3])  // Y
  {
  } else if (msg->axes[2] == -1 && msg->axes[6] == 1) {
  } else if (msg->axes[2] == -1 && msg->axes[6] == -1) {
  } else if (msg->axes[2] == -1 && msg->axes[7] == 1) {
  } else if (msg->axes[2] == -1 && msg->axes[7] == -1) {
  } else if (msg->axes[2] == -1 && msg->buttons[5])  //
  {
  } else if (msg->axes[2] == -1 && msg->buttons[4]) {
  } else if (msg->axes[2] == -1 && msg->axes[5] == -1) {
  }
//  else if(msg->axes[2]==-1&&msg->axes[7]==1)//up
//  {
//  }
//  else if(msg->axes[2]==-1&&msg->axes[7]==-1)//down
//  {
//  }
  else if (msg->axes[5] == -1 && msg->buttons[0])  // A
  {
  } else if (msg->axes[5] == -1 && msg->buttons[1])  // B
  {
  } else if (msg->axes[5] == -1 && msg->buttons[2])  // X
  {
  } else if (msg->axes[5] == -1 && msg->axes[6] == 1)  // left
  {
  } else if (msg->axes[5] == -1 && msg->axes[6] == -1)  // right
  {
  } else if (msg->axes[5] == -1 && msg->buttons[3])  // Y
  {
  } else if (msg->axes[5] == -1 && msg->buttons[10]) {
  } else if (msg->axes[5] == -1 && msg->buttons[4]) {
  } else if (msg->axes[5] == -1 && msg->buttons[5]) {
  } else if (msg->buttons[0]) {
    arm_position.position.x = 0.21;
    arm_position.position.y = -0.02;
    arm_position_pub.publish(arm_position);
  } else if (msg->buttons[1]) {
    arm_position.position.x = 0.21;
    arm_position.position.y = 0.1;
    arm_position_pub.publish(arm_position);
  } else if (msg->buttons[2]) {
    arm_gripper.x = 1;
    arm_gripper_pub.publish(arm_gripper);
  } else if (msg->buttons[3]) {
    arm_gripper.x = 0;
    arm_gripper_pub.publish(arm_gripper);
  } else if (msg->buttons[4])  // L1
  {
  } else if (msg->buttons[5])  // R1 
  {
    actionlib_msgs::GoalID temp;
    cancel_goal_pub.publish(temp);
  } else if (msg->buttons[6]) {
  } else if (msg->buttons[7]) {
  } else if (msg->buttons[9]) {
  } else if (msg->buttons[10]) {
  } else
    control_trigger_ = Default;

  if (publish_vel_) {
    geometry_msgs::Twist vel{};
    if (diff_drive_) {
      if (msg->axes[0] || msg->axes[1]) {
        vel.angular.z = msg->axes[0] * max_angular_velocity_;
        vel.linear.x = msg->axes[1] * max_linear_velocity_;
        if (msg->axes[3] || msg->axes[4]) {
          vel.angular.z = msg->axes[0] * max_angular_velocity_ +
                          msg->axes[3] * max_angular_velocity_ / 2.0;
          vel.linear.x = msg->axes[1] * max_linear_velocity_ +
                         msg->axes[4] * max_linear_velocity_ / 2.0;
        }
      } else if (msg->axes[3] || msg->axes[4]) {
        vel.angular.z = msg->axes[3] * max_angular_velocity_ / 2.0;
        vel.linear.x = msg->axes[4] * max_linear_velocity_ / 2.0;
        if (msg->axes[0] || msg->axes[1]) {
          vel.angular.z = msg->axes[0] * max_angular_velocity_ +
                          msg->axes[3] * max_angular_velocity_ / 2.0;
          vel.linear.x = msg->axes[1] * max_linear_velocity_ +
                         msg->axes[4] * max_linear_velocity_ / 2.0;
        }
      }
    } else {
      if (msg->axes[0] || msg->axes[1] || msg->axes[2] != 1 ||
          msg->axes[5] != 1) {
        vel.linear.y = msg->axes[0] * max_linear_velocity_;
        vel.linear.x = msg->axes[1] * max_linear_velocity_;
        vel.angular.z =
            (-msg->axes[2] + msg->axes[5]) * max_angular_velocity_ / 2;
        if (msg->axes[3] || msg->axes[4]) {
          vel.linear.y = msg->axes[0] * max_linear_velocity_ +
                         msg->axes[3] * max_linear_velocity_ / 2.0;
          vel.linear.x = msg->axes[1] * max_linear_velocity_ +
                         msg->axes[4] * max_linear_velocity_ / 2.0;
          vel.angular.z =
              (-msg->axes[2] + msg->axes[5]) * max_angular_velocity_ / 2;
        }
      } else if (msg->axes[3] || msg->axes[4]) {
        vel.linear.y = msg->axes[3] * max_linear_velocity_ / 2.0;
        vel.linear.x = msg->axes[4] * max_linear_velocity_ / 2.0;
        vel.angular.z =
            (-msg->axes[2] + msg->axes[5]) * max_angular_velocity_ / 2;
        if (msg->axes[0] || msg->axes[1]) {
          vel.linear.y = msg->axes[0] * max_linear_velocity_ +
                         msg->axes[3] * max_linear_velocity_ / 2.0;
          vel.linear.x = msg->axes[1] * max_linear_velocity_ +
                         msg->axes[4] * max_linear_velocity_ / 2.0;
          vel.angular.z =
              (-msg->axes[2] + msg->axes[5]) * max_angular_velocity_ / 2;
        }
      }
    }
    joy_vel_pub.publish(vel);
  }
}

int main(int argc, char ** argv){
  ros::init(argc, argv, "control_teleop_node");
  ros::NodeHandle nh_("~");
  ros::Rate loop_rate(30);
  double max_joy_linear_velocity, max_joy_angular_velocity;
  nh_.param("max_joy_linear_velocity", max_joy_linear_velocity, (double)0.5);
  nh_.param("max_joy_angular_velocity", max_joy_angular_velocity, (double)1.0);
  std::shared_ptr<CONTROLTELEOP::ControlTeleop> controlTeleop = std::make_shared<CONTROLTELEOP::ControlTeleop>(
      true, max_joy_linear_velocity, max_joy_linear_velocity, false);
  ROS_INFO("Control_teleop_node is running.");
  ros::spin();
}

