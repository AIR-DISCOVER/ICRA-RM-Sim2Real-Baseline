#ifndef CARTO_NAVIGATION_CONTROL_TELEOP_H
#define CARTO_NAVIGATION_CONTROL_TELEOP_H
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <utility>

#include "atomic"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include <actionlib_msgs/GoalID.h>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
namespace CONTROLTELEOP {
enum ControlTrigger {
  Default
};
class ControlTeleop {
 private:
  const double WATCHDOG_PERIOD_ = 2.0;
  ros::NodeHandle nh_;
  ros::Timer joy_watchdog_timer_;
  ros::Subscriber Joy_sub_;
  ros::Publisher joy_vel_pub,arm_position_pub,arm_gripper_pub, cancel_goal_pub;
  std::atomic_bool joy_alive_{};
  bool publish_vel_{};
  bool diff_drive_{};
  ControlTrigger control_trigger_{};
  double max_linear_velocity_{};
  double max_angular_velocity_{};
  mutable boost::shared_mutex control_trigger_mutex_;

 public:
  explicit ControlTeleop(bool publish_vel = false,
                         double max_linear_velocity = 0.8,
                         double max_angular_velocity = 0.5,
                         bool diff_drive = true);
  ~ControlTeleop() = default;
  void JoyCallback(const sensor_msgs::JoyConstPtr &msg);
  void Joywatchdog(const ros::TimerEvent &e);
  ControlTrigger getControlTrigger() {
    boost::unique_lock<boost::shared_mutex> writLock(control_trigger_mutex_);
    ControlTrigger temp{control_trigger_};
    control_trigger_ = Default;
    return temp;
  };
};
}  // namespace CONTROLTELEOP

#endif  // CARTO_NAVIGATION_CONTROL_TELEOP_H
