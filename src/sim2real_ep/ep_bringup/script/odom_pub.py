#!/usr/bin/env python
import tf
import rospy
import robomaster
from robomaster import robot
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3

global odom_publisher
global imu_publisher
global pos_x
global pos_y
global ang_z
global vel_x
global vel_y
global ang_vel_z

def sub_position_handler(position_info):
    x, y, z = position_info
    #print("chassis position: x:{0}, y:{1}, z:{2}".format(x, y, z))
    global pos_x
    global pos_y
    pos_x = x
    pos_y = y
    if rospy.is_shutdown():
        sys.exit(1)

def sub_attitude_info_handler(attitude_info):
    yaw, pitch, roll = attitude_info
    #print("chassis attitude: yaw:{0}, pitch:{1}, roll:{2} ".format(yaw, pitch, roll))
    global ang_z
    ang_z = yaw
    if rospy.is_shutdown():
        sys.exit(1)

def sub_imu_info_handler(imu_info):
    acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_info
    #print("chassis imu: acc_x:{0}, acc_y:{1}, acc_z:{2}, gyro_x:{3}, gyro_y:{4}, gyro_z:{5}".format(
    #    acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z))
    global ang_vel_z
    global imu_publisher
    global ang_z
    ang_vel_z = gyro_z
    imu = Imu()

    imu.header.stamp= rospy.Time.now()
    imu.header.frame_id = "imu_link"

    imu.orientation_covariance[0] = 0
    imu.orientation_covariance[1] = 0
    imu.orientation_covariance[2] = 0
    imu.orientation_covariance[3] = 0
    imu.orientation_covariance[4] = 0
    imu.orientation_covariance[5] = 0
    imu.orientation_covariance[6] = 0
    imu.orientation_covariance[7] = 0
    imu.orientation_covariance[8] = 0
    imu.angular_velocity_covariance[8]=0
    
    imu.angular_velocity_covariance[0]=0
    imu.angular_velocity_covariance[1]=0
    imu.angular_velocity_covariance[2]=0
    imu.angular_velocity_covariance[3]=0
    imu.angular_velocity_covariance[4]=0
    imu.angular_velocity_covariance[5]=0
    imu.angular_velocity_covariance[6]=0
    imu.angular_velocity_covariance[7]=0

    imu.linear_acceleration_covariance[0]=0
    imu.linear_acceleration_covariance[1]=0
    imu.linear_acceleration_covariance[2]=0
    imu.linear_acceleration_covariance[3]=0
    imu.linear_acceleration_covariance[4]=0
    imu.linear_acceleration_covariance[5]=0
    imu.linear_acceleration_covariance[6]=0
    imu.linear_acceleration_covariance[7]=0
    imu.linear_acceleration_covariance[8]=0

    orientation = tf.transformations.quaternion_from_euler(0, 0, ang_z)
    imu.orientation=orientation

    imu.linear_acceleration.x = acc_x
    imu.linear_acceleration.y = acc_y
    imu.linear_acceleration.z = acc_z

    imu.angular_velocity.x = gyro_x
    imu.angular_velocity.y = gyro_y
    imu.angular_velocity.z = gyro_z

    imu_publisher.publish(imu)
    print(acc_x, acc_y, acc_z)


def sub_velocity_info_handler(imu_info):
    vgx, vgy, vgz, vbx, vby, vbz = imu_info
    #print("chassis velocity: vgx:{0}, vgy:{1}, vgz:{2}, vbx:{3}, vby:{4}, vbz:{5}".format(
    #    vgx, vgy, vgz, vbx, vby, vbz))
    global vel_x
    global vel_y
    vel_x = vbx
    vel_y = vby
    
    global pos_x
    global pos_y
    global ang_z
    global ang_vel_z
    global odom_publisher
    
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, ang_z)
    
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    
    odom.pose.pose = Pose(Point(pos_x, pos_y, 0.), Quaternion(*odom_quat))
    
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vel_x, vel_y, 0), Vector3(0, 0, ang_vel_z))
    
    odom_publisher.publish(odom)
    if rospy.is_shutdown():
        sys.exit(1)
    #print(pos_x, pos_y, ang_z, vel_x, vel_y, ang_vel_z)
    

if __name__ == '__main__':
    rospy.init_node('ep_odom_node', anonymous=True)
    global odom_publisher
    global imu_publisher
    odom_publisher = rospy.Publisher("/ep/odom", Odometry, queue_size=10)
    imu_publisher = rospy.Publisher("/ep/imu", Imu, queue_size=1)
    # 如果本地IP 自动获取不正确，手动指定本地IP地址
    # robomaster.config.LOCAL_IP_STR = "192.168.2.20"
    ep_robot = robot.Robot()

    # 指定连接方式为AP 直连模式
    ep_robot.initialize(conn_type='rndis')

    ep_chassis = ep_robot.chassis

    # 订阅底盘位置信息
    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)
    #ep_chassis.sub_imu(freq=10, callback=sub_imu_info_handler)
    ep_chassis.sub_velocity(freq=10, callback=sub_velocity_info_handler)
    #ep_chassis.unsub_position()
    #ep_chassis.unsub_attitude()
    
    #ep_robot.close()
