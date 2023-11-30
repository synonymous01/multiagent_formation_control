#!/usr/bin/env/ python3
from robomaster import robot
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import numpy as np


def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]


class rover:
    def __init__(self) -> None:
        self.imu_pub = rospy.Publisher('imu_data', Imu, queue_size=10)
        self.pose_pub = rospy.Publisher('pose_data', Point, queue_size=10)
        self.angle_pub = rospy.Publisher('angle_data', )
        rospy.init_node('robomaster', anonymous=True)
        self.robo = robot.Robot()
        self.robo.initialize(conn_type="rndis")
        self.chass = self.robo.chassis
        self.chass.sub_imu(10, self.update_imu)
        self.chass.sub_position(0, 5, self.update_pose)
        self.chass.sub_attitude(5, self.update_angles)


    def update_imu(self, imu_info):
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_info
        sending = Imu()
        gyro_x = gyro_x * (180/np.pi)
        gyro_y = gyro_y * (180/np.pi)
        gyro_z = gyro_z * (180/np.pi)
        sending.linear_acceleration.x = acc_x
        sending.linear_acceleration.y = acc_y
        sending.linear_acceleration.z = acc_z
        sending.angular_velocity.x = gyro_x
        sending.angular_velocity.y = gyro_y
        sending.angular_velocity.z = gyro_z
        self.imu_pub.publish(sending)

    def update_pose(self, pose_info):
        x, y, z = pose_info
        sending = Point()
        sending.x = x
        sending.y = y
        sending.z = z
        self.pose_pub.publish(sending)

    def update_angles(self, angle_info):
        yaw, pitch, roll = angle_info
        angles = get_quaternion_from_euler(roll, pitch, yaw)
        sending = Quaternion()
        sending.x = angles[0]
        sending.y = angles[1]
        sending.z = angles[2]
        sending.w = angles[3]
        self.angle_pub.publish(sending)


robo = rover()
try:
    while True:
        pass
except KeyboardInterrupt:
    pass


    






    