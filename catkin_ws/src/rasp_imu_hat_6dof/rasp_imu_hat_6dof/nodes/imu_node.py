#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# Copyright: 2016-2020 https://www.corvin.cn ROS小课堂
# Author: corvin
# Description: 为树莓派IMU扩展板所使用的配套代码，由于默认
#    扩展板与树莓派使用IIC连接。所以这里的代码是直接从IIC接口
#    中读取IMU模块的三轴加速度、角度、四元数，然后组装成ROS
#    中的IMU消息格式，发布到/imu话题中，这样有需要的节点可以
#    直接订阅该话题即可获取到imu扩展板当前的数据。
# History:
#    20191031:Initial this file.
#    20191209:新增发布IMU芯片温度的话题，发布频率与发布imu数据频率相同.
#    20200406:增加可以直接发布yaw数据的话题.
#    20200410:增加通过service方式发布yaw数据,方便其他节点调用.
#    20200703:修改三轴线性加速度的方向,沿着各轴正方向数据为正,否则加速度就为负值,单位m/s2.
import rospy
import math

from rasp_imu_hat_6dof.srv import GetYawData,GetYawDataResponse
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from rasp_imu_hat_6dof.cfg import imuConfig
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from imu_data import MyIMU


degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0
yaw = 0.0

# ros server return Yaw data
def return_yaw_data(req):
    return GetYawDataResponse(yaw)

# Callback for dynamic reconfigure requests
def reconfig_callback(config, level):
    global imu_yaw_calibration
    imu_yaw_calibration = config['yaw_calibration']
    rospy.loginfo("Set imu_yaw_calibration to %f" % (imu_yaw_calibration))
    return config

rospy.init_node("imu_node")

# Get DIY config param
data_topic_name = rospy.get_param('~pub_data_topic', 'imu')
temp_topic_name = rospy.get_param('~pub_temp_topic', 'imu_temp')
link_name  = rospy.get_param('~link_name', 'imu_link')
pub_hz = rospy.get_param('~pub_hz', '20')
yaw_topic_name = rospy.get_param('~yaw_topic', 'yaw_topic')

data_pub = rospy.Publisher(data_topic_name, Imu, queue_size=1)
temp_pub = rospy.Publisher(temp_topic_name, Float32, queue_size=1)
yaw_pub = rospy.Publisher(yaw_topic_name, Float32, queue_size=1)
srv = Server(imuConfig, reconfig_callback)  # define dynamic_reconfigure callback
yaw_srv = rospy.Service('imu_node/GetYawData', GetYawData, return_yaw_data)

imuMsg = Imu()
yawMsg = Float32()

# Orientation covariance estimation
imuMsg.orientation_covariance = [
0.0025 , 0 , 0,
0, 0.0025, 0,
0, 0, 0.0025
]

# Angular velocity covariance estimation
imuMsg.angular_velocity_covariance = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

# linear acceleration covariance estimation
imuMsg.linear_acceleration_covariance = [
0.04 , 0 , 0,
0 , 0.04, 0,
0 , 0 , 0.04
]

seq=0
# sensor accel g convert to m/s^2.
accel_factor = 9.806

myIMU = MyIMU(0x50)
rate = rospy.Rate(pub_hz)

rospy.loginfo("IMU Module is working ...")
while not rospy.is_shutdown():
    myIMU.get_YPRAG()

    #rospy.loginfo("yaw:%f pitch:%f  roll:%f", myIMU.raw_yaw,
    #              myIMU.raw_pitch, myIMU.raw_roll)
    yaw_deg = float(myIMU.raw_yaw)
    yaw_deg = yaw_deg + imu_yaw_calibration
    if yaw_deg >= 180.0:
        yaw_deg -= 360.0

    #rospy.loginfo("yaw_deg: %f", yaw_deg)
    yaw = yaw_deg*degrees2rad
    pitch = float(myIMU.raw_pitch)*degrees2rad
    roll  = float(myIMU.raw_roll)*degrees2rad

    yawMsg.data = yaw
    yaw_pub.publish(yawMsg)

    # Publish imu message
    #rospy.loginfo("acc_x:%f acc_y:%f acc_z:%f", myIMU.raw_ax,
    #              myIMU.raw_ay, myIMU.raw_az)
    imuMsg.linear_acceleration.x = -float(myIMU.raw_ax)*accel_factor
    imuMsg.linear_acceleration.y = -float(myIMU.raw_ay)*accel_factor
    imuMsg.linear_acceleration.z = -float(myIMU.raw_az)*accel_factor

    imuMsg.angular_velocity.x = float(myIMU.raw_gx)*degrees2rad
    imuMsg.angular_velocity.y = float(myIMU.raw_gy)*degrees2rad
    imuMsg.angular_velocity.z = float(myIMU.raw_gz)*degrees2rad

    # From IMU module get quatern param
    #myIMU.get_quatern()
    #imuMsg.orientation.x = myIMU.raw_q1
    #imuMsg.orientation.y = myIMU.raw_q2
    #imuMsg.orientation.z = myIMU.raw_q3
    #imuMsg.orientation.w = myIMU.raw_q0

    # Calculate quatern param from roll,pitch,yaw by ourselves
    q = quaternion_from_euler(roll, pitch, yaw)
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]

    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = link_name
    imuMsg.header.seq = seq
    seq = seq + 1
    data_pub.publish(imuMsg)

    # Get imu module temperature, then publish to topic
    myIMU.get_temp()
    temp_pub.publish(myIMU.temp)

    rate.sleep()

