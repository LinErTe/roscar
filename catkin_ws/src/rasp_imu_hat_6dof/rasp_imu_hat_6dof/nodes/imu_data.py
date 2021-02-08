#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# Copyright: 2016-2019 https://www.corvin.cn ROS小课堂
# Author: corvin
# Description: 为树莓派IMU扩展板所使用的配套代码，由于默认
#    扩展板与树莓派使用IIC连接。所以这里的代码是直接从IIC接口
#    中读取IMU模块的三轴加速度、三轴角速度、四元数。
# History:
#    20191031: Initial this file.
#    20191209：Add get imu chip temperature function-get_temp().
#    20200702: 读取各数据后需要使用np.short转换才能进行后续数据处理.
import smbus
import rospy
import numpy as np

class MyIMU(object):
    def __init__(self, addr):
        self.addr = addr
        self.i2c = smbus.SMBus(1)

    def get_YPRAG(self):
        try:
            roll_tmp  = self.i2c.read_i2c_block_data(self.addr, 0x3d, 2)
            pitch_tmp = self.i2c.read_i2c_block_data(self.addr, 0x3e, 2)
            yaw_tmp   = self.i2c.read_i2c_block_data(self.addr, 0x3f, 2)

            ax_tmp  = self.i2c.read_i2c_block_data(self.addr, 0x34, 2)
            ay_tmp  = self.i2c.read_i2c_block_data(self.addr, 0x35, 2)
            az_tmp  = self.i2c.read_i2c_block_data(self.addr, 0x36, 2)
            #rospy.loginfo("Read i2c accX:" + str(ax_tmp))
            #rospy.loginfo("Read i2c accY:" + str(ay_tmp))
            #rospy.loginfo("Read i2c accZ:" + str(az_tmp))

            gx_tmp  = self.i2c.read_i2c_block_data(self.addr, 0x37, 2)
            gy_tmp  = self.i2c.read_i2c_block_data(self.addr, 0x38, 2)
            gz_tmp  = self.i2c.read_i2c_block_data(self.addr, 0x39, 2)
        except IOError:
            rospy.logerr("Read IMU YPRAG date error !")
        else:
            self.raw_roll  = float(((roll_tmp[1]<<8) |roll_tmp[0])/32768.0*180.0)
            self.raw_pitch = float(((pitch_tmp[1]<<8)|pitch_tmp[0])/32768.0*180.0)
            self.raw_yaw  = (yaw_tmp[1] << 8 | yaw_tmp[0])/32768.0*180

            self.raw_ax = float(np.short(np.short(ax_tmp[1]<<8)|ax_tmp[0])/32768.0*16.0)
            self.raw_ay = float(np.short(np.short(ay_tmp[1]<<8)|ay_tmp[0])/32768.0*16.0)
            self.raw_az = float(np.short(np.short(az_tmp[1]<<8)|az_tmp[0])/32768.0*16.0)
            #rospy.loginfo("Read raw accX:" + str(self.raw_ax))
            #rospy.loginfo("Read raw accY:" + str(self.raw_ay))
            #rospy.loginfo("Read raw accZ:" + str(self.raw_az))

            self.raw_gx = float(np.short(np.short(gx_tmp[1]<<8)|gx_tmp[0])/32768.0*2000.0)
            self.raw_gy = float(np.short(np.short(gy_tmp[1]<<8)|gy_tmp[0])/32768.0*2000.0)
            self.raw_gz = float(np.short(np.short(gz_tmp[1]<<8)|gz_tmp[0])/32768.0*2000.0)

    def get_quatern(self):
        try:
            q0 = self.i2c.read_i2c_block_data(self.addr, 0x51, 2)
            q1 = self.i2c.read_i2c_block_data(self.addr, 0x52, 2)
            q2 = self.i2c.read_i2c_block_data(self.addr, 0x53, 2)
            q3 = self.i2c.read_i2c_block_data(self.addr, 0x54, 2)
        except IOError:
            rospy.logerr("Read IMU quaternion date error !")
        else:
            self.raw_q0 = float((np.short((q0[1]<<8)|q0[0]))/32768.0)
            self.raw_q1 = float((np.short((q1[1]<<8)|q1[0]))/32768.0)
            self.raw_q2 = float((np.short((q2[1]<<8)|q2[0]))/32768.0)
            self.raw_q3 = float((np.short((q3[1]<<8)|q3[0]))/32768.0)

    def get_two_float(self, data, n):
        data = str(data)
        a, b, c = data.partition('.')
        c = (c+"0"*n)[:n]
        return ".".join([a,c])

    def get_temp(self):
        try:
            temp = self.i2c.read_i2c_block_data(self.addr, 0x40, 2)
        except IOError:
            rospy.logerr("Read IMU temperature data error !")
        else:
            self.temp = float((temp[1]<<8)|temp[0])/100.0
            #self.temp = float(self.get_two_float(self.temp, 2)) #keep 2 decimal places

