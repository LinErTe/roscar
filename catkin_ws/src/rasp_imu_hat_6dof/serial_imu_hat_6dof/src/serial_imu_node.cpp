/****************************************************
 * Copyright:2016-2020 www.corvin.cn ROS小课堂
 * Description:使用串口来读取IMU模块的数据,并通过
 *   topic将数据发布出来.芯片中默认读取出来的加速度数据
 *   单位是g,需要将其转换为ROS中加速度规定的m/s2才能发布.
 * Author: corvin
 * History:
 *   20200403: init this file.
 *   20200406:增加发布yaw数据的话题.
 *   20200703:修改三轴加速度数据正负,按照ROS规定沿着各轴
 *     正方向时数据为正,否则为负.单位m/s2。
*****************************************************/
#include <ros/ros.h>
#include <tf/tf.h>
#include "imu_data.h"
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>


int main(int argc, char **argv)
{
    float yaw, pitch, roll;
    std::string imu_dev;
    int baud = 0;
    int dataBit = 0;
    std::string parity;
    int stopBit = 0;

    std::string link_name;
    std::string imu_topic;
    std::string temp_topic;
    std::string yaw_topic;
    int pub_hz = 0;
    float degree2Rad = 3.1415926/180.0;
    float acc_factor = 9.806;

    ros::init(argc, argv, "imu_data_pub_node");
    ros::NodeHandle handle;

    ros::param::get("~imu_dev", imu_dev);
    ros::param::get("~baud_rate", baud);
    ros::param::get("~data_bits", dataBit);
    ros::param::get("~parity", parity);
    ros::param::get("~stop_bits", stopBit);

    ros::param::get("~link_name", link_name);
    ros::param::get("~pub_data_topic", imu_topic);
    ros::param::get("~pub_temp_topic", temp_topic);
    ros::param::get("~yaw_topic", yaw_topic);
    ros::param::get("~pub_hz", pub_hz);

    int ret = initSerialPort(imu_dev.c_str(), baud, dataBit, parity.c_str(), stopBit);
    if(ret < 0)
    {
        ROS_ERROR("init serial port error !");
        closeSerialPort();
        return -1;
    }
    ROS_INFO("IMU module is working... ");

    ros::Publisher imu_pub = handle.advertise<sensor_msgs::Imu>(imu_topic, 1);
    ros::Publisher yaw_pub = handle.advertise<std_msgs::Float32>(yaw_topic, 1);
    ros::Rate loop_rate(pub_hz);

    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = link_name;
    std_msgs::Float32 yaw_msg;
    while(ros::ok())
    {
        if(getImuData() < 0)
            break;

        imu_msg.header.stamp = ros::Time::now();
        roll  = getAngleX()*degree2Rad;
        pitch = getAngleY()*degree2Rad;
        yaw   = getAngleZ()*degree2Rad;
        if(yaw >= 3.1415926)
            yaw -= 6.2831852;

        //publish yaw data
        yaw_msg.data = yaw;
        yaw_pub.publish(yaw_msg);

        //ROS_INFO("yaw:%f pitch:%f roll:%f",yaw, pitch, roll);
        imu_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        imu_msg.orientation_covariance = {0.0025, 0, 0,
                                          0, 0.0025, 0,
                                          0, 0, 0.0025};

        imu_msg.angular_velocity.x = getAngularX()*degree2Rad;
        imu_msg.angular_velocity.y = getAngularY()*degree2Rad;
        imu_msg.angular_velocity.z = getAngularZ()*degree2Rad;
        imu_msg.angular_velocity_covariance = {0.02, 0, 0,
                                               0, 0.02, 0,
                                               0, 0, 0.02};

        //ROS_INFO("acc_x:%f acc_y:%f acc_z:%f",getAccX(), getAccY(), getAccZ());
        imu_msg.linear_acceleration.x = -getAccX()*acc_factor;
        imu_msg.linear_acceleration.y = -getAccY()*acc_factor;
        imu_msg.linear_acceleration.z = -getAccZ()*acc_factor;
        imu_msg.linear_acceleration_covariance = {0.04, 0, 0,
                                                   0, 0.04, 0,
                                                   0, 0, 0.04};

        imu_pub.publish(imu_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    closeSerialPort();
    return 0;
}

