#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "std_msgs/String.h"

sensor_msgs::Imu::ConstPtr pose_msg;

void chatterCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void local_pos_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    pose_msg = msg;
    ROS_INFO("Orientation: x=%f, y=%f, z=%f, w = %f", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox_razor");

    ros::NodeHandle nh;

    ros::Subscriber local_pos_sub = nh.subscribe<sensor_msgs::Imu>("/imu", 10, local_pos_callback);

    ros::spin();

    return 0;
}