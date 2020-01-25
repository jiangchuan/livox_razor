#include <sstream>
#include <fstream>
#include <sys/stat.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#define ROS_RATE 20

sensor_msgs::Imu::ConstPtr imu_msg;
double roll = 0.0, pitch = 0.0, yaw = 0.0;

double to_degrees(double r)
{
    return r * 180.0 / M_PI;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // ROS_INFO("orientation: x=%f, y=%f, z=%f, w=%f", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Quaternion qtn = tf2::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    qtn.normalize();
    tf2::Matrix3x3 m(qtn);
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("Razor roll = %1.1f degrees, pitch = %1.1f degrees, yaw = %1.1f degrees\n", to_degrees(roll), to_degrees(pitch), to_degrees(yaw));
}

void local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // ROS_INFO("orientation: x=%f, y=%f, z=%f, w=%f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf2::Quaternion qtn = tf2::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    qtn.normalize();
    tf2::Matrix3x3 m(qtn);
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("      Mavros roll = %1.1f degrees, pitch = %1.1f degrees, yaw = %1.1f degrees\n", to_degrees(roll), to_degrees(pitch), to_degrees(yaw));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox_razor");
    ros::NodeHandle nh;
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 10, imu_callback); // Razor IMU
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_pos_callback); // Mavros IMU
    ros::spin();
    return 0;
}
