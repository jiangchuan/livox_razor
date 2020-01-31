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


void computeEulerAngles(float dqx, float dqy, float dqz, float dqw)
{
    float ysqr = dqy * dqy;
    float t0 = -2.0f * (ysqr + dqz * dqz) + 1.0f;
    float t1 = +2.0f * (dqx * dqy - dqw * dqz);
    float t2 = -2.0f * (dqx * dqz + dqw * dqy);
    float t3 = +2.0f * (dqy * dqz - dqw * dqx);
    float t4 = -2.0f * (dqx * dqx + ysqr) + 1.0f;
  
	// Keep t2 within range of asin (-1, 1)
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;
  
    pitch = asin(t2) * 2;
    roll = atan2(t3, t4);
    yaw = atan2(t1, t0);
}


void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    ROS_INFO("orientation: x=%f, y=%f, z=%f, w=%f", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

    tf2::Quaternion qtn = tf2::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    // qtn.normalize();
    // tf2::Quaternion qtn_local = qtn * tf2::Quaternion(lidarx, lidary, lidarz, 0.0) * qtn.inverse().normalize();

    tf2::Quaternion qtn_inv = qtn.inverse();
    ROS_INFO("After inverse: x=%f, y=%f, z=%f, w=%f", qtn_inv.getX(), qtn_inv.getY(), qtn_inv.getZ(), qtn_inv.getW());

    tf2::Quaternion qtn_local = qtn * tf2::Quaternion(1, 2, 3, 0.0) * qtn_inv;
    double localx = qtn_local.getX();
    double localy = qtn_local.getY();
    double localz = qtn_local.getZ();
    double localw = qtn_local.getW();
    //ROS_INFO("After rotation: x=%f, y=%f, z=%f, w=%f", localx, localy, localz, localw);


    // tf2::Quaternion qtn = tf2::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    // qtn.normalize();
    // tf2::Matrix3x3 m(qtn);
    // m.getRPY(roll, pitch, yaw);

    computeEulerAngles(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    //ROS_INFO("Razor roll = %1.1f degrees, pitch = %1.1f degrees, yaw = %1.1f degrees\n", to_degrees(roll), to_degrees(pitch), to_degrees(yaw));
}

void local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ROS_INFO("orientation: x=%f, y=%f, z=%f, w=%f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf2::Quaternion qtn = tf2::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    qtn.normalize();
    tf2::Matrix3x3 m(qtn);
    m.getRPY(roll, pitch, yaw);
    //ROS_INFO("      Mavros roll = %1.1f degrees, pitch = %1.1f degrees, yaw = %1.1f degrees\n", to_degrees(roll), to_degrees(pitch), to_degrees(yaw));
    // Roll: -180 - 180, to right is positive
    // Pitch: 0 -- -90, to up is positive, synmetric to z axis
    // Yaw: To North is 0, 0 -- 180, 0 -- -180 to west. 

    //Calibration: 
    // xmin = -261.72, turn up 
    // xmax = 267.46, turn down
    // ymin = -283.94, turn left
    // ymax = 276.73, turn right
    // zmin = -283.69, facing down
    // zmax = 294.07, facing up

    // gyro x avg = -0.02
    // gyro y avg = -0.00
    // gyro z avg = -0.01
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox_razor");
    ros::NodeHandle nh;
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 10, imu_callback); // Razor IMU
    //ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_pos_callback); // Mavros IMU
    ros::spin();
    return 0;
}
