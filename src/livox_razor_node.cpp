#include <geometry_msgs/Point32.h>
#include <livox_razor/CustomMsg.h>
#include <livox_razor/CustomPoint.h>
#include <ros/ros.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <stdio.h>
#include <sys/stat.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>

#include <boost/thread/thread.hpp>
#include <fstream>
#include <sstream>

#define ROS_RATE 20
#define SAVE_SIZE 500

int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
std::string lidar_dir;
std::string imu_dir;
std::string lidar_timedir;
std::string imu_timedir;
std::map<boost::thread::id, int> lidar_num_writes_map;
std::map<boost::thread::id, time_t> lidar_last_time_map;
std::map<boost::thread::id, int> imu_num_writes_map;
std::map<boost::thread::id, time_t> imu_last_time_map;

std::stringstream imustream;
int imucount = 0;

time_t get_time() {
    time_t now = time(0);
    tm *ltm = gmtime(&now);
    year = 1900 + ltm->tm_year;
    month = 1 + ltm->tm_mon;
    day = ltm->tm_mday;
    hour = ltm->tm_hour;
    minute = ltm->tm_min;
    second = ltm->tm_sec;
    return now;
}

std::string digit2str(int num) {
    if (num < 10)
        return "0" + std::to_string(num);
    return std::to_string(num);
}

std::string get_time_str() {
    return std::to_string(year) + "-" + digit2str(month) + "-" + digit2str(day) + "_" + digit2str(hour) + "-" + digit2str(minute);
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    // gps_msg = msg;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &imu_msg) {
    double imu_t = imu_msg->header.stamp.sec * 1.0 + imu_msg->header.stamp.nsec / 1000000000.0;

    time_t now = get_time();
    std::string time_str = get_time_str();
    boost::thread::id this_id = boost::this_thread::get_id();
    if (imu_last_time_map.find(this_id) == imu_last_time_map.end()) {
        imu_last_time_map[this_id] = 0;
    }
    if (difftime(now, imu_last_time_map[this_id]) > 60) {
        imu_timedir = imu_dir + time_str + "/";
        int status = mkdir(imu_timedir.c_str(), 0777);
        imu_last_time_map[this_id] = now;
    }
    if (imu_num_writes_map.find(this_id) == imu_num_writes_map.end()) {
        imu_num_writes_map[this_id] = 0;
    } else {
        imu_num_writes_map[this_id]++;
    }

    std::stringstream ss;
    ss << imu_timedir << time_str << "-" << digit2str(second) << "_" << this_id << "_" << imu_num_writes_map[this_id] << ".csv";
    std::string imu_filename = ss.str();

    if (imucount % 500 == 0) {
        std::fstream file;
        file.open(imu_filename, std::ios::out | std::ios::app);
        file << imustream.rdbuf();
        file.close();

        imustream.str(std::string()); // clear stream
    }


    imustream << std::setprecision(4) << imu_msg->orientation.x << ",";
    imustream << std::setprecision(4) << imu_msg->orientation.y << ",";
    imustream << std::setprecision(4) << imu_msg->orientation.z << ",";
    imustream << std::setprecision(4) << imu_msg->orientation.w << ",";
}

void livox_callback(const livox_razor::CustomMsg::ConstPtr &lidar_msg) {
    time_t now = get_time();
    std::string time_str = get_time_str();
    boost::thread::id this_id = boost::this_thread::get_id();
    if (lidar_last_time_map.find(this_id) == lidar_last_time_map.end()) {
        lidar_last_time_map[this_id] = 0;
    }
    if (difftime(now, lidar_last_time_map[this_id]) > 60) {
        lidar_timedir = lidar_dir + time_str + "/";
        int status = mkdir(lidar_timedir.c_str(), 0777);
        lidar_last_time_map[this_id] = now;
    }
    if (lidar_num_writes_map.find(this_id) == lidar_num_writes_map.end()) {
        lidar_num_writes_map[this_id] = 0;
    } else {
        lidar_num_writes_map[this_id]++;
    }

    std::stringstream ss;
    ss << lidar_timedir << time_str << "-" << digit2str(second) << "_" << this_id << "_" << lidar_num_writes_map[this_id] << ".csv";
    std::string lidar_filename = ss.str();

    int pts_size = lidar_msg->points.size();
    float ijump = (float)pts_size / (float)SAVE_SIZE;
    float isum = 0.0f;
    int is = 1;

    double lidar_t0 = lidar_msg->header.stamp.sec * 1.0 + lidar_msg->header.stamp.nsec / 1000000000.0;
    // std::cout << "lidar_t0 = " << lidar_t0 << " sec = " << lidar_msg->header.stamp.sec << " nsec = " << lidar_msg->header.stamp.nsec << std::endl;
    std::stringstream stream;
    for (int i = 0; i < pts_size; i++) {
        if (isum <= (float)i) {
            livox_razor::CustomPoint point = lidar_msg->points[i];

            // Save to csv
            if (point.x > 1e-6 || fabs(point.y) > 1e-6 || fabs(point.z) > 1e-6) {
                double lidar_t = lidar_t0 + point.offset_time / 1000000000.0;
                stream << std::setprecision(18) << lidar_t << ",";
                stream << std::setprecision(6) << point.x << ",";
                stream << std::setprecision(6) << point.y << ",";
                stream << std::setprecision(6) << point.z << ",";
                stream << unsigned(point.reflectivity) << "\n";
            }
            is++;
            isum += ijump;
        }
    }

    // ROS_INFO("Point cloud size: %d", pts_size);
    if (pts_size > 0) {
        std::fstream file;
        file.open(lidar_filename, std::ios::out | std::ios::app);
        file << stream.rdbuf();
        file.close();
    }
}

int main(int argc, char **argv) {
    // std::string root_dir = "/home/ubuntu/livox_data/";
    std::string root_dir = "/home/jiangchuan/livox_data/";
    lidar_dir = root_dir + "lidar/";
    imu_dir = root_dir + "imu/";
    int status = mkdir(root_dir.c_str(), 0777);
    status = mkdir(lidar_dir.c_str(), 0777);
    status = mkdir(imu_dir.c_str(), 0777);

    ros::init(argc, argv, "livox_razor");
    ros::NodeHandle nh;
    // ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("qxgps", 5, gps_callback);  // QX GPS
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 100, imu_callback);  // Razor IMU
    // ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu/data", 100, imu_callback);     // Adis IMU
    ros::Subscriber livox_sub = nh.subscribe<livox_razor::CustomMsg>("livox/lidar", 5, livox_callback);

    ros::Rate rate((double)ROS_RATE);  // The setpoint publishing rate MUST be faster than 2Hz

    ros::AsyncSpinner aSpinner(0);  // Set 0: use a thread for each CPU core
    aSpinner.start();
    ros::waitForShutdown();

    return 0;
}
