#include <geometry_msgs/Point32.h>
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

sensor_msgs::Imu::ConstPtr imu_msg;
sensor_msgs::NavSatFix::ConstPtr gps_msg;

int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
std::string rootdir;
std::string timedir;
std::string gps_filename;

std::map<boost::thread::id, int> num_writes_map;
std::map<boost::thread::id, time_t> last_time_map;

time_t get_time() {
    time_t now = time(0);
    tm *ltm = localtime(&now);
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

void saveRawData(sensor_msgs::PointCloud2::ConstPtr livox_msg) {
    if (imu_msg && gps_msg && livox_msg) {
        time_t now = get_time();
        std::string time_str = get_time_str();

        boost::thread::id this_id = boost::this_thread::get_id();
        if (last_time_map.find(this_id) == last_time_map.end()) {
            last_time_map[this_id] = 0;
        }
        if (difftime(now, last_time_map[this_id]) > 60) {
            timedir = rootdir + time_str + "/";
            int status = mkdir(timedir.c_str(), 0777);
            last_time_map[this_id] = now;
        }

        if (num_writes_map.find(this_id) == num_writes_map.end()) {
            num_writes_map[this_id] = 0;
        } else {
            num_writes_map[this_id]++;
        }

        std::stringstream ss;
        ss << timedir << time_str << "-" << digit2str(second) << "_" << this_id << "_" << num_writes_map[this_id] << ".csv";
        gps_filename = ss.str();

        sensor_msgs::PointCloud pt_cloud;
        sensor_msgs::convertPointCloud2ToPointCloud(*livox_msg, pt_cloud);
        sensor_msgs::ChannelFloat32 channel = pt_cloud.channels[0];
        std::stringstream stream;
        int pts_size = pt_cloud.points.size();

        float ijump = (float)pts_size / (float)SAVE_SIZE;
        float isum = 0.0f;
        int is = 1;
        for (int i = 0; i < pts_size; i++) {
            if (isum <= (float)i) {
                geometry_msgs::Point32 point = pt_cloud.points[i];
                // ROS_INFO("Point cloud %d: x=%f, y=%f, z=%f, intensity = %f", i, point.x, point.y, point.z, channel.values[i]);

                // Save to csv
                if (point.x > 1e-6 || fabs(point.y) > 1e-6 || fabs(point.z) > 1e-6) {
                    // 3. GPS rod shift
                    stream << std::setprecision(10) << gps_msg->latitude << ",";
                    stream << std::setprecision(11) << gps_msg->longitude << ",";
                    stream << std::setprecision(7) << gps_msg->altitude << ",";
                    stream << int(gps_msg->status.status) << ",";

                    stream << std::setprecision(4) << imu_msg->orientation.x << ",";
                    stream << std::setprecision(4) << imu_msg->orientation.y << ",";
                    stream << std::setprecision(4) << imu_msg->orientation.z << ",";
                    stream << std::setprecision(4) << imu_msg->orientation.w << ",";

                    stream << std::setprecision(4) << point.x << ",";
                    stream << std::setprecision(4) << point.y << ",";
                    stream << std::setprecision(4) << point.z << ",";
                    stream << channel.values[i] << "\n";  // Reflectivity
                }
                is++;
                isum += ijump;
            }
        }

        // ROS_INFO("Point cloud size: %d", pts_size);
        if (pts_size > 0) {
            std::fstream file;
            file.open(gps_filename, std::ios::out | std::ios::app);
            file << stream.rdbuf();
            file.close();
        }
    }
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg) {
    imu_msg = msg;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    gps_msg = msg;
}

void livox_callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    saveRawData(msg);
}

int main(int argc, char **argv) {
    rootdir = "/home/ubuntu/livox_data/";
    int status = mkdir(rootdir.c_str(), 0777);

    ros::init(argc, argv, "livox_razor");
    ros::NodeHandle nh;
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 1, imu_callback);          // Razor IMU
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("qxgps", 1, gps_callback);  // QX GPS
    ros::Subscriber livox_sub = nh.subscribe<sensor_msgs::PointCloud2>("livox/lidar", 1, livox_callback);
    ros::Rate rate((double)ROS_RATE);  // The setpoint publishing rate MUST be faster than 2Hz

    ros::AsyncSpinner aSpinner(0);  // Set 0: use a thread for each CPU core
    aSpinner.start();
    ros::waitForShutdown();

    return 0;
}
