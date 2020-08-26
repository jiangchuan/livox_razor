#include <livox_razor/CustomMsg.h>
#include <livox_razor/CustomPoint.h>
#include <ros/ros.h>
#include <stdio.h>
#include <sys/stat.h>

#include <boost/thread/thread.hpp>
#include <fstream>
#include <sstream>

#define ROS_RATE 20
#define SAVE_SIZE 500

int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
std::string lidar_dir;
std::string lidar_timedir;
std::map<boost::thread::id, int> lidar_num_writes_map;
std::map<boost::thread::id, time_t> lidar_last_time_map;

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
    std::string root_dir = "/home/ubuntu/livox_data/";
    // std::string root_dir = "/home/jiangchuan/livox_data/";
    lidar_dir = root_dir + "lidar/";
    int status = mkdir(root_dir.c_str(), 0777);
    status = mkdir(lidar_dir.c_str(), 0777);

    ros::init(argc, argv, "livox_razor");
    ros::NodeHandle nh;
    ros::Subscriber livox_sub = nh.subscribe<livox_razor::CustomMsg>("livox/lidar", 5, livox_callback);

    ros::Rate rate((double)ROS_RATE);  // The setpoint publishing rate MUST be faster than 2Hz
    ros::AsyncSpinner aSpinner(0);     // Set 0: use a thread for each CPU core
    aSpinner.start();
    ros::waitForShutdown();

    return 0;
}
