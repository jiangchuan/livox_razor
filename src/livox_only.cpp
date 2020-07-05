#include <sstream>
#include <fstream>
#include <sys/stat.h>

#include <ros/ros.h>
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
#include <boost/thread/thread.hpp>

#include <stdio.h>

#define ROS_RATE 20
#define SAVE_SIZE 5000

double localx = 0.0, localy = 0.0, localz = 0.0;
int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;

std::string rootdir;
std::string timedir;
std::string gps_filename;

bool use_pi = true;

std::map<boost::thread::id, int> num_writes_map;
std::map<boost::thread::id, time_t> last_time_map;

time_t get_time()
{
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

std::string digit2str(int num)
{
    if (num < 10)
        return "0" + std::to_string(num);
    return std::to_string(num);
}

std::string get_time_str()
{
    return std::to_string(year) + "-" + digit2str(month) + "-" + digit2str(day) + "_" + digit2str(hour) + "-" + digit2str(minute) + "-" + digit2str(second);
}

void saveRawData(sensor_msgs::PointCloud2::ConstPtr livox_msg)
{
    if (livox_msg)
    {
        time_t now = get_time();
        std::string time_str = get_time_str();

        boost::thread::id this_id = boost::this_thread::get_id();

        if (last_time_map.find(this_id) == last_time_map.end())
        {
            last_time_map[this_id] = 0;
        }

        if (difftime(now, last_time_map[this_id]) > 60)
        {
            timedir = rootdir + time_str + "/";
            int status = mkdir(timedir.c_str(), 0777);
        }
        last_time_map[this_id] = now;

        if (num_writes_map.find(this_id) == num_writes_map.end())
        {
            num_writes_map[this_id] = 0;
        }
        else
        {
            num_writes_map[this_id]++;
        }

        std::stringstream ss;
        ss << timedir << time_str << "_" << this_id << "_" << num_writes_map[this_id] << ".csv";
        gps_filename = ss.str();

        sensor_msgs::PointCloud pt_cloud;
        sensor_msgs::convertPointCloud2ToPointCloud(*livox_msg, pt_cloud);
        sensor_msgs::ChannelFloat32 channel = pt_cloud.channels[0];

        std::stringstream stream;
        int pts_size = pt_cloud.points.size();

        float ijump = (float)pts_size / (float)SAVE_SIZE;
        float isum = 0.0f;
        int is = 1;
        for (int i = 0; i < pts_size; i++)
        {
            if (isum <= (float)i)
            {
                geometry_msgs::Point32 point = pt_cloud.points[i];
                // ROS_INFO("Point cloud %d: x=%f, y=%f, z=%f, intensity = %f", i, point.x, point.y, point.z, channel.values[i]);

                // Save to csv
                if (point.x > 1e-6 || fabs(point.y) > 1e-6 || fabs(point.z) > 1e-6)
                {
                    // 3. GPS rod shift
                    stream << std::setprecision(10) << 0.0 << ",";
                    stream << std::setprecision(11) << 0.0 << ",";
                    stream << std::setprecision(7) << 0.0 << ",";

                    stream << std::setprecision(4) << 0.0 << ",";
                    stream << std::setprecision(4) << 0.0 << ",";
                    stream << std::setprecision(4) << 0.0 << ",";
                    stream << std::setprecision(4) << 0.0 << ",";

                    stream << std::setprecision(4) << point.x << ",";
                    stream << std::setprecision(4) << point.y << ",";
                    stream << std::setprecision(4) << point.z << ",";
                    stream << channel.values[i] << "\n"; // Reflectivity
                }

                is++;
                isum += ijump;
            }
        }

        // ROS_INFO("Point cloud size: %d", pts_size);
        if (pts_size > 0)
        {
            std::fstream file;
            file.open(gps_filename, std::ios::out | std::ios::app);
            file << stream.rdbuf();
            file.close();
        }

    }
}

void livox_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    saveRawData(msg);
}

int main(int argc, char **argv)
{
    if (use_pi)
    {
        rootdir = "/home/ubuntu/livox_data/";
    }
    else
    {
        rootdir = "/home/jiangchuan/livox_data/";
    }
    int status = mkdir(rootdir.c_str(), 0777);

    ros::init(argc, argv, "livox_razor");
    ros::NodeHandle nh;
    ros::Subscriber livox_sub = nh.subscribe<sensor_msgs::PointCloud2>("livox/lidar", 1, livox_callback);
    ros::Rate rate((double)ROS_RATE); // The setpoint publishing rate MUST be faster than 2Hz

    ros::AsyncSpinner aSpinner(0); // Set 0: use a thread for each CPU core
    aSpinner.start();
    ros::waitForShutdown();

    return 0;
}
