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

#include <stdio.h>
#include <pigpio.h>

#define ROS_RATE 20
#define SAVE_SIZE 10000
#define LED_JUMP 10

sensor_msgs::Imu::ConstPtr imu_msg;
sensor_msgs::NavSatFix::ConstPtr gps_msg;

double localx = 0.0, localy = 0.0, localz = 0.0;
int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;

std::string rootdir;
std::string timedir;
std::string gps_filename;
int num_writes = 0;
int led_value = 0;
time_t last_write_time = 0;

bool use_pi = true;
// bool use_pi = false;
int spin_count = 0;

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

std::string get_time_str()
{
    std::stringstream sstm;
    sstm << year << "-";
    if (month < 10)
    {
        sstm << "0";
    }
    sstm << month << "-";
    if (day < 10)
    {
        sstm << "0";
    }
    sstm << day << "_";
    if (hour < 10)
    {
        sstm << "0";
    }
    sstm << hour << "-";
    if (minute < 10)
    {
        sstm << "0";
    }
    sstm << minute << "-";
    if (second < 10)
    {
        sstm << "0";
    }
    sstm << second;
    return sstm.str();
}

void set_led()
{
    if (led_value == 0)
    {
        time_t now = time(0);
        if (difftime(now, last_write_time) < 1)
        {
            led_value = 1;
            gpioWrite(24, 1); /* on */
        }
    }
    else
    {
        led_value = 0;
        gpioWrite(24, 0); /* off */
    }
}

void saveRawData(sensor_msgs::PointCloud2::ConstPtr livox_msg)
{
    if (imu_msg && gps_msg && livox_msg)
    {
        time_t now = get_time();
        std::string time_str = get_time_str();
        if (difftime(now, last_write_time) > 60)
        {
            timedir = rootdir + time_str + "/";
            int status = mkdir(timedir.c_str(), 0777);
        }
        last_write_time = now;

        // gps_filename = timedir + time_str + "_" + std::to_string(num_writes) + "_" + std::to_string(boost::this_thread::get_id()) + ".csv";

        std::stringstream ss;
        ss << timedir << time_str << "_" << num_writes << "_" << boost::this_thread::get_id() << ".csv";
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
                    stream << std::setprecision(10) << gps_msg->latitude << ",";
                    stream << std::setprecision(11) << gps_msg->longitude << ",";
                    stream << std::setprecision(7) << gps_msg->altitude << ",";

                    stream << std::setprecision(4) << imu_msg->orientation.x << ",";
                    stream << std::setprecision(4) << imu_msg->orientation.y << ",";
                    stream << std::setprecision(4) << imu_msg->orientation.z << ",";
                    stream << std::setprecision(4) << imu_msg->orientation.w << ",";

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

        num_writes++;

        if (use_pi && spin_count % LED_JUMP == 0)
        {
            set_led();
        }
        spin_count++;
    }
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu_msg = msg;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    gps_msg = msg;
}

void livox_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    saveRawData(msg);
}

int main(int argc, char **argv)
{
    if (use_pi)
    {
        if (gpioInitialise() < 0)
        {
            fprintf(stderr, "pigpio initialisation failed\n");
            return 1;
        }
        /* Set GPIO modes */
        gpioSetMode(24, PI_OUTPUT);

        rootdir = "/home/ubuntu/livox_data/";
    }
    else
    {
        rootdir = "/home/jiangchuan/livox_data/";
    }
    int status = mkdir(rootdir.c_str(), 0777);

    ros::init(argc, argv, "livox_razor");
    ros::NodeHandle nh;
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 10, imu_callback);         // Razor IMU
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("qxgps", 10, gps_callback); // QX GPS
    ros::Subscriber livox_sub = nh.subscribe<sensor_msgs::PointCloud2>("livox/lidar", 10, livox_callback);
    ros::Rate rate((double)ROS_RATE); // The setpoint publishing rate MUST be faster than 2Hz

    ros::AsyncSpinner s(4); // Use 4 threads
    // ROS_INFO_STREAM("Main loop in thread:" << boost::this_thread::get_id());
    s.start();
    ros::waitForShutdown();

    /* Stop DMA, release resources */
    gpioTerminate();

    return 0;
}
