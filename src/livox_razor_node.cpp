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
#include <pigpio.h>

///////////////////////////////////////
#include "mongodb_store/message_store.h"
#include "geometry_msgs/Pose.h"
#include <boost/foreach.hpp>
#include <cassert>
///////////////////////////////////////

#define ROS_RATE 20
#define SAVE_SIZE 5000
#define LED_JUMP 10

sensor_msgs::Imu::ConstPtr imu_msg;
sensor_msgs::NavSatFix::ConstPtr gps_msg;

double localx = 0.0, localy = 0.0, localz = 0.0;
int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;

std::string rootdir;
std::string timedir;
std::string gps_filename;
int led_value = 0;

bool use_pi = true;

std::map<boost::thread::id, int> num_writes_map;
std::map<boost::thread::id, time_t> last_time_map;

////////
mongodb_store::MessageStoreProxy *messageStore;
////////

// time_t get_time()
// {
//     time_t now = time(0);
//     tm *ltm = localtime(&now);
//     year = 1900 + ltm->tm_year;
//     month = 1 + ltm->tm_mon;
//     day = ltm->tm_mday;
//     hour = ltm->tm_hour;
//     minute = ltm->tm_min;
//     second = ltm->tm_sec;
//     return now;
// }

// std::string digit2str(int num)
// {
//     if (num < 10)
//         return "0" + std::to_string(num);
//     return std::to_string(num);
// }

// std::string get_time_str()
// {
//     return std::to_string(year) + "-" + digit2str(month) + "-" + digit2str(day) + "_" + digit2str(hour) + "-" + digit2str(minute) + "-" + digit2str(second);
// }

// void set_led()
// {
//     if (second % 2 == 0)
//     {
//         gpioWrite(24, 1); /* on */
//     }
//     else
//     {
//         gpioWrite(24, 0); /* off */
//     }
// }

// void saveRawData(sensor_msgs::PointCloud2::ConstPtr livox_msg)
// {
//     if (imu_msg && gps_msg && livox_msg)
//     {
//         time_t now = get_time();
//         std::string time_str = get_time_str();

//         boost::thread::id this_id = boost::this_thread::get_id();

//         if (last_time_map.find(this_id) == last_time_map.end())
//         {
//             last_time_map[this_id] = 0;
//         }

//         if (difftime(now, last_time_map[this_id]) > 60)
//         {
//             timedir = rootdir + time_str + "/";
//             int status = mkdir(timedir.c_str(), 0777);
//         }
//         last_time_map[this_id] = now;

//         if (num_writes_map.find(this_id) == num_writes_map.end())
//         {
//             num_writes_map[this_id] = 0;
//         }
//         else
//         {
//             num_writes_map[this_id]++;
//         }

//         std::stringstream ss;
//         ss << timedir << time_str << "_" << this_id << "_" << num_writes_map[this_id] << ".csv";
//         gps_filename = ss.str();

//         sensor_msgs::PointCloud pt_cloud;
//         sensor_msgs::convertPointCloud2ToPointCloud(*livox_msg, pt_cloud);
//         sensor_msgs::ChannelFloat32 channel = pt_cloud.channels[0];

//         std::stringstream stream;
//         int pts_size = pt_cloud.points.size();

//         float ijump = (float)pts_size / (float)SAVE_SIZE;
//         float isum = 0.0f;
//         int is = 1;
//         for (int i = 0; i < pts_size; i++)
//         {
//             if (isum <= (float)i)
//             {
//                 geometry_msgs::Point32 point = pt_cloud.points[i];
//                 // ROS_INFO("Point cloud %d: x=%f, y=%f, z=%f, intensity = %f", i, point.x, point.y, point.z, channel.values[i]);

//                 // Save to csv
//                 if (point.x > 1e-6 || fabs(point.y) > 1e-6 || fabs(point.z) > 1e-6)
//                 {
//                     // 3. GPS rod shift

//                     geometry_msgs::Pose p;
//                     p.position.x = point.x;
//                     p.position.y = point.y;
//                     p.position.z = point.z;

//                     p.orientation.x = imu_msg->orientation.x;
//                     p.orientation.y = imu_msg->orientation.y;
//                     p.orientation.z = imu_msg->orientation.z;
//                     p.orientation.w = imu_msg->orientation.w;

//                     messageStore->insertNamed("Livox", p);
//                 }

//                 is++;
//                 isum += ijump;
//             }
//         }

//         set_led();
//     }
// }

// void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
// {
//     imu_msg = msg;
// }

// void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
// {
//     gps_msg = msg;
// }

// void livox_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
// {
//     saveRawData(msg);
// }

int main(int argc, char **argv)
{
    // if (use_pi)
    // {
    //     if (gpioInitialise() < 0)
    //     {
    //         fprintf(stderr, "pigpio initialisation failed\n");
    //         return 1;
    //     }
    //     /* Set GPIO modes */
    //     gpioSetMode(24, PI_OUTPUT);

    //     rootdir = "/home/ubuntu/livox_data/";
    // }
    // else
    // {
    //     rootdir = "/home/jiangchuan/livox_data/";
    // }
    // int status = mkdir(rootdir.c_str(), 0777);

    ros::init(argc, argv, "livox_razor");
    ros::NodeHandle nh;
    messageStore = new mongodb_store::MessageStoreProxy(nh);
    // mongodb_store::MessageStoreProxy messageStore1(nh);

    geometry_msgs::Pose p;
    std::string name("my pose");

    //Insert something with a name, storing id too
    std::string id(messageStore->insertNamed(name, p));

    // insert(message, meta={}, wait=True)

    std::cout << "Pose \"" << name << "\" inserted with id " << id << std::endl;
    p.position.z = 666;
    messageStore->updateID(id, p);

    // ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 1, imu_callback);         // Razor IMU
    // ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("qxgps", 1, gps_callback); // QX GPS
    // ros::Subscriber livox_sub = nh.subscribe<sensor_msgs::PointCloud2>("livox/lidar", 1, livox_callback);
    // ros::Rate rate((double)ROS_RATE); // The setpoint publishing rate MUST be faster than 2Hz

    // ros::AsyncSpinner aSpinner(0); // Set 0: use a thread for each CPU core
    // aSpinner.start();
    // ros::waitForShutdown();

    // /* Stop DMA, release resources */
    // gpioTerminate();

    return 0;
}

// { "_id" : ObjectId("5ef9149e1d41c811f6b647ac"), "position" : { "y" : 0, "x" : 0, "z" : 111 }, "orientation" : { "y" : 0, "x" : 0, "z" : 0, "w" : 0 }, "_meta" : { "name" : "my pose", "inserted_at" : ISODate("2020-06-28T22:07:26.381Z"), "timestamp" : NumberLong("1593382046381707906"), "stored_type" : "geometry_msgs/Pose", "published_at" : ISODate("2020-06-28T22:07:26.381Z"), "inserted_by" : "/livox_razor_node", "stored_class" : "geometry_msgs.msg._Pose.Pose" } }
// { "_id" : ObjectId("5ef9149e1d41c811f6b647ad"), "position" : { "y" : 0, "x" : 0, "z" : 222 }, "orientation" : { "y" : 0, "x" : 0, "z" : 0, "w" : 0 }, "_meta" : { "name" : "my pose", "inserted_at" : ISODate("2020-06-28T22:07:26.389Z"), "timestamp" : NumberLong("1593382046389272928"), "stored_type" : "geometry_msgs/Pose", "published_at" : ISODate("2020-06-28T22:07:26.389Z"), "inserted_by" : "/livox_razor_node", "stored_class" : "geometry_msgs.msg._Pose.Pose" } }
