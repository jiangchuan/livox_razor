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

#include "mongodb_store/message_store.h"
#include "geometry_msgs/Pose.h"

#define ROS_RATE 20
#define SAVE_SIZE 5000

sensor_msgs::Imu::ConstPtr imu_msg;
sensor_msgs::NavSatFix::ConstPtr gps_msg;
mongodb_store::MessageStoreProxy *messageStore;

void saveRawData(sensor_msgs::PointCloud2::ConstPtr livox_msg)
{
    if (imu_msg && gps_msg && livox_msg)
    {
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

                // Save to mongodb
                if (point.x > 1e-6 || fabs(point.y) > 1e-6 || fabs(point.z) > 1e-6)
                {
                    geometry_msgs::Pose p;
                    p.position.x = point.x;
                    p.position.y = point.y;
                    p.position.z = point.z;

                    p.orientation.x = imu_msg->orientation.x;
                    p.orientation.y = imu_msg->orientation.y;
                    p.orientation.z = imu_msg->orientation.z;
                    p.orientation.w = imu_msg->orientation.w;

                    mongo::BSONObjBuilder mbuilder;
                    mbuilder << "lat" << gps_msg->latitude << "lon" << gps_msg->longitude << "alt" << gps_msg->altitude;
                    mongo::BSONObj meta = mbuilder.obj();

                    messageStore->insert(p, meta, false);
                }

                is++;
                isum += ijump;
            }
        }
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
    ros::init(argc, argv, "livox_razor");
    ros::NodeHandle nh;
    messageStore = new mongodb_store::MessageStoreProxy(nh);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 1, imu_callback);         // Razor IMU
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("qxgps", 1, gps_callback); // QX GPS
    ros::Subscriber livox_sub = nh.subscribe<sensor_msgs::PointCloud2>("livox/lidar", 1, livox_callback);
    ros::Rate rate((double)ROS_RATE); // The setpoint publishing rate MUST be faster than 2Hz

    ros::AsyncSpinner aSpinner(0); // Set 0: use a thread for each CPU core
    aSpinner.start();
    ros::waitForShutdown();

    return 0;
}


// { "_id" : ObjectId("5efa46271d41c86e877adbc7"), "position" : { "y" : 0.2720000147819519, "x" : 1.565000057220459, "z" : -0.43700000643730164 }, "orientation" : { "y" : -0.06278365356834364, "x" : 0.020597344504871315, "z" : -0.8897095705294458, "w" : 0.4517198715490241 }, "_meta" : { "name" : "Joe", "inserted_at" : ISODate("2020-06-29T19:51:03.077Z"), "timestamp" : NumberLong("1593460263077534914"), "age" : 33, "stored_type" : "geometry_msgs/Pose", "published_at" : ISODate("2020-06-29T19:51:03.077Z"), "inserted_by" : "/livox_razor_node", "stored_class" : "geometry_msgs.msg._Pose.Pose" } }
// { "_id" : ObjectId("5efa46271d41c86e877adbc8"), "position" : { "y" : 0.2750000059604645, "x" : 1.5829999446868896, "z" : -0.4390000104904175 }, "orientation" : { "y" : -0.062471927920228085, "x" : 0.021018633606793687, "z" : -0.8897744562067628, "w" : 0.4516158681282055 }, "_meta" : { "name" : "Joe", "inserted_at" : ISODate("2020-06-29T19:51:03.190Z"), "timestamp" : NumberLong("1593460263190537929"), "age" : 33, "stored_type" : "geometry_msgs/Pose", "published_at" : ISODate("2020-06-29T19:51:03.190Z"), "inserted_by" : "/livox_razor_node", "stored_class" : "geometry_msgs.msg._Pose.Pose" } }

// { "_id" : ObjectId("5efa3a4b1d41c80509b5eda1"), "position" : { "y" : 0.24799999594688416, "x" : 1.8619999885559082, "z" : 0.2280000001192093 }, "orientation" : { "y" : -0.0638838332414191, "x" : 0.02176226838737317, "z" : -0.8883246526398942, "w" : 0.4542296457049339 }, "_meta" : { "inserted_at" : ISODate("2020-06-29T19:00:27.822Z"), "timestamp" : NumberLong("1593457227822076082"), "stored_type" : "geometry_msgs/Pose", "published_at" : ISODate("2020-06-29T19:00:27.822Z"), "inserted_by" : "/livox_razor_node", "stored_class" : "geometry_msgs.msg._Pose.Pose" } }
// { "_id" : ObjectId("5efa3a4b1d41c80509b5eda2"), "position" : { "y" : 0.25200000405311584, "x" : 1.8609999418258667, "z" : 0.2329999953508377 }, "orientation" : { "y" : -0.06364907938063434, "x" : 0.021459671925886697, "z" : -0.8882608147383813, "w" : 0.45440180696728977 }, "_meta" : { "inserted_at" : ISODate("2020-06-29T19:00:27.828Z"), "timestamp" : NumberLong("1593457227828783035"), "stored_type" : "geometry_msgs/Pose", "published_at" : ISODate("2020-06-29T19:00:27.828Z"), "inserted_by" : "/livox_razor_node", "stored_class" : "geometry_msgs.msg._Pose.Pose" } }

// { "_id" : ObjectId("5efa1a4e1d41c80ea859cf86"), "position" : { "y" : 0.43299999833106995, "x" : 1.5750000476837158, "z" : -0.14100000262260437 }, "orientation" : { "y" : -0.0627335013509859, "x" : 0.020707887978900934, "z" : -0.8894762840765605, "w" : 0.45218097179011496 }, "_meta" : { "name" : "Livox", "inserted_at" : ISODate("2020-06-29T16:43:58.234Z"), "timestamp" : NumberLong("1593449038234296083"), "stored_type" : "geometry_msgs/Pose", "published_at" : ISODate("2020-06-29T16:43:58.234Z"), "inserted_by" : "/livox_razor_node", "stored_class" : "geometry_msgs.msg._Pose.Pose" } }
// { "_id" : ObjectId("5efa1a4e1d41c80ea859cf87"), "position" : { "y" : 0.4399999976158142, "x" : 1.5839999914169312, "z" : -0.1420000046491623 }, "orientation" : { "y" : -0.0627335013509859, "x" : 0.020707887978900934, "z" : -0.8894762840765605, "w" : 0.45218097179011496 }, "_meta" : { "name" : "Livox", "inserted_at" : ISODate("2020-06-29T16:43:58.240Z"), "timestamp" : NumberLong("1593449038240856885"), "stored_type" : "geometry_msgs/Pose", "published_at" : ISODate("2020-06-29T16:43:58.240Z"), "inserted_by" : "/livox_razor_node", "stored_class" : "geometry_msgs.msg._Pose.Pose" } }

// { "_id" : ObjectId("5ef9149e1d41c811f6b647ac"), "position" : { "y" : 0, "x" : 0, "z" : 111 }, "orientation" : { "y" : 0, "x" : 0, "z" : 0, "w" : 0 }, "_meta" : { "name" : "my pose", "inserted_at" : ISODate("2020-06-28T22:07:26.381Z"), "timestamp" : NumberLong("1593382046381707906"), "stored_type" : "geometry_msgs/Pose", "published_at" : ISODate("2020-06-28T22:07:26.381Z"), "inserted_by" : "/livox_razor_node", "stored_class" : "geometry_msgs.msg._Pose.Pose" } }
// { "_id" : ObjectId("5ef9149e1d41c811f6b647ad"), "position" : { "y" : 0, "x" : 0, "z" : 222 }, "orientation" : { "y" : 0, "x" : 0, "z" : 0, "w" : 0 }, "_meta" : { "name" : "my pose", "inserted_at" : ISODate("2020-06-28T22:07:26.389Z"), "timestamp" : NumberLong("1593382046389272928"), "stored_type" : "geometry_msgs/Pose", "published_at" : ISODate("2020-06-28T22:07:26.389Z"), "inserted_by" : "/livox_razor_node", "stored_class" : "geometry_msgs.msg._Pose.Pose" } }
