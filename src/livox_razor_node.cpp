#include "mongodb_store/message_store.h"
#include "geometry_msgs/Pose.h"
#include <boost/foreach.hpp>

#include <sstream>
#include <cassert>

using namespace geometry_msgs;
using namespace mongodb_store;
using namespace std;
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
        ros::init(argc, argv, "example_mongodb_store_cpp_client");
        ros::NodeHandle nh;

        //Create object which does the work for us.
        MessageStoreProxy messageStore(nh);

        //This is the message we want to store
        Pose p;
        string name("my pose");
        //Insert something with a name, storing id too
        messageStore.insertNamed(name, p);

        p.position.z = 666;
        messageStore.insertNamed(name, p);
        p.position.z = 111;
        messageStore.insertNamed(name, p);
        p.position.z = 222;
        messageStore.insertNamed(name, p);

        vector< boost::shared_ptr<Pose> > results;

        //Get it back, by default get one
        if(messageStore.queryNamed<Pose>(name, results)) {

                BOOST_FOREACH( boost::shared_ptr<Pose> p,  results)
                {
                        ROS_INFO_STREAM("Got by name: " << *p);
                }
        }

        results.clear();
        // get all poses, should show updated named position
        messageStore.query<Pose>(results);
        BOOST_FOREACH( boost::shared_ptr<Pose> p,  results)
        {
                ROS_INFO_STREAM("Got: " << *p);
        }

        return 0;
}