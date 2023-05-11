/* Header file with functions, classes, namespaces using in 'control node'
 */

#include "PAPI.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

/**************************************************** DEFINES ****************************************************/
namespace PAPI_ros
{
    namespace heartbeat
    {
        class Sender
        {
        public:
            Sender();
            Sender(const ros::NodeHandle &_nh);

            void heartbeatPublish(const std::string &_node_name);

        private:
            ros::NodeHandle nh;
            ros::Publisher pub;
        };

        class Listener
        {
        public:
            Listener();
            Listener(const ros::NodeHandle &_nh);

            // Callback function for heartbeat listener
            void heartbeatCallBack(const std_msgs::String::ConstPtr &_heartbeat);

        private:
            ros::NodeHandle nh;
            ros::Subscriber sub;
        };
    }

}

/*************************************************** IMPLEMENTS ***************************************************/

/****************************** heartbeat ******************************/

PAPI_ros::heartbeat::Sender::Sender() {}

PAPI_ros::heartbeat::Sender::Sender(const ros::NodeHandle &_nh = ros::NodeHandle()) : nh(_nh),
                                                                                      pub(nh.advertise<std_msgs::String>("heartbeat", 10))
{
}

void PAPI_ros::heartbeat::Sender::heartbeatPublish(const std::string &_node_name)
{
    int node_pid = PAPI::system::getPID(_node_name);

    std::stringstream ss;
    ss << node_pid;
    std_msgs::String msg;
    msg.data = ss.str();

    // ros::Rate loop_rate(1);
    // while (ros::ok())
    // {
    //     pub.publish(msg);

    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    pub.publish(msg);
}

/********************************/

PAPI_ros::heartbeat::Listener::Listener() {}

PAPI_ros::heartbeat::Listener::Listener(const ros::NodeHandle &_nh = ros::NodeHandle()) : nh(_nh),
                                                                                          sub(nh.subscribe("heartbeat", 10, &PAPI_ros::heartbeat::Listener::heartbeatCallBack, this))
{
}

void PAPI_ros::heartbeat::Listener::heartbeatCallBack(const std_msgs::String::ConstPtr &_heartbeat)
{
}
