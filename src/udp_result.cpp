#include "udp_bridge/udp_bridge.h"

std::map<udp_bridge::Channel,std::string> udp_bridge::UDPROSNode::topic_map;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_result");

    std::string host = "localhost";
    if (argc > 1)
        host = argv[1];

    int send_port = 7370;
    int receive_port = 7371;
    
    udp_bridge::UDPROSNode udpRosNode(host,send_port,receive_port);
    //change below.
    udpRosNode.addSender<geographic_msgs::GeoPointStamped,udp_bridge::position>("/udp/generated_position");
    udpRosNode.spin();
    return 0;
}
