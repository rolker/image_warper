// Author : Deepak Narayan
//Reference : https://github.com/rolker/halo_radar/blob/master/src/halo_radar/halo_radar.cpp - Authored by Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright May 2020, All rights reserved.


#include "iostream"
#include "ros/ros.h"    
#include <netinet/in.h> //Internet Address family and for socket
#include <unistd.h>     //for socket close and constants
#include "string.h"     //for memset and strings.
#include <mutex>        //for mutexes and locks.
#include "x265.h"       //for h265 encoding
#include <sensor_msgs/Image.h>  //for sensor messages.

class output_server{
public:
    output_server()
    :exitFlag(false){
        //will need to modify this node to access the h265 version of video.
        finalImage_Subscriber = node_handle_1.subscribe("finalimage/image_raw", 10, &output_server::udp_videoTransferCallback, this);
        //<check> pass ipaddress here.
        int listen_socket = createListenerSocket(1737);
        if (listen_socket < 0){
            perror("Server Listener socket creation failed.");
            return; //<check> using a return in constructor is fine, but will it cause issue if object.method() is called in next line in main?
        }
        processData();
    }
    
    ~output_server(){
        {
            const std::lock_guard<std::mutex> lock(exitFlagMutex);
            exitFlag = true;
        }
    }
    
    
private:
    ros::NodeHandle node_handle_1;
    ros::Subscriber finalImage_Subscriber;
    bool exitFlag;
    std::mutex exitFlagMutex;
    void udp_videoTransferCallback(const sensor_msgs::Image::ConstPtr& inpMsg){
        
    }
    //<check> passing of the ipaddress should be required, either as const std::string or as 
    int createListenerSocket(uint16_t port){
        //1. Create an unbound UDP socket and returns socket file descriptor.
        //AF_INET - for IPV4. Use AF_INET6 for IPV6.
        int retCode = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if(retCode < 0) //if error, return the negative value.
            return retCode;
        int one = 1;
        //2.Setting socketoptions - SO_REUSEADDR, SO_RCVTIMEO
        if (setsockopt(retCode, SOL_SOCKET, SO_REUSEADDR, (const char *)&one, sizeof(one)))
        {
            close(retCode); //closes socket file descriptor
            return -1;
        }
        //create a timeout of 1s 0microseconds
        timeval timeout; timeout.tv_sec = 1;timeout.tv_usec = 0;
        if (setsockopt (retCode, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
        {
            close(retCode);
            return -1;
        }
        //3. For listener, specifies an ipaddress and port for the family.
        sockaddr_in listenAddress;
        //initialize listeneraddress memory block to all zeroes.
        memset(&listenAddress, 0, sizeof(listenAddress));
        //set family and ip addr, port.
        listenAddress.sin_family = AF_INET;
        //htonl is hosttonetwork long. in host, Least Significant Byte is first, in network, Most Significant Byte first.
        //<check> - we need to change this. INADDR_ANY - accepts connections to any ip on the machine
        listenAddress.sin_addr.s_addr = htonl(INADDR_ANY);
        listenAddress.sin_port = htons(port);
        //4.Bind the socket to the ipaddress
        if (bind(retCode, (sockaddr *)&listenAddress, sizeof(listenAddress)) < 0)
        {
            close(retCode);
            return -1;
        }
    }
    
    void processData(){
        uint8_t in_data[65535]; //buffer
        while (true){
            //do we need the mutex here for this case?
            {//to give a special scope so that mutex lock scopes out immediately after the braces end.
                const std::lock_guard<std::mutex> lock(exitFlagMutex);
                if (exitFlag)
                    break;
            }
        
            
        }
        

    }

};

int main(int argc, char** argv){
    ros::init(argc, argv, "video_server_node");
    output_server server;
        
    while(ros::ok()){
        
        ros::spinOnce();
    }
    
}
