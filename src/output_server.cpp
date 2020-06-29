#include "iostream"
#include "ros/ros.h"    
#include <netinet/in.h> //Internet Address family and for socket
#include <unistd.h>     //for close
#include "string.h"     //for memset and strings.

class output_server{
public:
    output_server(){
        //will need to modify this node to access the h265 version of video.
        finalImage_Subscriber = node_handle_1.subscribe("finalimage/image_raw", 10, &output_server::udp_videoTransferCallback, this);
    }
    
    ~output_server(){}
    
    
private:
    ros::NodeHandle node_handle_1;
    ros::Subscriber finalImage_Subscriber;
    void udp_videoTransferCallback(){
        
    }
    //<check> passing of the ipaddress should be required, either as const std::string or as 
    int createListenerSocket(const std::string inpIPAddress, uint16_t port){
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
        listenAddress.sin_port = port;
        //4.Bind the socket to the ipaddress
        if (bind(retCode, (sockaddr *)&listenAddress, sizeof(listenAddress)) < 0)
        {
            close(retCode);
            return -1;
        }
    }

};

int main(int argc, char** argv){
    ros::init(argc, argv, "udp_video_node");
    output_server server;
        
    while(ros::ok()){
        
        ros::spinOnce();
    }
    
}


// #include <iostream>
// #include <boost/asio.hpp>   //for networking and i/o
// #include <boost/thread.hpp> //for multithreading
// #include <boost/bind/bind.hpp>   //for binding parameters of functions to its signatures.
// 
// using namespace std;
// using boost::asio::ip::tcp;
// 
// class output_server{
// public: 
//     output_server(boost::asio::io_context& io)
//     :io_context_obj(io), 
//     {
//         //acceptor object created bound to port 1737 for ipv4.
//         //<check> should we have another acceptor object and port for ipv6?
//         acceptor(io_context_obj, tcp::endpoint(tcp::v4(), 1737));        
//     }
//     ~output_server(){
//      cout << "Final count is : " << counter << endl;   
//     }
//     void process_connections(){
//         //create a new connection object in order to create a socket.
//         tcp_connection::pointer new_connection = tcp_connection::create(io_context_obj);
//         //creates a new socket and asynchronously accepts it and processes the request(so does this happen in another thread, need to check.
//         //<check> does this happen in another thread
//         acceptor.async_accept(new_connection->socket, boost::bind(&output_server::handle_request, this, new_connection, boost::asio::placeholders:error));
//         
//     }
//     //this does not take error as a paramter anymore.
//     void print(){
//         
//     }
// private:
//     /*boost::asio::steady_timer timer1;
//     int counter;*/
//     boost::asio::io_context io_context_obj;
//     tcp::acceptor acceptor;
//     //<check> do we need to handle more issues/errors during accept?
//     void handle_request(tcp_connection::pointer new_connection, const boost::system::error_code& error){
//         if (!error){
//             //<check> this probably happens in same thread
//             new_connection->processRequest();
//         }
//         //this probably happens in same thread after each client request completes processing, so this should be changed.
//         process_connections();
//     }
// };
// 
// 
// //asynchronous callback, will be executed if any thread completes the asynch wait.
// //void serverCallback(const boost::system::error_code& /*e*/){
// void print(const boost::system::error_code& /*e*/, , boost::asio::steady_timer* timer1, int* counter){  
//     cout << "Asynchronous print" << endl;
//     //note : there is no explicit call to the io_context to ask it to stop.
//     if (*counter < 5){
//         cout << *counter << endl;
//         ++(*counter); //here this is ok, because only 1 thread uses this callback. else would be an issue.
//         timer1->expires_at(timer1->expiry() + boost::asio::chrono::seconds(1));
//         //here, boost::bind is used to match the function parameters in its signature.
//         timer1->asynch_wait(boost::bind(print, boost::asio::placeholders::error, timer1, counter));
//     }
// }
// 
// class tcp_connection
//     : public boost::enable_shared_from_this<tcp_connection>
// {
//     public:
//         typedef boost::shared_ptr<tcp_connection> pointer;
//         
//         static pointer create(boost::asio::io_context& io){
//             return pointer(new tcp_connection(io));
//         }
//         
//         
// };
// 
// int main(int argc, char** argv){
//     try{
//         //for all n/w operations, io_context object is required.
//         boost::asio::io_context io_context_1;     
//         output_server server(io_context_1);
//         server.process_connections();
//         io_context_1.run(); // will return when work is completed.
// 
//     }
//     catch(exception& e){//note that cout, cerr are non-threadsafe
//         cerr << e.what() << endl;        
//     }
// /*    int count = 0;
//     //boost objects usually always takes the first parameter as the io_context object in their constructors.
//     boost::asio::steady_timer t(io_context_1, boost::asio::chrono::seconds(1));
//     t.asynch_wait(boost::bind(print, boost::asio::placeholders::error, &t, &count)); //non-blocking
//     //t.wait();
//     cout << "Print on sequential execution";
//     //the callback for the asynchronous wait or any other methods will only work when io_context.run() is called. Else it will never be invoked. run() will execute as long as there is work to do. For the asynchronous wait, the work is the 5second wait + the work being done in the callback function. So in another thread, the wait, but in same thread, the callback?
//     cout << "Final count is : " << count << endl;
// */    
// }
