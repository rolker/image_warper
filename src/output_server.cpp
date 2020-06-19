#include <iostream>
#include <boost/asio.hpp>   //for networking and i/o
#include <boost/thread.hpp> //for multithreading

//asynchronous callback, will be executed if any thread completes the asynch wait.
void serverCallback(const boost::system::error_code& /*e*/){
    
    
}

int main(int argc, char** argv){
    //for all n/w operations, io_context object is required.
    boost::asio::io_context io_1;
    //boost objects usually always takes the first parameter as the io_context object in their constructors.
    boost::asio::steady_timer t(io_1, boost::asio::chrono::seconds(5));
    t.asynch_wait(&print); //non-blocking
    //t.wait();
    cout << "Print on sequential execution";
    
}
