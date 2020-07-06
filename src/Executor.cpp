// Author : Deepak Narayan
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright Jan 2020, All rights reserved.

#include "cameraSetup.h"
#include <thread>
#include <mutex>

using namespace std;
using namespace cv;

// Variables declaration
const int CONST_NO_OF_PIXELS_Y_ROWS = 500;
const int CONST_NO_OF_PIXELS_X_COLS = 1000;
//<check - how do we get the number of cameras?>
const int NO_OF_CAMERAS = 5;

Mat finalImage(CONST_NO_OF_PIXELS_Y_ROWS, CONST_NO_OF_PIXELS_X_COLS, CV_8UC3);
sensor_msgs::ImagePtr msg;
image_transport::Publisher finalimage_publisher;
vector<cameraSetup*> cameraVector; //<check>
vector<string> camera_names;
vector<thread> camera_threads;
shared_ptr<mutex> sharedMutexPtr = make_shared<mutex>();

//will this cause issue if we multithread it?
void dynamicConfigurecallback(image_warper::image_warperConfig &config, uint32_t level) {
    for (int i = 0; i < NO_OF_CAMERAS; i++){
        cameraVector[i]->setCameraTransformDelay(config.transformation_delay);
        cout << "value of dyn param has been set to : " << config.transformation_delay << ", for camera " << cameraVector[i]->camera_name << endl;
    }
}

// Define a function/ lambda expression for the threads and callable.
void callableFunc(std::string name, ros::NodeHandle& handle, cv::Mat& image, sensor_msgs::ImagePtr& message, image_transport::Publisher& publisher_obj, shared_ptr<mutex> ptrSharedMutex, int y_rows, int x_cols) { 
        //we get a pointer back. do we need new -> we do. destructor will take care of the delete operation.
        //cout << "thread called : " << endl;
        cameraVector.push_back(new cameraSetup(name, handle, image, message, publisher_obj, ptrSharedMutex, y_rows, x_cols));  
}
    
//bool videoStreamCallbackForVR(){
//    }


int main(int argc, char** argv){
    for (int i = 0; i < NO_OF_CAMERAS; i++){
        camera_names.push_back("pano_" + to_string(i+1));
        cout << "camera names are : " << camera_names[i] << endl;
    }
    //<check how to remove the memory leak> - write a destructor inside camerasetup but also we need a delete.
    ros::init(argc, argv, "imageStabilize_360VR");  //node name.
    ros::NodeHandle nodeHandler1;
    dynamic_reconfigure::Server<image_warper::image_warperConfig> dynamic_config_server;
    //declare callback variable
    dynamic_reconfigure::Server<image_warper::image_warperConfig>::CallbackType f;
    //define callback. we dont need 'this' because we are not writing it as a class.
    
    ros::NodeHandle nodeHandler2_pub;
    image_transport::ImageTransport imgTransp(nodeHandler2_pub);
    finalimage_publisher = imgTransp.advertise("finalimage/image_raw", 1);
    //IMportant, we have to pass the callable as &callablename, ekse tuple error is coming.
    //std::thread thread1(&callableFunc,"thread1", std::ref(nodeHandler1), std::ref(finalImage), std::ref(msg), std::ref(finalimage_publisher), sharedMutexPtr, CONST_NO_OF_PIXELS_Y_ROWS, CONST_NO_OF_PIXELS_X_COLS);
    for (int i = 0; i < NO_OF_CAMERAS; i++){
        //<check> do from here - undefined ref, need to modify config files.
        //camera_threads.push_back(std::thread(callableFunc,camera_names[i], std::ref(nodeHandler1), std::ref(finalImage), std::ref(msg), std::ref(finalimage_publisher), sharedMutexPtr, CONST_NO_OF_PIXELS_Y_ROWS, CONST_NO_OF_PIXELS_X_COLS));
        callableFunc(camera_names[i], nodeHandler1, finalImage, msg, finalimage_publisher, sharedMutexPtr, CONST_NO_OF_PIXELS_Y_ROWS, CONST_NO_OF_PIXELS_X_COLS);
        
    }
    //subscribers for camera image data
    /*ros::Subscriber inputImage_Subscriber1 = nodeHandler1.subscribe("/pano_1/image_raw",10, inputImage_camera1CallBack);
    ros::Subscriber inputImage_Subscriber2 = nodeHandler1.subscribe("/pano_2/image_raw",10, inputImage_camera2CallBack);
    */    
    
    //for testing in Rviz
    //ros::init(argc,argv, "final_image_publisher1");
    //ros::ServiceServer nodeHandler2_pub.advertiseService(const string& service, videoStreamCallbackForVR);
    ros::Rate loops_per_sec(50); //setting no of loops per second. It is the Hz value.
    //cout << "camera 1 name is : " << cameraVector[0]->camera_name << endl;
    //cout << "camera 2 name is : " << cameraVector[1]->camera_name << endl;
    // rate of subscriber is actually the rate at which the publisher publishes.
    
    f = boost::bind(&dynamicConfigurecallback,  _1, _2);
    dynamic_config_server.setCallback(f);
    ros::MultiThreadedSpinner spinner(NO_OF_CAMERAS);
    //while(ros::ok()){
//         if (!finalImage.empty()){
//             cout << "entered whiletrue loop" << endl;
//             msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", finalImage).toImageMsg(); //bgr8 is blue green red with 8UC3
//             if ((msg != nullptr) && (finalimage_publisher!= NULL)){
//                 finalimage_publisher.publish(msg);
//                 cout << "final image published.." << endl;
//             }   
//             else{
//              cout << "Image publishing condition failed for this iteration." << endl;   
//             }
//         }
//         else{
//             cout << "final image is empty.." << endl;
//         }
        //ros::spinOnce();
        
        spinner.spin();
    //}
    
}
