// Author : Deepak Narayan
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright Jan 2020, All rights reserved.

#include "cameraSetup.h"

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

//will this cause issue if we multithread it?
void dynamicConfigurecallback(image_warper::image_warperConfig &config, uint32_t level) {
    for (int i = 0; i < NO_OF_CAMERAS; i++){
        cameraVector[i]->setCameraTransformDelay(config.transformation_delay);
        cout << "value of dyn param has been set to : " << config.transformation_delay << ", for camera " << cameraVector[i]->camera_name << endl;
    }
}

//bool videoStreamCallbackForVR(){
//    }


int main(int argc, char** argv){
    string camera_names[] = {"pano_1", "pano_2", "pano_3", "pano_4", "pano_5"}; 
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
    for (int i = 0; i < NO_OF_CAMERAS; i++){
        //<check> do from here - undefined ref, need to modify config files.
        //we get a pointer back. do we need new -> we do. destructor will take care of the delete operation.
        cameraVector.push_back(new cameraSetup(camera_names[i], nodeHandler1, finalImage, msg, finalimage_publisher, CONST_NO_OF_PIXELS_Y_ROWS, CONST_NO_OF_PIXELS_X_COLS));
    }
    //subscribers for camera image data
    /*ros::Subscriber inputImage_Subscriber1 = nodeHandler1.subscribe("/pano_1/image_raw",10, inputImage_camera1CallBack);
    ros::Subscriber inputImage_Subscriber2 = nodeHandler1.subscribe("/pano_2/image_raw",10, inputImage_camera2CallBack);
    */    
    
    //for testing in Rviz
    //ros::init(argc,argv, "final_image_publisher1");
    //ros::ServiceServer nodeHandler2_pub.advertiseService(const string& service, videoStreamCallbackForVR);
    ros::Rate loops_per_sec(50); //setting no of loops per second. It is the Hz value.
    cout << "camera 1 name is : " << cameraVector[0]->camera_name << endl;
    cout << "camera 2 name is : " << cameraVector[1]->camera_name << endl;
    // rate of subscriber is actually the rate at which the publisher publishes.
    
    f = boost::bind(&dynamicConfigurecallback,  _1, _2);
    dynamic_config_server.setCallback(f);
    
    while(ros::ok()){
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
        ros::spinOnce();
    }
    
}
