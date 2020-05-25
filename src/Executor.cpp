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

//functions declaration
/*void info_camera1CallBack(const sensor_msgs::CameraInfo::ConstPtr&);
void inputImage_camera1CallBack(const sensor_msgs::CameraInfo::ConstPtr&);
*/

bool checkCameraResolution() {
    //do we need to check all cameras together?
    //not here, required in the Camera class.
    
}

bool checkCameraData() {
    //do we need to check all cameras together?
}


/*void parseYamlFile(string fileName){
    
    cout << "fileName : " << fileName << endl;
    FileStorage fs(fileName, FileStorage::READ);
    //fs.open(fileName, FileStorage::READ);
    if (!fs.isOpened()){
        cerr << "Failed to open yaml file " << fileName << " for extracting calibration parameters." << endl;
        exit(1); //<check if we need to terminate like this>
    }
    //FileNode camera_name = fs["camera_name"];   - use this in case of need.
    FileNode camera_matrix = fs["camera_matrix"];
    FileNode distortion_coefficients = fs["distortion_coefficients"];
    camera_intrinsics = Mat(3, 3, CV_8UC1); //think it should be CV_32FC1 bcos warp expects it in float. Got an error otherwise while using warp
    camera_intrinsics = (float)(camera_matrix["data"]);
    camera_dist_coefficients = Mat(1, 5, CV_8UC1); //think it should be CV_32FC1
    camera_dist_coefficients = (float)(distortion_coefficients["data"]);
    cout << "camera_matrix1: " << camera_intrinsics << endl;
    cout << "distortion_coefficients1: " << camera_dist_coefficients << endl;
    fs.release();
}*/


int main(int argc, char** argv){
    string camera_names[] = {"pano_1", "pano_2", "pano_3", "pano_4", "pano_5"}; 
    //<check how to remove the memory leak> - write a destructor inside camerasetup but also we need a delete.
    ros::init(argc, argv, "imageStabilize_360VR");
    ros::NodeHandle nodeHandler1;
    for (int i = 0; i < NO_OF_CAMERAS; i++){
        //<check> do from here - undefined ref, need to modify config files.
        //we get a pointer back. do we need new -> we do. destructor will take care of the delete operation.
        cameraVector.push_back(new cameraSetup(camera_names[i], nodeHandler1, finalImage, CONST_NO_OF_PIXELS_Y_ROWS, CONST_NO_OF_PIXELS_X_COLS));
    }
    //subscribers for camera image data
    /*ros::Subscriber inputImage_Subscriber1 = nodeHandler1.subscribe("/pano_1/image_raw",10, inputImage_camera1CallBack);
    ros::Subscriber inputImage_Subscriber2 = nodeHandler1.subscribe("/pano_2/image_raw",10, inputImage_camera2CallBack);
    */    
    
    //for testing in Rviz
    //ros::init(argc,argv, "final_image_publisher1");
    ros::NodeHandle nodeHandler2_pub;
    image_transport::ImageTransport imgTransp(nodeHandler2_pub);
    finalimage_publisher = imgTransp.advertise("finalimage/image_raw", 1000);
    ros::Rate loops_per_sec(50); //setting no of loops per second. It is the Hz value.
    cout << "camera 1 name is : " << cameraVector[0]->camera_name << endl;
    cout << "camera 2 name is : " << cameraVector[1]->camera_name << endl;
    // rate of subscriber is actually the rate at which the publisher publishes.
    while(ros::ok()){
        if (!finalImage.empty()){
            //cout << "entered whiletrue loop" << endl;
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", finalImage).toImageMsg(); //bgr8 is blue green red with 8UC3
            if ((msg != nullptr) && (finalimage_publisher!= NULL)){
                finalimage_publisher.publish(msg);
            }            
        }
        else{
            cout << "final image is empty.." << endl;
        }
        ros::spinOnce();
    }
    
}
