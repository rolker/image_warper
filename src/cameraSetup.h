#ifndef CAMERA_SETUP_H
#define CAMERA_SETUP_H

#include "iostream"
#include "ros/ros.h"
#include <opencv2/core/mat.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core.hpp>                                 // for hconcat
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>                              // for namedWindow and waitkey(testing)
#include <opencv2/stitching/detail/warpers.hpp>
#include <opencv2/core/persistence.hpp>                     //file storage
#include <image_transport/image_transport.h>                //image transport for publisher(for rviz)
#include <opencv2/core/mat.hpp>
#include <tf2_ros/transform_listener.h>                     //for transforming frames
#include <tf2/LinearMath/Quaternion.h>                      //for creating quartenion
#include <tf2/LinearMath/Matrix3x3.h>                       //for converting quartenion to rotation matrix
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>            //to convert from TransformStamped.rotation to Quartenion
#include <opencv2/calib3d.hpp>                              //for creating a optimal camera matrix
//for dynamic parameterization
//below source : http://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile
#include "dynamic_reconfigure/server.h"
//The below config header file will be generated and saved.
#include "image_warper/cameraDelayConfig.h"
#include "boost/bind.hpp"
#include <thread>
#include <mutex>  //for multithreading locks.
#include "opencv2/stitching/detail/blenders.hpp" //for blending the images
#include <opencv2/stitching/detail/util.hpp> //for resultROI used for blending.
#include <opencv2/core/cvstd.hpp>           //for shared pointers.
#include <opencv2/imgproc/imgproc.hpp>      //for converting to gray scale.
#include <queue>

//#include <Eigen/Geometry>                                   //for quartenion to rotation matrix
//#include <tf2/LinearMath/Transform.h>                       //to convert from TransformStamped to Quartenion
//#include <tf2/geometry_msgs/Transform.h>                       //to convert from TransformStamped to Quartenion
//#include <geometry_msgs/Transform.h>                       //to convert from TransformStamped to Quartenion

class cameraSetup{
public:
    //cameraSetup();
    //cameraSetup(std::string);
    //cameraSetup(std::string, ros::NodeHandle&); //if we pass same handle, we can reuse same handle for all 6 cameras.
    cameraSetup(std::string, ros::NodeHandle&, cv::Mat&, sensor_msgs::ImagePtr&, image_transport::Publisher&, int, int);
    ~cameraSetup();
    void setCameraTransformDelay(double_t delay);
    void setCameraBlendAreaInPixels(int16_t pixels);
    void popCameraQueue();
    static std::mutex* getSharedMutex(){//public method which returns the static mutex - may need to change this concept.
     return &sharedMutex;   
    }

    
    std::string camera_name; //name is without the first forward slash in the topic, eg: pano_1
    int camera_height; //camera resolution height
    int camera_width; //camera resolution width
    //<check>have to initialize below in constructor - wud have liked to create a struct here instead of 4 variables and 4 queues.
    cv::Mat current_image;
    cv::Mat current_mask;
    cv::Point current_tl;
    cv::Size current_image_size;

    
private:
    // Variables declaration
    //<check>Do we need seperate Mat objects for undistort or can we overwrite on same? currently not overwriting.
    cv::Mat cam_data, undistorted_cam_data;
    cv::Mat camera_intrinsics, camera_dist_coefficients, rotation_matrix;
    cv::Mat finalCameraImage;
    sensor_msgs::ImagePtr msg;
    image_transport::Publisher finalimage_publisher;
    //static std::mutex sharedMutex; //shared across all objects/cameras
    int image_y_rows;
    int image_x_cols;
    tf2_ros::Buffer* tfBuffer;
    cv::Mat new_optimal_camera_matrix;
    std::queue<cv::Mat> camera_imageQueue;
    std::queue<cv::Mat> camera_maskQueue;
    std::queue<cv::Point> camera_imageTopLeftQueue;
    
    //This is very important. The TransformListener has to be declared as class or global variable. It needs time.
    //As per documentation,  The TransformListener object should be scoped to persist otherwise its cache will be unable to fill and almost every query will fail.Once the listener is created, it starts receiving tf2 transformations over the wire, and buffers them for up to 10 seconds.
    tf2_ros::TransformListener* tfListener;
//    image_transport::ImageTransport imgTransp_subscriber;
//    image_transport::Subscriber inputImage_Subscriber;
    ros::Subscriber camerainfo_Subscriber;
    ros::Subscriber inputImage_Subscriber;
    ros::Subscriber tf_Subscriber;
    ros::Subscriber tf_static_Subscriber;    
    double_t camera_transform_delay; //setting it as double_t bcos in cfg file, we do the same.
    int blend_type = cv::detail::Blender::MULTI_BAND; ///Blender::MULTI_BAND or Blender::FEATHER or Blender::NO
    cv::Ptr<cv::detail::Blender> blender; //pointer for blender as from sample in opencv
    int16_t pixels_to_blend; //setting it as int16_t bcos in cfg file, we do the same as int_t.
    cv::Mat dummy_white_Mat;
    cv::Mat undistort_dummy_white_Mat;
    static std::mutex sharedMutex; //shared across all objects/cameras
    //saving previous tl, image_size in case we dont get camera image for current iteration. So we create a black image with prev tl, prev img size and use that for building the panorama.
    cv::Point prev_tl;
    cv::Size prev_image_size;
    
    //Methods declaration
    bool checkCameraResolution();   //checks resolution of the camera if its valid.
    bool checkCameraData();         //checks input camera data if its valid.
    bool checkCameraParameters();   //checks input camera parameters(camera_intrinsics, camera_dist_coefficients, rotation_matrix) if its valid.
    bool calculateRotationMatrix(ros::Time);
    void calculateTransformationMatrix(); // unsure if not required.
    void setIntrinsics(const cv::Mat intrinsics);   //helper method
    void setDistortionCoefficients(const cv::Mat distortion_coef);  //helper method
    void setRotationMatrix(const cv::Mat rotationMat);  //helper method
    void setCameraParameters(const cv::Mat intrinsics, const cv::Mat distortion_coef, const cv::Mat rotationMat);
    void info_cameraCallBack(const sensor_msgs::CameraInfo::ConstPtr&);
    void inputImage_cameraCallBack(const sensor_msgs::Image::ConstPtr&);
    void process_3D_Map();
};

#endif
