// Author : Deepak Narayan
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright Jan 2020, All rights reserved.

#include "cameraSetup.h"
#include <thread>
#include <mutex>
#include <ros/callback_queue.h>
//#include "videoio.hpp" //for testing reading from video files, this can be removed later.

using namespace std;
using namespace cv;

// Variables declaration
const int CONST_NO_OF_PIXELS_Y_ROWS = 500;
const int CONST_NO_OF_PIXELS_X_COLS = 1000;
//<check - how do we get the number of cameras?>
const int NO_OF_CAMERAS = 6;
Mat finalImage(CONST_NO_OF_PIXELS_Y_ROWS, CONST_NO_OF_PIXELS_X_COLS, CV_8UC3, Scalar::all(0));
sensor_msgs::ImagePtr msg;
image_transport::Publisher finalimage_publisher;

vector<cameraSetup*> cameraVector; //<check>
vector<string> camera_names;
vector<thread> camera_threads;
int blend_type = cv::detail::Blender::MULTI_BAND; ///Blender::MULTI_BAND or Blender::FEATHER or Blender::NO
cv::Ptr<cv::detail::Blender> blender; //pointer for blender as from sample in opencv
bool debugOn = false;


//This causes issue if we use MultiThreadedSpinner as the main thread will be blocked. So use AsyncSpinner. AsyncSpinner also has an issue that it will process the dynamic callback only when callback q is the global queue.
//note : if we have more than 6 cameras, we need to add an entry to the cfg file.
void dynamicConfigurecallback(image_warper::cameraDelayConfig &config, uint32_t level) {
        double_t delay;
    int16_t pixels = config.blend_parameter_in_pixels;
    for (int i=0; i < NO_OF_CAMERAS; i++){
        switch(i){
            case(0) :
            {
                cout << "case0 reached" << endl;
                delay = config.transformation_delay_cam_1;  
                break;
            }
            case(1) :
            {
                cout << "case1 reached" << endl;
                delay = config.transformation_delay_cam_2;
                break;
            }
            case(2) :
            {    
                cout << "case2 reached" << endl;
                delay = config.transformation_delay_cam_3;
                break;
            }
            case(3) :
            {    
                cout << "case3 reached" << endl;
                delay = config.transformation_delay_cam_4;
                break;
            }
            case(4) :
            {    
                cout << "case4 reached" << endl;
                delay = config.transformation_delay_cam_5;
                break;
            }
            case(5) :
            {    
                cout << "case5 reached" << endl;
                delay = config.transformation_delay_cam_6;
                break;
            }
            
            default :
            {    
                cout << "default case reached" << endl;
                delay = config.transformation_delay_cam_default;
            }
        }
        cameraVector[i]->setCameraTransformDelay(delay);
        cout << "value of dyn param has been set to : " << delay << ", for camera " << cameraVector[i]->camera_name << endl;
        cameraVector[i]->setCameraBlendAreaInPixels(pixels);
        cout << "value of blend pixels has been set to : " << pixels << ", for camera " << cameraVector[i]->camera_name << endl;
    }
}

// //setting blend number of pixels
// void callbackBlend(image_warper::cameraDelayConfig &config, uint32_t level) {
//     int16_t pixels = config.blend_parameter_in_pixels;
//     for (int i=0; i < NO_OF_CAMERAS; i++){
//         cameraVector[i]->setCameraBlendAreaInPixels(pixels);
//         cout << "value of blend pixels has been set to : " << pixels << ", for camera " << cameraVector[i]->camera_name << endl;
//     }    
// }



// Define a function/ lambda expression for the threads and callable.
void callableFunc(std::string name, ros::NodeHandle& handle, cv::Mat& image, sensor_msgs::ImagePtr& message, image_transport::Publisher& publisher_obj, int y_rows, int x_cols) { 
        //we get a pointer back. do we need new -> we do. destructor will take care of the delete operation.
        //cout << "thread called : " << endl;
        cameraVector.push_back(new cameraSetup(name, handle, image, message, publisher_obj, y_rows, x_cols));  
}
    
//bool videoStreamCallbackForVR(){
//    }

void checkSyncUpofPoppedQueues(int imageIdToSyncOn){
 //cout << "camera image id : " << camera_name << "-" << current_image_id << endl;
    for (int i = 0; i < NO_OF_CAMERAS; i++){
        //if current image id is not same as image id to sync on, keep on popping.
        while(true){
            if (cameraVector[i]->current_image_id < imageIdToSyncOn){
                cout << "popping : " << cameraVector[i]-> camera_name << endl;
                cameraVector[i]-> popCameraQueue();
            }
            else{
                break;                
            }
        }
        
    }    
}

int main(int argc, char** argv){
    /*Below code is to try open a video file and convert to a ROS topic.
    
    // V Imp : change the value in constructor to -1 and -1 for width and height.
    vector<VideoCapture*> captureVector;
    for (int i = 0; i < NO_OF_CAMERAS; i++){
        captureVector.push_back(new VideoCapture("/home/ubuntu/data/" + to_string(i+1) + "_2020-07-07_14-10-25.mp4")); // open file
        if(!captureVector[i]->isOpened()){  // check if we succeeded
            cout << "not open 1" << endl;
        return -1;
        }
    }
    ros::NodeHandle nodeHandler_inputVideo;
    image_transport::ImageTransport inpVideoTransport(nodeHandler_inputVideo);
    CameraPublisher cam(inpVideoTransport, nodeHandler_inputVideo, "camera")
    */
/*    Mat edges;
    namedWindow("edges",1);
    while (true){
        Mat frame;
        *captureVector[4] >> frame; // get a new frame from camera
        imshow("edges", frame);
        if(waitKey(30) >= 0) break;
    }
*/

    /* Original code starts here
    */
        
    for (int i = 0; i < NO_OF_CAMERAS; i++){
        camera_names.push_back("pano_" + to_string(i+1));
        cout << "camera names are : " << camera_names[i] << endl;
    }
    
    //<check how to remove the memory leak> - write a destructor inside camerasetup but also we need a delete.
    ros::init(argc, argv, "imageStabilize_360VR");  //node name.
    ros::NodeHandle nodeHandler1;
    //not using a new callbackQ anymore cos it doesnt pick the dynamic reconfigure callback.
    ros::CallbackQueue my_callback_queue; //seperate callback queue for the cameras, instead of using the global callback queue for ros nodes. - this is not working for the dynamic reconfigure callback, so I have reverted to the global Q.
    //nodeHandler1.setCallbackQueue(&my_callback_queue);
    my_callback_queue.callAvailable(ros::WallDuration());
    
    //Dynamic server declaration
    dynamic_reconfigure::Server<image_warper::cameraDelayConfig> dynamic_config_server;
    //dynamic_reconfigure::Server<image_warper::cameraDelayConfig> server_blend;
    //Declare callback variable
    dynamic_reconfigure::Server<image_warper::cameraDelayConfig>::CallbackType f;
    //dynamic_reconfigure::Server<image_warper::cameraDelayConfig>::CallbackType f_blend;
    //define callback. we dont need 'this' because we are not writing it as a class.
    
    //ros::NodeHandle nodeHandler2_pub;
    //image_transport::ImageTransport imgTransp(nodeHandler2_pub);
    image_transport::ImageTransport imgTransp(nodeHandler1);
    //if publisher has queue_size = 1, it drops older msgs if not sent yet. Keeping it to 10, so that publisher can buffer upto 10 msgs.
    finalimage_publisher = imgTransp.advertise("finalimage/image_raw", 50);
    //IMportant, we have to pass the callable as &callablename, ekse tuple error is coming.
    //std::thread thread1(&callableFunc,"thread1", std::ref(nodeHandler1), std::ref(finalImage), std::ref(msg), std::ref(finalimage_publisher), sharedMutexPtr, CONST_NO_OF_PIXELS_Y_ROWS, CONST_NO_OF_PIXELS_X_COLS);
    for (int i = 0; i < NO_OF_CAMERAS; i++){
        //<check> do from here - undefined ref, need to modify config files.
        //camera_threads.push_back(std::thread(callableFunc,camera_names[i], std::ref(nodeHandler1), std::ref(finalImage), std::ref(msg), std::ref(finalimage_publisher), sharedMutexPtr, CONST_NO_OF_PIXELS_Y_ROWS, CONST_NO_OF_PIXELS_X_COLS));
        callableFunc(camera_names[i], nodeHandler1, finalImage, msg, finalimage_publisher, CONST_NO_OF_PIXELS_Y_ROWS, CONST_NO_OF_PIXELS_X_COLS);
        
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
    //f_blend = boost::bind(&callbackBlend,  _1, _2);
    dynamic_config_server.setCallback(f);
    //server_blend.setCallback(f_blend);
    //ros::MultiThreadedSpinner spinner(NO_OF_CAMERAS);
    //ros::MultiThreadedSpinner spinner(0);
    //spinner.spin(&my_callback_queue);

    //Mutex is non-copyable and non-movable in C++. So we cannot use an assignment operator. Hence have accessed it from a static method using pointers.
    
    std::mutex* sharedMainMutex = cameraSetup::getSharedMutex(); //shared across all objects/cameras
    //check redo this.
    //if (sharedMainMutex == nullptr){
    //    ROS_ERROR("Shared Mutex is NULL. Exiting..");
    //        return;
    //}
    cv::Size finalImagesize = cv::Size(CONST_NO_OF_PIXELS_Y_ROWS,CONST_NO_OF_PIXELS_X_COLS);
    finalImage.create(finalImagesize, CV_8UC3);
    finalImage.at<int>(0,0) = (255, 255, 255); //point on tl is mad white so that blender wont truncate.
    finalImage.at<int>(499,999) = (255, 255, 255); //point on br is made white
    //cout << "finalImage tl value : " << finalImage(Range(0,2), Range(0,2)) << endl;
    cout << "final size : " << finalImage.size() << endl;

    ros::AsyncSpinner async_spinner(0);
    async_spinner.start();
    cout << "Async spinner has been started - updated." << endl;
    while(ros::ok){
        //blending code starts here    
        cout << ".." << endl;
        //cout << "cameraVector.size() " << cameraVector.size() << endl;
        vector<Mat> cameraImages;
        vector<Mat> cameraImageMasks;
        vector<Point> corners;  //tl corners of all images
        vector<Size> sizes;     //sizes of all images
        //lets pop 1 image each and then check if all are same ids.
        priority_queue<int> imageIds;
        for (int i = 0; i < NO_OF_CAMERAS; i++){
            int id = cameraVector[i]-> popCameraQueue();
            imageIds.push(id);
        }
        //whatever is the highest value of image id, we will sync all images to that.
        checkSyncUpofPoppedQueues(imageIds.top());
        for (int i = 0; i < NO_OF_CAMERAS; i++){
            //all images, masks, tl corners and sizes will get saved to member Variables for each camera.
            //use this wherever you read the camera images, corners n sizes.
            //<check> will this cause a deadlock bcos c++ docs say twice locking by same thread may deadlock. Maybe i am misunderstanding? - it deadlocks if i nest the function within lock.
            
            {
                std::lock_guard<std::mutex> lock(*sharedMainMutex);
                //cout << "camera name : " << cameraVector[i]->camera_name << endl;
                //cout << "cameraVector[i]->current_image_size " << cameraVector[i]->current_image_size << endl;
                //cout << "cameraVector[i]-> current_mask size: " << cameraVector[i]-> current_mask.size() << endl;
                //cout << "cameraVector[i]->current_tl : " << cameraVector[i]->current_tl << endl;
                //cout << "Processing : cameraVector[i]->current_image_id : " << cameraVector[i]->current_image_id << endl;
                int rows = cameraVector[i]->current_image_size.height;
                int cols = cameraVector[i]->current_image_size.width; 
                int w = 0;
                Point tp1 = cameraVector[i]->current_tl;
                Point tp2(0,0); //optional for split images for negative coordinates.
                bool second_mask_reqd = false; //true if tp1.x < 0 and if it gets split.
                Mat img2, img2_s; //optional for split images for negative coordinates.
                Mat mask2_warped_gray_img; //optional for split images for negative coordinates.
                cv::Size s2 = cameraVector[i]->current_image_size;
                int orig_width = cameraVector[i]->current_image_size.width; //copying for later use
                if (tp1.x < 0){//manually changed from neg to 0 - affects pano_2 it seems
                //cout << "last point of image as per this is : tp1.x + orig_width " << tp1.x + orig_width << endl;
                    if (tp1.x + orig_width > 0){//2 masks required
                        //tp1.x = tp1.x + image_x_cols;
                        second_mask_reqd = true;
                        //img1 is the negative part being processed, img2 is from 0,0.
                        //for image1
                        //note use the image_x_cols only for updating points.
                        //if we use it for getting parts of image, it will go out of bounds, as our image is much smaller than the panorama.
                        w = -1 * tp1.x; //width is absolute value of tp1.x
                        tp1.x = tp1.x + CONST_NO_OF_PIXELS_X_COLS; //get top left of image 1. tp1.y remains same as before.
                        tp2.x = 0;                    //tl of image 2 starts from 0.
                        tp2.y = tp1.y;                //tp2.y remains same as tp1.y.
                        //For range, left end is inclusive, rt end is exclusive, and rows begin from 0, not 1.
                        //so here lets say tp1.x = -15 and original width = 100. so, img1 width = 15. img2 width = 100-15 = 85.
                        //split is from (1000+(-15)) to 999 and from 0 to 84, which is , (original width -img1 width - 1).
                        //so Range(985, 1000) and Range(0, 85)
                        //cout << "printing roi img2 : " << endl;
                        /*cout << "w : " << w << endl;
                        cout << "orig_width : " << orig_width << endl;
                        cout << "img2 starts : 0 " << endl;
                        cout << "img2 ends(exclusive) : orig_width-w : " << orig_width-w << endl;
                        cout << "img1 starts : new tp1.x : " <<  tp1.x << endl;
                        cout << "img1 ends(exclusive) : image_x_cols : " << image_x_cols << endl;
                        cout << camera_name << " : temp_warped_img size before split : " << s << endl;*/
                        cameraVector[i]->current_image(Range::all(), Range(w, orig_width)).copyTo(img2); //deep copy - do this first
                        s2 = img2.size();
                        /*cout << "img2 size : " << s2 << endl;
                        cout << "img1 starts : new tp1.x : " <<  tp1.x << endl;
                        cout << "img1 ends(exclusive) : image_x_cols : " << image_x_cols << endl;*/
                        //<check> if this works.
                        cameraVector[i]->current_image = cameraVector[i]->current_image(Range::all(), Range(0,w)); //overwriting temp_warped_img - do this second
                        cameraVector[i]->current_image_size = cameraVector[i]->current_image.size();
                        mask2_warped_gray_img = cameraVector[i]->current_mask(Range::all(), Range(w,orig_width));
                        cameraVector[i]->current_mask = cameraVector[i]->current_mask(Range::all(), Range(0,w));
                        //do from here
                        //cout << "img1 size : " << s << endl;
                        /*no need for this i guess
                        s.width = w;    //updating width of mask1
                        //for image2            
                        s2.width = orig_width - w;    
                        */     
                    }
                    else{//only 1 mask required
                    tp1.x = tp1.x + CONST_NO_OF_PIXELS_X_COLS; //update top left of image 1.     
                    }
                }
                cameraVector[i]->current_tl = tp1;
                corners.push_back(cameraVector[i]->current_tl); //tp1 is same as current_tl
                sizes.push_back(cameraVector[i]->current_image_size);
                cameraImages.push_back(cameraVector[i]->current_image);
                cameraImageMasks.push_back(cameraVector[i]->current_mask);
                if (second_mask_reqd){
                    corners.push_back(tp2);
                    sizes.push_back(s2);
                    cameraImages.push_back(img2);
                    cameraImageMasks.push_back(mask2_warped_gray_img);
                }
                //<check> have to split masks too.
                
            }
        }//end of looping through cameras.

            blend_type = cv::detail::Blender::FEATHER;
            blender = cv::detail::Blender::createDefault(blend_type, false); //false given for CUDA processing
            Size dst_sz = cv::detail::resultRoi(corners, sizes).size();
            //Rect resultROI = cv::detail::resultRoi(corners, sizes);
            //Mat ROI = finalCameraImage(resultROI);
            //imshow("resultROI", ROI);
            //change this to global variable
            float blend_strength = 10;
            float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
            //reference : https://docs.opencv.org/3.4/d9/dd8/samples_2cpp_2stitching_detailed_8cpp-example.html#a53
            if (blend_type == cv::detail::Blender::FEATHER){
                cv::detail::FeatherBlender* feather_blender = dynamic_cast<cv::detail::FeatherBlender*>(blender.get());
                feather_blender->setSharpness(1.f/blend_width);
                //feather_blender->setSharpness(1);
                //cout << "Feather blender, sharpness: " << feather_blender->sharpness();                
            }
            else if (blend_type == cv::detail::Blender::MULTI_BAND){
                cv::detail::MultiBandBlender* multiband_blender = dynamic_cast<cv::detail::MultiBandBlender*>(blender.get());
                multiband_blender-> setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
                //multiband_blender->setNumBands(1);
                //cout << camera_name << endl;
                //cout << "Multi-band blender, number of bands: " << multiband_blender->numBands();
            }
            else {//this should never be invoked
             ROS_ERROR("Error : %s", "Blend type is invalid.");
             
             //break;
             //return;
            }
            blender->prepare(corners, sizes);
            Mat finalCameraImage_s, finalCameraImage, result_mask; //Mats for blended result
            int top_left_final_image = 0;
            //create a min priority Q.
            std::priority_queue<int, vector<int>, greater<int>> tl_y_coord_Queue;
            {   //use this wherever you read the camera images, corners n sizes.
                std::lock_guard<std::mutex> lock(*sharedMainMutex);
                //cameraImages can have 2 split images from same camera image. It covers all cameras.
                for (int i = 0; i < cameraImages.size(); i++){
                    (cameraImages[i]).convertTo(cameraImages[i], CV_16SC3);
                    //try printing tl and size here and see.
                    blender->feed(cameraImages[i], cameraImageMasks[i], corners[i]);
                    //cout << "Point "<< i << " is : " << corners[i] << endl;
                    //cout << "x is " << corners[i].x << endl;
                    tl_y_coord_Queue.push(corners[i].y);
                }
                top_left_final_image = tl_y_coord_Queue.top();
                blender->blend(finalCameraImage_s, result_mask);
            }
            finalCameraImage_s.convertTo(finalCameraImage, CV_8UC3); //converting back to Unsigned
            //cout << "reached debug 1" << endl;
            if (!result_mask.empty()){
                //cout << "result mask size ; " << result_mask.size() << endl;
                //cout << "finalCameraImage size ; " << finalCameraImage.size() << endl;
                //finalCameraImage.copyTo(finalImage, result_mask); 
/*                //trying with alpha mattes.
                // Multiply the foregrounds with the alpha mattes
                multiply(result_mask, finalCameraImage, finalCameraImage);
                Mat beta;
                // Multiply the background with ( 1 - alpha)
                subtract(beta, result_mask, beta);
                beta.convertTo(beta, CV_32FC3);    //do we need this?         
                multiply(beta, finalImage, finalImage);
                // Add the masked foreground and background.
                add(finalCameraImage, finalImage, finalImage);  
*/              
                //cout << "finalImage size before copy; " << finalImage.size() << endl;
                //cout << "finalImage top left is : " << top_left_final_image << endl;
                //finalCameraImage.copyTo(finalImage(cv::Rect(0,top_left_final_image, finalCameraImage.cols, finalCameraImage.rows)));
                //using the blended image as the mask itself.
                //CONST_NO_OF_PIXELS_Y_ROWS, CONST_NO_OF_PIXELS_X_COLS
                //finalCameraImage.copyTo(finalImage, finalCameraImage);
                finalCameraImage.copyTo(finalImage, result_mask);
                if (debugOn){
                    cout << "finalCameraImage size ; " << finalCameraImage.size() << endl;   
                    cout << "finalImage size ; " << finalImage.size() << endl;   
                    cout << "result_mask size ; " << result_mask.size() << endl;   
                }
            }
            //cout << "finalImage : " << finalImage.size().height << "," << finalImage.size().width << endl;
            if (!finalImage.empty()){
                cout << "final Image not empty" << endl;
                msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", finalImage).toImageMsg(); //bgr8 is blue green red with 8UC3
                if ((msg != nullptr) && (finalimage_publisher!= NULL)){
                    finalimage_publisher.publish(msg);
                    //cout << "final image published.." << endl;
                }   
                else{
                cout << "Image publishing condition failed for this iteration." << endl;   
                }
            }
            else{
                cout << "final image is empty.." << endl;
            }
        if (!ros::master::check()){
            //means master got stopped.
            cout << "master unavailable" << endl;
            async_spinner.stop(); //AsyncSpinner stops when node 
            
            for (int i = 0; i < NO_OF_CAMERAS; i++){
                delete cameraVector[i];
                cout << "deleting ... " << endl;
            }

            break;
        }
    }
        //blending code ends here

}
