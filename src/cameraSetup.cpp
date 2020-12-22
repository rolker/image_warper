// Author : Deepak Narayan
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright Jan 2020, All rights reserved.

#include "cameraSetup.h"

using namespace std;
using namespace cv;
std::mutex cameraSetup::sharedMutex; //this has been done in Executor.cpp


bool cameraSetup::checkCameraResolution() {
    if ((camera_height ==  -1) || (camera_width ==  -1)){
     return false;
    }
    else{
     return true;   
    }   
}

bool cameraSetup::checkCameraData() {
    return (!cam_data.empty()); //returns true if cam_data is not empty.
}

bool cameraSetup::checkCameraParameters() {
    return (!((camera_intrinsics.empty())||(camera_dist_coefficients.empty())||(rotation_matrix.empty()))); //returns true only if all 3 camera parameters are non empty.
}

bool cameraSetup::calculateRotationMatrix(ros::Time capture_time){//if there is an issue during calculation, it will sleep through and try again in next cycle.
    //cout << "reached calculateRotationMatrix" << endl;
    //think we can remove the below 2 subscribers bcos we are using transform now.
    //tf_Subscriber = handle.subscribe("/tf",10, &cameraSetup::tf_cameraCallBack, this);
    //tf_static_Subscriber = handle.subscribe("/tf_static",10, &cameraSetup::tf_static_cameraCallBack, this);
    //<check> is there a tf2 subscriber??
    geometry_msgs::TransformStamped transformStampedValue;
    tf2::Quaternion q_value;
    tf2::Quaternion scalar(0.7071068, 0, 0, 0.7071068); //- about x axis
    //tf2::Quaternion scalar(0, 0.7071068, 0, 0.7071068); //- about y axis
    //<check> quartenion is getting rotation only, not the translation
    //<check> if transformStamped is initially not obtained, what should be done? think we have to introduce in callback an if statement.
    try{
        //parameters are target frame, source frame, time at which we want to transform(Time(0) is latest transform.), duration before timeout.
        //transformStamped - will get us the translation and rotation. I am using the rotation for warp. Not using translation currently.
        //transformStamped.transform.translation and transformStamped.transform.rotation
        //transformStampedValue = tfBuffer->lookupTransform("north_up_base_link" , camera_name + "_optical", capture_time+ros::Duration(camera_transform_delay));
        if (!tfBuffer->canTransform("north_up_base_link" , camera_name + "_optical", ros::Time(capture_time+ros::Duration(camera_transform_delay)), ros::Duration(10))){
            return false;   //if this cannot be done, then return false immediately.
        }
        transformStampedValue = tfBuffer->lookupTransform("north_up_base_link" , camera_name + "_optical", ros::Time(capture_time+ros::Duration(camera_transform_delay)));
        //convert msg into a quartenion - do we need to consider only the rotation part here by using transformStamped.transform.rotation? - yes.
        tf2::convert(transformStampedValue.transform.rotation , q_value);
        q_value = scalar * q_value;
//        cout << "q value.x " << q_value.x << endl;
        q_value.normalize();
        tf2::Matrix3x3 matrix;
        tf2::Vector3 dvec;
        matrix.setRotation(q_value);
        //cout << "q value.x " << q_value.x << endl;
        //cout << "rotation matrix before conversion of " + camera_name + " is : " << rotation_matrix << endl;
        for (int i=0; i< 3; i++){
            //cout << "row i is : " << matrix.getRow(i) << endl; // printing this does not work, it will give junk chars.
            dvec = matrix.getRow(i);
                for(int j=0; j<3; j++){
                //cout << "i , j is : " << i << " , " << j << endl;
                //cout << dvec[j] << endl;
                rotation_matrix.at<float>(i,j) = dvec[j];
            }
        }
        
        //cout << "rotation matrix of " + camera_name + " is : " << rotation_matrix << endl;
        //<check> how to add Eigen/Geometry - it might be very helpful later
        //cout << "transformStamped calculated for : " << camera_name << endl;
        return true;
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        //ros::Duration(1.0).sleep();
    }
    return false;
}


string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

void cameraSetup::process_3D_Map() {
    //cout << "process_3D_Map reached" << endl;
    Mat temp_warped_img, temp_warped_img_s, mask_warped_img, mask_warped_gray_img;
    bool second_mask_reqd = false; //true if tp1.x < 0 and if it gets split.
    Mat img2, img2_s; //optional for split images for negative coordinates.
    Mat mask2_warped_img, mask2_warped_gray_img; //optional for split images for negative coordinates.
    Point tp2(0,0); //optional for split images for negative coordinates.
    Mat finalCameraImage_mask=Mat(finalCameraImage.size(), CV_8UC1, Scalar::all(255));
    Mat finalCameraImage_s; //for blender feed, we need 16SC3 type.
    cv::detail::SphericalWarper sphWarp1(int(image_x_cols/(2*3.14)));
    //cv::detail::CylindricalWarper cylWarp1(int(image_x_cols/(2*3.14)));
    if (!undistorted_cam_data.empty()){
        //cout << "entered into processing part of process_3D_Map" << endl;
        //cout << "undistorted data ..... ===> " << undistort_inp(Range(1000, 1005),  Range(1000, 1002)) << endl;
        //cout << endl << "warping is being done" << endl;
        //Point tp1 is the top left corner of image
        Point tp1 = sphWarp1.warp(undistorted_cam_data, new_optimal_camera_matrix, rotation_matrix, INTER_LINEAR, 0, temp_warped_img);
        //warping the mask
        sphWarp1.warp(undistort_dummy_white_Mat, new_optimal_camera_matrix, rotation_matrix, INTER_LINEAR, 0, mask_warped_img);
        //converting to grayscale.
        cvtColor(mask_warped_img, mask_warped_gray_img, COLOR_BGR2GRAY);
        //using original camera matrix - Point tp1 = sphWarp1.warp(undistorted_cam_data, camera_intrinsics, rotation_matrix, INTER_LINEAR, 0, temp_warped_img);
        //whichever camdata is coming in callback, we need to pass that. Also the rotation matrix corresponding to that camera.
        //Point tp1 = cylWarp1.warp(undistorted_cam_data, new_optimal_camera_matrix, rotation_matrix, INTER_LINEAR, 0, temp_warped_img);
        //cout << "point is : " << tp1.x << "," << tp1.y << endl;
        
        if (tp1.y < 0){
            ROS_ERROR("Point returned from Spherical Warp has negative y-coordinate: %s", "tp1.y is negative");
            return;
        }
        
        //code 25aug starts here
        {//using same mutex for the invariant.
            std::lock_guard<std::mutex> lock(sharedMutex);
            //cout << "images and attributes being pushed into queues " << endl;
            camera_imageQueue.push(temp_warped_img);
            camera_maskQueue.push(mask_warped_gray_img);
            camera_imageTopLeftQueue.push(tp1);
        }
        //code 25aug ends
        
        /*uncomment this to get prev working code
        cv::Size s = temp_warped_img.size();
        int rows = s.height;
        int cols = s.width;  
        //cout << "point coordinates are : " << tp1 << endl;
        //cout << endl << "warping completed successfully." << endl;
        //<check - the correct image is not populating for cam1_distorted when i put the show window statements here.>
        //cout <<  "Size of warped Mat is : " <<  rows <<  ", " <<  cols <<  endl;
        //cout <<  "size variable of warped Mat is given as " <<  s <<  endl;        
        //cout << "point coordinates after calculating endpoints are : " << tp1.x + rows << "," << tp1.y + cols << endl;
        //Mat temp = finalCameraImage(cv::Rect(1,1,rows,cols));  //creates a pointer to the ROI of finalCameraImage
        //temp_warped_img.copyTo(temp); //use copyTo, do not use assignment operator as the pointer will get changed.
        
                
        vector<Point> corners(2); //1 for current camera image and 2 for previous final image
        vector<Size> sizes(2); //1 for current camera image and 2 for previous final image
        

        //working code with mask below
        //cout << camera_name << " : " << "Point is : " << tp1.x << "," << tp1.y << endl;
        int w = 0;
        cv::Size s2 = s;
        int orig_width = s.width; //copying for later use
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
                tp1.x = tp1.x + image_x_cols; //get top left of image 1. tp1.y remains same as before.
                tp2.x = 0;                    //tl of image 2 starts from 0.
                tp2.y = tp1.y;                //tp2.y remains same as tp1.y.
                //For range, left end is inclusive, rt end is exclusive, and rows begin from 0, not 1.
                //so here lets say tp1.x = -15 and original width = 100. so, img1 width = 15. img2 width = 100-15 = 85.
                //split is from (1000+(-15)) to 999 and from 0 to 84, which is , (original width -img1 width - 1).
                //so Range(985, 1000) and Range(0, 85)
                //cout << "printing roi img2 : " << endl;
                temp_warped_img(Range::all(), Range(w, orig_width)).copyTo(img2); //deep copy - do this first
                s2 = img2.size();
                temp_warped_img = temp_warped_img(Range::all(), Range(0,w)); //overwriting temp_warped_img - do this second
                s = temp_warped_img.size();
                
                //cout << "img1 size : " << s << endl;
                
                

            }
            else{//only 1 mask required
               tp1.x = tp1.x + image_x_cols; //update top left of image 1.     
            }
        }
        //In opencv coordinate s/m, Y = 0 at top left and increases downwards.
//        corners[1] = Point(0,499);; //corner of image
        //cout << "y value corner1 : " << corners[1].y << endl;
        corners[0] = Point(0,0); //corner of prev final image
        sizes[0] = finalCameraImage.size(); //size of prev final image
        corners[1] = tp1;
        //cout << "y value corner0 : " << corners[0].y << endl;
        sizes[1] = s; //size of image
        if (second_mask_reqd){
         corners.push_back(tp2);
         sizes.push_back(s2);
        }
        
        //no need for delete as I am not using 'new'.
        {//using same mutex for the invariant.
            std::lock_guard<std::mutex> lock(sharedMutex);
            camera_imageQueue.push(temp_warped_img);
            camera_maskQueue.push(mask_warped_gray_img);
            camera_imageTopLeftQueue.push(tp1);
            
            s = temp_warped_img.size();
            rows = s.height;
            cols = s.width;  
            //cout << camera_name << endl;
            //cout << "image size after warp before blend is : " << rows << "," << cols << endl;
            //cout << "Point is : " << tp1.x << "," << tp1.y << endl;
            
            
            // new code starts here
            blend_type = cv::detail::Blender::MULTI_BAND;
            //blend_type = cv::detail::Blender::MULTI_BAND;
            blender = cv::detail::Blender::createDefault(blend_type, false); //false given for CUDA processing
            Size dst_sz = cv::detail::resultRoi(corners, sizes).size();
            cout << "camera_name : " << camera_name << " : dst_sz : " << dst_sz << endl;
            Rect resultROI = cv::detail::resultRoi(corners, sizes);
            Mat ROI = finalCameraImage(resultROI);
            //imshow("resultROI", ROI);
            //change this to global variable
            float blend_strength = 1;
            float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
            //reference : https://docs.opencv.org/3.4/d9/dd8/samples_2cpp_2stitching_detailed_8cpp-example.html#a53
            if (blend_type == cv::detail::Blender::FEATHER){
                cv::detail::FeatherBlender* feather_blender = dynamic_cast<cv::detail::FeatherBlender*>(blender.get());
                //feather_blender->setSharpness(1.f/blend_width);
                feather_blender->setSharpness(1);
                //cout << "Feather blender, sharpness: " << feather_blender->sharpness();                
            }
            else if (blend_type == cv::detail::Blender::MULTI_BAND){
                cv::detail::MultiBandBlender* multiband_blender = dynamic_cast<cv::detail::MultiBandBlender*>(blender.get());
                //multiband_blender-> setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
                multiband_blender->setNumBands(1);
                //cout << camera_name << endl;
                //cout << "Multi-band blender, number of bands: " << multiband_blender->numBands();
            }
            else {//this should never be invoked
             ROS_ERROR("Error : %s", "Blend type is invalid.");
             return;
            }
            
            blender->prepare(corners, sizes);
            //finalCameraImage_mask - spans entire image
            temp_warped_img.convertTo(temp_warped_img_s, CV_16SC3);
            finalCameraImage.convertTo(finalCameraImage_s, CV_16SC3);
            blender->feed(finalCameraImage_s, finalCameraImage_mask, corners[0]);
            if (second_mask_reqd){
                img2.convertTo(img2_s, CV_16SC3);
                //blender->feed(temp_warped_img_s, mask_warped_gray_img, corners[1]);
                //blender->feed(img2_s, mask_warped_gray_img, corners[2]);
                
                //blender->feed(temp_warped_img_s, Mat(temp_warped_img_s.size(), CV_8UC1, Scalar::all(255)), corners[1]);
                //blender->feed(img2_s, Mat(img2.size(), CV_8UC1, Scalar::all(255)), corners[2]);
                blender->feed(temp_warped_img_s, mask_warped_gray_img(Range::all(), Range(0,w)), corners[1]);
                blender->feed(img2_s, mask_warped_gray_img(Range::all(), Range(w,orig_width)), corners[2]);
            }
            else{
                blender->feed(temp_warped_img_s, mask_warped_gray_img, corners[1]);
            }
            Mat result_mask;
            blender->blend(finalCameraImage_s, result_mask);
            finalCameraImage_s.convertTo(finalCameraImage, CV_8UC3); //converting back to Unsigned
            //cout << "finalCameraImage : " << finalCameraImage.size().height << "," << finalCameraImage.size().width << endl;
            //new code ends here
            
            
            */ //uncomment ends here but there is another section to uncomment
            /*
            //trying with alpha matte - fails at subtract and multiply.            
            //Issue is that the matrices have to be same size, but the mat object for the new camera image is much lower than the final image. Need to rethink.
            Mat finalCameraImage_float, temp_warped_img_float, alpha;
            Mat img2_float, alpha2;
            finalCameraImage.convertTo(finalCameraImage_float, CV_32FC3);
            temp_warped_img.convertTo(temp_warped_img_float, CV_32FC3);
            if (second_mask_reqd){
                img2.convertTo(img2_float, CV_32FC3);
                //alpha matte 1
                mask_warped_img(Range::all(), Range(0,w)).convertTo(alpha, CV_32FC3, 1.0/255); 
                //alpha matte 2
                mask_warped_img(Range::all(), Range(w,orig_width)).convertTo(alpha2, CV_32FC3, 1.0/255); 
            }
            else{
                //only 1 alpha matte
                mask_warped_img.convertTo(alpha, CV_32FC3, 1.0/255); 
            }
            Mat ouImage = Mat::zeros(finalCameraImage_float.size(), finalCameraImage_float.type());
            Mat beta = Mat::ones(finalCameraImage_float.size(), finalCameraImage_float.type());
            if (second_mask_reqd){
                cout << "reached : " << camera_name << endl;
                // Multiply the foregrounds with the alpha mattes
                multiply(alpha, temp_warped_img_float, temp_warped_img_float);
                multiply(alpha2, img2, img2);
                // Multiply the background with ( 1 - alpha - alpha2)
                subtract(beta, alpha, beta);
                subtract(beta, alpha2, beta);
                beta.convertTo(beta, CV_32FC3);            
                multiply(beta, finalCameraImage_float, finalCameraImage_float);
                // Add the masked foreground and background.
                add(temp_warped_img_float, finalCameraImage_float, ouImage);
                add(img2_float, ouImage, ouImage);
            }
            else{
                cout << "reached : " << camera_name << endl;
                // Multiply the foreground with the alpha matte
                cout << "alpha.size : " << alpha.size() << endl;
                cout << "temp_warped_img_float.size : " << temp_warped_img_float.size() << endl;
                multiply(alpha, temp_warped_img_float, temp_warped_img_float);
                cout << "temp_warped_img_float.size after multiply : " << temp_warped_img_float.size() << endl;
                // Multiply the background with ( 1 - alpha )
                cout << "beta.size : " << beta.size() << endl;
                cout << "alpha.size : " << alpha.size() << endl;
                subtract(beta, alpha, beta);
                cout << "beta.size after subtract: " << beta.size() << endl;
                beta.convertTo(beta, CV_32FC3);       
                cout << "beta.size after convert: " << beta.size() << endl;
                cout << "finalCameraImage_float.size : " << finalCameraImage_float.size() << endl;
                multiply(Scalar::all(1.0) - alpha, finalCameraImage_float, finalCameraImage_float);
                cout << "finalCameraImage_float.size after multiply : " << finalCameraImage_float.size() << endl;
                // Add the masked foreground and background.
                cout << "temp_warped_img_float.size : " << temp_warped_img_float.size() << endl;
                cout << "finalCameraImage_float.size : " << finalCameraImage_float.size() << endl;
                cout << "ouImage.size : " << ouImage.size() << endl;
                add(temp_warped_img_float, finalCameraImage_float, ouImage);
                cout << "ouImage.size after final add : " << ouImage.size() << endl;
            }
            finalCameraImage = ouImage;
            //alpha matte code ends here
            */
            
            
            /*current working code starts here
            Vec3b* Aitt;
            Vec3b* Bitt;

            //<check>have to add a check if its black cell or not.
            for(int i = 0; i < rows; i++)
            {   
                if ((tp1.y + i >= 0) && (tp1.y + i < image_y_rows)){//case when y coordinate is within the allowed range(0-image_y_rows)
                    Aitt = finalCameraImage.ptr<cv::Vec3b>(tp1.y + i);
                }
                else if (tp1.y + i >= image_y_rows){//case when y coordinate has exceeded the allowed range and is greater
                    Aitt = finalCameraImage.ptr<cv::Vec3b>((-1 * image_y_rows) + tp1.y + i);
                }
                else{//case when y coordinate is less than the allowed range
                    //cout << "tp1.x + i, should be < 0 : " << tp1.x + i << endl;
                    //~5000 rows and 10000 columns available in final image.
                    Aitt = finalCameraImage.ptr<cv::Vec3b>((image_y_rows) + tp1.y + i);
                    //cout << "finalCameraImage.size : " << finalCameraImage.size() << endl;
                }
                Bitt = temp_warped_img.ptr<cv::Vec3b>(i);
   
                //matrix starts from 0 - which is the top left. - we are changing value of each point
                for(int j = 0; j < cols; j++){//case when x coordinate is within the allowed range(0-image_x_cols)
                    if (!((Bitt[j][0]==0) && (Bitt[j][1]==0) && (Bitt[j][2]==0))){//if pixel is not black, copy the values to final image                
                        if ((tp1.x + j >= 0) && (tp1.x + j < image_x_cols)){
                            //cout << "tp1.y + j : " << tp1.y + j << endl;
                            //if (Bitt[j] !=0)  //<check>start from here
                            Aitt[tp1.x + j] = Bitt[j];
                        }
                        else if(tp1.x + j >= image_x_cols){//case when x coordinate has exceeded the allowed range and is greater
                            Aitt[(-1 * image_x_cols) + tp1.x + j] = Bitt[j];
                        }
                        else{//case when x coordinate is less than the allowed range
                            Aitt[(image_x_cols) + tp1.x + j] = Bitt[j];
                        }    
                    }
                }

            }
            current working code ends  here */

            
            
            //Mat ROI_finalImage2 = finalCameraImage(Range(100,110),  Range(100,110));
            //cout << "ROI_finalcameraImage2 is : " <<  endl << ROI_finalImage2 <<  endl; 
            
            /*cv::Size s2 = temp_warped_img.size();
            int rows2 = s2.height;
            int cols2 = s2.width;*/
            //cout <<  "Size of temp_warped_img Mat is : " <<  rows2 <<  ", " <<  cols2 <<  endl;
            //cout <<  "size variable of temp_warped_img Mat is given as " <<  s2 <<  endl;
            //Mat ROI_final_image = finalCameraImage(Range(tp1.x,tp1.y),  Range(tp1.x + 1,tp1.y + 1));
            //cout << "ROI_final_image is : " <<  endl << ROI_final_image <<  endl; 
            /*namedWindow("cam1_inputImage_cameraCallBackdistorted_insidewarper", WINDOW_AUTOSIZE);
            imshow("cam1_distorted_insidewarper", cam_data);
            namedWindow("cam1_undistorted", WINDOW_AUTOSIZE);
            imshow("cam1_undistorted", undistorted_cam_data);
            namedWindow("warpedImage", WINDOW_AUTOSIZE);
            imshow("warpedImage", temp_warped_img);*/
    //        namedWindow("ROI_temp", WINDOW_AUTOSIZE);
    //        imshow("ROI_temp", ROI_temp);
            /*namedWindow("finalImage", 0); //only windows without autosize can be resized
            imshow("finalImage", finalCameraImage);
            resizeWindow("finalImage", 500, 1000);
            waitKey();*/
            
            /* //second section of commenting begins here
            if (!finalCameraImage.empty()){
                //cout << "entered whiletrue loop" << endl;
                msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", finalCameraImage).toImageMsg(); //bgr8 is blue green red with 8UC3, mono8 for single channel
                //msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", result_mask).toImageMsg(); //bgr8 is blue green red with 8UC3, mono8 for single channel
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
            //cout << "---------------------processing done-------------------------------------------------" << this->camera_name << endl;
        }*/
    }
    else{
     cout << "undistort has empty data." << endl;   
    }
    
}

//Note : There is a CameraSubscriber in ROS, which I did not initially know about, which can be used for this, where we do not need 2 callbacks, instead we can use a single callback for it.

void cameraSetup::info_cameraCallBack(const sensor_msgs::CameraInfo::ConstPtr& inpMsg){
    //cout << "info_cameraCallBack reached." << endl;
    camera_height = inpMsg->height;
    camera_width = inpMsg->width;
    //cout <<  camera_name << ", " <<  camera_height << ", " <<  camera_width <<  endl;   
}

void cameraSetup::inputImage_cameraCallBack(const sensor_msgs::Image::ConstPtr& inpMsg){
    //cout << "inputImage_cameraCallBack reached." << endl;
    //cout <<  camera_name << endl;
    // First convert ROS image message to Open CV Mat using CVbridge. - if we need to modify the data,  we need to copy this. I am only referencing the original data here to prevent usage of too much memory.
    cv_bridge::CvImageConstPtr cv_cam_ptr; 
    try{
        cv_cam_ptr = cv_bridge::toCvShare(inpMsg,sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cam_data = cv_cam_ptr->image;
    ros::Time time_of_capture =  inpMsg->header.stamp;
    //cout << "time of capture is : " << time_of_capture << endl;
    
    if ((checkCameraResolution()) && (checkCameraData())) {
        //cout << "inputImage_cameraCallBack reached. - 1" << endl;
        bool success = calculateRotationMatrix(time_of_capture);
        /*namedWindow("cam1_distorted", WINDOW_AUTOSIZE);
        imshow("cam1_distorted", cam_data);*/
        //last parameter (optional), newCameraMatrix – Camera matrix of the distorted image. 
        //By default, it is the same as cameraMatrix but you may additionally scale and shift the result by using a different matrix.
        //used if we want a subset of resulting image.
        //<change this>
        if (checkCameraParameters() && success){//checks if intrinsics, distortion coefficients and rotation matrix has been set and the parameters havent been set before.
            setRotationMatrix(rotation_matrix);
            // Performing undistort on image.
            cv::Size distort_size = cam_data.size();
            int distort_type = cam_data.type();
            //cout << "distorted image height : " << distort_size.height << "," << " distorted image width : " << distort_size.width << endl;            
            new_optimal_camera_matrix = getOptimalNewCameraMatrix(camera_intrinsics, camera_dist_coefficients, distort_size, 1);

            undistort(cam_data, undistorted_cam_data, camera_intrinsics, camera_dist_coefficients, new_optimal_camera_matrix);
            //cout << "undistorted image height : " << undistorted_cam_data.size().height << "," << " undistorted image width : " << undistorted_cam_data.size().width << endl;

            //I am using a caching scheme here.
            //For the first time, if the data doesnt come in properly,
            //then we have 
            if ((dummy_white_Mat.size()!=cam_data.size()) || (dummy_white_Mat.type()!=cam_data.type())){
                /*creating a new white Mat object and then undistorting it.*/
                try{
                    dummy_white_Mat = Mat(distort_size, distort_type, Scalar(255, 255, 255));
                    //undistort the white image.
                    undistort(dummy_white_Mat, undistort_dummy_white_Mat, camera_intrinsics, camera_dist_coefficients, new_optimal_camera_matrix);   
                    /*namedWindow("undistort_dummy_white_Mat", WINDOW_FREERATIO);
                    imshow("undistort_dummy_white_Mat", undistort_dummy_white_Mat);
                    waitKey();*/
                }
                catch(Exception &e){
                    ROS_WARN("%s",e.what());
                }
            }
            /*cout << "undistort_cam_data type" << type2str(undistorted_cam_data.type()) << endl;
            cout << undistorted_cam_data.type() << endl;
            cout << "cam_data type" << type2str(cam_data.type()) << endl;
            cout << cam_data.type() << endl;*/
            if (!new_optimal_camera_matrix.empty())
                process_3D_Map();
        } 
    }
}

void cameraSetup::setIntrinsics(const Mat intrinsics){
    camera_intrinsics = intrinsics;
}   

void cameraSetup::setDistortionCoefficients(const Mat d_coef){
    camera_dist_coefficients = d_coef;
}

void cameraSetup::setRotationMatrix(const Mat rotMat){
    rotMat.copyTo(rotation_matrix);    //deep copy
}

void cameraSetup::setCameraParameters(const Mat intrinsics, const Mat distortion_coef, const Mat rotationMat){
    setIntrinsics(intrinsics);
    setDistortionCoefficients(distortion_coef);
    setRotationMatrix(rotationMat);
}


/*cameraSetup::cameraSetup()
:camera_name("unassigned name"),camera_height(-1), camera_width(-1){    
    //cout << "cameraSetup::cameraSetup() entered" << endl;
}   

cameraSetup::cameraSetup(string name)
:camera_name(name),camera_height(-1), camera_width(-1){   
    //cout << "cameraSetup::cameraSetup(string name) entered" << endl;
}

cameraSetup::cameraSetup(string name, ros::NodeHandle& handle)
:camera_name(name),camera_height(-1), camera_width(-1){  
    //cout << "cameraSetup::cameraSetup(string name, ros::NodeHandle& handle) entered" << endl;
    //Creates subscriber for Camera Info
    camerainfo_Subscriber = handle.subscribe("/" + camera_name + "/camera_info",10, &cameraSetup::info_cameraCallBack, this);
    //Creates subscriber for Camera Image
    inputImage_Subscriber = handle.subscribe("/" + camera_name + "/image_raw",10, &cameraSetup::inputImage_cameraCallBack, this);
    //imgTransp_subscriber(handle);
    //inputImage_Subscriber = imgTransp_subscriber.subscribe("/" + camera_name + "/image_raw",10, &cameraSetup::inputImage_cameraCallBack, this); //<check if compile issue here>
    //cout << "reached end of 2nd const" << endl;
}
*/

void cameraSetup::setCameraTransformDelay(double_t delay){
    camera_transform_delay = delay;
}


void cameraSetup::setCameraBlendAreaInPixels(int16_t pixels){
    pixels_to_blend = pixels;
}

void cameraSetup::popCameraQueue(){
    int qSize;
    {//popping under the same mutex and lock.
        std::lock_guard<std::mutex> lock(sharedMutex);
        qSize = camera_imageQueue.size();
        
    }
    int counter = 0;
    //bool 
    do{
        //pop only deletes the element in C++, use front to access it first
        if (qSize!=0){
            std::lock_guard<std::mutex> lock(sharedMutex);
            current_image = camera_imageQueue.front();
            //cout << "current_image.type() : "<< type2str(current_image.type()) << endl;
            current_image_size = current_image.size();
            camera_imageQueue.pop();
                
            //since they are locked by a shared Mutex, when the first Q has an object, the other Queues will also have it.
            current_mask = camera_maskQueue.front();
            //cout << "current_mask.type() : "<< type2str(current_mask.type()) << endl;
            camera_maskQueue.pop();
            current_tl = camera_imageTopLeftQueue.front();
            camera_imageTopLeftQueue.pop();
            prev_tl = current_tl; //to be used for next iteration
            prev_image_size = current_image_size; //to be used for next iteration
            break;
        }
        else{
            counter++; //local variable
/*          
            if (counter==100){//sends a black image if no frame received for 100 times of loop
                std::lock_guard<std::mutex> lock(sharedMutex);
                current_image_size = prev_image_size;
                current_tl = prev_tl;
                int rows = current_image_size.height;
                int cols = current_image_size.width;                 
                current_image = Mat(rows, cols, CV_8UC3, Scalar(0,0,0)); //8uc3
                current_mask = Mat(rows, cols, CV_8UC1, Scalar(1)); //8uc1
                break;
            }
*/
            std::lock_guard<std::mutex> lock(sharedMutex);
            qSize = camera_imageQueue.size();
        }
    }
    while(true);

}


//Note : This will cause an issue if we parallelise the code. We will have to synchronize it around the finalImage object because all camera objects will be updating the final image, if so.
cameraSetup::cameraSetup(string name, ros::NodeHandle& handle, Mat& finalImage, sensor_msgs::ImagePtr& finalImageMsg, image_transport::Publisher& finalImagepub, int finalimage_rows, int finalimage_cols)
:camera_name(name),camera_height(1440), camera_width(2560){
    //entire constructor is locked by the mutex. Important, else object might be half constructed which can lead to segmentation issues.
    std::lock_guard<std::mutex> lock(sharedMutex);   
    rotation_matrix = cv::Mat(3, 3, CV_32F);
    //cout << "cameraSetup::cameraSetup(string name, ros::NodeHandle& handle, Mat& finalImage, int finalimage_rows, int finalimage_cols) entered" << endl;
    //Creates subscriber for Camera Info
    camerainfo_Subscriber = handle.subscribe("/" + camera_name + "/camera_info",10, &cameraSetup::info_cameraCallBack, this);
    //Creates subscriber for Camera Image
    inputImage_Subscriber = handle.subscribe("/" + camera_name + "/image_raw",10, &cameraSetup::inputImage_cameraCallBack, this);
    
    
    //3 shared variables which form the invariant
        finalCameraImage = finalImage;
        msg = finalImageMsg;
        finalimage_publisher = finalImagepub;
    //<check> - need to change the below to receive from a rostopic later or from a server. Will see.
    /*if (camera_name == "pano_1"){
        setCameraParameters((Mat_<float>(3,3) << 1884.288597944681, 0, 1281.298355259871, 0, 1584.885477343124, 636.5814917534733, 0, 0, 1), (Mat_<float>(1,5) << -0.4173570405287141, 0.1493134654766953, 0.008037288266852087, -0.0007342658995463636, 0), (Mat_<float>(3,3) << 1,0,0,0,1,0,0,0,1));
    }
    else{//<check> for test: only change is the rotation matrix(third parameter)
        setCameraParameters((Mat_<float>(3,3) << 1884.288597944681, 0, 1281.298355259871, 0, 1584.885477343124, 636.5814917534733, 0, 0, 1), (Mat_<float>(1,5) << -0.4173570405287141, 0.1493134654766953, 0.008037288266852087, -0.0007342658995463636, 0), (Mat_<float>(3,3) << 0.3090170,  0.0000000,  -0.9510565, 0,1,0, 0.9510565,  0.0000000,  0.3090170));
    }*/
    //<check> have to change this
    /* this is for old 5 camera set
    if (camera_name == "pano_1"){
        setIntrinsics((Mat_<float>(3,3) << 1884.288597944681, 0, 1281.298355259871, 0, 1584.885477343124, 636.5814917534733, 0, 0, 1));
        setDistortionCoefficients((Mat_<float>(1,5) << -0.4173570405287141, 0.1493134654766953, 0.008037288266852087, -0.0007342658995463636, 0));
    }
    else if (camera_name == "pano_2"){
        setIntrinsics((Mat_<float>(3,3) << 1878.837910095895, 0, 1259.945629059152, 0, 1591.233867405212, 663.1776502988188, 0, 0, 1));
        setDistortionCoefficients((Mat_<float>(1,5) << -0.4047674604376282, 0.1425073623947998, -0.0001881467757579265, 0.001850710303313134, 0));
    }
    else if (camera_name == "pano_3"){
        setIntrinsics((Mat_<float>(3,3) << 1901.32933265009, 0, 1231.156091133535, 0, 1594.511382372096, 773.091002986523, 0, 0, 1));
        setDistortionCoefficients((Mat_<float>(1,5) << -0.4224369016068015, 0.1660912854233553, -0.007215744449927984, 8.593959770815199e-05, 0));
    }
    else if (camera_name == "pano_4"){
        setIntrinsics((Mat_<float>(3,3) << 1870.019860293959, 0, 1313.799025812918, 0, 1582.156273699344, 682.300588101025, 0, 0, 1));
        setDistortionCoefficients((Mat_<float>(1,5) << -0.3935032570679224, 0.1242180693301808, 0.004878604571892903, -0.00313195579436501, 0));
    }
    else{
        setIntrinsics((Mat_<float>(3,3) << 1888.512798190576, 0, 1212.026930077162, 0, 1601.169599805217, 732.2253364795896, 0, 0, 1));
        setDistortionCoefficients((Mat_<float>(1,5) << -0.3836906545283715, 0.1237841545472208, -0.001575181292988737, 0.01157441927076925, 0));
    }
    */
    //for new camera set.
    if (camera_name == "pano_1"){
        setIntrinsics((Mat_<float>(3,3) << 1924.05428,0,1326.89046,0, 1922.47413, 660.37084, 0,0,1));
        setDistortionCoefficients((Mat_<float>(1,5) << -0.403924, 0.125745, 0.001224, -0.001637, 0));
    }
    else if (camera_name == "pano_2"){
        setIntrinsics((Mat_<float>(3,3) << 1827.37403, 0, 1295.73966, 0, 1544.94464, 677.26849, 0, 0, 1));
        setDistortionCoefficients((Mat_<float>(1,5) << -0.402708, 0.129145, 0.001809, -0.001750, 0));
    }
    else if (camera_name == "pano_3"){
        setIntrinsics((Mat_<float>(3,3) << 1903.74564,0,1290.99875, 0,  1902.30552, 603.91738, 0, 0, 1));
        setDistortionCoefficients((Mat_<float>(1,5) << -0.389676, 0.106579, 0.004054, -0.002224, 0));
    }
    else if (camera_name == "pano_4"){
        setIntrinsics((Mat_<float>(3,3) << 1894.62387,0,1254.10863, 0,  1890.51009,681.91008, 0, 0, 1));
        setDistortionCoefficients((Mat_<float>(1,5) << -0.386955, 0.112251, -0.002492, -0.000816, 0));
    }
    else if (camera_name == "pano_5"){
        setIntrinsics((Mat_<float>(3,3) << 1858.65666,0,1293.11574, 0,  1857.20712,735.85572, 0, 0, 1));
        setDistortionCoefficients((Mat_<float>(1,5) << -0.378799, 0.104279, -0.000161, -0.001951, 0));
    }
    else{
        setIntrinsics((Mat_<float>(3,3) << 1870.61482,0,  1283.57924, 0,1870.00689,703.55448, 0, 0, 1));
        setDistortionCoefficients((Mat_<float>(1,5) << -0.385998, 0.113376, -0.002907, -0.000099, 0));
    }
    
    
    tfBuffer = new tf2_ros::Buffer(ros::Duration(10), false);
    tfListener = new tf2_ros::TransformListener(*tfBuffer);  //for transform.
    image_y_rows = finalimage_rows;
    image_x_cols = finalimage_cols; 
}


cameraSetup::~cameraSetup(){
    //cout << "destructor"<< endl;
    cam_data.release();
    undistorted_cam_data.release();
    camera_intrinsics.release();
    camera_dist_coefficients.release();
    rotation_matrix.release();
    delete tfListener;
    delete tfBuffer;

}



