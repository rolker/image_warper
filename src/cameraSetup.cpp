// Author : Deepak Narayan
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright Jan 2020, All rights reserved.

#include "cameraSetup.h"

using namespace std;
using namespace cv;



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
    geometry_msgs::TransformStamped transformStamped;
    tf2::Quaternion q_value;
    tf2::Quaternion scalar(0.7071068, 0, 0, 0.7071068); //- about x axis
    //tf2::Quaternion scalar(0, 0.7071068, 0, 0.7071068); //- about y axis
    //<check> quartenion is getting rotation only, not the translation
    //<check> if transformStamped is initially not obtained, what should be done? think we have to introduce in callback an if statement.
    try{
        //parameters are target frame, source frame, time at which we want to transform(Time(0) is latest transform.), duration before timeout.
        //transformStamped - will get us the translation and rotation. I am using the rotation for warp. Not using translation currently.
        //transformStamped.transform.translation and transformStamped.transform.rotation
        transformStamped = tfBuffer->lookupTransform("north_up_base_link" , camera_name + "_optical", capture_time+ros::Duration(camera_transform_delay));
        //convert msg into a quartenion - do we need to consider only the rotation part here by using transformStamped.transform.rotation? - yes.
        tf2::convert(transformStamped.transform.rotation , q_value);
        q_value = scalar * q_value;
        q_value.normalize();
        tf2::Matrix3x3 matrix;
        tf2::Vector3 dvec;
        matrix.setRotation(q_value);
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
    //cout << "procenvpss_3D_Map reached" << endl;
    Mat temp_warped_img;
    cv::detail::SphericalWarper sphWarp1(int(image_x_cols/(2*3.14)));
    //cv::detail::CylindricalWarper cylWarp1(int(image_x_cols/(2*3.14)));
    if (!undistorted_cam_data.empty()){
        //cout << "entered into processing part of process_3D_Map" << endl;
        //cout << "undistorted data ..... ===> " << undistort_inp(Range(1000, 1005),  Range(1000, 1002)) << endl;
        //cout << endl << "warping is being done" << endl;
        //Point tp1 is the top left corner of image
        Point tp1 = sphWarp1.warp(undistorted_cam_data, new_optimal_camera_matrix, rotation_matrix, INTER_LINEAR, 0, temp_warped_img);
        //Point tp1 = sphWarp1.warp(undistorted_cam_data, camera_intrinsics, rotation_matrix, INTER_LINEAR, 0, temp_warped_img);
        //whichever camdata is coming in callback, we need to pass that. Also the rotation matrix corresponding to that camera.
        //Point tp1 = cylWarp1.warp(undistorted_cam_data, camera_intrinsics, rotation_matrix, INTER_LINEAR, 0, temp_warped_img);
        cv::Size s = temp_warped_img.size();
        int rows = s.height;
        int cols = s.width;        
        //cout << "point coordinates are : " << tp1 << endl;
        //cout << endl << "warping completed successfully." << endl;
        //<check - the correct image is not populating for cam1_distorted when i put the show window statements here.>
        //cout <<  "Size of warped Mat is : " <<  rows <<  ", " <<  cols <<  endl;
        //cout <<  "size variable of warped Mat is given as " <<  s <<  endl;        
        //cout << "point coordinates after calculating endpoints are : " << tp1.x + rows << "," << tp1.y + cols << endl;
        /*Mat temp = finalCameraImage(cv::Rect(1,1,rows,cols));  //creates a pointer to the ROI of finalCameraImage
        temp_warped_img.copyTo(temp); //use copyTo, do not use assignment operator as the pointer will get changed.
        */
        
        Vec3b* Aitt;
        Vec3b* Bitt;
        //no need for delete as I am not using 'new'.
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
            //find which coordinate exceeds the allowed range.
            //lowerlimit : -tp1.x
            //upperlimit : image_x_cols - tp1.x if positive else tp1.x - image_x_cols
            
            if (tp1.x >= 0){
                int val = image_x_cols - tp1.x;
                if (val >=0){//image start is where we want it to be.
                    if (val>=cols){//we can copy entire width of warped image.
                        
                    }
                    else{//we can copy from tp1.x to tp1.x+val and then the rest, i need to start from 0/start point.
                        
                    }
                }
                else{//val is negative, means image is too far ahead, so need to bring it back to start point.
                    //do from here
                }
                
            }
            else{//tp1.x is negative
                
            }
            
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
        
        /*for(int i = 0; i < rows; i++)
        {   
            if (tp1.x + i >= 0){
                //no need for delete as I am not using 'new'.
                Aitt = finalCameraImage.ptr<cv::Vec3b>(tp1.x + i);
            }
            else{
                //cout << "tp1.x + i, should be < 0 : " << tp1.x + i << endl;
                //~5000 rows and 10000 columns available in final image.
                Aitt = finalCameraImage.ptr<cv::Vec3b>((image_x_cols) + tp1.x + i);
                //cout << "finalCameraImage.size : " << finalCameraImage.size() << endl;
            }
            Bitt = temp_warped_img.ptr<cv::Vec3b>(i);
            //matrix starts from 0 - which is the top left.
            for(int j = 0; j < cols; j++){
                if (tp1.y + j >= 0){
                    //cout << "tp1.y + j : " << tp1.y + j << endl;
                    Aitt[tp1.y + j] = Bitt[j];
                }
                else{
                    Aitt[(image_y_rows) + tp1.y + j] = Bitt[j]; //because we have an width:height ratio of 2:1 
                }    
            }                
        }*/
        
        
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

    }
    else{
     cout << "undistort has empty data." << endl;   
    }
    
}


void cameraSetup::info_cameraCallBack(const sensor_msgs::CameraInfo::ConstPtr& inpMsg){
    cout << "info_cameraCallBack reached." << endl;
    camera_height = inpMsg->height;
    camera_width = inpMsg->width;
    cout <<  camera_name << ", " <<  camera_height << ", " <<  camera_width <<  endl;   
}

void cameraSetup::inputImage_cameraCallBack(const sensor_msgs::Image::ConstPtr& inpMsg){
    cout << "inputImage_cameraCallBack reached." << endl;
    cout <<  camera_name << endl;
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
    cout << "time of capture is : " << time_of_capture << endl;
    
    if ((checkCameraResolution()) && (checkCameraData())) {
        //cout << "inputImage_cameraCallBack reached. - 1" << endl;
        bool success = calculateRotationMatrix(time_of_capture);
        //cout << "success of calculateRotationMatrix : " << success << endl;
        //cout << "success of checkCameraParameters : " << checkCameraParameters() << endl;
        /*namedWindow("cam1_distorted", WINDOW_AUTOSIZE);
        imshow("cam1_distorted", cam_data);*/
        //last parameter (optional), newCameraMatrix – Camera matrix of the distorted image. 
        //By default, it is the same as cameraMatrix but you may additionally scale and shift the result by using a different matrix.
        //used if we want a subset of resulting image.
        //<change this>
        if (checkCameraParameters() && success){//checks if intrinsics, distortion coefficients and rotation matrix has been set and the parameters havent been set before.
            //cout << "inputImage_cameraCallBack reached. - 2" << endl;
            //<check> have to change below to setCameraParameters
            //setCameraParameters((Mat_<float>(3,3) << 1884.288597944681, 0, 1281.298355259871, 0, 1584.885477343124, 636.5814917534733, 0, 0, 1), (Mat_<float>(1,5) << -0.4173570405287141, 0.1493134654766953, 0.008037288266852087, -0.0007342658995463636, 0), rotation_matrix);
            setRotationMatrix(rotation_matrix);
            //setRotationMatrix((Mat::eye(3, 3, CV_32F)));
            /*if (camera_name == "pano_1")//y and x-axis 90deg
                setRotationMatrix((Mat_<float>(3,3) << 0.5000000,  0.5000000,  0.7071068, 0.5000000,  0.5000000, -0.7071068, -0.7071068,  0.7071068,  0.0000000));
                //setRotationMatrix((Mat::eye(3, 3, CV_32F)));
            else if (camera_name == "pano_2") //x-axis 90deg
                setRotationMatrix((Mat_<float>(3,3) << 1,0,0,0,0,-1, 0, 1, 0));
            else if (camera_name == "pano_3")//y-axis 90deg
                setRotationMatrix((Mat_<float>(3,3) << 0,0,1,0,1,0,-1,0,0));
            else if (camera_name == "pano_4")//z-axis 90deg
                setRotationMatrix((Mat_<float>(3,3) << 0,-1,0,1,0,0,0,0,1));
            else if (camera_name == "pano_5")//y and z-axis 90deg
                setRotationMatrix((Mat_<float>(3,3) << 0.0000000, -0.7071068,  0.7071068, 0.7071068,  0.5000000,  0.5000000, -0.7071068,  0.5000000,  0.5000000));*/
            // Performing undistort on image.
            cv::Size distort_size = cam_data.size();
            new_optimal_camera_matrix = getOptimalNewCameraMatrix(camera_intrinsics, camera_dist_coefficients, distort_size, 1);
            cout << "new_optimal_camera_matrix : " << new_optimal_camera_matrix;
            cout << "camera_intrinsics : " << camera_intrinsics;
            undistort(cam_data, undistorted_cam_data, camera_intrinsics, camera_dist_coefficients, new_optimal_camera_matrix);
            
            /*newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
            mapx, mapy = cv2.initUndistortRectifyMap(camera_intrinsics, camera_dist_coefficients, cam_data, new_optimal_camera_matrix, dim, 5)
            image = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)

            x, y, w, h = roi
            image = image[y:y + h, x:x + w]*/


            /*cout << "undistort_cam_data type" << type2str(undistorted_cam_data.type()) << endl;
            cout << undistorted_cam_data.type() << endl;
            cout << "cam_data type" << type2str(cam_data.type()) << endl;
            cout << cam_data.type() << endl;*/
            /*namedWindow("cam1_undistorted", WINDOW_AUTOSIZE);
            imshow("cam1_undistorted", undistort_cam1_data);
            waitKey(); //pbly will be better to overwrite the same Mat object  */
            if (!new_optimal_camera_matrix.empty())
                process_3D_Map();
        } 
    }
    //Mat ROI_cam1_data = cam_data(Range(1000, 1005),  Range(1000, 1002));
    //cout << "ROI_cam1_data is : " <<  ROI_cam1_data <<  endl; 
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

//Note : This will cause an issue if we parallelise the code. We will have to synchronize it around the finalImage object because all camera objects will be updating the final image, if so.
cameraSetup::cameraSetup(string name, ros::NodeHandle& handle, Mat& finalImage, int finalimage_rows, int finalimage_cols)
:camera_name(name),camera_height(-1), camera_width(-1){
    rotation_matrix = cv::Mat(3, 3, CV_32F);
    //cout << "cameraSetup::cameraSetup(string name, ros::NodeHandle& handle, Mat& finalImage, int finalimage_rows, int finalimage_cols) entered" << endl;
    //Creates subscriber for Camera Info
    camerainfo_Subscriber = handle.subscribe("/" + camera_name + "/camera_info",10, &cameraSetup::info_cameraCallBack, this);
    //Creates subscriber for Camera Image
    inputImage_Subscriber = handle.subscribe("/" + camera_name + "/image_raw",10, &cameraSetup::inputImage_cameraCallBack, this);
    finalCameraImage = finalImage;
    //<check> - need to change the below to receive from a rostopic later or from a server. Will see.
    /*if (camera_name == "pano_1"){
        setCameraParameters((Mat_<float>(3,3) << 1884.288597944681, 0, 1281.298355259871, 0, 1584.885477343124, 636.5814917534733, 0, 0, 1), (Mat_<float>(1,5) << -0.4173570405287141, 0.1493134654766953, 0.008037288266852087, -0.0007342658995463636, 0), (Mat_<float>(3,3) << 1,0,0,0,1,0,0,0,1));
    }
    else{//<check> for test: only change is the rotation matrix(third parameter)
        setCameraParameters((Mat_<float>(3,3) << 1884.288597944681, 0, 1281.298355259871, 0, 1584.885477343124, 636.5814917534733, 0, 0, 1), (Mat_<float>(1,5) << -0.4173570405287141, 0.1493134654766953, 0.008037288266852087, -0.0007342658995463636, 0), (Mat_<float>(3,3) << 0.3090170,  0.0000000,  -0.9510565, 0,1,0, 0.9510565,  0.0000000,  0.3090170));
    }*/
    //<check> have to change this
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



