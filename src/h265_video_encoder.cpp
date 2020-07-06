// Author : Deepak Narayan
//Reference : https://github.com/aws-robotics/kinesisvideo-encoder-ros1/tree/master/h264_video_encoder/src
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright May 2020, All rights reserved.
//This provides a ROS Node that will encode a stream of images into a H265 video stream.


int main(int argc, char ** argv)
{
    
  ros::init(argc, argv, "h265_video_encoder_node");
  ros::NodeHandle node_handle_video_encoder;  
  ros::Publisher publisher_video_encoder;
  image_transport::Subscriber image_sub;
  ros::Subscriber metadata_sub;
  std::unique_ptr<H264Encoder> encoder;
  uint64_t frame_num = 0;
  kinesis_video_msgs::KinesisImageMetadata metadata;
  Aws::Client::Ros1NodeParameterReader param_reader;

  InitializeCommunication(nh, metadata_sub, image_sub, pub,
                          encoder, frame_num, metadata, param_reader);
  
  //
  // run the node
  //
  ros::spin();
  
}
