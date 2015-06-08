/*
 * Copyright 2015 Fetch Robotics Inc.
 * Author: Sriramvarun Nidamarthy
 */

#include <iostream>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class TestImage
{
private:
 ros::NodeHandle nh_;
 ros::Subscriber sub_;
 cv::Mat prev_image_;

 bool flag_;

public:
 TestImage()
 {
  sub_ = nh_.subscribe("/head_camera/rgb/image_rect_color", 1, &TestImage::imageCB, this);
  flag_ = true;
 }

 void imageCB(const sensor_msgs::ImagePtr& image)
 {
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
   cv_ptr = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
   ROS_ERROR("failed: %s", e.what());
  }
  
  if(flag_)
  {
   prev_image_ = cv_ptr->image;
   flag_ = false;
  } 
  cv::Mat gray;
  cv::Mat difference = (cv_ptr->image - prev_image_);
  cv::cvtColor(difference, gray, CV_BGR2GRAY);
  cv::Scalar diff = cv::mean(gray);
//  double diff0= (double)(difference[0]); double diff1 =  difference[1]; double diff2 =  difference[2];
 // std::cout<<(pow(diff0,2)+pow(diff1,2)+pow(diff2,2))/(640*480)<<std::endl;
   std::cout<<diff[0]<<std::endl;
  prev_image_ = cv_ptr->image;
 }
};

int main(int argc, char** argv)
{
 ros::init(argc, argv, "test_image");
 TestImage obj;
 ros::spin();
 return 0;
}

