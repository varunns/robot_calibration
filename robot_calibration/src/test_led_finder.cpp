/*
 * Copyright 2015 Fetch Robotics Inc.
 * Author: Sriramvarun Nidamarthy
 */
#include <iostream>

#include <ros/ros.h>

#include "message_filters/subscriber.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <robot_calibration/capture/led_finder.h>
#include <robot_calibration/capture/feature_finder.h>

using namespace cv;

class TestLedFinder
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub_;
  tf::TransformListener tf_;
  tf::MessageFilter<sensor_msgs::PointCloud2>* tf_filter_;

  std::string target_frame_;

  bool flag_cloud_;
  bool flag_image_;
  int i_;
  pcl::PointCloud<pcl::PointXYZRGB> prev_cloud_;
  cv::Mat prev_image_;
  int count_;
  int h_count_;
public:
  TestLedFinder(): tf_(), target_frame_("wrist_roll_link")
  {
    point_cloud_sub_.subscribe(nh_,"/head_camera/depth_registered/points", 10);
    tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(point_cloud_sub_, tf_, target_frame_, 10);
    tf_filter_->registerCallback(boost::bind(&TestLedFinder::pcCallback, this, _1));
    //sub_ = nh_.subscribe("/head_camera/depth_registered/points", 1, &TestLedFinder::pcCallback, this);
    i_ = 0;
    flag_cloud_ = true;
    flag_image_ = true;
    count_ = 0;
    h_count_ = 0;
  }

  void pcCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    //converting PoinCloud2 to pcl and opencv image
    pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
    //sensor_msgs::PointCloud2 transformed_cloud;

    //tf_.transformPointCloud(target_frame_, *cloud, transformed_cloud);
    pcl::fromROSMsg(*cloud, cloud_in);
    sensor_msgs::Image ros_image;
    cv_bridge::CvImagePtr cv_ptr;
    
    pcl::toROSMsg(*cloud, ros_image);
    try
    {
      cv_ptr = cv_bridge::toCvCopy(ros_image  , sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //cv
   // std::vector<cv::Mat> channels(3);
   // cv::split(cv_ptr->image, channels);
    std::stringstream ss(std::stringstream::in | std::stringstream::out);
    ss<<"/tmp/color/image_"<<i_<<".jpg";
    imwrite(ss.str(), cv_ptr->image);  
    //imwrite(ss.str(), channels[0]);  
    i_++;

    differenceImage(cv_ptr->image);
    //differenceImage(channels[0]);
    //detecting Hough Circle and centroid 
//    houghFunction(cv_ptr->image, cloud_in, cloud->header);

  }

  void houghFunction(cv::Mat image, cv::Mat color)
  {  

    cv::Mat blur_image, gray_image;
    std::vector<cv::Vec3f> circles;
    circles.clear();
 //   cv::cvtColor( image, gray_image, CV_BGR2GRAY );
   // cv::GaussianBlur(image, blur_image, cv::Size(9,9), 2, 2);
    cv::HoughCircles(image, circles, CV_HOUGH_GRADIENT, 2, 1, 6, 8, 0, 0);
 // /   ROS_INFO("number of circles :  %d", circles.size());

 /*   if(circles.size() < 1)
    {
      return;
    }
*/
    for(size_t i = 0; i < circles.size(); i++)
    {
      cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // circle center
      cv::circle(color, center, 1, cv::Scalar(0,255,0), -1, 8, 0 );
      // circle outline
      cv::circle(color, center, radius, cv::Scalar(0,0,255), 1, 8, 0 ); 
    }

    std::stringstream ss(std::stringstream::in | std::stringstream::out);
    ss<<"/tmp/hough/image_"<<h_count_<<".jpg";
    imwrite(ss.str(), color);  
    h_count_++;
    //imwrite(ss.str(), channels[0]);  

 /*   if(circles.size() > 0)
    {
      pcl::PointXYZRGB pt = cloud_in(cvRound(circles[0][0]),cvRound(circles[0][1]));
      geometry_msgs::PointStamped pt_ros;
      pt_ros.point = pcl2RosPt(pt); 
      pt_ros.header = header;
      geometry_msgs::PointStamped transform_pt;
      tf_.transformPoint(target_frame_, pt_ros, transform_pt);
      ROS_INFO("%f        :     %f      :      %f", transform_pt.point.x, transform_pt.point.y, transform_pt.point.z);
    }*/
    
  }

  bool differenceCloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_in)
  {
    if(flag_cloud_)
    {
      prev_cloud_ = cloud_in;
      flag_cloud_ = false;
    }

    pcl::PointCloud<pcl::PointXYZRGB> diff_cloud = cloud_in;
    if(cloud_in.size() != prev_cloud_.size())
    {
      return false;
    }

    for(size_t i = 0; i < cloud_in.size(); i ++)
    {
      /*diff_cloud.points[i].x = cloud_in.points[i].x;
      diff_cloud.points[i].y = cloud_in.points[i].y;
      diff_cloud.points[i].z = cloud_in.points[i].z;*/
      diff_cloud.points[i].r = abs(cloud_in.points[i].r - prev_cloud_.points[i].r);
      diff_cloud.points[i].g = abs(cloud_in.points[i].g - prev_cloud_.points[i].g);
      diff_cloud.points[i].b = abs(cloud_in.points[i].b - prev_cloud_.points[i].b);
    }
    prev_cloud_ = cloud_in;
    sensor_msgs::PointCloud2 ros_cloud;
    sensor_msgs::Image diff_image;
    pcl::toROSMsg(diff_cloud, ros_cloud);
    pcl::toROSMsg(ros_cloud, diff_image);
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(diff_image  , sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return false;
    }
    //call in hough function

  }

  void differenceImage(const cv::Mat image_in)
  {
    cv::Mat diff_image, image_in_norm, image_in_gray;
    if(flag_image_)
    {
      cv::cvtColor(image_in, image_in_gray, CV_BGR2GRAY);
      cv::normalize(image_in_gray, image_in_norm, 0 , 255, NORM_MINMAX,  -1);  
      prev_image_ = image_in_norm;
      flag_image_ = false;
      count_++;
      return;
    }
    cv::Mat thresh;
    cv::cvtColor(image_in, image_in_gray, CV_BGR2GRAY);
    cv::normalize(image_in_gray, image_in_norm, 0 , 255, NORM_MINMAX,  -1);
    
    cv::Mat diff_notnorm = image_in_norm - prev_image_;
    //cv::Mat diff_image = image_in_norm - prev_image_;
    cv::normalize(diff_notnorm, diff_image, 0 , 255, NORM_MINMAX,  -1);
    threshold( diff_image, thresh, 190, 255,CV_THRESH_BINARY );

    std::stringstream ss(std::stringstream::in | std::stringstream::out);
    ss<<"/tmp/diff/image_"<<(i_-1)<<count_<<".jpg";
    imwrite(ss.str(), thresh);  
    count_++;
   // / houghFunction(diff_image, image_in);
    /*imshow("difference", diff_image);
    cv::waitKey(100);*/
    //call hough function

    prev_image_ = image_in_norm ;
    
  }

  geometry_msgs::Point pcl2RosPt(const pcl::PointXYZRGB pt)
  {
    geometry_msgs::Point point;
    point.x = pt.x;
    point.y = pt.y;
    point.z = pt.z;
    return point;
  }

};

int main(int argc,char** argv)
{
  ros::init(argc, argv, "test_led_finder");
  TestLedFinder obj;
  ros::spin();
  return 0;
}
