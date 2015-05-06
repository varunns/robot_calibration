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

    //detecting Hough Circle and centroid 
    houghFunction(cv_ptr->image, cloud_in, cloud->header);

  }

  void houghFunction(cv::Mat image, 
                     pcl::PointCloud<pcl::PointXYZRGB> cloud_in,
                     std_msgs::Header header)
  { 
    std::stringstream ss(std::stringstream::in | std::stringstream::out);
    ss<<"/tmp/image_"<<i_<<".jpg";
    imwrite(ss.str(), image);  
    i_++; 

    cv::Mat blur_image, gray_image;
    std::vector<cv::Vec3f> circles;
    circles.clear();
    cv::cvtColor( image, gray_image, CV_BGR2GRAY );
    cv::GaussianBlur(gray_image, blur_image, cv::Size(9,9), 2, 2);
    cv::HoughCircles(blur_image, circles, CV_HOUGH_GRADIENT, 1, 1, 6, 8, 0, 0);
 // /   ROS_INFO("number of circles :  %d", circles.size());
    for(size_t i = 0; i < circles.size(); i++)
    {
      cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // circle center
      cv::circle(image, center, 1, cv::Scalar(0,255,0), -1, 8, 0 );
      // circle outline
      cv::circle(image, center, radius, cv::Scalar(0,0,255), 1, 8, 0 ); 
    }

    if(circles.size() > 0)
    {
      pcl::PointXYZRGB pt = cloud_in(cvRound(circles[0][0]),cvRound(circles[0][1]));
      geometry_msgs::PointStamped pt_ros;
      pt_ros.point = pcl2RosPt(pt); 
      pt_ros.header = header;
      geometry_msgs::PointStamped transform_pt;
      tf_.transformPoint(target_frame_, pt_ros, transform_pt);
      ROS_INFO("%f        :     %f      :      %f", transform_pt.point.x, transform_pt.point.y, transform_pt.point.z);
    }
    
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


  }

  void differenceImage(const cv::Mat image_in)
  {
    if(flag_image_)
    {
      flag_image_ = false;
    }
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
