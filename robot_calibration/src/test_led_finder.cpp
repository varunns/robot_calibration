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

  bool flag_ = true;
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
  }

  void pcCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    //converting PoinCloud2 to pcl and opencv image
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    //sensor_msgs::PointCloud2 transformed_cloud;

    //tf_.transformPointCloud(target_frame_, *cloud, transformed_cloud);
    pcl::fromROSMsg(*cloud, pcl_cloud);
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

   /* if(flag_)
    {
      prev_cloud_ = pcl_cloud;
      prev_image_ = cv_ptr->image;
      flag_ = false;
    }
*/
    //detecting Hough Circle and centroid
    std::vector<cv::Vec3f> circles;
    circles.clear();
    cv::Mat clone_image = cv_ptr->image.clone();
    cv::Mat blur_image, gray_image;
    cv::cvtColor( cv_ptr->image, gray_image, CV_BGR2GRAY );
    cv::GaussianBlur(gray_image, blur_image, cv::Size(9,9), 2, 2);
    cv::HoughCircles(blur_image, circles, CV_HOUGH_GRADIENT, 1, 1, 6, 8, 0, 0);
 // /   ROS_INFO("number of circles :  %d", circles.size());
    for(size_t i = 0; i < circles.size(); i++)
    {
      cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // circle center
      cv::circle(clone_image, center, 1, cv::Scalar(0,255,0), -1, 8, 0 );
      // circle outline
      cv::circle(clone_image, center, radius, cv::Scalar(0,0,255), 1, 8, 0 ); 
    }

    if(circles.size() > 0)
    {
      pcl::PointXYZRGB pt = pcl_cloud(cvRound(circles[0][0]),cvRound(circles[0][1]));
      geometry_msgs::PointStamped pt_ros;
      pt_ros.point = pcl2RosPt(pt); 
      pt_ros.header = cloud->header;
      geometry_msgs::PointStamped transform_pt;
      tf_.transformPoint(target_frame_, pt_ros, transform_pt);
      ROS_INFO("%f        :     %f      :      %f", transform_pt.point.x, transform_pt.point.y, transform_pt.point.z);
    }
    
    imshow("image", cv_ptr->image);
    cv::waitKey(200);
    std::stringstream ss(std::stringstream::in | std::stringstream::out);
    ss<<"/tmp/image_"<<i_<<".jpg";
    imwrite(ss.str(), clone_image);  
    i_++;
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
