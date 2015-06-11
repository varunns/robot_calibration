
/*
 * Copyright (C) 2015 Fetch Robotics Inc.
 * Copyright (C) 2013-2014 Unbounded Robotics Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Author: Michael Ferguson

#include <math.h>
#include <robot_calibration/capture/led_finder.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo/photo.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <queue>
#include <sstream>
#include <boost/make_shared.hpp>

cv::RNG rng(12345);
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud_;
namespace robot_calibration
{

  double distancePoints(
  const geometry_msgs::Point p1,
  const geometry_msgs::Point p2)
{
  return std::sqrt((p1.x-p2.x) * (p1.x-p2.x) +
                   (p1.y-p2.y) * (p1.y-p2.y) +
                   (p1.z-p2.z) * (p1.z-p2.z));
}

LedFinder::LedFinder(ros::NodeHandle & n) :
  FeatureFinder(n),
  waiting_(false)
{
  ros::NodeHandle nh(n, "led_finder");

  // Setup the action client
  std::string topic_name;
  nh.param<std::string>("action", topic_name, "/gripper_led_action");
  client_.reset(new LedClient(topic_name, true));
  ROS_INFO("Waiting for %s...", topic_name.c_str());
  client_->waitForServer();

  // Setup subscriber
  nh.param<std::string>("topic", topic_name, "/points");
  subscriber_ = n.subscribe(topic_name,
                            1,
                            &LedFinder::cameraCallback,
                            this);

  // Publish where LEDs were seen
  publisher_ = n.advertise<sensor_msgs::PointCloud2>("led_points", 10);

  // Maximum distance LED can be from expected pose
  nh.param<double>("max_error", max_error_, 0.1);
  // Maximum relative difference between two LEDs
  nh.param<double>("max_inconsistency", max_inconsistency_, 0.01);

  // Parameters for detection
  nh.param<double>("threshold", threshold_, 1000.0);
  nh.param<int>("max_iterations", max_iterations_, 50);

  // Should we output debug image/cloud
  nh.param<bool>("debug", output_debug_, true);

  // durtation to keep led on.. for so many secs keep sending the goal

  // Parameters for LEDs themselves
  std::string gripper_led_frame;
  nh.param<std::string>("gripper_led_frame", gripper_led_frame, "wrist_roll_link");
  XmlRpc::XmlRpcValue led_poses;
  nh.getParam("poses", led_poses);
  ROS_ASSERT(led_poses.getType() == XmlRpc::XmlRpcValue::TypeArray);
  // Each LED has a code, and pose in the gripper_led_frame
  for (int i = 0; i < led_poses.size(); ++i)
  {
    codes_.push_back(static_cast<int>(led_poses[i]["code"]));
    codes_.push_back(0);  // assumes "0" is code for "OFF"

    double x, y, z;
    x = static_cast<double>(led_poses[i]["x"]);
    y = static_cast<double>(led_poses[i]["y"]);
    z = static_cast<double>(led_poses[i]["z"]);
    trackers_.push_back(CloudDifferenceTracker(gripper_led_frame, x, y, z));
  }
  //duration to keep the led on.... it now keeps sending goal continuosly for 2s
  led_duration_ = 5;
}

bool LedFinder::debug_flag_ = true;

void LedFinder::cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  
  if (waiting_)
  { 
    cloud_ptr_ = cloud;
    clouds_ptr_.push_back(cloud);
    if(clouds_ptr_.size() > 19)
    {
      waiting_ = false;
    }
  }
}

// Returns true if we got a message, false if we timeout.
bool LedFinder::waitForCloud()
{
  ros::Time ref_time = ros::Time::now();
  waiting_ = true;
  int count = 40;
  while (--count)
  {
    if (!waiting_)
    {
      // success
      return true;
    }
    ros::Duration(0.1).sleep();
  }
  ROS_ERROR("Failed to get cloud");
  return !waiting_;
}

bool LedFinder::find(robot_calibration_msgs::CalibrationData * msg)
{
  uint8_t code_idx = -1;

  std::vector<geometry_msgs::PointStamped> rgbd;
  std::vector<geometry_msgs::PointStamped> world;
  std::vector<geometry_msgs::PointStamped> world_hough_pt;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> prev_clouds;

  robot_calibration_msgs::GripperLedCommandGoal command;
  command.led_code = 0;
  client_->sendGoal(command);
  client_->waitForResult(ros::Duration(10.0));
  ros::Duration(0.5).sleep();

  //------------------------------------------------------------------------------------------------------------------->defining the shared_ptr for struct to hold all the info
  std::vector< CloudDifferenceTracker::TrackContoursPtr > led_respective_contours;
  
  for( size_t i = 0; i < trackers_.size(); i++)
  {
    CloudDifferenceTracker::TrackContoursPtr tmp(new CloudDifferenceTracker::TrackContours());
    led_respective_contours.push_back(tmp);
  }
  std::vector<int> index_check;
  index_check.resize(4);
  // Get initial cloud
  if (!waitForCloud())
  {
    ROS_INFO("waiting for initial cloud");
    return false;
  }

  /* previous clouds*/
  *prev_cloud = *cloud_ptr_;

  prev_clouds.resize(clouds_ptr_.size());
  prev_clouds = clouds_ptr_;
  bool check_first_time = false;
  //pcloud_ clouds_ptr_ = clouds_ptr_[0];
  
  // Initialize difference trackers
  for (size_t i = 0; i < trackers_.size(); ++i)
  {
    trackers_[i].reset(cloud_ptr_->size());
  }
  //ading manually image points for checking the location of led in 3d
  int cycles = 0;
  while (true)
  {
    // Toggle LED to next state
    code_idx = (code_idx + 1) % 8;
    command.led_code = codes_[code_idx];
    ros::Time ref_time = ros::Time::now();

    // time to keep leds on.... keep sending goal for 2s
      client_->sendGoal(command);
      client_->waitForResult(ros::Duration(10.0));
     
    // Get a point cloud
    if (!waitForCloud())
    {
      return false;
    }

    // Commands are organized as On-Off for each led.
    int tracker = code_idx/2;
    // Even indexes are turning on, Odd are turning off
    double weight = (code_idx%2 == 0) ? 1: -1;


    // Has each point converged?
    bool done = true;
    for (size_t t = 0; t < trackers_.size(); ++t)
    {
      done &= trackers_[t].isFound(cloud_ptr_, threshold_);
      //done &= trackers_[t].oisFound(clouds_ptr_, threshold_);
    }
    // We want to break only if the LED is off, so that pixel is not washed out
    if (done && (weight == -1))
    {
      break;
    }

    //getting the led point in the frame of camera------------------------------------------------------------>start
    geometry_msgs::PointStamped led_pt_gripperframe;
    geometry_msgs::PointStamped led_pt_cameraframe;
    led_pt_gripperframe.point = trackers_[tracker].point;
    led_pt_gripperframe.header.frame_id = trackers_[tracker].frame_;
    led_pt_gripperframe.header.stamp = ros::Time::now();

    tf::StampedTransform transform;
    try
    {
      listener_.transformPoint(clouds_ptr_[0]->header.frame_id, 
                               ros::Time(0),
                               led_pt_gripperframe,
                               trackers_[tracker].frame_, 
                               led_pt_cameraframe);
      //std::cout<<led_pt_gripperframe.point<<" "<<led_pt_cameraframe.point<<std::endl;
    }
    catch(const tf::TransformException &ex)
    {
      ROS_ERROR_STREAM("Failed to transform feature to " << clouds_ptr_[0]->header.frame_id);
      return false;
    }
   // led_pt_cameraframe = transform * led_pt_gripperframe.point;
    // call to obtain difference cloud and the max cloud
    diff_image_.release();
    std::vector<std::vector<cv::Point> > contours;
    pcl::PointXYZRGB pt;

    pt.x = led_pt_cameraframe.point.x;
    pt.y = led_pt_cameraframe.point.y;
    pt.z = led_pt_cameraframe.point.z;
    //getting the led point in the frame of camera ------------------------------------------------------------->end
                                                  

    trackers_[tracker].getDifferenceCloud(cloud_ptr_, prev_cloud, diff_image_, weight);
    trackers_[tracker].process(cloud_ptr_, prev_cloud, weight);
    if(cycles > 3)
    {
      trackers_[tracker].oprocess(pt, tracker, clouds_ptr_, prev_clouds, led_respective_contours, check_first_time);
    }

    if (++cycles > max_iterations_)
    {
      break;
    }

    *prev_cloud = *cloud_ptr_;
    prev_clouds = clouds_ptr_;
    clouds_ptr_.resize(0);
    check_first_time = true;
  }
 
  //-------------------------------------------------------------------------------------------Strart Using the populated vector of sharedPtr : led_respective_contours
  std::vector<cv::Rect> bounding_rect_leds;
  std::vector<int> area;
  std::vector<pcl::PointXYZRGB> led_pts;
  led_pts.resize(4);
  
  for( size_t i = 0 ; i < 4; i++)
  {
    cv::Rect bounding_box;
    getCandidateRoi(led_respective_contours[i]);
    std::cout<<std::sqrt(pow((led_respective_contours[i]->pt3d.x - led_respective_contours[i]->estimate_led.point.x),2)+
                         pow((led_respective_contours[i]->pt3d.y - led_respective_contours[i]->estimate_led.point.y),2)+
                         pow((led_respective_contours[i]->pt3d.z - led_respective_contours[i]->estimate_led.point.z),2))<<std::endl;
 //   getCandidateRoi2(led_respective_contours[i], led_pts[i]);
  }


  // Create PointCloud2 to publish
  sensor_msgs::PointCloud2 cloud;
  cloud.width = 0;
  cloud.height = 0;
  cloud.header.stamp = ros::Time::now();
  cloud.header.frame_id = cloud_ptr_->header.frame_id;
  sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
  cloud_mod.setPointCloud2FieldsByString(1, "xyz");
  cloud_mod.resize(4);
  sensor_msgs::PointCloud2Iterator<float> iter_cloud(cloud, "x");

  // Export results
  msg->observations.resize(2);
  msg->observations[0].sensor_name = "camera";  // TODO: parameterize
  msg->observations[1].sensor_name = "arm";     // TODO: parameterize
  for (size_t t = 0; t < trackers_.size(); ++t)
  {
    geometry_msgs::PointStamped rgbd_pt;
    geometry_msgs::PointStamped world_pt;
  
  /*
    if (!trackers_[t].getRefinedCentroid(cloud_ptr_, rgbd_pt))
    {

      ROS_ERROR_STREAM("No centroid for feature " << t);
      return false;
    }*/
    // Check that point is close enough to expected pose
    /*try
    {
      listener_.transformPoint(trackers_[t].frame_, 
                               ros::Time(0), 
                               rgbd_pt,
                               rgbd_pt.header.frame_id, 
                               world_pt);
    }
    catch(const tf::TransformException &ex)
    {
      ROS_ERROR_STREAM("Failed to transform feature to " << trackers_[t].frame_);
      return false;
    }*/
    //substituting point obtianed from getCandidateRoi
    try
    {
      listener_.transformPoint(trackers_[t].frame_, 
                               ros::Time(0), 
                               led_respective_contours[t]->estimate_led,
                               led_respective_contours[t]->estimate_led.header.frame_id, 
                               world_pt);
    }
    catch(const tf::TransformException &ex)
    {
      ROS_ERROR_STREAM("Failed to transform feature to " << trackers_[t].frame_);
      return false;
    }

    double distance = distancePoints(world_pt.point, trackers_[t].point);
    ROS_INFO("the distance of %d  : %f", led_respective_contours[t]->tracker_id, distance);
    if (distance > max_error_)
    {
      ROS_ERROR_STREAM("Feature was too far away from expected pose in " << trackers_[t].frame_ << ": " << distance);
      return false;
    }

    // Check that points are consistent with one another
    for (size_t t2 = 0; t2 < t; ++t2)
    {
      double expected = distancePoints(trackers_[t2].point, trackers_[t].point);
      double actual = distancePoints(msg->observations[0].features[t2].point, rgbd_pt.point);
      if (fabs(expected-actual) > max_inconsistency_)
      {
        ROS_ERROR_STREAM("Features not internally consistent: " << expected << " " << actual);
        return false;
      }
    }

    // Push back observation
    msg->observations[0].features.push_back(rgbd_pt);

    // Visualize
    iter_cloud[0] = rgbd_pt.point.x;
    iter_cloud[1] = rgbd_pt.point.y;
    iter_cloud[2] = rgbd_pt.point.z;
    ++iter_cloud;

    // Push back expected location of point on robot
    world_pt.header.frame_id = trackers_[t].frame_;
    world_pt.point = trackers_[t].point;
    msg->observations[1].features.push_back(world_pt);
  }

  // Final check that all points are valid
  if (msg->observations[0].features.size() != trackers_.size())
  {
    return false;
  }

    // Add debug cloud to message
  if (output_debug_)
  {
    pcl::toROSMsg(*cloud_ptr_, msg->observations[0].cloud);
  }

  // Publish results
  publisher_.publish(cloud);

  return true;
}

void LedFinder::getCandidateRoi2(CloudDifferenceTracker::TrackContoursPtr& tracker_in, pcl::PointXYZRGB& led_pt)
{

}

/*
 *@brief Takes in a single tracker point struct, determines the nearest ROI and calculates
         the centoid of the roi to determine the approx loc
 *@param The TrackerContourPtr Struct
 */
void LedFinder::getCandidateRoi(CloudDifferenceTracker::TrackContoursPtr& tracker_in)
{
  std::cout<<tracker_in->pt3d.x<<" "<<tracker_in->pt3d.y<<" "<<tracker_in->pt3d.z<<std::endl;
  cv::Mat graytmp;
  cv::Mat tmp = (tracker_in->diff_images)[3];
  cv::cvtColor(tmp, graytmp, CV_BGR2GRAY);
  cv::threshold(graytmp, graytmp, 2, 255, CV_THRESH_BINARY);
  cv::Mat dst;

  for(size_t i = 4; i < (tracker_in->diff_images).size(); i++)
  {
    cv::Mat gray;
    cv::cvtColor((tracker_in->diff_images)[i], gray, CV_BGR2GRAY);
    cv::threshold(gray, gray, 2, 255, CV_THRESH_BINARY);
    cv::bitwise_and(gray, graytmp, dst);
    graytmp = dst;
 // localDebugImage((tracker_in->rgb_image)[i], "/tmp/mean/image_");
    localDebugImage((tracker_in->diff_images)[i], "/tmp/mean/diff_");
  }
  std::vector<cv::Point2i> locations;

   //using all the non zero pixels , for getting the locations
  if(cv::countNonZero(dst) > 0)
  {
    cv::findNonZero(dst,locations);
    cv::rectangle((tracker_in->rgb_image)[1], cv::Rect((locations[0]).x, (locations[0]).y, 4, 4), cv::Scalar(0,0,255), 3, 8);
  }

  //debug
  localDebugImage(dst,"/tmp/mean/bitwise_");
 // localDebugImage((tracker_in->rgb_image)[1],"/tmp/mean/bitwise_");

  //Using any diff image values to populate the non zero locations
  cv::Mat diff_gray, color_gray;
  cv::cvtColor( (tracker_in->diff_images)[10], diff_gray, CV_BGR2GRAY);
  cv::cvtColor( (tracker_in->rgb_image)[10], color_gray, CV_BGR2GRAY);
  cv::Mat non_zero = cv::Mat::zeros(diff_gray.rows, diff_gray.cols, CV_8UC1);

  for( size_t i = 0; i < locations.size(); i++)
  {
    non_zero.at<uchar>((locations[i]).y,(locations[i]).x) = diff_gray.at<uchar>((locations[i]).y,(locations[i]).x);
  }

  //finding contours in the non_zero image
  cv::Mat canny_image;
  int canny_thresh = 60;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<std::vector<cv::Point> > contours_candidate;
  cv::Canny(non_zero, canny_image, canny_thresh, canny_thresh*2, 3);
  cv::findContours(canny_image, contours_candidate, hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );
/*

  for(int i = 0; i < contours_candidate.size(); i++)
  {
    for( int j = 0; j < contours_candidate[i].size(); j++)
    {
      cv::Point pt = (contours_candidate[i])[j];
      for( int k = 0; k < tracker_in->pclouds.size(); k++)
      {
        pcl::PointXYZRGB pt3 = (*tracker_in->pclouds[k])(pt.x, pt.y);
        if( isnan(pt3.x) || isnan(pt3.y) || isnan(pt3.z) )
        {
            continue;
        }
        else
        {
          std::cout<<pt3.x<<" "<<pt3.y<<" "<<pt3.z<<std::endl;
          break;
        }
      }
    }
  }*/
  std::cout<<"&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"<<std::endl;
  int min_dist = 100000;
  pcl::PointXYZRGB pt_min;
  int r, m;
  for( int i = 317; i < 337; i++)
  {
    for( int j = 233; j < 253; j++)
    {
      for( int k = 0; k < tracker_in->pclouds.size(); k++)
      {
        pcl::PointXYZRGB pt3;
        pt3 = (*tracker_in->pclouds[k])(i,j);
        if( isnan(pt3.x) || isnan(pt3.y) || isnan(pt3.z) )
        {
          continue;
        }

        double dist = std::sqrt(pow((pt3.x - tracker_in->pt3d.x),2)+pow((pt3.y - tracker_in->pt3d.y),2)+pow((pt3.z - tracker_in->pt3d.z),2));
        if(dist < min_dist)
        {
         min_dist = dist;
         pt_min.x = pt3.x;
         pt_min.y = pt3.y;
         pt_min.z = pt3.z;
         r = i;
         m = j;
         std::cout<<r<<" "<<m<<" "<<min_dist<<" "<<pt_min.x<<" "<<pt_min.y<<" "<<pt_min.z<<std::endl;
        }
      } 
    }
  }
  std::cout<<r<<" "<<m<<" "<<min_dist<<" "<<pt_min.x<<" "<<pt_min.y<<" "<<pt_min.z<<std::endl;
  std::cout<<"&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"<<std::endl;

  //getting the contour with max mean, as the mean should be highest for the position of led
  int max_sum = -1000;
  std::vector<cv::Point> max_contour;
  for( size_t i = 0; i < contours_candidate.size(); i++)
  {   
    float sum = 0;
    for( size_t j = 0; j < contours_candidate[i].size(); j++)
    {
      cv::Point pt = (contours_candidate[i])[j];
      sum +=  (int)color_gray.at<uchar>(pt.y, pt.x);
    }
    sum = sum/contours_candidate[i].size();
  //  std::cout<<"sum: "<<sum<<std::endl;
    if(sum > max_sum)
    {
      max_sum = sum;
      max_contour = contours_candidate[i];
    }
  }

  //debugging to find the contours
/*  std::vector<std::vector<cv::Point> > test_conts;
  test_conts.push_back(max_contour);

  for( size_t i = 0; i < test_conts.size(); i++)
  {
    cv::drawContours((tracker_in->diff_images)[10],test_conts,  i, cv::Scalar(0,0,255), 1, 8, cv::noArray(), 1, cv::Point());  
  }

  //localDebugImage((tracker_in->diff_images)[10], "/tmp/mean/test_");*/
  std::vector<pcl::PointXYZRGB> pt3ds;

  //calculate mid point of a contour
  bool flag = true;
  cv::Rect max_rect;
  if( max_contour.size() > 0 && flag)
  {
    max_rect = cv::boundingRect(max_contour);
    cv::rectangle((tracker_in->diff_images)[10], max_rect, cv::Scalar(0,255,0),1, 8);
  }


/*  for( int i = 0; i < contours_candidate.size(); i++)
  {
    for( int j = 0; j < contours_candidate[i].size(); j++)
    {
      cv::Point pt = (contours_candidate[i])[j];
      for( int k = 0; k < tracker_in->pclouds.size(); k++)
      {
        pcl::PointXYZRGB pt3 = (*tracker_in->pclouds[k])(pt.x, pt.y);
        if( isnan(pt3.x) || isnan(pt3.y) || isnan(pt3.z) )
        {
          continue;
        }
        else
        {
          pt3ds.push_back(pt3);
          std::cout<<pt3.x<<" "<<pt3.y<<" "<<pt3.z<<std::endl;
          break;
        }
      }
    }
  }
*/
  /*cv::circle((tracker_in->diff_images)[10], cv::Point(max_rect.x,max_rect.y), 8, cv::Scalar(0,0,255), 1, 8);
  localDebugImage((tracker_in->diff_images)[10], "/tmp/mean/test_");*/
  //adding weights based on the gray level
   for(int i = 0; i < max_contour.size(); i++)
   {
    pcl::PointXYZRGB pt3;
    cv::Point pt = max_contour[i];
      for( size_t j = 0; j < (tracker_in->pclouds).size(); j++ )
      { 
          pt3 = (*tracker_in->pclouds[j])(pt.x, pt.y);
          if( isnan(pt3.x) || isnan(pt3.y) || isnan(pt3.z) )
          {
            continue;
          }
          else
          {
            pt3ds.push_back(pt3);
         //   std::cout<<pt3.x<<" "<<pt3.y<<" "<<pt3.z<<std::endl;
            break;
          }
      } 
    }
  
  
/*  for( int i = 292; i < 297; i++)
  {
    for( int k = 278; k < 283; k++)
    {
      for( size_t j = 0; j < (tracker_in->pclouds).size(); j++ )
      { 
          pcl::PointXYZRGB pt3;
          pt3 = (*tracker_in->pclouds[j])(i, k);
          if( isnan(pt3.x) || isnan(pt3.y) || isnan(pt3.z) )
          {
            continue;
          }
          else
          {
            pt3ds.push_back(pt3);
            std::cout<<pt3.x<<" "<<pt3.y<<" "<<pt3.z<<std::endl;
            break;
          }
      } 
    }
  }*/


  pcl::PointXYZRGB centroid;
  //getWeightedCentroid(pt3ds, centroid);
  pcl::PointXYZRGB sum_pt;
  sum_pt.x = 0;
  sum_pt.y = 0;
  sum_pt.z = 0;
  //adding weights to find the centroid of the led
  for( size_t i = 0; i < pt3ds.size(); i++)
  {
    sum_pt.x += pt3ds[i].x;
    sum_pt.y += pt3ds[i].y;
    sum_pt.z += pt3ds[i].z;
  }
  tracker_in->estimate_led.point.x = sum_pt.x/(pt3ds.size());
  tracker_in->estimate_led.point.y = sum_pt.y/(pt3ds.size());
  tracker_in->estimate_led.point.z = sum_pt.z/(pt3ds.size());

/*  tracker_in->estimate_led.point.x = centroid.x;
  tracker_in->estimate_led.point.y = centroid.y;
  tracker_in->estimate_led.point.z = centroid.z;
*/
  std::cout<<" "<<"actual"<<": "<<tracker_in->pt3d.x<<" "<<tracker_in->pt3d.y<<" "<<tracker_in->pt3d.z<<std::endl;
  std::cout<<" "<<"centroided"<<": "<<sum_pt.x/pt3ds.size()<<" "<<sum_pt.y/pt3ds.size()<<" "<<sum_pt.z/pt3ds.size()<<std::endl;
 //std::cout<<" "<<"predicted"<<": "<<centroid.x<<" "<<centroid.y<<" "<<centroid.z<<std::endl;


}

void LedFinder::getWeightedCentroid(std::vector<pcl::PointXYZRGB> pts, pcl::PointXYZRGB& centroid)
{
  double min = 1000;
  double max = -1000;
  std::vector<double> gray_val;
  for( size_t i = 0; i < pts.size(); i++)
  {
    //std::cout<<pts[i].r<<" "<<pts[i].g<<" "<<pts[i].b<<std::endl;
    double gray = 0.2989*pts[i].r + 0.5870*pts[i].g + 0.1140*pts[i].b;
    gray_val.push_back(gray);
    if(gray < min)
    {
      min = gray;
    }

    if(gray > max)
    {
      max = gray;
    }
  }

  double total_weight = 0;
  pcl::PointXYZRGB sum_pt;
  sum_pt.x = 0;
  sum_pt.y = 0;
  sum_pt.z = 0;

  for( int i = 0; i < pts.size(); i++)
  {
    double curr_weight = min + (max - min)*(gray_val[i] - min)/(max - min);
    total_weight += curr_weight;
    sum_pt.x += curr_weight*pts[i].x;
    sum_pt.y += curr_weight*pts[i].y;
    sum_pt.z += curr_weight*pts[i].z;    
  }

  centroid.x = sum_pt.x/total_weight;
  centroid.y = sum_pt.y/total_weight;
  centroid.z = sum_pt.z/total_weight;

}

void LedFinder::localDebugImage(cv::Mat img, std::string str)
{
  ros::Time n = ros::Time::now();
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss<<str<<n<<".jpg";
  cv::imwrite(ss.str(), img);
}

LedFinder::CloudDifferenceTracker::CloudDifferenceTracker(
  std::string frame, double x, double y, double z) :
    frame_(frame)
{
  point.x = x;
  point.y = y;
  point.z = z;
}

  


void LedFinder::CloudDifferenceTracker::reset(size_t size)
{
  // Number of clouds processed.
  count_ = 0;
  // Maximum difference observed
  max_ = -1000.0;
  // Pixel tis was observed in
  max_idx_ = -1;

  // Setup difference tracker
  diff_.resize(size);
  for (std::vector<double>::iterator it = diff_.begin(); it != diff_.end(); ++it)
  {
    *it = 0.0;
  }
}

// Weight should be +/- 1 typically
bool LedFinder::CloudDifferenceTracker::process(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev,
  double weight)
{

  if (cloud->size() != diff_.size())
  {
    std::cerr << "Cloud size has changed." << std::endl;
    return false;
  }

  for (size_t i = 0; i < cloud->size(); i++)
  {
    diff_[i] += ((double)(cloud->points[i].b) - (double)(prev->points[i].b)) * weight;
    if (diff_[i] > max_)
    {
      max_ = diff_[i];
      max_idx_ = i;
    }
  }

  return true;
}

/*functions by varun*/
/*
 *@brief takes in current and preecious point clouds, populates 
         the tracker_contours structure for batch processing later
 *@param id of the led currently on
 *@param vector of clouds before
 *@param vector of clouds after
 *@param reference for the track_contours struct
 *@param to call constructor only the first time
 *
 */
bool LedFinder::CloudDifferenceTracker::oprocess( pcl::PointXYZRGB pt,
                                                  int tracker_id,
                                                  std::vector<pcloud_> cloud,
                                                  std::vector<pcloud_> prev,
                                                  std::vector<CloudDifferenceTracker::TrackContoursPtr>& track_contours,
                                                  bool check_first_time
                                                )
{
  if( (track_contours[tracker_id]->first_time) == false && check_first_time)
  {
   (track_contours[tracker_id])->first_time = true;
   (track_contours[tracker_id])->pt3d = pt;
   (track_contours[tracker_id])->tracker_id = tracker_id;
  }
  //cv_bridge image pointers
  std::vector<cv_bridge::CvImagePtr> cloud_image_ptr;
  std::vector<cv_bridge::CvImagePtr> prev_image_ptr;

  cv::Mat cloud_bits;
  cv::Mat prev_bits;

  //function call for initial processing to convert to cv::Mat
  convert2CvImagePtr(cloud, cloud_image_ptr);
  convert2CvImagePtr(prev, prev_image_ptr);
  //ROS_INFO("%d",cloud_image_ptr[0]->image.rows);

  cv::Mat cloud_pix_weighed(cloud_image_ptr[0]->image.rows, cloud_image_ptr[0]->image.cols, CV_8UC3, cv::Scalar(0,0,0));
  cv::Mat prev_pix_weighed(cloud_image_ptr[0]->image.rows, cloud_image_ptr[0]->image.cols, CV_8UC3, cv::Scalar(0,0,0));
  
  weightedSum(cloud_image_ptr, cloud_pix_weighed);
  weightedSum(prev_image_ptr, prev_pix_weighed);

  cv::Mat diff_image = cv::Mat(cloud_image_ptr[0]->image.rows, cloud_image_ptr[0]->image.cols, CV_8UC3, cv::Scalar(0,0,0));

  //calculate the difference Image
  cv::Mat img;
  cv::absdiff(cloud_pix_weighed, prev_pix_weighed, diff_image);
  
  //possible contours in the image
  std::vector<std::vector<cv::Point> > possible_contours;
//  possibleContours(diff_image, possible_contours);

  //Filling the members {std::vec<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>, diff_img, rgb_img}
  //--------------------------------------------------------------------------------------------------------------------------------------> TrackContour Pointer population
  //Pointclouds push_back 
  int size = std::min(cloud.size(), prev.size());
  for( int i = 0; i < size; i++)
  {
    (track_contours[tracker_id])->pclouds.push_back(cloud[i]);
    (track_contours[tracker_id])->pclouds.push_back(prev[i]); 
  }
  
  //push_back images
  (track_contours[tracker_id])->diff_images.push_back(diff_image);
  (track_contours[tracker_id])->rgb_image.push_back(cloud_pix_weighed);

  //--------------------------------------------------------------------------------------------------------------------------------------> TrackContour Pointer population
}


void LedFinder::CloudDifferenceTracker::possibleContours(cv::Mat& diff_image, std::vector<std::vector<cv::Point> >& contours)
{
  float canny_thresh = 60;
  cv::Mat canny_image;
  std::vector<cv::Vec4i> hierarchy;
  cv::Mat gray;
  cv::cvtColor(diff_image, gray, CV_BGR2GRAY);
  cv::Canny(diff_image, canny_image, canny_thresh, canny_thresh*2, 3);
  cv::findContours(canny_image, contours, hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );
}

/*
 * @brief create a weight_img = ( img(2)-img(1) )/img(2) , 
 *        use per element operations in opencv to calculate a 
 *        weighted image
 * @param array of image pointers
 * @param weighed image
 */
void LedFinder::CloudDifferenceTracker::weightedSum(std::vector<cv_bridge::CvImagePtr>& images, cv::Mat& result)
{

  cv::Mat tmp_weight(images[0]->image.rows, images[0]->image.cols, CV_8UC3, cv::Scalar(0,0,0));

  //considering the images in the middle so that more noisy initial images are avoided
  for(size_t i =5; i < 6  ; i++)
  {
    cv::add(tmp_weight,images[i]->image, result);
    tmp_weight = result;
  }

}


void LedFinder::CloudDifferenceTracker::convert2CvImagePtr(std::vector<pcloud_>& pcl_cloud, std::vector<cv_bridge::CvImagePtr>& cv_ptr)
{
  sensor_msgs::Image::Ptr ros_image(new sensor_msgs::Image);
  sensor_msgs::PointCloud2::Ptr ros_cloud(new sensor_msgs::PointCloud2);

  cv_ptr.resize(pcl_cloud.size());
  for(size_t i = 0; i < pcl_cloud.size(); i++)
  {
    cv_ptr[i].reset(new cv_bridge::CvImage);
      for(size_t j = 0; j < pcl_cloud[i]->size(); j++)
      {
        if(pcl_cloud[i]->points[j].z > 1.0 || isnan(pcl_cloud[i]->points[j].z))
        {
          pcl_cloud[i]->points[j].x = NAN;
          pcl_cloud[i]->points[j].y = NAN;
          pcl_cloud[i]->points[j].z = NAN;
          pcl_cloud[i]->points[j].r = 0;
          pcl_cloud[i]->points[j].g = 0;
          pcl_cloud[i]->points[j].b = 0; 
        }
      }

    pcl::toROSMsg(*(pcl_cloud[i]),*ros_cloud);
    pcl::toROSMsg(*ros_cloud, *ros_image);
    try
    {
      cv_ptr[i] = cv_bridge::toCvCopy(*ros_image, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cloud_rosimage is sorry: %s ", e.what());
    }
    
  }

}


 void LedFinder::CloudDifferenceTracker::debug_img(cv::Mat image, std::string string_in, int k, int l, float diff)
 {

  ros::Time n = ros::Time::now();
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss<<string_in<<n<<"_"<<k<<l<<"_"<<diff<<".jpg";
  imwrite(ss.str(), image);
 }


/*overloaded function added by varun*/
bool LedFinder::CloudDifferenceTracker::isFound(
  const pcloud_ clouds,
  double threshold)
{
/*  for(size_t i = 0; i < clouds.size(); i++)
  {
    if(max_ < threshold)
  }*/
}


bool LedFinder::CloudDifferenceTracker::getRefinedCentroid(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  geometry_msgs::PointStamped& point)
{
  ROS_INFO("In centroid");
  // Get initial centroid
  geometry_msgs::PointStamped p;
  point.point.x = cloud->points[max_idx_].x;
  point.point.y = cloud->points[max_idx_].y;
  point.point.z = cloud->points[max_idx_].z;

  // Do not accept NANs
  if (isnan(point.point.x) ||
      isnan(point.point.y) ||
      isnan(point.point.z))
  {
    return false;
  }

  // Get a better centroid
  int points = 0;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  for (size_t i = 0; i < cloud->size(); ++i)
  {
    if (isnan(cloud->points[i].x) ||
        isnan(cloud->points[i].y) ||
        isnan(cloud->points[i].z))
    {
      continue;
    }

    // Using highly likely points
    if (diff_[i] > (max_*0.75))
    {
      double dx = cloud->points[i].x - p.point.x;
      double dy = cloud->points[i].y - p.point.y;
      double dz = cloud->points[i].z - p.point.z;

      // That are less than 5mm from the max point
      if ( (dx*dx) + (dy*dy) + (dz*dz) < 0.22 )
      {
        sum_x += cloud->points[i].x;
        sum_y += cloud->points[i].y;
        sum_z += cloud->points[i].z;
        ++points;
      }
    }
  }

  if (points > 0)
  {
    point.point.x = sum_x/points;
    point.point.y = sum_y/points;
    point.point.z = sum_z/points;
  }

  /* Fill in the headers */
  point.header.seq = cloud->header.seq;
  point.header.frame_id = cloud->header.frame_id;
  point.header.stamp.fromNSec(cloud->header.stamp * 1e3);  // from pcl_conversion

  return true;
}

  bool LedFinder::CloudDifferenceTracker::getDifferenceCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev,
    cv::Mat& diff_image_,
    double weight)
  {
   
    std::vector< pcl::PointCloud<pcl::PointXYZRGB> > pcl_cloud;
    pcl_cloud.push_back(*cloud);
    pcl_cloud.push_back(*prev);
    std::vector<sensor_msgs::Image> image;
    std::vector<sensor_msgs::PointCloud2> ros_cloud;
    std::vector<cv_bridge::CvImagePtr> cv_ptr;
    std::vector<cv::Mat> cvimage;
    cvimage.resize(2);
    image.resize(2);
    ros_cloud.resize(2);
    cv_ptr.resize(2);
    
    for(int j = 0; j < 2; j++)
    {
     pcl::toROSMsg(pcl_cloud[j], ros_cloud[j]);
     pcl::toROSMsg(ros_cloud[j], image[j]);
     try
     {
        cv_ptr[j] = cv_bridge::toCvCopy(image[j], sensor_msgs::image_encodings::BGR8);
     }
     catch(cv_bridge::Exception& e)
     {
       ROS_ERROR("failed to convert: %s", e.what()); 
     }
    }
    std::stringstream ss(std::stringstream::in | std::stringstream::out);
    ros::Time n = ros::Time::now();
    ss<<"/tmp/color/image_"<<n<<".jpg";
    imwrite(ss.str(),cv_ptr[0]->image); 

    diff_image_ = cv_ptr[0]->image - cv_ptr[1]->image;
    return true;
  }

}  // namespace robot_calibration
