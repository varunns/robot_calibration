
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
  
  for( int i = 0; i < trackers_.size(); i++)
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
  std::vector<cv::Point>  debug_image_manual;
  debug_image_manual.resize(trackers_.size());
  debug_image_manual[0] = cv::Point(282,230);
  debug_image_manual[1] = cv::Point(257,287);
  debug_image_manual[2] = cv::Point(301,311);
  debug_image_manual[3] = cv::Point(328,247);
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
/*
    if (++cycles > max_iterations_)
    {
      return false;
    }*/
    /* previous clouds*/
    *prev_cloud = *cloud_ptr_;
    prev_clouds = clouds_ptr_;
    clouds_ptr_.resize(0);
    check_first_time = true;
  }
 
  //-------------------------------------------------------------------------------------------Strart Using the populated vector of sharedPtr : led_respective_contours
  std::vector<pcl::PointXYZRGB> led_pts;
  std::vector<cv::Rect> bounding_rect_leds;
  std::vector<int> area;
  //Should be replaced by a bost pointer
  std::vector<std::vector<cv::Point> > probable_contours;
  std::vector<cv::Point> most_probable_contour;
//  for( int i = 0 ; i < led_respective_contours.size(); i++)
   for( int i = 0 ; i  < 1; i++)
  {
    cv::Rect bounding_box;
    getCandidateRoi(led_respective_contours[i], bounding_box, debug_image_manual[i]);
    //checkMostProbableCandidate(led_respective_contours[i], probable_contours, most_probable_contour);
    bounding_rect_leds.push_back(bounding_box);
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
    geometry_msgs::PointStamped hough_pt;
  

    if (!trackers_[t].getRefinedCentroid(cloud_ptr_, rgbd_pt))
    {

      ROS_ERROR_STREAM("No centroid for feature " << t);
      return false;
    }
    // Check that point is close enough to expected pose
    try
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
    }
    double distance = distancePoints(world_pt.point, trackers_[t].point);
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

/*void LedFinder::checkMostProbableCandidate(CloudDifferenceTracker::TrackContoursPtr tracker_in, 
                                           std::vector<std::vector<cv::Point> > probable_contours, 
                                           std::vector<cv::Point> most_probable_contour)
{

}*/




void LedFinder::getCandidateRoi(CloudDifferenceTracker::TrackContoursPtr tracker_in, 
                                cv::Rect& rect,
                                cv::Point debug_pixel)
{

  cv::Mat graytmp;
  cv::Mat tmp = (tracker_in->diff_images)[3];
  cv::cvtColor(tmp, graytmp, CV_BGR2GRAY);
  cv::threshold(graytmp, graytmp, 5, 255, CV_THRESH_BINARY);
  cv::Mat dst;

  //Using a bitwise-AND on all the depth images to determine the most existent pixesl
  //Also using only the images in middle, again to avoid any noisy diff images
  for(int i = 4; i < (tracker_in->diff_images).size(); i++)
  {
    cv::Mat gray;
    cv::cvtColor((tracker_in->diff_images)[i], gray, CV_BGR2GRAY);
    cv::threshold(gray, gray, 5, 255, CV_THRESH_BINARY);
    cv::bitwise_and(gray, graytmp, dst);
    graytmp = dst;
//    localDebugImage((tracker_in->rgb_image)[i], "/tmp/mean/image_");
    localDebugImage((tracker_in->diff_images)[i], "/tmp/mean/diff_");
  }
  std::vector<cv::Point2i> locations;

  //using all the non zero pixels , for getting the locations
  if(cv::countNonZero(dst) > 0)
  {
    cv::findNonZero(dst,locations);
  }

  //debug
  localDebugImage(dst,"/tmp/mean/bitwise_");
  

  //Using any diff image values to populate the non zero locations
  cv::Mat diff_gray, color_gray;
  cv::cvtColor( (tracker_in->diff_images)[10], diff_gray, CV_BGR2GRAY );
  cv::cvtColor( (tracker_in->rgb_image)[10], color_gray, CV_BGR2GRAY);
  
  cv::Mat non_zero = cv::Mat::zeros(diff_gray.rows, diff_gray.cols, CV_8UC1);
  for( int i = 0; i < locations.size(); i++)
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
  cv::rectangle((tracker_in->rgb_image)[1], cv::Rect(debug_pixel.x,debug_pixel.y,5,5),cv::Scalar(0,255,0),1,8);

  localDebugImage(non_zero,"/tmp/mean/non_zero_");
  localDebugImage((tracker_in->rgb_image)[1],"/tmp/mean/bitwised_");
 
  for( int i = 0; i < contours_candidate.size(); i++)
  {
    cv::drawContours((tracker_in->diff_images)[10],contours_candidate,  i, cv::Scalar(0,185,155), 1, 8, cv::noArray(), 1, cv::Point());  
  }

  std::vector<pcl::PointXYZRGB> pt3ds;
  //for all the conooutrs obtained calculate the debug points, center
  for(int i = 0; i < contours_candidate.size(); i++)
  {

    std::vector<cv::Point> contour = contours_candidate[i];
    std::vector<pcl::PointXYZRGB> pt3ds_temp;
    pcl::PointXYZRGB sum_pt;
    sum_pt.x = 0;
    sum_pt.y = 0;
    sum_pt.z = 0;

    for(int j = 0; j < contour.size(); j++)
    {   
      pcl::PointXYZRGB pt3;
      cv::Point pt = contour[j];

      for( int k = 0; k < (tracker_in->pclouds).size(); k++ )
      {

        pt3 = (*tracker_in->pclouds[k])(pt.x, pt.y);
        if( !isnan(pt3.x) && !isnan(pt3.y) && !isnan(pt3.z) )
        {
           pt3ds_temp.push_back(pt3);
        }

      }

      for( int k = 0; k < pt3ds_temp.size(); k++)
      {
        sum_pt.x  = sum_pt.x + pt3ds_temp[k].x;
        sum_pt.y  = sum_pt.y + pt3ds_temp[k].y;
        sum_pt.z  = sum_pt.z + pt3ds_temp[k].z;
      }

      sum_pt.x = sum_pt.x/(pt3ds_temp.size());
      sum_pt.y = sum_pt.y/(pt3ds_temp.size());
      sum_pt.z = sum_pt.z/(pt3ds_temp.size());
      
    }
    std::cout<<" "<<"predicted : "<<sum_pt.x<<" "<<sum_pt.y<<" "<<sum_pt.z<<std::endl;
    pt3ds.push_back(sum_pt);
    pt3ds_temp.clear();
    contour.clear();
  }

  std::cout<<"debug_pixel:"<<debug_pixel<<std::endl;
  pcl::PointXYZRGB deb_pt3; 
  for( int k = 0; k < (tracker_in->pclouds).size(); k++ )
  {
    pcl::PointXYZRGB pt = (*tracker_in->pclouds[k])(debug_pixel.x,debug_pixel.y);

    if(isnan(pt.x) && isnan(pt.y) && isnan(pt.z) )
    {
      continue;
    }

    deb_pt3 = pt;
    break;
  }

  std::cout<<"tracker_id : "<<tracker_in->tracker_id<<std::endl;
  std::cout<<"Debugged  : "<<deb_pt3.x<<" "<<deb_pt3.y<<" "<<deb_pt3.z<<std::endl;
  std::cout<<"FromTransform : "<<tracker_in->pt3d.x<<" "<<tracker_in->pt3d.y<<" "<<tracker_in->pt3d.z<<std::endl;
  for( int i = 0; i < pt3ds.size(); i++)
  {
    std::cout<<" "<<"Debug_diff:   "<< i <<std::sqrt(pow((pt3ds[i].x-deb_pt3.x),2)+pow((pt3ds[i].y-deb_pt3.y),2)+pow((pt3ds[i].z-deb_pt3.z),2))<<std::endl;
    std::cout<<" "<<"Finder_diff:   "<< i <<std::sqrt(pow((pt3ds[i].x-tracker_in->pt3d.x),2)+pow((pt3ds[i].y-tracker_in->pt3d.y),2)+pow((pt3ds[i].z-tracker_in->pt3d.z),2))<<std::endl;
  }
  

}

bool LedFinder::findInMatchedContours(std::vector<cv::Point> contour, std::vector<std::vector<cv::Point> >  matched_contours)
{
  for( int i = 0; i < matched_contours.size(); i++)
  {
    if( cv::matchShapes(contour, matched_contours[i], CV_CONTOURS_MATCH_I1, 0) == 0 )
    {
      return true;
    }
  }

  return false;
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
  // Pixel this was observed in
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
 *@brief the function processes n number of pointclouds that come before and after
 *       finds the weighted sum of respective vectors, and then the difference
 *@param vector of clouds before
 *@param vector of clouds after
 *@param weight assigned whether on or off
 *
 *@note weighted some is done to avoid the noise originating from 
 *      external illumination(can further be invested)
 *      there is little repetetion, the code can still be modified
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
  possibleContours(diff_image, possible_contours);

  /*Determing the contour nearest to the LED,
    determine the enclosing circle, and the get a bounding box
    to search for the nearest 3D points */
  std::vector<std::vector<cv::Point> > final_contours;
  std::priority_queue<ContourDistPtr, std::vector<ContourDistPtr>, CompareContourDist > contour_dist_queue;
  for( int i = 0; i < possible_contours.size(); i++)
  {
    cv::Point2f center;
    float r;
    float dist;
    cv::minEnclosingCircle(possible_contours[i], center, r);
   
    if(!calcDistQueue(pt, cloud_pix_weighed, cloud, prev, center, r, dist))
    {
      continue;
    }
    else
    {
      ContourDistPtr dist_tmp(new ContourDist(possible_contours[i], dist) );
      contour_dist_queue.push(dist_tmp);
    }
  }

  while(!contour_dist_queue.empty())
  {
    final_contours.push_back(contour_dist_queue.top()->contour);
    contour_dist_queue.pop();
  }

  //Adding the final_contours to respective tracker  pointer and filling the other members {std::vec<contours>, std::vec<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>, diff_img, rgb_img}
  //--------------------------------------------------------------------------------------------------------------------------------------> TrackContour Pointer population

  if(final_contours.size() != 0)                                     
  {
    for( int i = 0; i < final_contours.size(); i ++)
    {
      (track_contours[tracker_id])->all_contours.push_back(final_contours[i]);  
    }
    
  }

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


bool LedFinder::CloudDifferenceTracker::calcDistQueue(pcl::PointXYZRGB pt,  
                                                      cv::Mat color_img,
                                                      std::vector<pcloud_> cloud, 
                                                      std::vector<pcloud_> prev, 
                                                      cv::Point2f center,
                                                      float radius,
                                                      float& dist_cand)
{
  cv::Point cvpt = cv::Point(0,0);
  cvpt.x = (int)center.x;
  cvpt.y = (int)center.y;
  pcl::PointXYZRGB non_nan_pt;
  //adding a distance vector and considering all the 3D points possible in all the point clouds
  std::vector<pcl::PointXYZRGB> p3vec;
  
  //check if the region is on the borders
  if( center.x < 30 || center.x > 600 || center.y < 30 || center.y > 450)
  {
    return false;
  }
  
  //check if more than 40% of the points in the region are black i.e out of pass thru
  cv::Mat gray;
  cv::Rect roi = cvRect(cvpt.x - 10, cvpt.y - 10, 20, 20);
  cv::cvtColor(color_img(roi), gray, CV_BGR2GRAY);
  if( ((float)cv::countNonZero(gray))/400 < 0.9)
  {
    return false;
  }

  // define a small roi 
  for(int i = (cvpt.x - 10); i < (cvpt.x + 10); i++)
  {
    for( int j = (cvpt.y - 10); j < (cvpt.y + 10); j++)
    {

      //continue if the point is nan else use the first non nan point to get the distance from led_pt
      int limit = std::min(cloud.size(), prev.size());
      for( int k = 0; k < limit; k++)
      {
       
       pcl::PointXYZRGB pt1 = (*(cloud[k]))(i,j);
       pcl::PointXYZRGB pt2 = (*(prev[k]))(i,j); 

       if(!isnan(pt1.x) && !isnan(pt1.y) && !isnan(pt1.z))
       {
        non_nan_pt = pt1;
        p3vec.push_back(pt1);
       }

       if(!isnan(pt2.x) && !isnan(pt2.y) && !isnan(pt2.z))
       {
        non_nan_pt = pt2;
        p3vec.push_back(pt2);
       }

       else
       {
        continue;
       }

      }

    }
  }
  std::vector<float> distance;
  if(p3vec.size() > 0)
  {
    float dist = (non_nan_pt.x - pt.x)*(non_nan_pt.x - pt.x)+
                 (non_nan_pt.y - pt.y)*(non_nan_pt.y - pt.y)+
                 (non_nan_pt.z - pt.z)*(non_nan_pt.z - pt.z);
    distance.push_back(dist);
  }
  std::sort(distance.begin(), distance.end());
  if( distance[0] > 0.005)
  {
  //  std::cout<<dist<<std::endl;
    return false;
  }
  dist_cand = distance[0];
  return true;
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
  for(int i = 15; i < 16  ; i++)
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
    for(int j = 0; j < pcl_cloud[i]->size(); j++)
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

/*added by varun*/
/*bool LedFinder::CloudDifferenceTracker::getHoughCirclesCentroid(
  const pcl::PointCloud<pcl::PointXYZRGB> cloud,
  geometry_msgs::PointStamped& hough_pt)
{
  ROS_INFO("in HOUGH here");
  sensor_msgs::PointCloud2 ros_cloud;
  sensor_msgs::Image image;
  ROS_INFO("after image");
  pcl::toROSMsg(cloud, ros_cloud);
  pcl::toROSMsg(ros_cloud, image);
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge is acting funny: %s", e.what());
    return false;
  }
  ROS_INFO("after conversion");
  ros::Time n = ros::Time::now();
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss<<"/tmp/image_"<<n<<".jpg";
  imwrite(ss.str(), cv_ptr->image);
  std::vector<cv::Vec3f> circles;

  cv::Mat gray_image, norm_image;
  cv::cvtColor( cv_ptr->image, gray_image, CV_BGR2GRAY );
  cv::normalize(gray_image, norm_image, 0 , 255, 32,  -1); //enum for NORM_MINMAX is 32 namespace cv is not declared
  cv::threshold(gray_image, gray_image, 200, 255, CV_THRESH_BINARY);
  cv::HoughCircles(norm_image, circles, CV_HOUGH_GRADIENT, 1, 1, 6, 8, 0, 0);

  if(circles.size() < 0)
  {
    return false;
  }

 
  for(size_t i = 0; i < circles.size(); i++)
  {
    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);
    ROS_INFO("RADIUS OF THE CIRCLE : %d", radius);
    // circle center
    cv::circle(cv_ptr->image, center, 1, cv::Scalar(0,255,0), -1, 8, 0 );
    // circle outline
    cv::circle(cv_ptr->image, center, radius, cv::Scalar(0,0,255), 1, 8, 0 ); 
  }
  pcl::PointXYZRGB pt = (cloud)(cvRound(circles[0][0]),cvRound(circles[0][1]));
  hough_pt.point.x = pt.x; hough_pt.point.y = pt.y; hough_pt.point.z = pt.z;

  n = ros::Time::now();
  ss.str(" ");
  ss<<"tmp/image_"<<n<<".jpg";
  imwrite(ss.str(), cv_ptr->image);

  return true;

}*/

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

 /*   cv::Mat gray;
    cv::cvtColor(diff_image_, gray, CV_BGR2GRAY);
    cv::threshold(gray, gray, 40, 255, CV_THRESH_BINARY);
    ss.str(" ");   
    ss<<"/tmp/diff/image_"<<n<<".jpg";
    imwrite(ss.str(), gray);*/
    return true;
  }

  bool LedFinder::CloudDifferenceTracker::getContourCircle(cv::Mat&  image,
                                                           geometry_msgs::PointStamped& point)
  {
 /*  cv::Mat gray_image, norm_image, canny_image;

   cv::cvtColor(image, gray_image, CV_BGR2GRAY);
   cv::normalize(image, norm_image, 0, 255, 32, -1);
   std::stringstream ss( std::stringstream::in | std::stringstream::out)
   ros::Time now = ros::Time::now();
   ss<<"/tmp/imagebef_"<<now<<".jpg";
   imwrite(ss.str(),nowm_image);
   cv::threshold(norm_image, norm_image, 190,255, CV_THRESH_BINARY);
   cv::Canny(norm_image, canny_image, 10, 30, 3);

   std::vector< std::vector<cv::Point> > contours;
   std::vector<cv::Vec4i> hierarchy;
   cv::findContours(canny_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

   
   for(size_t i = 0; i < contours.size(); i++)
   {
    cv::Scalar color =  cv::Scalar(0,0,255);
    cv::drawContours(image, contours, i, color, 2, 8, hierarchy, 0 , cv::Point());
   }  */
  }

}  // namespace robot_calibration
