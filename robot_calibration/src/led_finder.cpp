
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
    if(clouds_ptr_.size() > 7)
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
  int count = 20;
  while (--count)
  {
    if (!waiting_)
    {
      // success
      return true;
    }
    ros::Duration(0.05).sleep();
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
    
    pcl::PointXYZRGB pt;
    pt.x = led_pt_cameraframe.point.x;
    pt.y = led_pt_cameraframe.point.y;
    pt.z = led_pt_cameraframe.point.z;
    
    //getting the led point in the frame of camera ------------------------------------------------------------->end
    trackers_[tracker].process(cloud_ptr_, prev_cloud, weight);


    /*A condition pn cycles is used as the initial difference images are
      not necessarily discernible
    */

    if(cycles > 3)
    {
      trackers_[tracker].oprocess(pt, tracker, clouds_ptr_, prev_clouds, led_respective_contours, check_first_time);
    }

    if (++cycles > max_iterations_)
    {
      break;
    }
  
  /*if (++cycles > max_iterations_)
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
  std::vector<pcl::PointXYZRGB> original_pts;
  original_pts.resize(4);
  for( size_t i = 0; i < led_respective_contours.size(); i++)
  {
    original_pts[i] = led_respective_contours[i]->pt3d;
  }
  
  for( size_t i = 0 ; i < led_respective_contours.size(); i++)
  {
    cv::Rect bounding_box;
    pcl::PointXYZRGB tmp_pt3;
    getCandidateRoi(led_respective_contours[i],original_pts,tmp_pt3);
    led_pts.push_back(tmp_pt3);
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

/*
 *@brief Takes in the tracker_in, that contains all the images, pointclouds,
         tracker_id, detects the led using highest probability approach, and 
         detects the 3d points
 * @param struct tracker_in as described above
 * @param reference pcl point to store the detected center
 */
void LedFinder::getCandidateRoi(CloudDifferenceTracker::TrackContoursPtr tracker_in, 
                                std::vector<pcl::PointXYZRGB> original_pts,
                                pcl::PointXYZRGB& tmp_pt3)
{

  cv::Mat graytmp;
  cv::Mat tmp = (tracker_in->diff_images)[0];
  cv::cvtColor(tmp, graytmp, CV_BGR2GRAY);
  cv::threshold(graytmp, graytmp, 5, 255, CV_THRESH_BINARY);
  cv::Mat dst;

  //Using a bitwise-AND on all the depth images to determine the most (probable to 
  //generalize the method further)existent pixels
  //Also using only the images in middle, again to avoid any noisy diff images
  for( size_t i = 4; i < (tracker_in->diff_images).size(); i++)
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
  cv::cvtColor( (tracker_in->diff_images)[0], diff_gray, CV_BGR2GRAY );
  cv::cvtColor( (tracker_in->rgb_image)[0], color_gray, CV_BGR2GRAY);
  
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
  

  localDebugImage(non_zero,"/tmp/mean/non_zero_");
  localDebugImage((tracker_in->rgb_image)[1],"/tmp/mean/bitwised_");
 
  for( size_t i = 0; i < contours_candidate.size(); i++)
  {
    cv::drawContours((tracker_in->diff_images)[2],contours_candidate,  i, cv::Scalar(0,185,155), 1, 8, cv::noArray(), 1, cv::Point());  
  }


  
    std::vector<cv::Point> max_contour;
    double max = -1000.0;
    if(contours_candidate.size() > 0)
    {
      for(size_t i = 0; i < contours_candidate.size(); i++)
      {
        double sum = 0;
        std::vector<cv::Point> tmp_contour = contours_candidate[i];
        for(size_t j = 0; j < tmp_contour.size(); j++)
        {
          sum += (double)color_gray.at<uchar>(tmp_contour[j].y, tmp_contour[j].x);
        }
        sum = sum/tmp_contour.size();
        if(sum > max)
        {
          max = sum;
          max_contour = tmp_contour;
        }
      }
    }

std::vector< std::vector<cv::Point> > contour_test;
contour_test.push_back(max_contour);

//for debugging, need to add a debug flag
for( size_t i = 0 ; i < contour_test.size(); i++)
{
  cv::drawContours((tracker_in->rgb_image)[11], contour_test, i, cv::Scalar(0,255,0), 1, 8, cv::noArray(), 1, cv::Point());
}
localDebugImage((tracker_in->rgb_image)[11], "/tmp/mean/contour_" );
  std::cout<<"max_contour"<<max_contour.size()<<std::endl;
  pcl::PointXYZRGB pt3ds;
  std::vector<pcl::PointXYZRGB> cand_pts;
  //for all the conooutrs obtained calculate the debug points, center
  if(max_contour.size() > 0)
  {   
    for( size_t i = 0; i < max_contour.size(); i++)
    {
      for( size_t j = 0; j < (tracker_in->pclouds).size(); j++)
      {
        pcl::PointXYZRGB pt = (*tracker_in->pclouds[j])((max_contour[i]).x, (max_contour[i]).y);
        if( !isnan(pt.x) && !isnan(pt.y) && !isnan(pt.z) )
        {
          cand_pts.push_back(pt);
        }
      }
    }
  }
  
  pcl::PointXYZRGB sum_pt;
  for( size_t i = 0; i < cand_pts.size(); i++)
  {
    sum_pt.x += cand_pts[i].x;
    sum_pt.y += cand_pts[i].y;
    sum_pt.z += cand_pts[i].z;
  }
  std::cout<<"sum :"<<sum_pt.x<<" "<<sum_pt.y<<" "<<sum_pt.z<<std::endl;
  if(cand_pts.size() > 0)
  {
    pt3ds.x = sum_pt.x/cand_pts.size();
    pt3ds.y = sum_pt.y/cand_pts.size();
    pt3ds.z = sum_pt.z/cand_pts.size();
  }

  std::vector<double> distance_estimate;
  std::vector<double> distance_tf;

  float min_dist = 1000;
  
  int index2;
  for( size_t j = 0; j < original_pts.size(); j++)
  {
    float dist = std::sqrt(pow((pt3ds.x - original_pts[j].x),2) + pow((pt3ds.y - original_pts[j].y),2) + pow((pt3ds.z - original_pts[j].z),2));
    std::cout<<"dist : "<<dist<<std::endl;
    if(dist < min_dist)
    {
      min_dist = dist;
      index2 = j;
     }
  }
  
  tmp_pt3 = pt3ds;

  std::cout<<"pt3ds"<<pt3ds.x<<" "<<pt3ds.y<<" "<<pt3ds.z<<std::endl;
  std::cout<<"FromTransform : "<<tracker_in->pt3d.x<<" "<<tracker_in->pt3d.y<<" "<<tracker_in->pt3d.z<<std::endl;
  std::cout<<original_pts[index2].x<<" "<<original_pts[index2].y<<" "<<original_pts[index2].z<<" "<<std::endl;
  std::cout<<min_dist<<std::endl;

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

bool LedFinder::CloudDifferenceTracker::isFound(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  double threshold)
{
  // Returns true only if the max exceeds threshold
  if (max_ < threshold)
  {
    return false;
  }

  // AND the current index is a valid point in the cloud.
  if (isnan(cloud->points[max_idx_].x) ||
      isnan(cloud->points[max_idx_].x) ||
      isnan(cloud->points[max_idx_].x))
  {
    return false;
  }

  return true;
}

/*functions by varun*/
/*
 *@brief Takes in all the accumulated prev and current pointclouds, 
         calculates the difference image and populates a struct that
         contains all of the above for batch processing
 *@param tracker_id, used to identify the led
 *@param vector of clouds before
 *@param vector of clouds after
 *@param struct used to save all the data pertaining to the current frame
 *
 *@note I have considered atleast 20 clouds till now, to identify the led,
        The logic is based on the fact that led pixels are the most probable
        ones to appear in almost al the frames, so a probability of above, say
        may be 98% can be used, Also another check is to take a single frame 
        andget the least probable pixels with respect to gray levels are the led
        pixels, this joint probability can be used to accuratel determine led pixels
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

  //function call for initial processing to convert to cv::Mat
  convert2CvImagePtr(cloud, cloud_image_ptr);
  convert2CvImagePtr(prev, prev_image_ptr);

  cv::Mat diff_image = cv::Mat(cloud_image_ptr[0]->image.rows, cloud_image_ptr[0]->image.cols, CV_8UC3, cv::Scalar(0,0,0));

  //calculate the difference Image
  cv::Mat img;
  cv::absdiff(cloud_image_ptr[3]->image, prev_image_ptr[3]->image, diff_image);
  //--------------------------------------------------------------------------------------------------------------------------------------->TrackContour population
  //Pointclouds push_back
  int size = std::min(cloud.size(), prev.size());
  for( int i = 0; i < size; i++)
  {
    (track_contours[tracker_id])->pclouds.push_back(cloud[i]);
    (track_contours[tracker_id])->pclouds.push_back(prev[i]); 
  }
  
  //push_back images
  (track_contours[tracker_id])->diff_images.push_back(diff_image);

  //---------------------------------------------------------------------------------------------------------------------------------------->TrackContour Pointer population
  if(cloud.empty() || prev.empty() )
  {
    return false;
  }

  return true;
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


}  // namespace robot_calibration
