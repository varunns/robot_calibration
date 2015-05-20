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
  int count = 20;
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
  //pcloud_ clouds_ptr_ = clouds_ptr_[0];
  
  // Initialize difference trackers
  for (size_t i = 0; i < trackers_.size(); ++i)
  {
    trackers_[i].reset(cloud_ptr_->size());
  }
  std::vector<cv::Mat> diff_images_vector;
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
  
    //call to obtain difference cloud and the max cloud
    diff_image_.release();
    trackers_[tracker].getDifferenceCloud(cloud_ptr_, prev_cloud, diff_image_, weight);
    trackers_[tracker].process(cloud_ptr_, prev_cloud, weight);
    trackers_[tracker].oprocess(clouds_ptr_, prev_clouds, weight);


    if (++cycles > max_iterations_)
    {
      return false;
    }
    /* previous clouds*/
    *prev_cloud = *cloud_ptr_;
    prev_clouds = clouds_ptr_;
    clouds_ptr_.resize(0);
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
    

    /* code added for hough transform*/
    if (!trackers_[t].getContourCircle(diff_image_, hough_pt))
    {
      ROS_ERROR_STREAM("No centroid for feature " << t);
      return false;
    }    
    // the transformed point
    try
    {
      listener_.transformPoint(trackers_[t].frame_, ros::Time(0), hough_pt,
                               hough_pt.header.frame_id, hough_pt);
    }
    catch(const tf::TransformException &ex)
    {
      ROS_ERROR_STREAM("Failed to transform hough feature to " << trackers_[t].frame_);
      return false;
    }

    /* code added for hough transform*/



    if (!trackers_[t].getRefinedCentroid(cloud_ptr_, rgbd_pt))
    {
      ROS_ERROR_STREAM("No centroid for feature " << t);
      return false;
    }
    // Check that point is close enough to expected pose
    try
    {
      listener_.transformPoint(trackers_[t].frame_, ros::Time(0), rgbd_pt,
                               rgbd_pt.header.frame_id, world_pt);
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
bool LedFinder::CloudDifferenceTracker::oprocess(
  std::vector<pcloud_> cloud,
  std::vector<pcloud_> prev,
  double weight)
{
  //cv_bridge image pointers
  std::vector<cv_bridge::CvImagePtr> cloud_image_ptr;
  std::vector<cv_bridge::CvImagePtr> prev_image_ptr;

  cv::Mat cloud_bits;
  cv::Mat prev_bits;

  //function call for initial processing to convert to cv::Mat
  convert2CvImagePtr(cloud, cloud_image_ptr);
  convert2CvImagePtr(prev, prev_image_ptr);
  ROS_INFO("%d",cloud_image_ptr[0]->image.rows);
  //perform a bitwise AND

  cv::Mat cloud_pix_weighed(cloud_image_ptr[0]->image.rows, cloud_image_ptr[0]->image.cols, CV_8UC3, cv::Scalar(0,0,0));
  cv::Mat prev_pix_weighed(cloud_image_ptr[0]->image.rows, cloud_image_ptr[0]->image.cols, CV_8UC3, cv::Scalar(0,0,0));
  
  weightedSum(cloud_image_ptr, cloud_pix_weighed);
  weightedSum(prev_image_ptr, prev_pix_weighed);
  //debug_img(cloud_pix_weighed,"/tmp/mean/cloud_", 0, 0, 0);  
  debug_img(prev_pix_weighed,"/tmp/mean/prev_", 0, 0, 0);  
  cv::Mat diff_pix = cv::Mat(cloud_image_ptr[0]->image.rows, cloud_image_ptr[0]->image.cols, CV_8UC3, cv::Scalar(0,0,0));
//  cv::Mat diff1_pix = cv::Mat(cloud_image_ptr[0]->image.rows, cloud_image_ptr[0]->image.cols, CV_8UC3, cv::Scalar(0,0,0));
  //cv::absdiff(prev_pix_weighed, cloud_pix_weighed, diff_pix);
  //calculate the difference Image
  cv::Mat img;
  differenceImage(cloud_pix_weighed, prev_pix_weighed, diff_pix, img);
  debug_img(diff_pix,"/tmp/mean/diff_", 0, 0, 0);
/*  debug_img(thresh, "/tmp/mean/thresh_", 0, 0, 0);
  debug_img(cloud_image_ptr[0]->image, "/tmp/mean/image_", 0, 0, 0);*/
}

/*
 the function has to be modified for dark space by considering the point clouds
*/
void LedFinder::CloudDifferenceTracker::differenceImage(cv::Mat image1, cv::Mat image2, cv::Mat& diff_image, cv::Mat img)
{
  int count = 0;
  cv::Mat diff1_image, canny;
  float max_mean = -100;
  float max_dev = -100;
  cv::Scalar mean;
  cv::Scalar std_dev;
  float mean_val;
  float dev_val;
  cv::Point pt;
  cv::absdiff(image1, image2, diff1_image);

  //creating a blob template
  /*cv::Mat templte(13, 13, CV_8UC3, cv::Scalar(0,0,0));
  for(int i = 0; i < templte.rows; i++)
  {
    for(int j = 0; j < templte.cols; j++)
    {
      if( i > 3 && i < 9 )
      { 
        if( j > 3 && j < 9 )
        {
          templte.at<cv::Scalar>(j,i) = cv::Scalar(255,255,255);
        }
      }
    }
  }
  cv::Mat imcovar;
  
  double min_mahala_dist = 1000;
  cv::Mat tmp, tmp_thresh;
  cv::cvtColor(image1, tmp_thresh, CV_BGR2GRAY);
  cv::threshold(tmp_thresh, tmp, 175, 255, CV_THRESH_BINARY);
  cv::Canny( tmp, canny, 20, 20*3, 3 );
  //cv::cvtColor(tmp, img, CV_GRAY2BGR);
  for(int i = 50; i < image1.rows - 50; i++)
  {
    for(int j = 50; j < image1.cols - 50; j++)
    {
      cv::Rect rect(j, i, 15, 15);
      if(cv::countNonZero(tmp(rect) < 200 ) || cv::countNonZero(canny(rect)) > 9)
      {
        continue;
      }
      cv::Rect rect1(j, i, 10, 10);
      double mahala_curr =  cv::Mahalanobis(templte, templte, imcovar);
      if(mahala_curr < min_mahala_dist)
      {
        min_mahala_dist = mahala_curr;
        pt = cv::Point(j,i);
      }

    }
  }

  cv::rectangle(diff1_image, cv::Rect(pt.x, pt.y, 10, 10), cv::Scalar(0,0,255), 1 ,8);
  debug_img(diff1_image,"/tmp/mean/diff1_", 0, 0, 0);
  debug_img(tmp,"/tmp/mean/cloud_", 0, 0, 0);  
  debug_img(templte, "/tmp/mean/template_",0,0,0);
  debug_img(canny,"/tmp/mean/canny_", 0, 0, 0);  */

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

  std::vector<cv::Mat> weights(images.size());
  cv::Mat weight(images[0]->image.rows, images[0]->image.cols, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat norm_weight(images[0]->image.rows, images[0]->image.cols, CV_64F, cv::Scalar(0));
  cv::Mat weighted_image(images[0]->image.rows, images[0]->image.cols, CV_64F, cv::Scalar(0));
  cv::Mat tmp_weight(images[0]->image.rows, images[0]->image.cols, CV_8UC3, cv::Scalar(0,0,0));

  //Calculating the weight in a different loop as the we need the overall weight to normalize, 
  //if everything is done int he same loop the image saturates
  //TODO is to just use the cv::Array instead of cv::Mat and 
  //non-opencv options for multiplication and division  

  std::vector<cv::Mat> channels(3);

  for(int i = 0; i < images.size(); i++)
  {
    cv::add(tmp_weight,0.05*(images[i]->image), result);
    tmp_weight = result;
  }
//  cv::fastNlMeansDenoisingColoredMulti(img, result, 5, 5, 10, 10, 7, 21);

}

/*void LedFinder::CloudDifferenceTracker::planeFit()*/

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

/*/*bool LedFinder::CloudDifferenceTracker::oisFound(
  const pcloud_ cloud,
  double threshold)
{
  // Returns true only if the max exceeds threshold
  if (max_ < threshold)
  {
    return false;
  }*/

  // AND the current index is a valid point in the cloud.
/*  if (isnan(cloud->points[max_idx_].x) ||
      isnan(cloud->points[max_idx_].x) ||
      isnan(cloud->points[max_idx_].x))
  {
    return false;
  }

  return true;
}*/

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
