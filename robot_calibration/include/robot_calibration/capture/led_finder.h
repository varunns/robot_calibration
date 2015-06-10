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

#ifndef ROBOT_CALIBRATION_CAPTURE_LED_FINDER_H
#define ROBOT_CALIBRATION_CAPTURE_LED_FINDER_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <robot_calibration/capture/feature_finder.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <robot_calibration_msgs/GripperLedCommandAction.h>
#include <actionlib/client/simple_action_client.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <queue>
  
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud_;

namespace robot_calibration
{

/** @brief This class processes the point cloud input to find the LED. */
class LedFinder : public FeatureFinder
{
  /** @brief Internally used within LED finder to track each of several LEDs. */
  struct CloudDifferenceTracker
  {


        //struct to save all the contours belonging to the same tracker in a 
    struct TrackContours
    {
      //to be filled first time
      bool first_time;
      pcl::PointXYZRGB pt3d;
      int tracker_id;
      //to be filled in process
      std::vector<std::vector<cv::Point> > all_contours;
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pclouds;
      std::vector<cv::Mat> diff_images;
      std::vector<cv::Mat> rgb_image;
      geometry_msgs::PointStamped estimate_led;

      TrackContours()
      {
        first_time = false;
      }
      TrackContours(bool flag, pcl::PointXYZRGB pt, int id)
      {
        first_time = flag;
        pt3d = pt;
  tracker_id = id;
      }
    };

    typedef boost::shared_ptr<TrackContours> TrackContoursPtr;

    CloudDifferenceTracker(std::string frame, double x, double y, double z);

    // Weight should be +/- 1 typically
    bool process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev,
                 double weight);

    // Have we found the LED?
    bool isFound(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                 double threshold);

    // Gives a refined centroid using multiple points
    bool getRefinedCentroid(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                            geometry_msgs::PointStamped& point);

    // Reset the tracker
    void reset(size_t size);

    // Looking at clouds to Obtain max_cloud and diff cloud
    bool getDifferenceCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev,
                            cv::Mat& image,
                            double weight);

    /* Overloaded/un-overloaded left functions added by varun*/
    bool oprocess(pcl::PointXYZRGB pt,
                  int tracker_id,
                  std::vector<pcloud_> cloud,
                  std::vector<pcloud_> prev,
                  std::vector<TrackContoursPtr>& trackContourPtr,
                  bool check_first_time
                  );

    // function to determine possible contours
    void possibleContours(cv::Mat& diff_image, 
                          std::vector<std::vector<cv::Point> >& centers);




    // Calculate the weighted sum of the images
    void weightedSum(std::vector<cv_bridge::CvImagePtr>& images, 
                     cv::Mat& result);

    // Convert pointcloud to cv_image ptr
    void convert2CvImagePtr(std::vector<pcloud_>& pcl_cloud, 
                            std::vector<cv_bridge::CvImagePtr>& cv_ptr);

    // trying out looking for contours
    bool getContourCircle(cv::Mat& cloud,
                          geometry_msgs::PointStamped& point);

    /*for debuggin*/
    void debug_img(cv::Mat image, 
                   std::string string_in,
                   int k,
                   int l,
                   float diff);



    /*struct holding all the contours and their distance */
    struct ContourDist
    {
      std::vector<cv::Point> contour;
      float dist;
      
      ContourDist()
      {

      }

      ContourDist(std::vector<cv::Point> contour_in, float mean_distance)
      {
        contour = contour_in;
        dist = mean_distance;
      }
    };

    //struct for combination queue sorting rule, min at the top
    typedef boost::shared_ptr<ContourDist> ContourDistPtr;
    struct CompareContourDist
    {
      bool operator()(ContourDistPtr a, ContourDistPtr b)
      {
        return(a->dist > b->dist);
      }
    };
    
    int count_;
    std::vector<double> diff_;
    double max_;
    int max_idx_;
    std::string frame_;  // frame of led coordinates
    geometry_msgs::Point point;  //coordinates of led this is tracking
  };
  
 
  typedef actionlib::SimpleActionClient<robot_calibration_msgs::GripperLedCommandAction> LedClient;

public:
  LedFinder(ros::NodeHandle & n);
   static bool debug_flag_;
  /**
   * \brief Attempts to find the led in incoming data.
   * \param msg CalibrationData instance to fill in with led point information.
   * \returns True if point has been filled in.
   */
  bool find(robot_calibration_msgs::CalibrationData * msg);
   
  //getting the common contours among different frames to obtain the most repeated nd hence the deesired frame
  void getCandidateRoi(CloudDifferenceTracker::TrackContoursPtr& tracker_in);
  void getCandidateRoi2(CloudDifferenceTracker::TrackContoursPtr& tracker_in, pcl::PointXYZRGB& led_pt);
  void findProb(cv::Mat img);


  static bool getDebug()
  {
    return debug_flag_;
  }

  static void setDebug(bool flag)
  {
    debug_flag_ = flag;
  }

  void localDebugImage(cv::Mat img, std::string str);
 // void getMostAccuratePt(cv::Mat img, std::vector<cv::Point> contour, cv::Point& pt);

private:

struct Hist
{
  std::vector<cv::Point> pts;
  pcl::PointCloud<pcl::PointXYZRGB> pt3;
};

  struct PointsAndDist
  {
    pcl::PointXYZRGB pt_led;
    pcl::PointXYZRGB pt_tf;
    float dist;
    PointsAndDist(pcl::PointXYZRGB pt1, pcl::PointXYZRGB pt2)
    {
      pt_led = pt1;
      pt_tf = pt2;
      dist = std::sqrt(pow((pt1.x-pt2.x),2)+pow((pt1.x-pt2.y),2));
    }
  };

  typedef boost::shared_ptr<PointsAndDist> PointsAndDistPtr;
  struct ComparePointsAndDist
  {
    bool operator()(PointsAndDistPtr a, PointsAndDistPtr b)
    {
      return(a->dist > b->dist);
    }
  };

  void cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  bool waitForCloud();

  ros::Subscriber subscriber_;  /// Incoming sensor_msgs::PointCloud2
  ros::Publisher publisher_;  /// Outgoing sensor_msgs::PointCloud2
  boost::scoped_ptr<LedClient> client_;

  bool waiting_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr_;
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clouds_ptr_;

  std::vector<CloudDifferenceTracker> trackers_;
  std::vector<uint8_t> codes_;

  tf::TransformListener listener_;

  /*
   * ROS Parameters
   */
  double max_error_;    /// Maximum distance led can be from expected pose
  double max_inconsistency_;

  double threshold_;    /// Minimum value of diffs in order to trigger that this is an LED
  int max_iterations_;  /// Maximum number of cycles before we abort finding the LED

  bool output_debug_;   /// Should we output debug image/cloud?
  cv::Mat diff_image_;
  float led_duration_; //led duration.. to keep led on for so many secs
};

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_CAPTURE_LED_FINDER_H 