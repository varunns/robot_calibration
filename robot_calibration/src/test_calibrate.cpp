
#include <fstream>
#include <ctime>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>

#include <std_msgs/String.h>
#include <robot_calibration_msgs/CalibrationData.h>

#include <robot_calibration/capture/chain_manager.h>
#include <robot_calibration/capture/checkerboard_finder.h>
#include <robot_calibration/capture/led_finder.h>
#include <robot_calibration/depth_camera.h>

#include <camera_calibration_parsers/parse.h>
#include <robot_calibration/ceres/optimizer.h>
#include <robot_calibration/depth_camera.h>

#include <boost/foreach.hpp> 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_calibrate");
  ros::NodeHandle nh("~");

  bool verbose;
  nh.param<bool>("verbose", verbose, false);

  // The calibration data
  std_msgs::String description_msg;
  std::vector<robot_calibration_msgs::CalibrationData> data;

  // What bag to use to load calibration poses out of (for capture)
  std::string pose_bag_name("calibration_poses.bag");
  if (argc > 1)
    pose_bag_name = argv[1];

  // All ROS callbacks are processed in a separate thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  if (pose_bag_name.compare("--from-bag") != 0)
  {
    // no name provided for a calibration bag file, must do capture
    robot_calibration::ChainManager chain_manager_(nh);
    robot_calibration::FeatureFinder * finder_;
    if (nh.hasParam("led_finder"))
    {
      finder_ = new robot_calibration::LedFinder(nh);
    }
    else
    {
      finder_ = new robot_calibration::CheckerboardFinder(nh);
    }
  }
}