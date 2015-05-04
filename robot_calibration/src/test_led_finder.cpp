/*
 * Copyright 2015 Fetch Robotics Inc.
 * Author: Sriramvarun Nidamarthy
 */
#include <iostream>

#include <ros/ros.h>
#include <robot_calibration/capture/led_finder.h>
#include <robot_calibration/capture/feature_finder.h>

int main(int argc,char** argv)
{
  ros::init(argc, argv, "test_led_finder");
  ros::NodeHandle nh;
  robot_calibration::FeatureFinder * finder = new robot_calibration::LedFinder(nh);
  robot_calibration_msgs::CalibrationData msg;
  finder->find(&msg);
  ros::spin();
  return 0;
}
