#include <math.h>
#include <robot_calibration/capture/led_finder.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <queue>
#include <sstream>

class TestImages
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  cv::Mat prev_image_;
  bool flag_;

public:
  TestImages()
  {
    sub_ = nh_.subscribe("/head_camera/rgb/image_rect_color", 1, &TestImages::imageCB, this);
    flag_ = true;
  }

  void imageCB(const sensor_msgs::ImageConstPtr& image)
  { 
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }

    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("sorry state : %s", e.what());
    }

    if(flag_)
    {
      prev_image_ = cv_ptr->image;
      flag_ = false;
    }
    cv::Mat gray, no_illuminance, yuv_image, no_y;
    cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
    //std::vector<cv::Mat> channels(3);
    //cv::cvtColor(cv_ptr->image, yuv_image, CV_BGR2YUV);
    //cv::split(yuv_image, channels);
    //cv::equalizeHist(channels[0], channels[0]);
    //std::vector<cv::Mat> new_channels(3);
   /* cv::Mat tmp(channels[0].rows, channels[0].cols, CV_8UC1);
    tmp.setTo(cv::Scalar(0));
    new_channels[0] = tmp;
    new_channels[1] = channels[1];
    new_channels[2] = channels[2];*/
    //cv::merge(channels, no_y);
    //cv::cvtColor(no_y, no_y, CV_YUV2BGR);
    //cv::normalize(, no_y, 0, 1, 32);
    debug_pic(gray, "/tmp/test/image_", 0, 0, 0);

  }

  void debug_pic(cv::Mat image, std::string string_in, int k, int l, float diff)
 {
  ros::Time n = ros::Time::now();
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss<<string_in<<n<<"_"<<k<<l<<"_"<<diff<<".jpg";
  imwrite(ss.str(), image);
 }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_images");
  TestImages obj;
  ros::spin();
  return 0;
}