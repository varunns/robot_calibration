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

    debug_pic(cv_ptr->image, "/tmp/test/image_", 0, 0, 0);

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