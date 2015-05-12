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
  int i;

public:
  TestImages()
  {
    sub_ = nh_.subscribe("/head_camera/rgb/image_rect_color", 1, &TestImages::imageCB, this);
    flag_ = true;
    i = 0;
  }

  void imageCB(const sensor_msgs::ImageConstPtr& image)
  { 
    i++;

    if(i < 10)
    {
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }

    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("sorry state : %s", e.what());
    }
    cv::Mat yuv_image;
    if(flag_)
    {
      std::vector<cv::Mat> channels(3);
      cv::cvtColor(cv_ptr->image, yuv_image, CV_BGR2YUV);
      cv::split(yuv_image, channels);
      prev_image_ = channels[0];
      flag_ = false;
    }
    cv::Mat curr_gray, prev_gray, no_illuminance,no_y, canny;
    //cv::cvtColor(prev_image_, prev_gray, CV_BGR2GRAY);
   // cv::cvtColor(cv_ptr->image, curr_gray, CV_BGR2GRAY);
    std::vector<cv::Mat> channels(3);
    cv::cvtColor(cv_ptr->image, yuv_image, CV_BGR2YUV);
    cv::split(yuv_image, channels);
    cv::Mat diff = channels[0] - prev_image_;
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
   // diffHist(diff_image);
    
    debug_pic(channels[0], "/tmp/test/image_", 0, 0, 0);

  }

  void diffHist(cv::Mat image)
  {
    int histSize = 16;
    float range[] = {0, 256};
    const float* histRange = { range };
    cv::Mat diff_hist;
    bool uniform = true; bool accumulate = false;
    cv::calcHist(&image, 1, 0, cv::Mat(), diff_hist, 1, &histSize, &histRange, uniform, accumulate);

    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );
    cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0) );
    cv::normalize(diff_hist, diff_hist, 0, histImage.rows, 32, -1, cv::Mat() );
    for( int i = 1; i < histSize; i++ )
    {
      line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(diff_hist.at<float>(i-1)) ) ,
                       cv::Point( bin_w*(i), hist_h - cvRound(diff_hist.at<float>(i)) ),
                       cv::Scalar( 255, 0, 0), 2, 8, 0  );
    }
    //double diff = cv::compareHist
    debug_pic(histImage,"/tmp/test/hist_image_", 0, 0, 0);
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