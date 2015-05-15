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
  std::vector<cv::Mat> image_in;

public:
  TestImages()
  {
    sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &TestImages::imageCB, this);
    flag_ = true;
    i = 0;
  }

  void imageCB(const sensor_msgs::PointCloud2::Ptr& ros_cloud)
  {     
    sensor_msgs::Image::Ptr ros_image(new sensor_msgs::Image);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr.reset(new cv_bridge::CvImage);

    pcl::toROSMsg(*ros_cloud, *ros_image);
    try
    {
      cv_ptr = cv_bridge::toCvCopy(*ros_image, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cloud_rosimage is sorry: %s ", e.what());
    }
    cv::Mat mat;
    (cv_ptr->image).convertTo(mat, CV_32SC3);
    image_in.push_back(mat);
    if(image_in.size() > 4)
    {
      testFunc(image_in);
      image_in.clear();
    }
  }

  void testFunc(std::vector<cv::Mat> images)
  {
    cv::Mat tmp_img = cv::Mat::zeros(images[0].rows, images[0].cols, CV_32SC3);
    std::cout<<"I am here"<<std::endl;
    for(int i = 0; i < images.size(); i++)
    {
      tmp_img = tmp_img+images[i];
    }
    std::cout<<"I am outta here"<<std::endl;
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