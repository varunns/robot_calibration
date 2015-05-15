#include <math.h>
#include <robot_calibration/capture/led_finder.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

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

class TestImages
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  cv::Mat prev_image_;
  bool flag_;
  int i;
  std::vector<cv::Mat> images;
  typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud_;
public:
  TestImages()
  {
    sub_ = nh_.subscribe("/head_camera/depth_registered/points", 1, &TestImages::pcCB, this);
    pub_ = nh_.advertise<geometry_msgs::Point>("/color_diff", 10);
    flag_ = true;
    i = 0;
    

  }

  void pcCB(const sensor_msgs::PointCloud2ConstPtr& points)
  { 
    pcloud_ pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcloud_ pass_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*points, *pcl_cloud);

    // Create the filtering object
    std::vector<int> index_in;
    pcl::IndicesConstPtr index_rem;
    pcl::PassThrough<pcl::PointXYZRGB> pass (true);
    pass.setInputCloud(pcl_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(index_in);
    index_rem = pass.getRemovedIndices();
    //std::cout<<"former: "<<index_rem->size()<<std::endl;
    // Set all filtered out points to white
    for(int i = 0; i < index_rem->size(); i++)
    {
      pcl_cloud->points[index_rem->at(i)].x = NAN;
      pcl_cloud->points[index_rem->at(i)].y = NAN;
      pcl_cloud->points[index_rem->at(i)].z = NAN;
      pcl_cloud->points[index_rem->at(i)].r = 255;
      pcl_cloud->points[index_rem->at(i)].g = 255;
      pcl_cloud->points[index_rem->at(i)].b = 255;
    }
    
    /*plane fitting*/
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (pcl_cloud);
    seg.segment (*inliers, *coefficients);
     //extract indices\
    
    //std::vector<int>* index_out(new std::vector<int>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_filter(true);
    extract_filter.setInputCloud(pcl_cloud);
    extract_filter.setIndices (inliers);
    extract_filter.setNegative(true);

    std::vector<int> index_in1;
    pcl::IndicesConstPtr index_rem1;
    extract_filter.filter(index_in1);
    index_rem1 = extract_filter.getRemovedIndices();
    for(int i = 0; i < index_rem1->size(); i++)
    {
      pcl_cloud->points[index_rem1->at(i)].x = NAN;
      pcl_cloud->points[index_rem1->at(i)].y = NAN;
      pcl_cloud->points[index_rem1->at(i)].z = NAN;
      pcl_cloud->points[index_rem1->at(i)].r = 255;
      pcl_cloud->points[index_rem1->at(i)].g = 255;
      pcl_cloud->points[index_rem1->at(i)].b = 255;
    }


    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*pcl_cloud,ros_cloud);
    sensor_msgs::ImagePtr img(new sensor_msgs::Image);
    pcl::toROSMsg(ros_cloud, *img);

    cv_bridge::CvImagePtr cv;
    try
    {
      cv = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("sorry state : %s", e.what());
    }
    debug_img(cv->image, "/tmp/mean/img_",0,0,0);

  }


  void testAgain(  std::vector<cv::Mat>  images)
  {
    std::vector<cv::Mat> diss(images.size());
    cv::Mat all;
    for(int i = 1; i < images.size(); i++)
    {
    /*  for(int j = 0; j < (images[i].rows/2); j++)
      {
        cv::Mat tmp1 = images[i](cv::Rect(0, 2*j, 640, 2));
        cv::normalize(tmp1, tmp1, 0, 1, 32);
        cv::Mat tmp2 = images[i-1](cv::Rect(0, 2*j, 640, 2));
        cv::normalize(tmp2, tmp2, 0, 1, 32);
        cv::Mat diff = tmp1 - tmp2;
        std::cout<<cv::mean(diff)<<std::endl;
      }
      std::cout<<"*88888888888888888888888888888888888888888888888888888888888888"<<std::endl;*/
      cv::Mat diff = images[i] - images[i-1];
      diss.push_back(diff);
   
    }
   // eliminate_wedges(diss, all);

      
  }

/*
  void diffCalc(cv::Vec3b* p1, cv::Vec3b* p2)
  {
    for(int i = 0; i < 3; i++)
    {
      if( ((*p1)[i] - (*p2)[i]) > 0 )
      {
        (*p1)[i] = (*p1)[i] + (*p1)[i] - (*p2)[i];
      }
      else
      {
        (*p1)[i] = (*p2)[i]; 
      }
    }
  }*/

  void debug_img(cv::Mat image, std::string string_in, int k, int l, float diff)
  {

    ros::Time n = ros::Time::now();
    std::stringstream ss(std::stringstream::in | std::stringstream::out);
    ss<<string_in<<n<<"_"<<k<<l<<"_"<<diff<<".jpg";
    imwrite(ss.str(), image);
  }

/*  void process(std::vector<cv::Mat> images)
  {
    std::vector<cv::MatND> hist_base;
    std::vector<cv::Mat> hsv(images.size());
    
    for(int i = 0; i < images.size(); i++)
    {
      cv::MatND tmp_hist;
      //cv::cvtColor(images[i], hsv[i], CV_BGR2HSV);
      constHist(images[i], tmp_hist);
      hist_base.push_back(tmp_hist);
    }

    for(int i = 1; i < images.size(); i++)
    {
      double diff = cv::compareHist(hist_base[i], hist_base[i-1], CV_COMP_CORREL);
      std::cout<<diff<<std::endl;
    }

  }

  void constHist(cv::Mat image, cv::MatND& hist)
  {

    //drawing the histogram
    //int h_bins = 50; int s_bins = 60; 
    int r_bins = 64; int g_bins = 64; int b_bins =  64;
    //int histSize[] = { h_bins, s_bins };
    int histSize[] = { r_bins, g_bins, b_bins };
    //int hist_size[] = {h_bins, s_bins};
    //float h_ranges[] = {0,180};
    //float s_ranges[] = {0,256};
    float r_ranges[] = {0,255};
    float g_ranges[] = {0,255};
    float b_ranges[] = {0,255};


    //const float* ranges[] = {h_ranges, s_ranges};
    const float* ranges[] = {r_ranges, g_ranges, b_ranges};
    int channels[] = {0,1,2};

    cv::calcHist(&image, 1, channels, cv::Mat(), hist, 2, histSize, ranges, true, false);
    cv::normalize(hist, hist, 0, 1, 32, -1, cv::Mat());
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
*/
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_images");
  TestImages obj;
  ros::spin();
  return 0;
}