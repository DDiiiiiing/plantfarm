#include <vector>
#include <cmath>
#include <string>
#include "CTPL/ctpl.h"

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <plantfarm/YoloResult.h>
#include <plantfarm/YoloResultList.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

class SeperateAbnormal
{
 public:
  SeperateAbnormal(std::string rgb_topic, std::string depth_topic, std::string yolo_topic, std::string plane_topic)
  {
    ROS_INFO("wait for realsense");
    while(nh_.hasParam("/camera/realsense2_camera/serial_no")!=1);
    
    ROS_INFO("seperate abnormal node start!!!");
    // subscriber
    rgb_sub_.subscribe(nh_, rgb_topic, 1);
    depth_sub_.subscribe(nh_, depth_topic, 1);
    sync_.reset(new Sync(MySyncPolicy(10), rgb_sub_, depth_sub_));
    sync_->registerCallback(boost::bind(&SeperateAbnormal::rs_callback, this, _1, _2));
    yolo_sub = nh_.subscribe(yolo_topic, 1, &SeperateAbnormal::yolo_callback, this);
    // publisher
    nh_.advertise<std_msgs::Int32MultiArray>(plane_topic, 500);

    // get image size
    nh_.getParam("/camera/realsense2_camera/color_width", image_w);
    nh_.getParam("/camera/realsense2_camera/color_height", image_h);
    if(image_w==-1 || image_h==-1)
    {
      ROS_ERROR("please check realsense in connected in USB3.0 mode");
      throw "please check realsense in connected in USB3.0 mode";
    }
  }
  ~SeperateAbnormal()
  {
    ROS_INFO("seperate abnormal node end!!!");
    cv::destroyAllWindows();
  }

  // subscribe rgb, depth image topic in sync
  // with contour info from yolo, masking the area of depth image
  void rs_callback(const sensor_msgs::ImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth)
  {
    cv_bridge::CvImagePtr rgb_ptr, depth_ptr;
    try
    {
      rgb_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
      depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    abnormal_depths_.clear();
    for(int i=0; i<abnormal_contours.size(); i++)
    {
      cv::Mat contour_mask = cv::Mat::zeros(depth_ptr->image.size(), CV_16UC1);
      drawContours(contour_mask, abnormal_contours, i, cv::Scalar(int(std::pow(2,16)-1)), -1);
      cv::Mat abnormal_depth = cv::Mat::zeros(contour_mask.size(), CV_16UC1);
      cv::bitwise_and(depth_ptr->image*16, contour_mask, abnormal_depth);
      abnormal_depths_.push_back(abnormal_depth);

      std::string num = std::to_string(i);
      cv::imshow("ab dep"+num, abnormal_depth);
    }
    
    cv::imshow("rgb", rgb_ptr->image);
    cv::imshow("d16", depth_ptr->image * 16);
    cv::waitKey(1);
  }

  // subscribe yolo topic
  // get contour of the abnormal
  void yolo_callback(const plantfarm::YoloResultListPtr &yolo)
  {
    auto yolo_returns = yolo->ret;
    abnormal_contours.clear();

    for(auto yolo_ret : yolo_returns)
    {
      // cls
      // 0 : abnormal
      // 1 : plant
      int16_t cls = yolo_ret.cls;
      if(cls != 0) continue;
      if(yolo_ret.x.size() != yolo_ret.y.size()) throw std::invalid_argument("the nuber of x and y point different");

      static std::vector<cv::Point> contour;
      for(int i=0; i<yolo_ret.x.size(); i++)
      {
        static cv::Point temp;
        temp.x = int(yolo_ret.x[i]*image_w);
        temp.y = int(yolo_ret.y[i]*image_h);
        contour.push_back(temp);
      }
      abnormal_contours.push_back(contour);
      contour.clear();
    }
  }
  inline std::vector<cv::Mat> get_depth(){return abnormal_depths_;}
  ros::Publisher plane_pub;

 private:
  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  ros::Subscriber yolo_sub;

  int image_w, image_h;
  cv::Mat depth_image;
  std::vector<std::vector<cv::Point>> abnormal_contours;
  std::vector<cv::Mat> abnormal_depths_; //vectors of input depth data. masked depth image.
};

pcl::PointCloud<pcl::PointXYZ> depth2pointcloud(cv::Mat depth_image)
{
  pcl::PointCloud<pcl::PointXYZ> ret;
  return ret;
}

std::vector<int> ransac(pcl::PointCloud<pcl::PointXYZ> cloud)
{
  std::vector<int> ret;
  return ret;
}

std::vector<int> depth2plane(int id, cv::Mat depth_image)
{
  auto cloud = depth2pointcloud(depth_image);
  std::vector<int> plane = ransac(cloud);
  return plane;
}

int main(int argc, char **argv)
{
  // subscribe
  std::string rgb_topic = "/camera/color/image_raw";
  std::string depth_topic = "/camera/aligned_depth_to_color/image_raw";
  std::string yolo_topic = "/yolov5/result";
  // publish
  std::string plane_topic = "/planes";
  std_msgs::Int32MultiArray plane_coeff;

  ros::init(argc, argv, "serperate_abnormal_node");
  ros::Rate rate(30);
  SeperateAbnormal sep_abn(rgb_topic, depth_topic, yolo_topic, plane_topic);
  
  ctpl::thread_pool p(int(std::thread::hardware_concurrency())); //make worker as much as the number of cpus
  std::vector<cv::Mat> abnormal_depths; //vectors of input depth data. masked depth image.
  std::vector<std::future<std::vector<int>>> abnormal_planes; //vectors of output plane coefficient.

  
  while(ros::ok())
  {
    abnormal_depths=sep_abn.get_depth();
    abnormal_planes.clear();
    for(auto& d_img : abnormal_depths) //calc pointcloud conversion and ransac with pool
    {
      abnormal_planes.push_back(p.push(std::ref(depth2plane), d_img));
    }

    try
    {
      plane_coeff.data.clear();  
      for(auto plane : abnormal_planes) //extract data from pool return
      {
        plane_coeff.data = plane.get();
        
        sep_abn.plane_pub.publish(plane_coeff);
      }
    }
    catch (std::exception &e)
    {
      std::cout<<"caught exception"<<std::endl;
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
