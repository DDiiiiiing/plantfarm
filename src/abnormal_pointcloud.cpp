#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <boost/array.hpp>
#include "thread-pool/BS_thread_pool.hpp"
#include <thread>
// ROS Headers
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <plantfarm/YoloResult.h>
#include <plantfarm/YoloResultList.h>
// OpenCV Headers
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
// PCL Headers
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>

// https://github.com/bshoshany/thread-pool
// generate thread pool with a number of cpus
BS::thread_pool pool(std::thread::hardware_concurrency());

class SeperateAbnormal
{
public:
  SeperateAbnormal(std::string depth_topic, std::string intrinsic_topic, std::string yolo_topic, std::string pointcloud_topic)
  {
    ROS_INFO("wait for realsense");
    while(nh_.hasParam("/camera/realsense2_camera/serial_no")!=1);
    
    ROS_INFO("abnormal pointcloud node start!!!");
    // subscriber
    depth_sub       = nh_.subscribe(depth_topic, 5, &SeperateAbnormal::rs_callback, this);
    intrinsic_sub   = nh_.subscribe(intrinsic_topic, 1, &SeperateAbnormal::intrinsics_callback, this);    
    yolo_sub        = nh_.subscribe(yolo_topic, 10, &SeperateAbnormal::yolo_callback, this);
    empty_sub       = nh_.subscribe("/empty", 1, &SeperateAbnormal::empty_callback, this);
    // publisher
    pointcloud_pub  = nh_.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 5);
    // get image size
    nh_.getParam("/camera/realsense2_camera/color_width", image_w);
    nh_.getParam("/camera/realsense2_camera/color_height", image_h);
    if(image_w==-1 || image_h==-1)
    {
      ROS_ERROR("please check realsense in connected in USB3.0 mode");
      throw "please check realsense in connected in USB3.0 mode";
    }
    
    //for test
    // image_w=1280;
    // image_h=720;
  }
  ~SeperateAbnormal()
  {
    ROS_INFO("abnormal pointcloud node end!!!");
    cv::destroyAllWindows();
  }

  // subscribe depth image and intrinsic topic in sync
  // with contour info from yolo, masking the area of depth image
  void rs_callback(const sensor_msgs::ImageConstPtr &depth)
  {
    // read depth image
    cv_bridge::CvImagePtr depth_ptr;
    try
    {
      depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1);
      depth_image = depth_ptr->image;
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
  // subscribe yolo topic
  // get contour of the abnormal
  void yolo_callback(const plantfarm::YoloResultListPtr &yolo)
  {
    auto yolo_returns = yolo->ret;
    abnormal_contours.clear();
    if(abnormal_contours.capacity() > 100) abnormal_contours.shrink_to_fit(); // shrink memory
    
    // std::cout<<"yolo returns size : "<<yolo_returns.size()<<std::endl;
    for(auto yolo_ret : yolo_returns)
    {
      // cls
      // 0 : abnormal
      // 1 : plant
      int16_t cls = yolo_ret.cls;
      // std::cout<<"cls : "<<cls<<" size : "<<yolo_ret.x.size()<<std::endl;
      
      if(cls != 0) continue; // only abnormal
      if(yolo_ret.x.size() <= 2) continue; //ignore empty contour
      if(yolo_ret.x.size() != yolo_ret.y.size()) throw std::invalid_argument("the nuber of x and y point different");

      static std::vector<cv::Point> contour;
      contour.clear();
      for(int i=0; i<yolo_ret.x.size(); i++)
      {
        static cv::Point temp;
        temp.x = int(yolo_ret.x[i]*image_w);
        temp.y = int(yolo_ret.y[i]*image_h);
        contour.push_back(temp);
      }
      abnormal_contours.push_back(contour);
    }
  }
  // subscribe intrinsic topic
  void intrinsics_callback(const sensor_msgs::CameraInfoPtr &intrinsic)
  {
    K = intrinsic->K;
    D = intrinsic->D;
  }
  // subscribe empty topic. this is for 
  void empty_callback(const std_msgs::EmptyPtr &empty)
  {
    pool.push_task(&SeperateAbnormal::image_pipeline, this, depth_image, abnormal_contours);
    // image_pipeline(depth_image, abnormal_contours);
  }
  // make mask image with contour
  cv::Mat make_contour_mask(cv::Mat &depth_image, std::vector<std::vector<cv::Point>> contour, int idx)
  {
    cv::Mat contour_mask = cv::Mat::zeros(depth_image.size(), CV_16UC1);
    drawContours(contour_mask, contour, idx, cv::Scalar(int(std::pow(2,16)-1)), -1);
    cv::Mat abnormal_depth = cv::Mat::zeros(contour_mask.size(), CV_16UC1);
    cv::bitwise_and(depth_image, contour_mask, abnormal_depth);

    // cv::imshow("depth_image", depth_image);
    // cv::imshow("contour_mask", contour_mask);
    // cv::imshow("abnormal_depth", abnormal_depth);
    // cv::waitKey(1);
    return abnormal_depth;
  }
  // convert depth image to pointcloud and publish
  pcl::PointCloud<pcl::PointXYZ> depth_to_pointcloud(cv::Mat depth_image)
  {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    int width = depth_image.cols;
    int height = depth_image.rows;
    cloud.clear();
    // cloud.width = width;
    // cloud.height = height;
    cloud.is_dense = false;
    // cloud.points.resize(cloud.width * cloud.height);

    // Get the camera intrinsics
    double fx = K.at(0);
    double fy = K.at(4);
    double cx = K.at(2);
    double cy = K.at(5);

    for (int v = 0; v < height; v++)
    {
      for (int u = 0; u < width; u++)
      {
        // https://github.com/IntelRealSense/librealsense/blob/5e73f7bb906a3cbec8ae43e888f182cc56c18692/include/librealsense2/rsutil.h#L46
        // get a data of an element from depth image
        uint16_t depth = depth_image.at<uint16_t>(v, u);
        // Skip over pixels with a depth value of zero, which is used to indicate no data
        if(depth==0) continue;

        float x = (u - cx) / fx;
        float y = (v - cy) / fy;

        // Apply distortion
        // float r2 = x * x + y * y;
        // float f = 1 + D.at(0) * r2 + D.at(1) * r2 * r2 + D.at(4) * r2 * r2 * r2;
        // float ux = x * f + 2 * D.at(2) * x * y + D.at(3) * (r2 + 2 * x * x);
        // float uy = y * f + 2 * D.at(3) * x * y + D.at(2) * (r2 + 2 * y * y);
        // x = ux;
        // y = uy;

        pcl::PointXYZ point;
        point.x = float(depth * x / 1000.0);
        point.y = float(depth * y / 1000.0);
        point.z = float(depth / 1000.0);

        // cloud.points[v * width + u] = point;
        cloud.push_back(point); 
        // std::cout<<"u : "<<u<<" | v : "<<v<<" | x : "<<point.x<<" | y : "<<point.y<<" | z : "<<point.z<<""<<std::endl;
      }
    }

    // print_pc(cloud);
    return cloud;
  }
  // publish pointcloud
  void publish_pointcloud(pcl::PointCloud<pcl::PointXYZ> cloud)
  {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = "camera_link";
    cloud_msg.header.stamp = ros::Time::now();

    pointcloud_pub.publish(cloud_msg);
  }
  // image pipline
  void image_pipeline(cv::Mat depth_image, std::vector<std::vector<cv::Point>> contours)
  {
    if(depth_image.empty()) return;
    if(contours.empty()) return;
    for(int i=0; i<contours.size(); i++)
    {
      cv::Mat abnormal_depth = make_contour_mask(depth_image, contours, i);
      pcl::PointCloud<pcl::PointXYZ> cloud = depth_to_pointcloud(abnormal_depth);
      publish_pointcloud(cloud);
    }
  }
  template <class T>
  void print_pc(pcl::PointCloud<T>& cloud)
  {
    int count = 0;
    if(count>0) return;
    for (const auto& pt: cloud.points){
        cout << count++ << ": ";
        cout <<pt.x << ", "<<pt.y << ", "<< pt.z << endl;
    }
  }
private:
  ros::NodeHandle nh_; // local node handler

  ros::Subscriber intrinsic_sub;    // intrinsic
  ros::Subscriber depth_sub;        // depth image
  ros::Subscriber yolo_sub;         // yolo result
  ros::Subscriber empty_sub;        // empty topic
  ros::Publisher pointcloud_pub;    // pointcloud

  int image_w, image_h;
  boost::array<double, 9> K;        // camera intrinsics
  std::vector<double> D;                 // distortion coefficients
  cv::Mat depth_image;
  std::vector<std::vector<cv::Point>> abnormal_contours;
};

int main(int argc, char **argv)
{
  // subscribe
  std::string depth_topic       = "/camera/aligned_depth_to_color/image_raw";
  std::string intrinsic_topic   = "/camera/aligned_depth_to_color/camera_info";
  std::string yolo_topic        = "/yolov5/result";
  // publish
  std::string pc_topic          = "/abnormal_pointcloud";
  
  // ros base
  ros::init(argc, argv, "serperate_abnormal_node");
  ros::NodeHandle nh; //main node handler
  ros::Publisher empty_pub = nh.advertise<std_msgs::Empty>("/empty", 1); //publish empty topic
  SeperateAbnormal sep_abn(depth_topic, intrinsic_topic, yolo_topic, pc_topic);
  ros::Rate rate(30); // rate of empty publisher

  while(ros::ok())
  {
    empty_pub.publish(std_msgs::Empty());
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

