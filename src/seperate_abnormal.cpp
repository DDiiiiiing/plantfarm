#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include "thread-pool/BS_thread_pool.hpp"
// ROS Headers
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <plantfarm/YoloResult.h>
#include <plantfarm/YoloResultList.h>
// Message Filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// OpenCV Headers
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>

// depth to pointcloud conversion thread pointcloud_pool
BS::thread_pool pointcloud_pool(4); // number of workers
// mask depth image with contour
BS::thread_pool contour_pool(4);

class SeperateAbnormal
{
 public:
  SeperateAbnormal(std::string depth_topic, std::string intrinsic_topic, std::string yolo_topic, std::string pointcloud_topic)
  {
    ROS_INFO("wait for realsense");
    while(nh_.hasParam("/camera/realsense2_camera/serial_no")!=1);
    
    ROS_INFO("seperate abnormal node start!!!");
    // subscriber
    depth_sub = nh_.subscribe(depth_topic, 5, &SeperateAbnormal::rs_callback, this);
    intrinsic_sub = nh_.subscribe(intrinsic_topic, 1, &SeperateAbnormal::intrinsics_callback, this);    
    yolo_sub = nh_.subscribe(yolo_topic, 10, &SeperateAbnormal::yolo_callback, this);
    // publisher
    pc_topic_ = pointcloud_topic;

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
    ROS_INFO("seperate abnormal node end!!!");
    cv::destroyAllWindows();
  }
  void advertise()  {pointcloud_pub = nh_.advertise<sensor_msgs::PointCloud2>(pc_topic_, 5);}

  // subscribe depth image and intrinsic topic in sync
  // with contour info from yolo, masking the area of depth image
  void rs_callback(const sensor_msgs::ImageConstPtr &depth)
  {
    // read depth image
    cv_bridge::CvImagePtr depth_ptr;
    try
    {
      depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // mask depth image with contour
    if(abnormal_contours.size()==0) {
      ROS_INFO("no abnormal contour");
      std::cout<<static_cast<int>(abnormal_contours.size())<<std::endl;
      return;
    }
    auto ab_contours = &abnormal_contours;
    BS::multi_future<std::vector<cv::Mat>> abnormal_depths_future = contour_pool.parallelize_loop(static_cast<int>(ab_contours->size()),
          [&depth_ptr, &ab_contours](const int a, const int b) -> std::vector<cv::Mat>
          {
            std::vector<cv::Mat> abnormal_depths;
            abnormal_depths.clear();
            for(int i=a; i<b; ++i)
            {
              // std::cout<<"i "<<i<<" contour size "<<(*ab_contours).at(i).size()<<std::endl;
              cv::Mat contour_mask = cv::Mat::zeros(depth_ptr->image.size(), CV_16UC1);
              drawContours(contour_mask, *ab_contours, i, cv::Scalar(int(std::pow(2,16)-1)), -1);
              cv::Mat abnormal_depth = cv::Mat::zeros(contour_mask.size(), CV_16UC1);
              cv::bitwise_and(depth_ptr->image*16, contour_mask, abnormal_depth);
        
              abnormal_depths.push_back(abnormal_depth);
            }
            return abnormal_depths;
          });
    abnormal_contours.clear();
    if(abnormal_contours.capacity() > 100) abnormal_contours.shrink_to_fit(); // shrink memory

    // for display
    int j = 0;
    auto adfg = abnormal_depths_future.get();
    for (auto adf : adfg)
    {
      for (auto ad : adf)
      {
        pointcloud_pool.push_task(&SeperateAbnormal::depth_to_pointcloud, this, ad);
        // cv::imshow("abnormal_depth"+std::to_string(j++), ad);
      }
    }

    cv::imshow("masked", adfg[0][0]);
    // std::cout<<adfg[0][0]<<std::endl;
    cv::imshow("d16", depth_ptr->image * 16);
    cv::waitKey(0);
  }

  // subscribe yolo topic
  // get contour of the abnormal
  void yolo_callback(const plantfarm::YoloResultListPtr &yolo)
  {
    auto yolo_returns = yolo->ret;
    for(auto yolo_ret : yolo_returns)
    {
      // cls
      // 0 : abnormal
      // 1 : plant
      int16_t cls = yolo_ret.cls;
      if(cls != 0) continue;
      if(yolo_ret.x.size() <= 2) continue; //ignore empty contour
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
    std::cout<<"ac sz "<<static_cast<int>(abnormal_contours.size())<<std::endl;
  }

  void intrinsics_callback(const sensor_msgs::CameraInfoPtr &intrinsic)
  {
    K = intrinsic->K;
  }

  // convert depth image to pointcloud and publish
  inline void depth_to_pointcloud(cv::Mat depth_image)
  {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    int width = depth_image.cols;
    int height = depth_image.rows;
    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    // camera intrinsic
    while(K.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    float fx = K[0];
    float fy = K[4];
    float cx = K[2];
    float cy = K[5];
 
    for (int v = 0; v < height; v++)
    {
      for (int u = 0; u < width; u++)
      {
        // Get the depth of the current pixel
        float depth = depth_image.at<float>(v, u);
        // Skip invalid pixels (depth = 0)
        if (depth <= 1.0e-5) continue;

        // Convert the pixel to a point in the point cloud
        pcl::PointXYZ point;
        point.x = (u - cx) * depth / fx;
        point.y = (v - cy) * depth / fy;
        point.z = depth;
        cloud->at(v * width + u) = point;
      }
    }

    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "abnormal";
    cloud_msg.header.stamp = ros::Time::now();

    pointcloud_pub.publish(cloud_msg);
    // ROS_INFO("publish pointcloud");
  }

private:
  ros::NodeHandle nh_; // local node handle

  ros::Subscriber intrinsic_sub; // intrinsic
  ros::Subscriber depth_sub; // depth image
  ros::Subscriber yolo_sub; // yolo result
  ros::Publisher pointcloud_pub;
  std::string pc_topic_;

  int image_w, image_h;
  boost::array<double, 9UL> K; // intrinsic matrix
  std::vector<std::vector<cv::Point>> abnormal_contours;
};

int main(int argc, char **argv)
{
  // subscribe
  std::string depth_topic = "/camera/aligned_depth_to_color/image_raw";
  std::string intrinsic_topic = "/camera/aligned_depth_to_color/camera_info";
  std::string yolo_topic = "/yolov5/result";
  // publish
  std::string pc_topic = "/abnormal_pointcloud";
  // ros base
  ros::init(argc, argv, "serperate_abnormal_node");
  SeperateAbnormal sep_abn(depth_topic, intrinsic_topic, yolo_topic, pc_topic);
  // poincloud publisher
  sep_abn.advertise();
  ros::Rate rate(30);

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
