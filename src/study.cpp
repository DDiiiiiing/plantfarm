#include <vector>
#include <string>
#include <boost/array.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Core>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
// subscribe depth image
// convert them into correspoding pointcloud
// and publish the pointcloud into sensor_msgs/PointCloud2

class PointCloudStudy
{
public:
  PointCloudStudy(string depth_topic, string intrinsics_topic, string pointcloud_topic)
  {
    ROS_INFO("PointCloudStudy start");
    depth_sub = nh.subscribe(depth_topic, 10, &PointCloudStudy::depthCallback, this);
    intrinsics_sub = nh.subscribe(intrinsics_topic, 10, &PointCloudStudy::intrinsics_callback, this);
    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 1);
  }
  ~PointCloudStudy()
  {
    ROS_INFO("PointCloudStudy end");
    cv::destroyAllWindows();
  }

  void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg)
  {
    ROS_INFO("depthCallback cb");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
      depth_image = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = depth_to_pointcloud(depth_image);

    sensor_msgs::PointCloud2 pointcloud_msg;
    pcl::toROSMsg(*cloud, pointcloud_msg);
    pointcloud_msg.header.frame_id = "camera_link";
    pointcloud_pub.publish(pointcloud_msg);
  }

  // subscribe intrinsic topic
  void intrinsics_callback(const sensor_msgs::CameraInfoPtr &intrinsic)
  {
    K = intrinsic->K;
    D = intrinsic->D;
  }

private:
ros::NodeHandle nh;
ros::Subscriber depth_sub;
ros::Subscriber intrinsics_sub;
ros::Publisher pointcloud_pub;
cv::Mat depth_image;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
boost::array<double, 9> K;
std::vector<double> D;

  pcl::PointCloud<pcl::PointXYZ> depth_to_pointcloud(cv::Mat depth_image)
  {
     pcl::PointCloud<pcl::PointXYZ> cloud;

    int width = depth_image.cols;
    int height = depth_image.rows;
    cloud.clear();
    cloud.is_dense = false;

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
 
        // // Apply distortion
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

        cloud.push_back(point); 
        // std::cout<<"u : "<<u<<" | v : "<<v<<" | x : "<<point.x<<" | y : "<<point.y<<" | z : "<<point.z<<""<<std::endl;
      }
    }

    return cloud;
  }
};

int main(int argc, char** argv)
{
  // subscribe
  std::string depth_topic       = "/camera/aligned_depth_to_color/image_raw";
  std::string intrinsic_topic   = "/camera/aligned_depth_to_color/camera_info";
  // publish
  std::string pc_topic          = "/pointcloud";

  ros::init(argc, argv, "pointcloud_study");
  PointCloudStudy pointcloud_study(depth_topic, intrinsic_topic, pc_topic);
  ros::spin();
  return 0;
}