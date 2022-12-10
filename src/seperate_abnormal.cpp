#include <vector>

#include <ros/ros.h>
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

class RSSubscriber
{
 public:
  RSSubscriber(std::string rgb_topic, std::string depth_topic, std::string yolo_topic)
  {
    ROS_INFO("wait for realsense")
    while(ros::hasParam("/camera/realsense2_camera/serial_no"));

    ROS_INFO("seperate abnormal node start!!!");
    rgb_sub_.subscribe(nh_, rgb_topic, 1);
    depth_sub_.subscribe(nh_, depth_topic, 1);
    sync_.reset(new Sync(MySyncPolicy(10), rgb_sub_, depth_sub_));
    sync_->registerCallback(boost::bind(&RSSubscriber::callback, this, _1, _2));
    yolo_sub = nh_.subscribe(yolo_topic, 1, &RSSubscriber::yolo_callback, this);

    nh_.getParam("/camera/realsense2_camera/color_width", image_w);
    nh_.getParam("/camera/realsense2_camera/color_height", image_h);
  }
  ~RSSubscriber()
  {
    ROS_INFO("seperate abnormal node end!!!");
    cv::destroyAllWindows();
  }

  void rs_callback(const sensor_msgs::ImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth)
  {
    cv_bridge::CvImagePtr rgb_ptr, depth_ptr;
    try
    {
      rgb_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
      depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16SC1);
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    auto depth8 = cv::Mat(depth_ptr->image);
    depth8 = depth_ptr->image / 2;
    cv::imshow("depth", depth8);
    cv::waitKey(1);
  }

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
      if(cls != 0) return;
      if(yolo_ret.x.size() != yolo_ret.y.size()) throw std::invalid_argument("the nuber of x and y point different");

      static std::vector<cv::Point> contour;
      for(int i=0; i<yolo_ret.x.size(); i++)
      {
        static cv::Point temp;
        temp.x = yolo_ret.x[i];
        temp.y = yolo_ret.y[i];

        contour.push_back(temp);
        temp.clear();
      }
      abnormal_contours.push_back(contour);
      contour.clear();
    }
  }

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
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serperate_abnormal_node");

  RSSubscriber synchronizer("/camera/color/image_raw", "/camera/aligned_depth_to_color/image_raw", "/yolov5/result");

  ros::spin();

  return 0;
}