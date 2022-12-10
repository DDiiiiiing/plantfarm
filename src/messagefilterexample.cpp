#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class RSSubscriber
{
 public:
  RSSubscriber(std::string rgb_topic, std::string depth_topic)
  {
    ROS_INFO("seperate abnormal node start!!!");
    sub_rgb_.subscribe(nh_, rgb_topic, 1);
    sub_depth_.subscribe(nh_, depth_topic, 1);
    sync_.reset(new Sync(MySyncPolicy(10), sub_rgb_, sub_depth_));
    sync_->registerCallback(boost::bind(&RSSubscriber::callback, this, _1, _2));
  }
  ~RSSubscriber()
  {
    ROS_INFO("seperate abnormal node end!!!");
    cv::destroyAllWindows();
  }

  void callback(const sensor_msgs::ImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth)
  {
    ROS_INFO("callback!!!");
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
    cv::imshow("rgb", rgb_ptr->image);
    cv::imshow("depth", depth8);
    cv::waitKey(1);
  }

 private:
  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::Image> sub_rgb_;
  message_filters::Subscriber<sensor_msgs::Image> sub_depth_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  cv::Mat rgb_image, depth_image;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serperate_abnormal_node");

  RSSubscriber synchronizer("/camera/color/image_raw", "/camera/aligned_depth_to_color/image_raw");

  ros::spin();

  return 0;
}