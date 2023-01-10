#include <vector>
#include <cmath>
#include <ctime>
#include <string>
#include <algorithm>
#include <boost/array.hpp>
#include "thread-pool/BS_thread_pool.hpp"
#include <thread>
// ros 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// #include <plantfarm/PreprocessedPointcloud.h>
// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>  
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Core>
// doosan robot
#include <dsr_msgs/GetCurrentPosx.h>
#include <dsr_msgs/GetCurrentRotm.h>
#include <dsr_msgs/RobotState.h>

// work flow
// 1. Get abnormal pointcloud(not ordered). And transform the pointcloud to robot coordinate.
// 2, To get a representive position of the abnormal pointcloud, choose N points from the pointcloud and get a median. 
// 3. If a distance of the median is far from the existing median points, give the pointcloud data a new ID for later registration.
// 4. If a distance is close to a existing point, classify the pointcloud as same group to the pointcloud. And the point for calculate the distance is updated to mean of two median points.
// 5. If a group of pointclouds is not updated for a long time, the group is deleted.
// 6. Whenever new pointcloud is added, the score of the group is updated. The score indicates whether the pointclouds are sufficient for registration.
// 7. If the score is high enough, the pointclouds are preprocessed and registered.

// a struct to save the pointcloud data and robot pose and rotm
typedef struct PointCloudId
{
  uint8_t id;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud;
  pcl::PointXYZ centroid;
  bool is_processed;
}PointCloudId;
int pointcloud_last = 0;

// generate thread pool with a number of cpus
BS::thread_pool pool(std::thread::hardware_concurrency());

class RegisterClassify
{
public:
  RegisterClassify(std::string abnormal_pointcloud_topic, std::string processed_pointcloud_topic)
  {
    ROS_INFO("wait for realsense");
    while(nh_.hasParam("/camera/realsense2_camera/serial_no")!=1);

    ROS_INFO("RegisterClassify node start!!!");
    // subscriber
    abnormal_pointcloud_sub = nh_.subscribe(abnormal_pointcloud_topic, 5, &RegisterClassify::abnormal_pointcloud_callback, this);
    robot_state_sub =         nh_.subscribe("/dsr01m1013/state", 5, &RegisterClassify::robot_state_callback, this);
    // publisher
    // processed_pointcloud_topic_pub = nh_.advertise<plantfarm::PreprocessedPointcloud>(processed_pointcloud_topic, 5);
    // doosan robot service client
    get_current_pose_client = nh_.serviceClient<dsr_msgs::GetCurrentPosx>("/dsr01m1013/aux_control/get_current_posx");
    get_current_rotm_client = nh_.serviceClient<dsr_msgs::GetCurrentRotm>("/dsr01m1013/aux_control/get_current_rotm");
  }
  ~RegisterClassify()
  {
    ROS_INFO("RegisterClassify node end!!!");
  }

  void abnormal_pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    ROS_INFO("abnormal!");
    clock_t start = clock();
    // get the current pose and rotm of the robot
    dsr_msgs::GetCurrentPosx get_current_pose_srv;
    dsr_msgs::GetCurrentRotm get_current_rotm_srv;
    get_current_pose_srv.request.ref = 0;
    get_current_rotm_srv.request.ref = 0;
    get_current_pose_client.call(get_current_pose_srv);
    get_current_rotm_client.call(get_current_rotm_srv);
    if(get_current_pose_srv.response.success == false)
    {
      ROS_ERROR("get_current_pose service failed");
      return;
    }
    if(get_current_rotm_srv.response.success == false)
    {
      ROS_ERROR("get_current_rotm service failed");
      return;
    }
    std::cout << "duration" << start-clock() << std::endl;
    
    // homogeneous transformation matrix from camera to endeffector
    Eigen::Matrix4d camera2endeffector;
    Eigen::Matrix4d endeffector2base;
    camera2endeffector << 9.99033876e-01,-4.03820491e-02,-1.73379364e-02,-32.13200871477923,
                          3.98650088e-02, 9.98778398e-01,-2.91974786e-02,-99.3960836718246,
                          1.84958104e-02, 2.84780933e-02, 9.99423285e-01,-7.012243399327414,
                          0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00;
    endeffector2base << get_current_rotm_srv.response.rot_matrix[0].data[0], get_current_rotm_srv.response.rot_matrix[0].data[1], get_current_rotm_srv.response.rot_matrix[0].data[2], get_current_pose_srv.response.task_pos_info[0].data[0],
                        get_current_rotm_srv.response.rot_matrix[1].data[0], get_current_rotm_srv.response.rot_matrix[1].data[1], get_current_rotm_srv.response.rot_matrix[1].data[2], get_current_pose_srv.response.task_pos_info[0].data[1],
                        get_current_rotm_srv.response.rot_matrix[2].data[0], get_current_rotm_srv.response.rot_matrix[2].data[1], get_current_rotm_srv.response.rot_matrix[2].data[2], get_current_pose_srv.response.task_pos_info[0].data[2],
                        0, 0, 0, 1;
    Eigen::Matrix4d camera2base = camera2endeffector * endeffector2base;
    std::cout << "c2e" << std::endl << camera2endeffector << std::endl;
    std::cout << "e2b" << std::endl << endeffector2base << std::endl;
    std::cout << "c2b" << std::endl << camera2base << std::endl;

    // transform the pointcloud to the robot coordinate
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    pcl::transformPointCloud(*cloud, *cloud_base, camera2base);

    // get centroid of pointcloud
    Eigen::Matrix<float, 4, 1> centroid_point;
    pcl::compute3DCentroid(*cloud_base, centroid_point);
    std::cout << "centroid" << std::endl << centroid_point << std::endl;

    int index = -1;
    // find same pointcloud within list
    for(int i=0; i<pointcloud_list.size(); ++i)
    {
      // calculate distance between centroid of pointcloud and centroid of pointcloud in list
      double distance; // square of distance
      distance = std::pow((centroid_point[0] - pointcloud_list[i].centroid.x),2) + std::pow((centroid_point[1] - pointcloud_list[i].centroid.y),2) + std::pow((centroid_point[2] - pointcloud_list[i].centroid.z),2);
      if(distance < 1.0e-1) //threshold
      {
        index = i;
        break;
      }
    }

    // new pointcloud
    if(index == -1)
    { 
      ROS_INFO("new pointcloud");
      PointCloudId pointcloud_tmp;
      pointcloud_tmp.id = pointcloud_last++;
      pointcloud_tmp.cloud.push_back(cloud_base);
      pointcloud_tmp.centroid.x = centroid_point[0];
      pointcloud_tmp.centroid.y = centroid_point[1];
      pointcloud_tmp.centroid.z = centroid_point[2];
      pointcloud_tmp.is_processed = false;
      pointcloud_list.push_back(pointcloud_tmp);
    }
    // existed pointcloud
    else
    {
      ROS_INFO("existed pointcloud");
      pointcloud_list.at(index).cloud.push_back(cloud_base);
      pointcloud_list.at(index).centroid.x = centroid_point[0];
      pointcloud_list.at(index).centroid.y = centroid_point[1];
      pointcloud_list.at(index).centroid.z = centroid_point[2];
      pointcloud_list.at(index).is_processed = false;
    }

    // preprocess pointcloud
    ROS_INFO("push task preprocess");
    pool.push_task(&RegisterClassify::preprocess_pointcloud, this);
  }

  void preprocess_pointcloud()
  {
    ROS_INFO("preprocess enter");
    for(auto pc_list : pointcloud_list)
    {
      if(pc_list.is_processed == true) continue;
      auto it = std::find(ignore_id.begin(), ignore_id.end(), pc_list.id);
      if(it != ignore_id.end()) continue; // if id is in ignore_id, skip
      ROS_INFO("preprocess pointcloud %d", pc_list.id);

      pc_list.is_processed = true;
      // preprocess pointcloud       
      // 1. downsampleing
      // 2. delete outliers
      // 3. registration
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_final;
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setMaximumIterations(100);
      for(int i=0; i<pc_list.cloud.size(); ++i)
      {
        // downsampleing
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsample (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> downsample;
        downsample.setInputCloud(pc_list.cloud.at(i));
        downsample.setLeafSize(0.01f, 0.01f, 0.01f);
        downsample.filter(*cloud_downsample);
        // delete outliers
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outlier (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier;
        outlier.setInputCloud(cloud_downsample);
        outlier.setMeanK(50);
        outlier.setStddevMulThresh(1.0);
        outlier.filter(*cloud_outlier);
        // registration
        if(i==0) pointcloud_final = cloud_outlier;
        else
        {
          icp.setInputSource(pointcloud_final);
          icp.setInputTarget(cloud_outlier);
          icp.align(*pointcloud_final);
        }
      }
      // publish preprocessed pointcloud with id
      // plantfarm::PreprocessedPointcloud preprocessed_pointcloud;
      // sensor_msgs::PointCloud2 preprocessed;
      // pcl::toROSMsg(*pointcloud_final, preprocessed);
      // preprocessed.header.frame_id = "camera_link";
      // preprocessed.header.stamp = ros::Time::now();
      // preprocessed_pointcloud.id = pc_list.id;
      // preprocessed_pointcloud.cloud = preprocessed;
      // processed_pointcloud_topic_pub.publish(preprocessed_pointcloud);
    }
  }

  void robot_state_callback(const dsr_msgs::RobotStateConstPtr& msg)
  {
    std::int32_t robot_state = msg->robot_state;
    std::int32_t robot_state_last = robot_state;
    if(robot_state_last != robot_state) ROS_INFO("%d", robot_state);

    // preprocess pointcloud on frequently called function
    pool.push_task(&RegisterClassify::preprocess_pointcloud, this);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber abnormal_pointcloud_sub;
  ros::Subscriber robot_state_sub;
  ros::Publisher processed_pointcloud_topic_pub;
  ros::ServiceClient get_current_pose_client;
  ros::ServiceClient get_current_rotm_client;


  std::vector<PointCloudId> pointcloud_list;
  std::vector<int> ignore_id; // if task is done, the id is considered to be ignored
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "register_classify_ransac");
  RegisterClassify register_classify_ransac("/abnormal_pointcloud", "processed_pointcloud");

  ros::spin();
  return 0;
}