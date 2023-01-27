#include <vector>
#include <cmath>
#include <ctime>
#include <string>
#include <algorithm>
// #include <boost/array.hpp>
#include "thread-pool/BS_thread_pool.hpp"
#include <thread>
#include <memory>
#include <chrono>
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
#include <pcl/registration/gicp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
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

// generate thread pool with a number of cpus
BS::thread_pool pool(std::thread::hardware_concurrency()); // preprocessing

class RegisterAbnormal
{
public:
  RegisterAbnormal(std::string abnormal_pointcloud_topic, std::string processed_pointcloud_topic)
  {
    ROS_INFO("wait for realsense");
    while (nh_.hasParam("/camera/realsense2_camera/serial_no") != 1);

    ROS_INFO("RegisterAbnormal node start!!!");
    // subscriber
    abnormal_pointcloud_sub = nh_.subscribe(abnormal_pointcloud_topic, 1, &RegisterAbnormal::abnormal_pointcloud_callback, this);
    // publisher
    pub_1          = nh_.advertise<sensor_msgs::PointCloud2>("/plantfarm/test1", 1);
    pub_registered = nh_.advertise<sensor_msgs::PointCloud2>("/plantfarm/registered", 1);

    // processed_pointcloud_topic_pub = nh_.advertise<plantfarm::PreprocessedPointcloud>(processed_pointcloud_topic, 5);
    // doosan robot service client
    get_current_pose_client = nh_.serviceClient<dsr_msgs::GetCurrentPosx>("/dsr01m1013/aux_control/get_current_posx");
    get_current_rotm_client = nh_.serviceClient<dsr_msgs::GetCurrentRotm>("/dsr01m1013/aux_control/get_current_rotm");
  }
  ~RegisterAbnormal()
  {
    ROS_INFO("RegisterAbnormal node end!!!");
    pool.~thread_pool();
  }

  void abnormal_pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    ROS_INFO("abnormal_pointcloud_callback");
    // get the current pose and rotm of the robot
    dsr_msgs::GetCurrentPosx get_current_pose_srv;
    dsr_msgs::GetCurrentRotm get_current_rotm_srv;
    get_current_pose_srv.request.ref = 0;
    get_current_rotm_srv.request.ref = 0;
    get_current_pose_client.call(get_current_pose_srv);
    get_current_rotm_client.call(get_current_rotm_srv);
    if (get_current_pose_srv.response.success == false)
    {
      ROS_ERROR("get_current_pose service failed");
      return;
    }
    if (get_current_rotm_srv.response.success == false)
    {
      ROS_ERROR("get_current_rotm service failed");
      return;
    }

    // homogeneous transformation matrix from camera to endeffector
    Eigen::Matrix4d camera2endeffector;
    Eigen::Matrix4d endeffector2base;
    camera2endeffector << 9.99033876e-01, -4.03820491e-02, -1.73379364e-02, -32.13200871477923,
        3.98650088e-02, 9.98778398e-01, -2.91974786e-02, -99.3960836718246,
        1.84958104e-02, 2.84780933e-02, 9.99423285e-01, -7.012243399327414,
        0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00;
    endeffector2base << get_current_rotm_srv.response.rot_matrix[0].data[0], get_current_rotm_srv.response.rot_matrix[0].data[1], get_current_rotm_srv.response.rot_matrix[0].data[2], get_current_pose_srv.response.task_pos_info[0].data[0],
        get_current_rotm_srv.response.rot_matrix[1].data[0], get_current_rotm_srv.response.rot_matrix[1].data[1], get_current_rotm_srv.response.rot_matrix[1].data[2], get_current_pose_srv.response.task_pos_info[0].data[1],
        get_current_rotm_srv.response.rot_matrix[2].data[0], get_current_rotm_srv.response.rot_matrix[2].data[1], get_current_rotm_srv.response.rot_matrix[2].data[2], get_current_pose_srv.response.task_pos_info[0].data[2],
        0, 0, 0, 1;
    Eigen::Matrix4d camera2base = camera2endeffector * endeffector2base;
    camera2base.block<3, 1>(0, 3) = camera2base.block<3, 1>(0, 3) / 1000.0;
    std::cout << "c2e" << std::endl
              << camera2endeffector << std::endl;
    std::cout << "e2b" << std::endl
              << endeffector2base << std::endl;
    std::cout << "c2b" << std::endl
              << camera2base << std::endl;

    // transform the pointcloud to the robot coordinate
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    pcl::transformPointCloud(*cloud, *cloud_base, camera2base);

    // publish cloud_base
    sensor_msgs::PointCloud2 cloud_base_msg;
    pcl::toROSMsg(*cloud_base, cloud_base_msg);
    cloud_base_msg.header.frame_id = "camera_link";
    cloud_base_msg.header.stamp = ros::Time::now();
    pub_1.publish(cloud_base_msg);

    // get centroid of pointclo/camera/realsense2_camera/resetd
    Eigen::Matrix<float, 4, 1> centroid_point;
    pcl::compute3DCentroid(*cloud_base, centroid_point);
    std::cout << "centroid" << std::endl
              << centroid_point << std::endl;

    int index = -1;
    // find same pointcloud within list
    for (int i = 0; i < pointcloudid_list.size(); ++i)
    {
      // calculate distance between centroid of pointcloud and centroid of pointcloud in list
      double distance; // square of distance
      distance =  std::pow((centroid_point[0] - pointcloudid_list[i]->centroid[0]), 2) 
                + std::pow((centroid_point[1] - pointcloudid_list[i]->centroid[1]), 2) 
                + std::pow((centroid_point[2] - pointcloudid_list[i]->centroid[2]), 2);
      if (distance < 1.0e-0) // threshold
      {
        index = i;
        std::cout << "close enough to existing point! " << i
                  << " distance : " << distance << std::endl;
        break;
      }
    }

    // new pointcloud
    if (index == -1)
    {
      ROS_ERROR("new pointcloud");
      std::shared_ptr<PointCloudId> pointcloudid_tmp(new PointCloudId());
      pointcloudid_tmp->id = pointcloud_last++;
      pointcloudid_tmp->time_stamp = std::chrono::high_resolution_clock::now();
      pointcloudid_tmp->new_cloud = cloud_base;
      pointcloudid_tmp->centroid = centroid_point;
      pointcloudid_tmp->is_processed = false;
      pointcloudid_tmp->is_new = true;
      pointcloudid_list.push_back(pointcloudid_tmp);
    }
    // existed pointcloud
    else
    {
      if (pointcloudid_list.at(index)->is_processed == false)
      {
        ROS_INFO("existed pointcloud is not processed yet");
        return;
      } 
      ROS_INFO("existed pointcloud");
      pointcloudid_list.at(index)->time_stamp = std::chrono::high_resolution_clock::now();
      pointcloudid_list.at(index)->new_cloud = cloud_base;
      pointcloudid_list.at(index)->centroid = centroid_point;
      pointcloudid_list.at(index)->is_processed = false;
      pointcloudid_list.at(index)->is_new = false;
    }
    // preprocess pointcloud
    ROS_INFO("push task preprocess");
    pool.push_task(&RegisterAbnormal::preprocess_pointcloud, this);
    pool.push_task(&RegisterAbnormal::publish_registered, this);
  }

  void preprocess_pointcloud()
  {
    const clock_t begin_time = clock();
    ROS_INFO("preprocess enter pointcloud size : %d", static_cast<int>(pointcloudid_list.size()));
    for (int i = 0; i < pointcloudid_list.size(); ++i)
    {
      if (pointcloudid_list[i]->is_processed == true)
        continue;
      // if id is in ignore_id, skip
      auto it = std::find(ignore_id.begin(), ignore_id.end(), pointcloudid_list[i]->id);
      if (it != ignore_id.end())
        continue; 

      ROS_INFO("preprocess pointcloud %d", pointcloudid_list[i]->id);
      // delete outliers
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rm_outlier(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier;
      outlier.setInputCloud(pointcloudid_list[i]->new_cloud);
      outlier.setMeanK(50);
      outlier.setStddevMulThresh(1.0);
      outlier.filter(*cloud_rm_outlier);

      // show clock time in ms with id
      std::cout << "preprocess ";
      std::cout << "id : " << pointcloudid_list[i]->id << " time : " << float(clock() - begin_time) / CLOCKS_PER_SEC * 1000 << std::endl;

      ROS_WARN("!!!");
      if(pointcloudid_list[i]->is_new == true)
      {
        ROS_INFO("first pointcloud");
        pointcloudid_list[i]->registered_cloud = cloud_rm_outlier;
        pointcloudid_list[i]->is_processed = true;
        pointcloudid_list[i]->is_new = false;
        return;
      }
      // registration. G-ICP algorithm
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_registered(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setMaxCorrespondenceDistance(1.0);
      icp.setTransformationEpsilon(0.001);
      icp.setMaximumIterations(100);

      icp.setInputSource(cloud_rm_outlier);
      icp.setInputTarget(pointcloudid_list[i]->registered_cloud);
      icp.align(*cloud_registered);
      ROS_WARN("registed");
      if (icp.hasConverged())
      {
        std::cout << "Id : " << pointcloudid_list[i]->id << std::endl;
        std::cout << "ICP converged." << std::endl;
        std::cout << "ICP score is " << icp.getFitnessScore() << std::endl;
        std::cout << "ICP transformation " << icp.getFinalTransformation() << std::endl;
        pointcloudid_list[i]->registered_cloud = cloud_registered;
        pointcloudid_list[i]->is_processed = true;
      }
      else
      {
        std::cout << "ICP not converged." << std::endl;
      }

      //show clock time in ms with id
      std::cout << "regi ";
      std::cout << "id : " << pointcloudid_list[i]->id << " time : " << float(clock() - begin_time) / CLOCKS_PER_SEC * 1000 << std::endl;
    }
  }

  void publish_registered() // + check expired
  {
    ROS_INFO("publish_registered");
    for (int i = 0; i < pointcloudid_list.size(); ++i)
    {
      // check if the pointcloudid is expired
      std::chrono::duration<float> diff = std::chrono::system_clock::now() - pointcloudid_list[i]->time_stamp;
      std::chrono::seconds sec = std::chrono::duration_cast<std::chrono::seconds>(diff);
      ROS_INFO("id : %d, time : %d", pointcloudid_list[i]->id, sec.count());
      if(sec.count() > expire_sec)
      {
        ROS_INFO("pointcloud %d is expired", pointcloudid_list[i]->id);
        pointcloudid_list.erase(pointcloudid_list.begin() + i);
        continue;
      }
      // check registered pointcloud is existing
      else if(pointcloudid_list[i]->is_new == true)
      {
        ROS_INFO("new pointcloud. skip");
        continue;
      }
      // test if registered cloud can represent the original object
      else if(score_pointcloud(pointcloudid_list[i]->registered_cloud)>-1) 
      {
        ROS_INFO("pointcloud %d is good", pointcloudid_list[i]->id);
        sensor_msgs::PointCloud2 registered;
        pcl::toROSMsg(*(pointcloudid_list[i]->registered_cloud), registered);
        registered.header.frame_id = "camera_link";
        registered.header.stamp = ros::Time::now();
        pub_registered.publish(registered);
      }
    }
  }

  // get a score of the pointcloud
  // the score is deviation 
  float score_pointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc)
  {
    float score = 0.0;
    return score;
  }

  //https://limhyungtae.github.io/2021-09-14-ROS-Point-Cloud-Library-(PCL)-12.-Generalized-Iterative-Closest-Point-(G-ICP)/
  void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc, pcl::PointCloud<pcl::PointXYZRGB> &pc_colored, const std::vector<int> &color)
  {
    int N = pc.points.size();

    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;
    for (int i = 0; i < N; ++i)
    {
      const auto &pt = pc.points[i];
      pt_tmp.x = pt.x;
      pt_tmp.y = pt.y;
      pt_tmp.z = pt.z;    
      pt_tmp.r = color[0];
      pt_tmp.g = color[1];
      pt_tmp.b = color[2];  
      pc_colored.points.emplace_back(pt_tmp);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber abnormal_pointcloud_sub;
  ros::Subscriber robot_state_sub;
  ros::Publisher processed_pointcloud_topic_pub;
  ros::Publisher pub_1;
  ros::Publisher pub_registered;
  ros::ServiceClient get_current_pose_client;
  ros::ServiceClient get_current_rotm_client;

  // a struct to save the pointcloud data and robot pose and rotm
  typedef struct PointCloudId
  {
    unsigned int id;
    std::chrono::time_point<std::chrono::system_clock> time_stamp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr registered_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud;
    Eigen::Matrix<float, 4, 1> centroid;
    bool is_processed;
    bool is_new;
  } PointCloudId;
  unsigned int pointcloud_last = 0;
  const int expire_sec = 10; // if the pointcloud is not updated for 10 sec, it is considered to be expired
  // pcl::visualization::PCLVisualizer viewer;
  std::vector<std::shared_ptr<PointCloudId>> pointcloudid_list;
  std::vector<int> ignore_id; // if task is done, the id is considered to be ignored
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "register_abnormal");
  RegisterAbnormal register_classify_ransac("/plantfarm/abnormal_pointcloud", "/plantfarm/registered_pointcloud");

  // run in 5 hz
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout<<"end"<<std::endl;
  return 0;
}