#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/Vertices.h>
#include <boost/thread/thread.hpp>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/region_growing_rgb.h>

#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/filters/project_inliers.h>

#include <thread>
#include <cmath>
#include <chrono>
#include <algorithm>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>


#include <pcl/keypoints/sift_keypoint.h>

#include <dsr_msgs/RobotState.h>
#include <dsr_msgs/GetCurrentPose.h>
#include <dsr_msgs/GetCurrentRotm.h>






using namespace std::chrono_literals;



class pcltest
{
  public:

  pcltest()
  {
    ROS_INFO("pcltest Node Start!");
    pointcloud_sub_ = n.subscribe("/camera/depth/color/points", 1, &pcltest::pointcloud_sub_cb, this);
    pointcloud_pub_ = n.advertise<sensor_msgs::PointCloud2>("/pointclouds", 1000);

  }


  ~pcltest()
  {
    ROS_INFO("pcltest Node STOP!");
  }

  void pointcloud_sub_cb(const sensor_msgs::PointCloud2ConstPtr& pointcloud_raw)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_after (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlierPoints_neg (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::ModelCoefficients::Ptr coefficients12 (new pcl::ModelCoefficients ());

    ros::ServiceClient srvGetpose = n.serviceClient<dsr_msgs::GetCurrentPose>("/dsr01m1013/system/get_current_pose");
    ros::ServiceClient srcGetrotm = n.serviceClient<dsr_msgs::GetCurrentRotm>("/dsr01m1013/aux_control/get_current_rotm");
    dsr_msgs::GetCurrentPose srv;
    dsr_msgs::GetCurrentRotm srv2;
    srv.request.space_type = 1;

    float robot_current_pose[6];
    float robot_current_rotm[9];
    if(srvGetpose.call(srv)){
      for(int i=0; i<6; i++){
        robot_current_pose[i] = srv.response.pos[i];}
      //return (srv.response.success);
    }

    cv::Mat currunt_r_temp;
    if(srcGetrotm.call(srv2)){
      currunt_r_temp = (cv::Mat_ <float>(3,3) <<
      srv2.response.rot_matrix[0].data[0], srv2.response.rot_matrix[0].data[1], srv2.response.rot_matrix[0].data[2],
      srv2.response.rot_matrix[1].data[0], srv2.response.rot_matrix[1].data[1], srv2.response.rot_matrix[1].data[2],
      srv2.response.rot_matrix[2].data[0], srv2.response.rot_matrix[2].data[1], srv2.response.rot_matrix[2].data[2]);
    }
    //cv::Mat currunt_r_inv = currunt_r_temp.inv();

    *cloud = cloudmsg2cloud(*pointcloud_raw);

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);       //(옵션) // Enable model coefficient refinement (optional).
    seg.setInputCloud (cloud);                 //입력
    seg.setModelType (pcl::SACMODEL_PLANE);    //적용 모델  // Configure the object to look for a plane.
    seg.setMethodType (pcl::SAC_RANSAC);       //적용 방법   // Use RANSAC method.
    seg.setMaxIterations (1000);               //최대 실행 수
    seg.setDistanceThreshold (0.01);          //inlier로 처리할 거리 정보   // Set the maximum allowed distance to the model.
    //seg.setRadiusLimits(0, 0.1);     // cylinder경우, Set minimum and maximum radii of the cylinder.
    seg.segment (*inliers, *coefficients);    //세그멘테이션 적용
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);//false
    extract.filter (*cloud);


#include <pcl/filters/passthrough.h>

    /*for(int i=0 ; i < cloud->size(); i++)
    {
      cloud->points[i].r = 255;
      cloud->points[i].g = 0;
      cloud->points[i].b = 0;
    }*/

      cv::Matx44f c2g = {9.99033876e-01 ,-4.03820491e-02, -1.73379364e-02, -32.13200871477923,
     3.98650088e-02, 9.98778398e-01, -2.91974786e-02, -99.3960836718246,
    1.84958104e-02, 2.84780933e-02, 9.99423285e-01, -7.012243399327414,
    0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00};

     cv::Matx44f g2b = {
                currunt_r_temp.at<float>(0,0), currunt_r_temp.at<float>(0,1), currunt_r_temp.at<float>(0,2), robot_current_pose[0],
                currunt_r_temp.at<float>(1,0), currunt_r_temp.at<float>(1,1), currunt_r_temp.at<float>(1,2), robot_current_pose[1],
                currunt_r_temp.at<float>(2,0), currunt_r_temp.at<float>(2,1), currunt_r_temp.at<float>(2,2), robot_current_pose[2],
                0, 0, 0, 1};



    std::cout << c2g << std::endl << g2b << std::endl;

    pcl::io::savePCDFile<pcl::PointXYZRGB>("before.pcd", *cloud);

    pcl::PointXYZRGB pt_temp;
    for(int i=0; i < cloud->points.size(); i++)
    {
      cv::Matx41f cloud_pt = {cloud->points[i].x*1000, cloud->points[i].y*1000, cloud->points[i].z*1000, 1};
      cv::Matx41f output = g2b*c2g*cloud_pt;
      /*cloud->points[i].x = output.val[0];
      cloud->points[i].y = output.val[1];
      cloud->points[i].z = output.val[2];*/
      pt_temp.x = output.val[0];
      pt_temp.y = output.val[1];
      pt_temp.z = output.val[2];
      pt_temp.r = cloud->points[i].r;
      pt_temp.g = cloud->points[i].g;
      pt_temp.b = cloud->points[i].b;
      cloud_after->push_back(pt_temp);
    }

    pcl::io::savePCDFile<pcl::PointXYZRGB>("after2.pcd", *cloud_after);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
    viewer->setBackgroundColor( 255.0, 255.0, 255.0 );
    viewer->addPointCloud(cloud, "cloud");
    viewer->addPointCloud(cloud_after, "cloud_after");

    std::cout << cloud->size() << std::endl;



    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud_dst;
    pcl::fromROSMsg(cloudmsg, cloud_dst);
    return cloud_dst;
  }

  sensor_msgs::PointCloud2 cloud2cloudmsg(pcl::PointCloud<pcl::PointXYZRGB> cloud_src)
  {
    sensor_msgs::PointCloud2 cloudmsg;
    pcl::toROSMsg(cloud_src, cloudmsg);
    cloudmsg.header.frame_id = "map";
    return cloudmsg;
  }




  public:

  ros::Publisher pointcloud_pub_;
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  private:

  ros::NodeHandle n;
  ros::Subscriber pointcloud_sub_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcltest");

  pcltest pt;

  while(ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}



