#include <limits.h> /* PATH_MAX = 4096 */
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 10000;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size(); ++i) 
    {
        cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    for (size_t i = 0; i < cloud.points.size(); ++i)
        std::cerr << " " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

    pcl::visualization::CloudViewer viewer("PCL Viewer");
    viewer.showCloud(cloud.makeShared());
    while (!viewer.wasStopped());

    return 0;
}