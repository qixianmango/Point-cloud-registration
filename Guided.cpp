#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

using point = pcl::PointXYZ;
using cloud = pcl::PointCloud<point>;

int main()
{
    cloud cloud_in, cloud_out;

    pcl::io::loadPCDFile<point>("D:\\PCLProjects\\data\\rabbit\\rabbit_whole.pcd", cloud_in);
    if (!cloud_in.size())
        return 0;

    pcl::KdTreeFLANN<point> kdtree;
    unsigned int k = 500;      // 参数1
    double epsilon = 0.05;    // 参数2
    kdtree.setInputCloud(cloud_in.makeShared());

    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");

    // Create two viewports, one for the original cloud and one for the filtered cloud
    int v1, v2;
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    // Visualize the original point cloud in the left viewport
    pcl::visualization::PointCloudColorHandlerCustom<point> color_handler_original(cloud_in.makeShared(), 255, 255, 255);
    viewer.addPointCloud(cloud_in.makeShared(), color_handler_original, "original_cloud", v1);

    for (size_t i = 0; i < cloud_in.size(); ++i)
    {
        std::vector<int> indices(0, 0);
        indices.reserve(k);
        std::vector<float> dist(0, 0.0);
        cloud neigh_points;
        dist.reserve(k);

        if (kdtree.nearestKSearch(cloud_in.points[i], k, indices, dist) > 0)
        {
            pcl::copyPointCloud(cloud_in, indices, neigh_points);
            point point_mean(0.0, 0.0, 0.0);
            double neigh_mean_2 = 0.0;

            for (auto neigh_point : neigh_points)
            {
                point_mean.x += neigh_point.x;
                point_mean.y += neigh_point.y;
                point_mean.z += neigh_point.z;
                neigh_mean_2 += (neigh_point.x * neigh_point.x) + (neigh_point.y * neigh_point.y) + (neigh_point.z * neigh_point.z);
            }

            point_mean.x /= neigh_points.size();
            point_mean.y /= neigh_points.size();
            point_mean.z /= neigh_points.size();
            neigh_mean_2 /= neigh_points.size();

            double point_mean_2 = (point_mean.x * point_mean.x) + (point_mean.y * point_mean.y) + (point_mean.z * point_mean.z);
            double a = (neigh_mean_2 - point_mean_2) / (neigh_mean_2 - point_mean_2 + epsilon);
            point b;
            b.x = (1.0 - a) * point_mean.x;
            b.y = (1.0 - a) * point_mean.y;
            b.z = (1.0 - a) * point_mean.z;

            point smoothed_point(0.0, 0.0, 0.0);
            smoothed_point.x = a * cloud_in.points[i].x + b.x;
            smoothed_point.y = a * cloud_in.points[i].y + b.y;
            smoothed_point.z = a * cloud_in.points[i].z + b.z;
            cloud_out.push_back(smoothed_point);
        }
    }

    // Visualize the filtered point cloud in the right viewport
    pcl::visualization::PointCloudColorHandlerCustom<point> color_handler_filtered(cloud_out.makeShared(), 255, 0, 0);
    viewer.addPointCloud(cloud_out.makeShared(), color_handler_filtered, "filtered_cloud", v2);

    std::cout << "滤波前点云数量: " << cloud_in.size() << " 个点" << std::endl;
    std::cout << "滤波后点云数量: " << cloud_out.size() << " 个点" << std::endl;

    viewer.spin();

    return 0;
}