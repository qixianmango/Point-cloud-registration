#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

int main(int argc, char** argv)
{
    // 记录程序开始时间
    auto program_start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\data\\rabbit\\rabbit_whole.pcd", *cloud);

    // 输出点云信息
    std::cout << "滤波前点云数量: " << cloud->size() << " 个点" << std::endl;

    // 体素滤波
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.003f, 0.003f, 0.003f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*voxel_filtered);

    // K最近邻搜索
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    pcl::PointIndicesPtr inds = std::make_shared<pcl::PointIndices>();

    for (size_t i = 0; i < voxel_filtered->points.size(); i++) {
        pcl::PointXYZ searchPoint;
        searchPoint.x = voxel_filtered->points[i].x;
        searchPoint.y = voxel_filtered->points[i].y;
        searchPoint.z = voxel_filtered->points[i].z;

        int K = 1;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            inds->indices.push_back(pointIdxNKNSearch[0]);
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr final_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, inds->indices, *final_filtered);

    // 输出滤波后点云信息
    std::cout << "滤波后点云数量: " << final_filtered->size() << " 个点" << std::endl;

    // 记录程序结束时间
    auto program_end = std::chrono::high_resolution_clock::now();

    // 计算程序运行时间
    auto program_duration = std::chrono::duration_cast<std::chrono::milliseconds>(program_end - program_start).count();

    // 输出程序运行时间
    std::cout << "程序运行时间: " << program_duration << " 毫秒" << std::endl;

    // 创建可视化对象
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    // 创建两个视口
    int v1, v2;
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    // 设置视口属性
    viewer.setBackgroundColor(0.0, 0.0, 0.0, v1);
    viewer.setBackgroundColor(0.0, 0.0, 0.0, v2);

    // 显示原始点云
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "原始点云", v1);

    // 显示滤波后的点云
    viewer.addPointCloud<pcl::PointXYZ>(final_filtered, "滤波后的点云", v2);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "滤波后的点云", v2);

    // 显示可视化窗口
    viewer.spin();

    return 0;
}