#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

int main(int argc, char** argv)
{
    // 记录程序开始时间
    auto program_start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\data\\rabbit\\rabbit_whole.pcd", *cloud);

    // 输出滤波前点云信息
    std::cout << "滤波前点云数量: " << cloud->size() << " 个点" << std::endl;

    // 创建体素滤波器对象
    pcl::VoxelGrid<pcl::PointXYZ> grid;

    // 设置输入点云
    grid.setInputCloud(cloud);

    // 设置体素的大小
    Eigen::Vector4f leaf_size{ 0.003, 0.003, 0.003, 0 };
    grid.setLeafSize(leaf_size);

    // 设置每一个体素内需要包含的最小点个数
    grid.setMinimumPointsNumberPerVoxel(3);

    // 执行体素滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    grid.filter(*cloud_filtered);

    // 记录程序结束时间
    auto program_end = std::chrono::high_resolution_clock::now();

    // 计算程序运行时间
    auto program_duration = std::chrono::duration_cast<std::chrono::milliseconds>(program_end - program_start).count();

    // 输出滤波后点云信息
    std::cout << "滤波后点云数量: " << cloud_filtered->size() << " 个点" << std::endl;
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
    viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, "滤波后的点云", v2);

    // 显示可视化窗口
    viewer.spin();

    return 0;
}