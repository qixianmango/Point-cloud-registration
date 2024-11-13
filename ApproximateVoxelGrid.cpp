#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

int main(int argc, char** argv)
{
    // 记录程序开始时间
    auto program_start = std::chrono::high_resolution_clock::now();

    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\data\\rabbit\\rabbit_whole.pcd", *cloud);

    // 输出滤波前点云信息
    std::cout << "滤波前点云数量: " << cloud->size() << " 个点" << std::endl;

    // ApproximateVoxelGrid滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> avf;
    avf.setInputCloud(cloud);
    avf.setLeafSize(0.005, 0.005, 0.005);
    avf.filter(*filtered_cloud);

    // 记录程序结束时间
    auto program_end = std::chrono::high_resolution_clock::now();

    // 计算程序运行时间
    auto program_duration = std::chrono::duration_cast<std::chrono::milliseconds>(program_end - program_start).count();

    // 输出滤波后点云信息
    std::cout << "滤波后点云数量: " << filtered_cloud->size() << " 个点" << std::endl;
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
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "原始点云", v1);

    // 显示滤波后的点云
    viewer.addPointCloud<pcl::PointXYZ>(filtered_cloud, "滤波后的点云", v2);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "滤波后的点云", v2);

    // 显示可视化窗口
    viewer.spin();

    return 0;
}