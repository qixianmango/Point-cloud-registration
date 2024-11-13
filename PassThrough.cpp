#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

int main()
{
    // 记录程序开始时间
    auto program_start = std::chrono::high_resolution_clock::now();

    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 从PCD文件加载点云数据
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("n1.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file\n");
        return (-1);
    }

    // 创建滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(5, 100);
    pass.filter(*cloud_filtered);

    // 记录程序结束时间
    auto program_end = std::chrono::high_resolution_clock::now();

    // 计算程序运行时间
    auto program_duration = std::chrono::duration_cast<std::chrono::milliseconds>(program_end - program_start).count();

    // 输出点云信息
    std::cout << "滤波前点云数量: " << cloud->size() << " 个点" << std::endl;
    std::cout << "滤波后点云数量: " << cloud_filtered->size() << " 个点" << std::endl;
    std::cout << "程序运行时间: " << program_duration << " 毫秒" << std::endl;

    // 可视化
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    // 创建两个视口，左边显示原始点云，右边显示滤波后的点云
    int v1, v2;
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    // 设置视口属性
    viewer.setBackgroundColor(0.0, 0.0, 0.0, v1);
    viewer.setBackgroundColor(0.0, 0.0, 0.0, v2);

    // 显示原始点云
    viewer.addPointCloud(cloud, "原始点云", v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "原始点云", v1);

    // 显示滤波后的点云
    viewer.addPointCloud(cloud_filtered, "滤波后的点云", v2);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "滤波后的点云", v2);


        pcl::io::savePCDFileBinary("n1.pcd", *cloud_filtered);

    // 显示可视化窗口
    viewer.spin();

    return 0;
}