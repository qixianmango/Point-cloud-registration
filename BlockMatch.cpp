#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

void VisualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud) {
    //-----------------------显示点云-----------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("显示点云"));

    int v1(0), v2(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("point clouds", 10, 10, "v1_text", v1);
    viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
    viewer->addText("filtered point clouds", 10, 10, "v2_text", v2);
    // 按照z字段进行渲染,将z改为x或y即为按照x或y字段渲染
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");
    viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud", v1);

    viewer->addPointCloud<pcl::PointXYZ>(filter_cloud, "cloud_filtered", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_filtered", v2);
    //viewer->addCoordinateSystem(1.0);
    //viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

int main()
{
    // 记录程序开始时间
    auto program_start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\data\\rabbit\\rabbit_whole.pcd", *cloud);

    // 输出滤波前点云信息
    std::cout << "滤波前点云数量: " << cloud->size() << " 个点" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // -----------------统计滤波-------------------
    // 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
    // 个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);   //设置待滤波的点云
    sor.setMeanK(50);           //设置在进行统计时考虑查询点邻近点数
    sor.setStddevMulThresh(1);  //设置判断是否为离群点的阈值，里边的数字表示标准差的倍数，1个标准差以上就是离群点。

    sor.filter(*cloud_filtered); //存储内点
    // 输出滤波后点云信息
    std::cout << "滤波后点云数量: " << cloud_filtered->size() << " 个点" << std::endl;

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
    viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, "滤波后的点云", v2);

    // 显示可视化窗口
    viewer.spin();

    return 0;
}