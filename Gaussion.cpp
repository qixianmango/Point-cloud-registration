#include <iostream>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/convolution_3d.h>  // 高斯滤波
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

int main()
{
    // 加载数据并开始计时
    auto start_time = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("D:\\PCLProjects\\data\\rabbit\\rabbit_whole.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file pcd\n");
        return -1;
    }

    auto end_time_load = std::chrono::high_resolution_clock::now();
    auto load_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_load - start_time).count();

    // 基于高斯核函数的卷积滤波实现
    pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> kernel;
    kernel.setSigma(1);                    // 高斯函数的标准方差，决定函数的宽度
    kernel.setThresholdRelativeToSigma(5); // 设置相对Sigma参数的距离阈值
    kernel.setThreshold(5);             // 设置距离阈值，若点间距离大于阈值则不予考虑

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
    convolution.setKernel(kernel); // 设置卷积核
    convolution.setInputCloud(cloud);
    convolution.setNumberOfThreads(8);
    convolution.setSearchMethod(tree);
    convolution.setRadiusSearch(0.01);

    auto start_time_convolution = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr gaussianFilter(new pcl::PointCloud<pcl::PointXYZ>);
    convolution.convolve(*gaussianFilter);
    auto end_time_convolution = std::chrono::high_resolution_clock::now();
    auto convolution_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_convolution - start_time_convolution).count();

    // 输出点云信息
    std::cout << "滤波前点云数量: " << cloud->size() << " 个点" << std::endl;
    std::cout << "滤波后点云数量: " << gaussianFilter->size() << " 个点" << std::endl;
    std::cout << "程序运行时间: " << convolution_duration << " 毫秒" << std::endl;

    // 显示点云
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ShowCloud"));
    viewer->setWindowName("高斯滤波");
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);
    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
    viewer->addText("Filtered point clouds", 10, 10, "v2_text", v2);

    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
    viewer->addPointCloud<pcl::PointXYZ>(gaussianFilter, "gaussianFilter", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "sample cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "gaussianFilter", v2);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}