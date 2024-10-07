#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h> // icp算法
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

using namespace std;

int
main(int argc, char** argv)
{

    // --------------------加载源点云-----------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\p1.pcd", *source);

    
    // -------------------加载目标点云----------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\p2.pcd", *target);


    // 声明计时器
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    
    //--------------------初始化ICP对象--------------------
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //----------------------icp核心代码--------------------
    icp.setInputSource(source);            // 源点云
    icp.setInputTarget(target);            // 目标点云
    icp.setTransformationEpsilon(1e-10);   // 为终止条件设置最小转换差异
    icp.setMaxCorrespondenceDistance(0.05);  // 设置对应点对之间的最大距离（此值对配准结果影响较大）。
    icp.setEuclideanFitnessEpsilon(0.05);  // 设置收敛条件是均方误差和小于阈值， 停止迭代；
    icp.setMaximumIterations(100);           // 最大迭代次数
    icp.setUseReciprocalCorrespondences(true);//设置为true,则使用相互对应关系
    // 计算需要的刚体变换以便将输入的源点云匹配到目标点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*icp_cloud);

    // 计算总运行时间
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "总时间： = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
    //cout << "\nICP has converged, score is " << icp.getFitnessScore() << endl;
    cout << "变换矩阵：\n" << icp.getFinalTransformation() << endl;
    // 使用创建的变换对为输入源点云进行变换
    pcl::transformPointCloud(*source, *icp_cloud, icp.getFinalTransformation());

    // ----------------点云可视化----------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("ICP配准结果"));
    viewer->setBackgroundColor(1.0, 1.0, 1.0);  // 设置背景颜色为白色
    // 对目标点云着色可视化 (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(target, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(target, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    // 对源点云着色可视化 (blue).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>source_color(source, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(source, source_color, "source cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");

    // 对转换后的源点云着色 (green)可视化.
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>icp_color(icp_cloud, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(icp_cloud, icp_color, "icp cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "icp cloud");

    //pcl::io::savePCDFile("D:\\LaTex\\article\\High-precision point calculation method\\pic\\ICP\\n1n2\\n1n2.pcd", *icp_cloud);
    // 启动可视化

    viewer->initCameraParameters();   //初始化摄像头参数
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    system("pause");

    return (0);
}