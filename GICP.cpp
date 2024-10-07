#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>  
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <chrono>

using namespace std;

int main()
{
    // 加载点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\d1.pcd", *target);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\d2.pcd", *source);

    cout << "读取源点云个数：" << source->points.size() << endl;
    cout << "读取目标点云个数：" << target->points.size() << endl;

    // 声明计时器
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // 初始化GICP对象
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

    // KD树加速搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
    tree1->setInputCloud(source);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
    tree2->setInputCloud(target);
    gicp.setSearchMethodSource(tree1);
    gicp.setSearchMethodTarget(tree2);

    // 设置GICP相关参数
    gicp.setInputSource(source);
    gicp.setInputTarget(target);
    gicp.setMaxCorrespondenceDistance(1);
    gicp.setTransformationEpsilon(1e-5);
    gicp.setEuclideanFitnessEpsilon(0.01);
    gicp.setMaximumIterations(50);

    // 计算需要的刚体变换以便将输入的源点云匹配到目标点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    gicp.align(*icp_cloud);


    // 计算总运行时间
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "总时间： = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
    cout << "变换矩阵：\n" << gicp.getFinalTransformation() << endl;

    // 使用变换矩阵对为输入点云进行变换
    pcl::transformPointCloud(*source, *icp_cloud, gicp.getFinalTransformation());

    // 点云可视化
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_final(new pcl::visualization::PCLVisualizer("配准结果"));
    viewer_final->setBackgroundColor(255, 255, 255);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(target, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_color(source, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(source, input_color, "input cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(icp_cloud, 0, 0, 255);
    viewer_final->addPointCloud<pcl::PointXYZ>(icp_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");

    //pcl::io::savePCDFileBinary("D:\\LaTex\\article\\High-precision point calculation method\\pic\\GICP\\n1n2\\n1n2.pcd", *icp_cloud);

    while (!viewer_final->wasStopped())
    {
        viewer_final->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}
