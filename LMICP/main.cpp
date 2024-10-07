#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp_nl.h> // LM-ICP
#include <pcl/console/time.h>
#include "LM-ICP.h"

int main()
{
    pcl::console::TicToc time;
    // Load source point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\mango\\Desktop\\AdTree-main\\data\\new1.pcd", *source);
    // Load target point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\mango\\Desktop\\AdTree-main\\data\\tree1.pcd", *target);

    // Randomly sample feature points
    pcl::PointCloud<pcl::PointXYZ>::Ptr s_k(new pcl::PointCloud<pcl::PointXYZ>);
    sample_point(source, s_k);
    pcl::PointCloud<pcl::PointXYZ>::Ptr t_k(new pcl::PointCloud<pcl::PointXYZ>);
    sample_point(target, t_k);

    time.tic();
    // LM-ICP
    pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> lmicp;
    lmicp.setInputSource(s_k);
    lmicp.setInputTarget(t_k);
    lmicp.setTransformationEpsilon(1e-10);
    lmicp.setMaxCorrespondenceDistance(10);
    lmicp.setEuclideanFitnessEpsilon(0.0001);
    lmicp.setMaximumIterations(50);
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    lmicp.align(*icp_cloud);

    std::cout << "总时间: " << time.toc() << "ms" << std::endl;
    std::cout << "旋转矩阵：\n" << lmicp.getFinalTransformation() << std::endl;

    // Transform the source point cloud
    pcl::transformPointCloud(*source, *icp_cloud, lmicp.getFinalTransformation());

    visualize_registration(source, target, icp_cloud);

    return 0;
}