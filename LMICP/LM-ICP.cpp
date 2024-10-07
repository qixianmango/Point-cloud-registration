#include "LM-ICP.h"

void sample_point(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_rsf, double count)
{
    pcl::RandomSample<pcl::PointXYZ> rs_src;
    rs_src.setInputCloud(cloud);
    rs_src.setSample(count);
    rs_src.filter(*cloud_rsf);
}

void visualize_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr& source, pcl::PointCloud<pcl::PointXYZ>::Ptr& target, pcl::PointCloud<pcl::PointXYZ>::Ptr& icp)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Registration Cloud"));
    int v1 = 0;
    viewer->createViewPort(0, 0, 1, 1, v1);  // 创建一个视口，显示全部点云
    viewer->setBackgroundColor(255, 255, 255, v1);

    // Original point clouds in green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source, 0, 255, 0);
    // Target point cloud in blue
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target, 0, 0, 255);
    // Transformed source point cloud in red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transe(icp, 255, 0, 0);
    viewer->addPointCloud(source, src_h, "source_cloud", v1);
    viewer->addPointCloud(target, tgt_h, "target_cloud", v1);
    viewer->addPointCloud(icp, transe, "icp_cloud", v1);

    viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
}
