#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/sample_consensus_prerejective.h>//　随机采样一致性配准


using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    //-------------------------法向量估计-----------------------
    pointnormal::Ptr normals(new pointnormal);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    n.setInputCloud(input_cloud);
    n.setNumberOfThreads(8);        // 设置openMP的线程数
    n.setSearchMethod(tree);        // 搜索方式
    n.setKSearch(10);               // K近邻点个数
    n.compute(*normals);            // 计算法线
    //-------------------------FPFH估计-------------------------
    fpfhFeature::Ptr fpfh(new fpfhFeature);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fest;
    fest.setNumberOfThreads(8);     //指定8核计算
    fest.setInputCloud(input_cloud);//输入点云
    fest.setInputNormals(normals);  //输入法线
    fest.setSearchMethod(tree);     //搜索方式
    fest.setKSearch(10);            //K近邻点个数
    fest.compute(*fpfh);            //计算FPFH

    return fpfh;
}

void visualize_pcd(pointcloud::Ptr pcd_src, pointcloud::Ptr pcd_tgt, pointcloud::Ptr pcd_final)
{
    pcl::visualization::PCLVisualizer viewer("Registration Viewer");

    // Set background color
    viewer.setBackgroundColor(255, 255, 255);

    // Add source point cloud in green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 0, 255, 0);
    viewer.addPointCloud(pcd_src, src_h, "source_cloud");

    // Add target point cloud in red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 255, 0, 0);
    viewer.addPointCloud(pcd_tgt, tgt_h, "target_cloud");

    // Add aligned point cloud in blue
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(pcd_final, 0, 0, 255);
    viewer.addPointCloud(pcd_final, final_h, "final_cloud");

    // Set camera position and orientation
    viewer.setCameraPosition(0, 0, -2, 0, -1, 0, 0);

    // Spin until the window is closed
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

int main(int argc, char** argv)
{
    // 声明计时器
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    //---------------------加载点云数据------------------------------
    pcl::PointCloud<PointT>::Ptr source(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile("D:\\PCLProjects\\pointcloud\\d1.pcd", *source);
    pcl::PointCloud<PointT>::Ptr target(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile("D:\\PCLProjects\\pointcloud\\d2.pcd", *target);

    //---------------计算源点云和目标点云的FPFH----------------------
    fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source);
    fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target);

    //--------------------RANSAC点云配准-----------------------------
    pcl::SampleConsensusPrerejective<PointT, PointT, pcl::FPFHSignature33> r_sac;
    r_sac.setInputSource(source);            // 源点云
    r_sac.setInputTarget(target);            // 目标点云
    r_sac.setSourceFeatures(source_fpfh);    // 源点云FPFH特征
    r_sac.setTargetFeatures(target_fpfh);    // 目标点云FPFH特征
    r_sac.setCorrespondenceRandomness(4);    // 在选择随机特征对应时，设置要使用的邻居的数量,数值越大，特征匹配的随机性越大。
    r_sac.setInlierFraction(0.5f);           // 所需的(输入的)inlier分数
    r_sac.setNumberOfSamples(3);             // 每次迭代中使用的采样点数量
    r_sac.setSimilarityThreshold(0.1f);      // 将底层多边形对应拒绝器对象的边缘长度之间的相似阈值设置为[0,1]，其中1为完全匹配。
    r_sac.setMaxCorrespondenceDistance(1.0f);// 内点，阈值 Inlier threshold
    r_sac.setMaximumIterations(100);         // RANSAC 　最大迭代次数
    pointcloud::Ptr align(new pointcloud);
    r_sac.align(*align);

    // 计算总运行时间
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "总时间： = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
    pcl::transformPointCloud(*source, *align, r_sac.getFinalTransformation());
    cout << "变换矩阵：\n" << r_sac.getFinalTransformation() << endl;

    //-------------------可视化------------------------------------
    visualize_pcd(source, target, align);

    return 0;
}
