#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>    // 体素下采样滤波
#include <pcl/features/normal_3d_omp.h>// 使用OMP需要添加的头文件
#include <pcl/features/fpfh_omp.h>     // fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/ia_ransac.h>// sac_ia算法
#include <pcl/registration/ndt.h>      // NDT配准算法
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef pcl::PointCloud<pcl::FPFHSignature33> FPFHFeature;

// 下采样滤波
PointCloud::Ptr voxel_grid_filter(PointCloud::Ptr& cloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(0.05, 0.05, 0.05);
    filter.setInputCloud(cloud);
    PointCloud::Ptr filtered_cloud(new PointCloud);
    filter.filter(*filtered_cloud);
    return filtered_cloud;
}

// 计算FPFH特征
FPFHFeature::Ptr compute_fpfh_feature(PointCloud::Ptr cloud)
{
    // 估计法向量
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    NormalCloud::Ptr normals(new NormalCloud);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setNumberOfThreads(4);
    ne.setSearchMethod(tree);
    ne.setKSearch(10);
    ne.compute(*normals);

    // 计算FPFH特征
    FPFHFeature::Ptr fpfh(new FPFHFeature);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
    fpfh_estimation.setNumberOfThreads(4);
    fpfh_estimation.setInputCloud(cloud);
    fpfh_estimation.setInputNormals(normals);
    fpfh_estimation.setSearchMethod(tree);
    fpfh_estimation.setKSearch(10);
    fpfh_estimation.compute(*fpfh);

    return fpfh;
}

// 可视化
void visualize_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr& source, pcl::PointCloud<pcl::PointXYZ>::Ptr& target, pcl::PointCloud<pcl::PointXYZ>::Ptr& icp)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Registration Cloud"));
    int v1 = 0;
    viewer->createViewPort(0, 0, 1, 1, v1);  // 创建一个视口，显示全部点云
    viewer->setBackgroundColor(255, 255, 255, v1);


    // 原始点云为绿色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source, 0, 255, 0);
    // 目标点云为红色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target, 255, 0, 0);
    // 旋转后的点云为蓝色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transe(icp, 0, 0, 255);

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


int main(int argc, char** argv)
{
    // 加载点云数据
    PointCloud::Ptr source_cloud(new PointCloud);
    PointCloud::Ptr target_cloud(new PointCloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\d1.pcd", *source_cloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\d2.pcd", *target_cloud);
    if (source_cloud->empty() || target_cloud->empty())
    {
        cout << "请确认点云文件名称是否正确" << endl;
        return -1;
    }

    // 声明计时器
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // 下采样滤波
    PointCloud::Ptr source_filtered = voxel_grid_filter(source_cloud);
    PointCloud::Ptr target_filtered = voxel_grid_filter(target_cloud);

    // 计算FPFH特征
    FPFHFeature::Ptr source_fpfh = compute_fpfh_feature(source_filtered);
    FPFHFeature::Ptr target_fpfh = compute_fpfh_feature(target_filtered);

    // 采样一致性SAC-IA初始配准
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(source_filtered);
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(target_filtered);
    sac_ia.setTargetFeatures(target_fpfh);
    sac_ia.setMinSampleDistance(0.005);       // 设置样本之间的最小距离
    sac_ia.setMaxCorrespondenceDistance(0.1);// 设置对应点对之间的最大距离
    sac_ia.setNumberOfSamples(100);          // 设置每次迭代计算中使用的样本数量
    sac_ia.setCorrespondenceRandomness(6);   // 设置在6个最近特征对应中随机选取一个
    PointCloud::Ptr aligned_cloud(new PointCloud);
    sac_ia.align(*aligned_cloud);
    Eigen::Matrix4f initial_transformation = sac_ia.getFinalTransformation();
    cout << "SAC-IA 初始变换矩阵：\n" << initial_transformation << endl;

    // 正态分布变换（NDT）
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setInputSource(source_filtered);
    ndt.setInputTarget(target_filtered);
    ndt.setStepSize(4);                      // 设置More-Thuente线搜索的最大步长
    ndt.setResolution(0.1);                 // 设置NDT网格结构的分辨率
    ndt.setMaximumIterations(35);            // 设置最大迭代次数
    ndt.setTransformationEpsilon(0.01); // 设置终止条件的最小转换差异
    PointCloud::Ptr output_cloud(new PointCloud);
    ndt.align(*output_cloud, initial_transformation); // 使用初始变换矩阵进行配准

    // 计算总运行时间
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "总时间： = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;

    Eigen::Matrix4f final_transformation = ndt.getFinalTransformation(); // 获取最终的变换矩阵
    cout << "NDT 最终变换矩阵：\n" << final_transformation << endl;


    // 使用变换矩阵对未进行滤波的原始源点云进行变换
    PointCloud::Ptr transformed_source_cloud(new PointCloud);
    pcl::transformPointCloud(*source_cloud, *transformed_source_cloud, final_transformation);

    // 可视化
    visualize_registration(source_cloud, target_cloud, transformed_source_cloud);

    return 0;
}