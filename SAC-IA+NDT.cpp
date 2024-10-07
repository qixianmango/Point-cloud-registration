#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>    // �����²����˲�
#include <pcl/features/normal_3d_omp.h>// ʹ��OMP��Ҫ��ӵ�ͷ�ļ�
#include <pcl/features/fpfh_omp.h>     // fpfh���ټ����omp(��˲��м���)
#include <pcl/registration/ia_ransac.h>// sac_ia�㷨
#include <pcl/registration/ndt.h>      // NDT��׼�㷨
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef pcl::PointCloud<pcl::FPFHSignature33> FPFHFeature;

// �²����˲�
PointCloud::Ptr voxel_grid_filter(PointCloud::Ptr& cloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(0.05, 0.05, 0.05);
    filter.setInputCloud(cloud);
    PointCloud::Ptr filtered_cloud(new PointCloud);
    filter.filter(*filtered_cloud);
    return filtered_cloud;
}

// ����FPFH����
FPFHFeature::Ptr compute_fpfh_feature(PointCloud::Ptr cloud)
{
    // ���Ʒ�����
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    NormalCloud::Ptr normals(new NormalCloud);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setNumberOfThreads(4);
    ne.setSearchMethod(tree);
    ne.setKSearch(10);
    ne.compute(*normals);

    // ����FPFH����
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

// ���ӻ�
void visualize_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr& source, pcl::PointCloud<pcl::PointXYZ>::Ptr& target, pcl::PointCloud<pcl::PointXYZ>::Ptr& icp)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Registration Cloud"));
    int v1 = 0;
    viewer->createViewPort(0, 0, 1, 1, v1);  // ����һ���ӿڣ���ʾȫ������
    viewer->setBackgroundColor(255, 255, 255, v1);


    // ԭʼ����Ϊ��ɫ
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source, 0, 255, 0);
    // Ŀ�����Ϊ��ɫ
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target, 255, 0, 0);
    // ��ת��ĵ���Ϊ��ɫ
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
    // ���ص�������
    PointCloud::Ptr source_cloud(new PointCloud);
    PointCloud::Ptr target_cloud(new PointCloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\d1.pcd", *source_cloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\d2.pcd", *target_cloud);
    if (source_cloud->empty() || target_cloud->empty())
    {
        cout << "��ȷ�ϵ����ļ������Ƿ���ȷ" << endl;
        return -1;
    }

    // ������ʱ��
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // �²����˲�
    PointCloud::Ptr source_filtered = voxel_grid_filter(source_cloud);
    PointCloud::Ptr target_filtered = voxel_grid_filter(target_cloud);

    // ����FPFH����
    FPFHFeature::Ptr source_fpfh = compute_fpfh_feature(source_filtered);
    FPFHFeature::Ptr target_fpfh = compute_fpfh_feature(target_filtered);

    // ����һ����SAC-IA��ʼ��׼
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(source_filtered);
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(target_filtered);
    sac_ia.setTargetFeatures(target_fpfh);
    sac_ia.setMinSampleDistance(0.005);       // ��������֮�����С����
    sac_ia.setMaxCorrespondenceDistance(0.1);// ���ö�Ӧ���֮���������
    sac_ia.setNumberOfSamples(100);          // ����ÿ�ε���������ʹ�õ���������
    sac_ia.setCorrespondenceRandomness(6);   // ������6�����������Ӧ�����ѡȡһ��
    PointCloud::Ptr aligned_cloud(new PointCloud);
    sac_ia.align(*aligned_cloud);
    Eigen::Matrix4f initial_transformation = sac_ia.getFinalTransformation();
    cout << "SAC-IA ��ʼ�任����\n" << initial_transformation << endl;

    // ��̬�ֲ��任��NDT��
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setInputSource(source_filtered);
    ndt.setInputTarget(target_filtered);
    ndt.setStepSize(4);                      // ����More-Thuente����������󲽳�
    ndt.setResolution(0.1);                 // ����NDT����ṹ�ķֱ���
    ndt.setMaximumIterations(35);            // ��������������
    ndt.setTransformationEpsilon(0.01); // ������ֹ��������Сת������
    PointCloud::Ptr output_cloud(new PointCloud);
    ndt.align(*output_cloud, initial_transformation); // ʹ�ó�ʼ�任���������׼

    // ����������ʱ��
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "��ʱ�䣺 = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;

    Eigen::Matrix4f final_transformation = ndt.getFinalTransformation(); // ��ȡ���յı任����
    cout << "NDT ���ձ任����\n" << final_transformation << endl;


    // ʹ�ñ任�����δ�����˲���ԭʼԴ���ƽ��б任
    PointCloud::Ptr transformed_source_cloud(new PointCloud);
    pcl::transformPointCloud(*source_cloud, *transformed_source_cloud, final_transformation);

    // ���ӻ�
    visualize_registration(source_cloud, target_cloud, transformed_source_cloud);

    return 0;
}