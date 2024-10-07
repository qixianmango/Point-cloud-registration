#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/sample_consensus_prerejective.h>//���������һ������׼


using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    //-------------------------����������-----------------------
    pointnormal::Ptr normals(new pointnormal);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    n.setInputCloud(input_cloud);
    n.setNumberOfThreads(8);        // ����openMP���߳���
    n.setSearchMethod(tree);        // ������ʽ
    n.setKSearch(10);               // K���ڵ����
    n.compute(*normals);            // ���㷨��
    //-------------------------FPFH����-------------------------
    fpfhFeature::Ptr fpfh(new fpfhFeature);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fest;
    fest.setNumberOfThreads(8);     //ָ��8�˼���
    fest.setInputCloud(input_cloud);//�������
    fest.setInputNormals(normals);  //���뷨��
    fest.setSearchMethod(tree);     //������ʽ
    fest.setKSearch(10);            //K���ڵ����
    fest.compute(*fpfh);            //����FPFH

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
    // ������ʱ��
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    //---------------------���ص�������------------------------------
    pcl::PointCloud<PointT>::Ptr source(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile("D:\\PCLProjects\\pointcloud\\d1.pcd", *source);
    pcl::PointCloud<PointT>::Ptr target(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile("D:\\PCLProjects\\pointcloud\\d2.pcd", *target);

    //---------------����Դ���ƺ�Ŀ����Ƶ�FPFH----------------------
    fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source);
    fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target);

    //--------------------RANSAC������׼-----------------------------
    pcl::SampleConsensusPrerejective<PointT, PointT, pcl::FPFHSignature33> r_sac;
    r_sac.setInputSource(source);            // Դ����
    r_sac.setInputTarget(target);            // Ŀ�����
    r_sac.setSourceFeatures(source_fpfh);    // Դ����FPFH����
    r_sac.setTargetFeatures(target_fpfh);    // Ŀ�����FPFH����
    r_sac.setCorrespondenceRandomness(4);    // ��ѡ�����������Ӧʱ������Ҫʹ�õ��ھӵ�����,��ֵԽ������ƥ��������Խ��
    r_sac.setInlierFraction(0.5f);           // �����(�����)inlier����
    r_sac.setNumberOfSamples(3);             // ÿ�ε�����ʹ�õĲ���������
    r_sac.setSimilarityThreshold(0.1f);      // ���ײ����ζ�Ӧ�ܾ�������ı�Ե����֮���������ֵ����Ϊ[0,1]������1Ϊ��ȫƥ�䡣
    r_sac.setMaxCorrespondenceDistance(1.0f);// �ڵ㣬��ֵ Inlier threshold
    r_sac.setMaximumIterations(100);         // RANSAC ������������
    pointcloud::Ptr align(new pointcloud);
    r_sac.align(*align);

    // ����������ʱ��
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "��ʱ�䣺 = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
    pcl::transformPointCloud(*source, *align, r_sac.getFinalTransformation());
    cout << "�任����\n" << r_sac.getFinalTransformation() << endl;

    //-------------------���ӻ�------------------------------------
    visualize_pcd(source, target, align);

    return 0;
}
