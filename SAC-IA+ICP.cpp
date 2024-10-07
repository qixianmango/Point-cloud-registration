#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <boost/thread.hpp>

using pcl::NormalEstimation;
using pcl::search::KdTree;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// ���ƿ��ӻ�����ɫ������
void visualize_pcd(PointCloud::Ptr pcd_src,
    PointCloud::Ptr pcd_tgt,
    PointCloud::Ptr pcd_final)
{
    pcl::visualization::PCLVisualizer viewer("Registration Viewer");
    viewer.setBackgroundColor(255, 255, 255); // ���ñ�����ɫΪ��ɫ
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 255, 0, 0); // ԭʼ������ɫ����Ϊ��ɫ
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 0, 255, 0); // Ŀ�������ɫ����Ϊ��ɫ
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(pcd_final, 0, 0, 255); // ת����ĵ�����ɫ����Ϊ��ɫ
    viewer.addPointCloud(pcd_src, src_h, "source cloud");
    viewer.addPointCloud(pcd_tgt, tgt_h, "target cloud");
    viewer.addPointCloud(pcd_final, final_h, "final cloud");
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

int main(int argc, char** argv)
{
    PointCloud::Ptr cloud_src_o(new PointCloud); // ԭʼ���ƣ�����׼
    pcl::io::loadPCDFile("D:\\PCLProjects\\pointcloud\\d1.pcd", *cloud_src_o);
    PointCloud::Ptr cloud_tgt_o(new PointCloud); // Ŀ�����
    pcl::io::loadPCDFile("D:\\PCLProjects\\pointcloud\\d2.pcd", *cloud_tgt_o);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // ȥ��NaNֵ
    std::vector<int> indices_src;
    pcl::removeNaNFromPointCloud(*cloud_src_o, *cloud_src_o, indices_src);

    // �����˲�
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(0.08, 0.08, 0.08);
    voxel_grid.setInputCloud(cloud_src_o);
    PointCloud::Ptr cloud_src(new PointCloud);
    voxel_grid.filter(*cloud_src);
    std::cout << "down size *cloud_src_o from " << cloud_src_o->size() << " to " << cloud_src->size() << std::endl;

    // ���㷨��
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_src;
    ne_src.setInputCloud(cloud_src);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree<pcl::PointXYZ>());
    ne_src.setSearchMethod(tree_src);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud<pcl::Normal>);
    ne_src.setRadiusSearch(0.02);
    ne_src.compute(*cloud_src_normals);

    // ȥ��NaNֵ
    std::vector<int> indices_tgt;
    pcl::removeNaNFromPointCloud(*cloud_tgt_o, *cloud_tgt_o, indices_tgt);

    // �����˲�
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_2;
    voxel_grid_2.setLeafSize(0.08, 0.08, 0.08);
    voxel_grid_2.setInputCloud(cloud_tgt_o);
    PointCloud::Ptr cloud_tgt(new PointCloud);
    voxel_grid_2.filter(*cloud_tgt);
    std::cout << "down size *cloud_tgt_o from " << cloud_tgt_o->size() << " to " << cloud_tgt->size() << std::endl;

    // ���㷨��
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_tgt;
    ne_tgt.setInputCloud(cloud_tgt);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree<pcl::PointXYZ>());
    ne_tgt.setSearchMethod(tree_tgt);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud<pcl::Normal>);
    ne_tgt.setRadiusSearch(0.02);
    ne_tgt.compute(*cloud_tgt_normals);

    // ����FPFH����
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_src;
    fpfh_src.setInputCloud(cloud_src);
    fpfh_src.setInputNormals(cloud_src_normals);
    pcl::search::KdTree<PointT>::Ptr tree_src_fpfh(new pcl::search::KdTree<PointT>);
    fpfh_src.setSearchMethod(tree_src_fpfh);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
    fpfh_src.setRadiusSearch(0.05);
    fpfh_src.compute(*fpfhs_src);

    // ����FPFH����
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_tgt;
    fpfh_tgt.setInputCloud(cloud_tgt);
    fpfh_tgt.setInputNormals(cloud_tgt_normals);
    pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<PointT>);
    fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
    fpfh_tgt.setRadiusSearch(0.05);
    fpfh_tgt.compute(*fpfhs_tgt);

    // ʹ��SAC-IA�㷨���г�ʼ��׼
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
    scia.setInputSource(cloud_src);
    scia.setInputTarget(cloud_tgt);
    scia.setSourceFeatures(fpfhs_src);
    scia.setTargetFeatures(fpfhs_tgt);
    PointCloud::Ptr sac_result(new PointCloud);
    scia.align(*sac_result);
    Eigen::Matrix4f sac_trans = scia.getFinalTransformation();

    // ʹ��ICP�㷨���н�һ������׼
    PointCloud::Ptr icp_result(new PointCloud);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_src);
    icp.setInputTarget(cloud_tgt_o);
    icp.setMaxCorrespondenceDistance(0.04);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-20);
    icp.setEuclideanFitnessEpsilon(0.1);
    icp.align(*icp_result, sac_trans);


    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "��ʱ��: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
    Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
    std::cout << "��ת����" << std::endl << icp_trans << std::endl;

    // ��ԭʼ���Ƹ���ICP�任��Ľ������ת��
    pcl::transformPointCloud(*cloud_src_o, *icp_result, icp_trans);

    // ���ӻ����
    visualize_pcd(cloud_src_o, cloud_tgt_o, icp_result);
    return (0);
}
