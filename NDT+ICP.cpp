#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h> //ndtͷ�ļ�
#include <pcl/registration/icp.h> //icpͷ�ļ�
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <chrono>

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// Ԥ�������
void pretreat(PointCloud::Ptr& pcd_cloud, PointCloud::Ptr& pcd_down, float LeafSize = 0.01) {
	//ȥ��NAN��
	std::vector<int> indices_src; //����ȥ���ĵ������
	pcl::removeNaNFromPointCloud(*pcd_cloud, *pcd_cloud, indices_src);
	//std::cout << "ɾ�� *cloud_source nan" << endl;
	//�²����˲�
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(LeafSize, LeafSize, LeafSize);
	voxel_grid.setInputCloud(pcd_cloud);
	voxel_grid.filter(*pcd_down);
	//cout << "down size *cloud from " << pcd_cloud->size() << " to " << pcd_down->size() << endl;
};

// ���ƿ��ӻ�
void visualize_pcd(PointCloud::Ptr& pcd_src, PointCloud::Ptr& pcd_tgt, PointCloud::Ptr& pcd_final)
{
	pcl::visualization::PCLVisualizer viewer("registration Viewer");
	//ԭʼ������ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 0, 255, 0);
	//Ŀ����ƺ�ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 255, 0, 0);
	//ƥ��õĵ�����ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(pcd_final, 0, 0, 255);

	viewer.setBackgroundColor(255, 255, 255);
	viewer.setWindowName("NDT+ICP��׼");
	viewer.addPointCloud(pcd_src, src_h, "source cloud");
	viewer.addPointCloud(pcd_tgt, tgt_h, "target cloud");
	viewer.addPointCloud(pcd_final, final_h, "result cloud");
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
	//��������ָ��
	PointCloud::Ptr cloud_source(new PointCloud);
	PointCloud::Ptr cloud_target(new PointCloud);
	// ���ص����ļ�

	pcl::io::loadPCDFile("D:\\PCLProjects\\pointcloud\\d1.pcd", *cloud_source);
	pcl::io::loadPCDFile("D:\\PCLProjects\\pointcloud\\d2.pcd", *cloud_target);
	


	

	PointCloud::Ptr cloud_src(new PointCloud);
	PointCloud::Ptr cloud_tar(new PointCloud);
	pretreat(cloud_source, cloud_src);
	pretreat(cloud_target, cloud_tar);

	//NDT��׼
	pcl::NormalDistributionsTransform<PointT, PointT> ndt;
	PointCloud::Ptr cloud_ndt(new PointCloud);

	ndt.setStepSize(0.1);              // ΪMore-Thuente������������󲽳�
	ndt.setResolution(0.5);              // ����NDT����ṹ�ķֱ��ʣ�VoxelGridCovariance��
	ndt.setMaximumIterations(25);      // ����ƥ�������������
	ndt.setTransformationEpsilon(0.01);// Ϊ��ֹ����������Сת������

	// ����Դ�������ݼ�
	ndt.setInputSource(cloud_src); // ��Դ�������ݼ�����Ϊ����Դ

	// ����Ŀ��������ݼ�
	ndt.setInputTarget(cloud_tar); // ��Ŀ��������ݼ�����ΪĿ��

	ndt.align(*cloud_ndt);
	Eigen::Matrix4f ndt_trans = ndt.getFinalTransformation(); // ��������ֵ

	//icp��׼�㷨
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	PointCloud::Ptr cloud_icp_registration(new PointCloud);
	//���ò���
	icp.setInputSource(cloud_src);
	icp.setInputTarget(cloud_tar);
	icp.setMaxCorrespondenceDistance(1);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.01);
	icp.setMaximumIterations(25);
	icp.align(*cloud_icp_registration, ndt_trans);

	Eigen::Matrix4f icp_trans = icp.getFinalTransformation();

	// ����������ʱ��
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "��ʱ�䣺 = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;

	cout << icp_trans << endl;

	pcl::transformPointCloud(*cloud_source, *cloud_icp_registration, icp_trans);

	//���ӻ�
	visualize_pcd(cloud_source, cloud_target, cloud_icp_registration);

	// ������׼�ĵ���
	//pcl::io::savePCDFileBinary("D:\\LaTex\\article\\High-precision point calculation method\\pic\\NDT+ICP\\n1n2\\n1n2.pcd", *cloud_icp_registration);
	return 0;
}