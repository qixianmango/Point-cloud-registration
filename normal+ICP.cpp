#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h> // icp�㷨
#include <pcl/registration/correspondence_rejection_surface_normal.h> //���÷��߼нǵ�ͷ�ļ�
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/angles.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>


using namespace std;

void cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals)
{

	//---------------------ƴ�ӵ��������뷨����Ϣ-------------------
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;//OMP����
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//����kdtree�����н��ڵ㼯����
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	n.setNumberOfThreads(10);//����openMP���߳���
	//n.setViewPoint(0,0,0);//�����ӵ㣬Ĭ��Ϊ��0��0��0��
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(10);//���Ʒ������ʱ����Ҫ���ѵĽ��ڵ��С
	//n.setRadiusSearch(0.03);//�뾶����
	n.compute(*normals);//��ʼ���з����
	//�����������뷨����Ϣƴ��
	pcl::concatenateFields(*cloud, *normals, *cloud_normals);
}

void visualize_three_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud3)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("�������ƿ��ӻ�"));
	viewer->setBackgroundColor(255, 255, 255); // ���ñ���ɫΪ��ɫ

	// ��ӵ�һ�����ƣ���ɫ��
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud1_color(cloud1, 0, 255, 0);
	viewer->addPointCloud(cloud1, cloud1_color, "cloud1");

	// ��ӵڶ������ƣ���ɫ��
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud2_color(cloud2, 255, 0, 0);
	viewer->addPointCloud(cloud2, cloud2_color, "cloud2");

	// ��ӵ��������ƣ���ɫ��
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud3_color(cloud3, 0, 0, 255);
	viewer->addPointCloud(cloud3, cloud3_color, "cloud3");

	// ��ʾ���Ʋ��������ӻ�ѭ��
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}
}


int main()
{
	// ������ʱ��
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	// --------------------����Դ����-----------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\d1.pcd", *source);
	cout << "��Դ�����ж�ȡ " << source->size() << " ����" << endl;
	// -------------------����Ŀ�����----------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\d2.pcd", *target);
	cout << "��Ŀ������ж�ȡ " << target->size() << " ����" << endl;
	//-----------------ƴ�ӵ����뷨����Ϣ-------------------
	pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	cloud_with_normal(source, source_with_normals);
	pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	cloud_with_normal(target, target_with_normals);
	//-------------------------ICP--------------------------
	pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
	icp.setInputSource(source_with_normals);
	icp.setInputTarget(target_with_normals);
	icp.setTransformationEpsilon(1e-10);   // Ϊ��ֹ����������Сת������
	icp.setMaxCorrespondenceDistance(10);  // ���ö�Ӧ���֮��������루��ֵ����׼���Ӱ��ϴ�
	// --------------��ӷ������н�Լ��---------------------
	pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr rej(new pcl::registration::CorrespondenceRejectorSurfaceNormal);
	float aipha = 10; // �н���ֵ
	auto threshold = cos(pcl::deg2rad(aipha));// ����cos(10��)���н���ֵתΪ����ֵ
	rej->setThreshold(threshold);          // ���÷���֮��ļн���ֵ��������Ǽн�����ֵ
	icp.addCorrespondenceRejector(rej);    // ��ӷ�����Լ����ICP�㷨��
	icp.setEuclideanFitnessEpsilon(0.001); // �������������Ǿ�������С����ֵ�� ֹͣ������
	icp.setMaximumIterations(35);          // ����������
	pcl::PointCloud<pcl::PointNormal>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointNormal>);
	icp.align(*icp_cloud);

	//cout << "\nICP has converged, score is " << icp.getFitnessScore() << endl;
		// ����������ʱ��
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "��ʱ�䣺 = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
	cout << "�任����\n" << icp.getFinalTransformation() << endl;
	// ʹ�ô����ı任��Ϊ�����Դ���ƽ��б任
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*source, *out_cloud, icp.getFinalTransformation());
	//pcl::io::savePCDFileASCII("save.pcd", *out_cloud);
	visualize_three_clouds(source, target, out_cloud);

	return 0;
}
