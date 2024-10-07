#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_fpcs.h> // 4PCS�㷨
#include <pcl/console/time.h>   // ����̨����ʱ��
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int
main(int argc, char** argv)
{
	pcl::console::TicToc time;
	// -----------------����Ŀ�����---------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\d2.pcd", *target_cloud) == -1)
	{
		PCL_ERROR("��ȡĿ�����ʧ�� \n");
		return (-1);
	}
	cout << "��Ŀ������ж�ȡ " << target_cloud->size() << " ����" << endl;

	// ------------------����Դ����---------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\d1.pcd", *source_cloud) == -1)
	{
		PCL_ERROR("��ȡԴ�����ʧ�� \n");
		return (-1);
	}
	cout << "��Դ�����ж�ȡ " << source_cloud->size() << " ����" << endl;
	time.tic();
	//--------------��ʼ��4PCS��׼����-------------------
	pcl::registration::FPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> fpcs;
	fpcs.setInputSource(source_cloud);  // Դ����
	fpcs.setInputTarget(target_cloud);  // Ŀ�����
	fpcs.setApproxOverlap(0.9);         // ����Դ��Ŀ��֮��Ľ����ص��ȡ�
	fpcs.setDelta(0.1);                // ���ó�������delta�����ڶ��ڲ�����Ĳ������м�Ȩ��
	//fpcs.setMaxComputationTime(100);  // ����������ʱ��(����Ϊ��λ)��
	fpcs.setNumberOfSamples(100);       // ������֤��׼Ч��ʱҪʹ�õĲ���������
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcs(new pcl::PointCloud<pcl::PointXYZ>);
	fpcs.align(*pcs);                   // ����任����
	cout << "FPCS��׼��ʱ�� " << time.toc() << " ms" << endl;
	cout << "�任����" << fpcs.getFinalTransformation() << endl;
	// ʹ�ô����ı任��Ϊ������ƽ��б任
	pcl::transformPointCloud(*source_cloud, *pcs, fpcs.getFinalTransformation());
	// ����ת�����Դ������Ϊ���յı任���
	//  pcl::io::savePCDFileASCII ("ro.pcd", *pcs);

	// ��ʼ�����ƿ��ӻ�����
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("��ʾ����"));
	viewer->setBackgroundColor(255, 255, 255);  //���ñ�����ɫΪ��ɫ

	// ��Ŀ�������ɫ���ӻ� (red).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(target_cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
	// ��Դ������ɫ���ӻ� (blue).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>input_color(source_cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(source_cloud, input_color, "input cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input cloud");
	// ��ת�����Դ������ɫ (green)���ӻ�.
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>output_color(pcs, 0, 0, 255);
	viewer->addPointCloud<pcl::PointXYZ>(pcs, output_color, "output cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");


	// �ȴ�ֱ�����ӻ����ڹر�
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}

	return (0);
}

