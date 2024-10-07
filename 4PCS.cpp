#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_fpcs.h> // 4PCS算法
#include <pcl/console/time.h>   // 控制台计算时间
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int
main(int argc, char** argv)
{
	pcl::console::TicToc time;
	// -----------------加载目标点云---------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\d2.pcd", *target_cloud) == -1)
	{
		PCL_ERROR("读取目标点云失败 \n");
		return (-1);
	}
	cout << "从目标点云中读取 " << target_cloud->size() << " 个点" << endl;

	// ------------------加载源点云---------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\d1.pcd", *source_cloud) == -1)
	{
		PCL_ERROR("读取源标点云失败 \n");
		return (-1);
	}
	cout << "从源点云中读取 " << source_cloud->size() << " 个点" << endl;
	time.tic();
	//--------------初始化4PCS配准对象-------------------
	pcl::registration::FPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> fpcs;
	fpcs.setInputSource(source_cloud);  // 源点云
	fpcs.setInputTarget(target_cloud);  // 目标点云
	fpcs.setApproxOverlap(0.9);         // 设置源和目标之间的近似重叠度。
	fpcs.setDelta(0.1);                // 设置常数因子delta，用于对内部计算的参数进行加权。
	//fpcs.setMaxComputationTime(100);  // 设置最大计算时间(以秒为单位)。
	fpcs.setNumberOfSamples(100);       // 设置验证配准效果时要使用的采样点数量
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcs(new pcl::PointCloud<pcl::PointXYZ>);
	fpcs.align(*pcs);                   // 计算变换矩阵
	cout << "FPCS配准用时： " << time.toc() << " ms" << endl;
	cout << "变换矩阵：" << fpcs.getFinalTransformation() << endl;
	// 使用创建的变换对为输入点云进行变换
	pcl::transformPointCloud(*source_cloud, *pcs, fpcs.getFinalTransformation());
	// 保存转换后的源点云作为最终的变换输出
	//  pcl::io::savePCDFileASCII ("ro.pcd", *pcs);

	// 初始化点云可视化对象
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("显示点云"));
	viewer->setBackgroundColor(255, 255, 255);  //设置背景颜色为黑色

	// 对目标点云着色可视化 (red).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(target_cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
	// 对源点云着色可视化 (blue).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>input_color(source_cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(source_cloud, input_color, "input cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input cloud");
	// 对转换后的源点云着色 (green)可视化.
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>output_color(pcs, 0, 0, 255);
	viewer->addPointCloud<pcl::PointXYZ>(pcs, output_color, "output cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");


	// 等待直到可视化窗口关闭
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}

	return (0);
}

