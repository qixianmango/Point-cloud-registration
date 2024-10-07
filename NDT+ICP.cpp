#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h> //ndt头文件
#include <pcl/registration/icp.h> //icp头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <chrono>

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 预处理过程
void pretreat(PointCloud::Ptr& pcd_cloud, PointCloud::Ptr& pcd_down, float LeafSize = 0.01) {
	//去除NAN点
	std::vector<int> indices_src; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*pcd_cloud, *pcd_cloud, indices_src);
	//std::cout << "删除 *cloud_source nan" << endl;
	//下采样滤波
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(LeafSize, LeafSize, LeafSize);
	voxel_grid.setInputCloud(pcd_cloud);
	voxel_grid.filter(*pcd_down);
	//cout << "down size *cloud from " << pcd_cloud->size() << " to " << pcd_down->size() << endl;
};

// 点云可视化
void visualize_pcd(PointCloud::Ptr& pcd_src, PointCloud::Ptr& pcd_tgt, PointCloud::Ptr& pcd_final)
{
	pcl::visualization::PCLVisualizer viewer("registration Viewer");
	//原始点云绿色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 0, 255, 0);
	//目标点云红色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 255, 0, 0);
	//匹配好的点云蓝色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(pcd_final, 0, 0, 255);

	viewer.setBackgroundColor(255, 255, 255);
	viewer.setWindowName("NDT+ICP配准");
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
	// 声明计时器
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	//创建点云指针
	PointCloud::Ptr cloud_source(new PointCloud);
	PointCloud::Ptr cloud_target(new PointCloud);
	// 加载点云文件

	pcl::io::loadPCDFile("D:\\PCLProjects\\pointcloud\\d1.pcd", *cloud_source);
	pcl::io::loadPCDFile("D:\\PCLProjects\\pointcloud\\d2.pcd", *cloud_target);
	


	

	PointCloud::Ptr cloud_src(new PointCloud);
	PointCloud::Ptr cloud_tar(new PointCloud);
	pretreat(cloud_source, cloud_src);
	pretreat(cloud_target, cloud_tar);

	//NDT配准
	pcl::NormalDistributionsTransform<PointT, PointT> ndt;
	PointCloud::Ptr cloud_ndt(new PointCloud);

	ndt.setStepSize(0.1);              // 为More-Thuente线搜索设置最大步长
	ndt.setResolution(0.5);              // 设置NDT网格结构的分辨率（VoxelGridCovariance）
	ndt.setMaximumIterations(25);      // 设置匹配迭代的最大次数
	ndt.setTransformationEpsilon(0.01);// 为终止条件设置最小转换差异

	// 设置源点云数据集
	ndt.setInputSource(cloud_src); // 将源点云数据集设置为输入源

	// 设置目标点云数据集
	ndt.setInputTarget(cloud_tar); // 将目标点云数据集设置为目标

	ndt.align(*cloud_ndt);
	Eigen::Matrix4f ndt_trans = ndt.getFinalTransformation(); // 声明并赋值

	//icp配准算法
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	PointCloud::Ptr cloud_icp_registration(new PointCloud);
	//设置参数
	icp.setInputSource(cloud_src);
	icp.setInputTarget(cloud_tar);
	icp.setMaxCorrespondenceDistance(1);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.01);
	icp.setMaximumIterations(25);
	icp.align(*cloud_icp_registration, ndt_trans);

	Eigen::Matrix4f icp_trans = icp.getFinalTransformation();

	// 计算总运行时间
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "总时间： = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;

	cout << icp_trans << endl;

	pcl::transformPointCloud(*cloud_source, *cloud_icp_registration, icp_trans);

	//可视化
	visualize_pcd(cloud_source, cloud_target, cloud_icp_registration);

	// 保存配准的点云
	//pcl::io::savePCDFileBinary("D:\\LaTex\\article\\High-precision point calculation method\\pic\\NDT+ICP\\n1n2\\n1n2.pcd", *cloud_icp_registration);
	return 0;
}