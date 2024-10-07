#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h> // icp算法
#include <pcl/registration/correspondence_rejection_surface_normal.h> //设置法线夹角的头文件
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/angles.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>


using namespace std;

void cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals)
{

	//---------------------拼接点云数据与法线信息-------------------
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;//OMP加速
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	n.setNumberOfThreads(10);//设置openMP的线程数
	//n.setViewPoint(0,0,0);//设置视点，默认为（0，0，0）
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(10);//点云法向计算时，需要所搜的近邻点大小
	//n.setRadiusSearch(0.03);//半径搜素
	n.compute(*normals);//开始进行法向计
	//将点云数据与法向信息拼接
	pcl::concatenateFields(*cloud, *normals, *cloud_normals);
}

void visualize_three_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud3)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("三个点云可视化"));
	viewer->setBackgroundColor(255, 255, 255); // 设置背景色为白色

	// 添加第一个点云（绿色）
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud1_color(cloud1, 0, 255, 0);
	viewer->addPointCloud(cloud1, cloud1_color, "cloud1");

	// 添加第二个点云（蓝色）
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud2_color(cloud2, 255, 0, 0);
	viewer->addPointCloud(cloud2, cloud2_color, "cloud2");

	// 添加第三个点云（红色）
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud3_color(cloud3, 0, 0, 255);
	viewer->addPointCloud(cloud3, cloud3_color, "cloud3");

	// 显示点云并启动可视化循环
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}
}


int main()
{
	// 声明计时器
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	// --------------------加载源点云-----------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\d1.pcd", *source);
	cout << "从源点云中读取 " << source->size() << " 个点" << endl;
	// -------------------加载目标点云----------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\d2.pcd", *target);
	cout << "从目标点云中读取 " << target->size() << " 个点" << endl;
	//-----------------拼接点云与法线信息-------------------
	pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	cloud_with_normal(source, source_with_normals);
	pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	cloud_with_normal(target, target_with_normals);
	//-------------------------ICP--------------------------
	pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
	icp.setInputSource(source_with_normals);
	icp.setInputTarget(target_with_normals);
	icp.setTransformationEpsilon(1e-10);   // 为终止条件设置最小转换差异
	icp.setMaxCorrespondenceDistance(10);  // 设置对应点对之间的最大距离（此值对配准结果影响较大）
	// --------------添加法向量夹角约束---------------------
	pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr rej(new pcl::registration::CorrespondenceRejectorSurfaceNormal);
	float aipha = 10; // 夹角阈值
	auto threshold = cos(pcl::deg2rad(aipha));// 计算cos(10°)将夹角阈值转为余弦值
	rej->setThreshold(threshold);          // 设置法线之间的夹角阈值，输入的是夹角余弦值
	icp.addCorrespondenceRejector(rej);    // 添加法向量约束到ICP算法中
	icp.setEuclideanFitnessEpsilon(0.001); // 设置收敛条件是均方误差和小于阈值， 停止迭代；
	icp.setMaximumIterations(35);          // 最大迭代次数
	pcl::PointCloud<pcl::PointNormal>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointNormal>);
	icp.align(*icp_cloud);

	//cout << "\nICP has converged, score is " << icp.getFitnessScore() << endl;
		// 计算总运行时间
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "总时间： = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
	cout << "变换矩阵：\n" << icp.getFinalTransformation() << endl;
	// 使用创建的变换对为输入的源点云进行变换
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*source, *out_cloud, icp.getFinalTransformation());
	//pcl::io::savePCDFileASCII("save.pcd", *out_cloud);
	visualize_three_clouds(source, target, out_cloud);

	return 0;
}
