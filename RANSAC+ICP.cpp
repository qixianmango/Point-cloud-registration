#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh_omp.h>     // fpfh加速计算的omp(多核并行计算)
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/registration/ia_ransac.h>//采样一致性
#include <pcl/registration/icp.h>      //icp配准
#include <pcl/visualization/pcl_visualizer.h> 
#include <boost/thread/thread.hpp>
using namespace std;
#pragma region
void extract_keypoint(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& keypoint,
	float LeafSize = 0.04, float radius = 0.04, float threshold = 5) // 参数分别为：体素格网的大小，法向量计算半径，夹角阈值（度）
{
	//体素下采样
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_down(new pcl::PointCloud<pcl::PointXYZ>);
	voxel_grid.setInputCloud(cloud);
	voxel_grid.setLeafSize(LeafSize, LeafSize, LeafSize);
	voxel_grid.filter(*pcd_down);
	//计算每一个点的法向量
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	n.setInputCloud(pcd_down);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	n.setSearchMethod(tree);
	//设置KD树搜索半径
	// n.setRadiusSearch (0.03);
	n.setKSearch(10);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	n.compute(*normals);

	float Angle = 0.0;
	float Average_Sum_AngleK = 0.0;//定义邻域内K个点法向量夹角的平均值
	vector<int>indexes;
	//--------------计算法向量夹角及夹角均值----------------
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  //建立kdtree对象
	kdtree.setInputCloud(pcd_down); //设置需要建立kdtree的点云指针

	vector<int> pointIdxRadiusSearch;  //保存每个近邻点的索引
	vector<float> pointRadiusSquaredDistance;  //保存每个近邻点与查找点之间的欧式距离平方
	pcl::PointXYZ searchPoint;
	for (size_t i = 0; i < pcd_down->points.size(); ++i) {
		searchPoint = pcd_down->points[i];
		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			float Sum_AngleK = 0.0;//定义K个邻近的点法向夹角之和
			/*计算法向量的夹角*/
			for (size_t m = 0; m < pointIdxRadiusSearch.size(); ++m) {

				Eigen::Vector3f
					v1(normals->points[i].data_n[0],
						normals->points[i].data_n[1],
						normals->points[i].data_n[2]),

					v2(normals->points[pointIdxRadiusSearch[m]].data_n[0],
						normals->points[pointIdxRadiusSearch[m]].data_n[1],
						normals->points[pointIdxRadiusSearch[m]].data_n[2]);

				Angle = pcl::getAngle3D(v1, v2, true);

			}
			Sum_AngleK += Angle;//邻域夹角之和
			Average_Sum_AngleK = Sum_AngleK / pointIdxRadiusSearch.size();//邻域夹角均值
			//-----------------提取特征点--------------------
			float t = pcl::deg2rad(threshold);
			if (Average_Sum_AngleK > t) {
				indexes.push_back(i);
			}
		}
	}
	pcl::copyPointCloud(*pcd_down, indexes, *keypoint);

	cout << "提取的特征点个数:" << keypoint->points.size() << endl;

};

pcl::PointCloud<pcl::FPFHSignature33>::Ptr compute_fpfh_feature(pcl::PointCloud<pcl::PointXYZ>::Ptr& keypoint)
{
	// 法向量估计
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	n.setInputCloud(keypoint);
	n.setSearchMethod(tree);
	n.setKSearch(10);
	n.compute(*normals);
	//------------------FPFH估计-------------------------
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
	f.setNumberOfThreads(8); //指定8核计算
	f.setInputCloud(keypoint);
	f.setInputNormals(normals);
	f.setSearchMethod(tree);
	f.setRadiusSearch(0.05);
	f.compute(*fpfh);

	return fpfh;

}

// 可视化
void visualize_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr& source, pcl::PointCloud<pcl::PointXYZ>::Ptr& target, pcl::PointCloud<pcl::PointXYZ>::Ptr& regist)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Registration"));
	viewer->setBackgroundColor(255, 255, 255); // 设置背景为白色

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source, 0, 255, 0);
	viewer->addPointCloud(source, src_h, "source cloud"); // 显示原始点云，绿色

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target, 255, 0, 0);
	viewer->addPointCloud(target, tgt_h, "target cloud"); // 显示目标点云，蓝色

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transe(regist, 0, 0, 255);
	viewer->addPointCloud(regist, transe, "registered cloud"); // 显示转换后的点云，红色

	viewer->spin();
}

#pragma endregion
int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("D:\\PCLProjects\\pointcloud\\d1.pcd", *source);

	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("D:\\PCLProjects\\pointcloud\\d2.pcd", *target);
	if (source->empty() || target->empty()) // 使用empty()函数判断点云是否加载成功
	{
		cout << "请确认点云文件名称是否正确" << endl;
		return -1;
	}
	else
	{
		cout << "从目标点云读取 " << target->size() << " 个点" << endl;
		cout << "从源点云中读取 " << source->size() << " 个点" << endl;
	}

	// 声明计时器
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	//1、 提取特征点
	pcl::PointCloud<pcl::PointXYZ>::Ptr s_k(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr t_k(new pcl::PointCloud<pcl::PointXYZ>);
	extract_keypoint(source, s_k);
	extract_keypoint(target, t_k);
	//2、计算源点云和目标点云的FPFH
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr sk_fpfh = compute_fpfh_feature(s_k);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr tk_fpfh = compute_fpfh_feature(t_k);

	//3、SAC配准
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
	scia.setInputSource(s_k);
	scia.setInputTarget(t_k);
	scia.setSourceFeatures(sk_fpfh);
	scia.setTargetFeatures(tk_fpfh);
	scia.setMinSampleDistance(0.01);
	scia.setNumberOfSamples(100);
	scia.setCorrespondenceRandomness(6);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sac_result(new pcl::PointCloud<pcl::PointXYZ>);
	scia.align(*sac_result);
	//std::cout << "sac has converged:" << scia.hasConverged() << "  score: " << scia.getFitnessScore() << endl;
	Eigen::Matrix4f sac_trans;
	sac_trans = scia.getFinalTransformation();
	
	//4、KD树改进的ICP配准
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//kdTree 加速搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	tree1->setInputCloud(s_k);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(t_k);
	icp.setSearchMethodSource(tree1);
	icp.setSearchMethodTarget(tree2);
	icp.setInputSource(s_k);
	icp.setInputTarget(t_k);
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setMaximumIterations(35);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.01);
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_result(new pcl::PointCloud<pcl::PointXYZ>);
	icp.align(*icp_result, sac_trans);
	pcl::transformPointCloud(*source, *icp_result, icp.getFinalTransformation());

	// 计算总运行时间
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "总时间： = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
	std::cout << "变换矩阵：" << endl << sac_trans << endl;
	// 5、可视化
	visualize_registration(source, target, icp_result);
	return 0;
}

