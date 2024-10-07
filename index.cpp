#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include<cmath>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>
using namespace std;
int main()
{
	//加载原始点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\data\\nefu\\nefu_3.pcd", *cloudA);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\data\\nefu\\nefu_4.pcd", *cloudB);

	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
	core.setInputSource(cloudA);
	core.setInputTarget(cloudB);
	pcl::Correspondences all;
	//core.determineCorrespondences(all_correspondences,0.05);//确定输入点云与目标点云之间的对应关系：

	core.determineReciprocalCorrespondences(all);   //确定输入点云与目标点云之间的交互对应关系。
	float sum = 0.0, sum_x = 0.0, sum_y = 0.0, sum_z = 0.0, rmse, rmse_x, rmse_y, rmse_z;
	vector<float>Co;
	for (size_t j = 0; j < all.size(); j++) {
		sum += all[j].distance;
		Co.push_back(all[j].distance);
		sum_x += pow((cloudB->points[all[j].index_match].x - cloudA->points[all[j].index_query].x), 2);
		sum_y += pow((cloudB->points[all[j].index_match].y - cloudA->points[all[j].index_query].y), 2);
		sum_z += pow((cloudB->points[all[j].index_match].z - cloudA->points[all[j].index_query].z), 2);
	}
	rmse = sqrt(sum / all.size());     //均方根误差
	rmse_x = sqrt(sum_x / all.size()); //X方向均方根误差
	rmse_y = sqrt(sum_y / all.size()); //Y方向均方根误差
	rmse_z = sqrt(sum_z / all.size()); //Z方向均方根误差
	vector<float>::iterator max = max_element(Co.begin(), Co.end());//获取最大距离的对应点
	vector<float>::iterator min = min_element(Co.begin(), Co.end());//获取最小距离的对应点
	cout << "匹配点对个数" << all.size() << endl;
	cout << "距离最大值" << sqrt(*max) * 100 << "厘米" << endl;
	cout << "距离最小值" << sqrt(*min) * 100 << "厘米" << endl;

	cout << "均方根误差" << rmse * 100 << "厘米" << endl;
	cout << "X均方根误差" << rmse_x * 100 << "厘米" << endl;
	cout << "Y均方根误差" << rmse_y * 100 << "厘米" << endl;
	cout << "Z均方根误差" << rmse_z * 100 << "厘米" << endl;

	return 0;
}
