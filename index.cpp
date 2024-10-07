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
	//����ԭʼ��������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\data\\nefu\\nefu_3.pcd", *cloudA);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\data\\nefu\\nefu_4.pcd", *cloudB);

	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
	core.setInputSource(cloudA);
	core.setInputTarget(cloudB);
	pcl::Correspondences all;
	//core.determineCorrespondences(all_correspondences,0.05);//ȷ�����������Ŀ�����֮��Ķ�Ӧ��ϵ��

	core.determineReciprocalCorrespondences(all);   //ȷ�����������Ŀ�����֮��Ľ�����Ӧ��ϵ��
	float sum = 0.0, sum_x = 0.0, sum_y = 0.0, sum_z = 0.0, rmse, rmse_x, rmse_y, rmse_z;
	vector<float>Co;
	for (size_t j = 0; j < all.size(); j++) {
		sum += all[j].distance;
		Co.push_back(all[j].distance);
		sum_x += pow((cloudB->points[all[j].index_match].x - cloudA->points[all[j].index_query].x), 2);
		sum_y += pow((cloudB->points[all[j].index_match].y - cloudA->points[all[j].index_query].y), 2);
		sum_z += pow((cloudB->points[all[j].index_match].z - cloudA->points[all[j].index_query].z), 2);
	}
	rmse = sqrt(sum / all.size());     //���������
	rmse_x = sqrt(sum_x / all.size()); //X������������
	rmse_y = sqrt(sum_y / all.size()); //Y������������
	rmse_z = sqrt(sum_z / all.size()); //Z������������
	vector<float>::iterator max = max_element(Co.begin(), Co.end());//��ȡ������Ķ�Ӧ��
	vector<float>::iterator min = min_element(Co.begin(), Co.end());//��ȡ��С����Ķ�Ӧ��
	cout << "ƥ���Ը���" << all.size() << endl;
	cout << "�������ֵ" << sqrt(*max) * 100 << "����" << endl;
	cout << "������Сֵ" << sqrt(*min) * 100 << "����" << endl;

	cout << "���������" << rmse * 100 << "����" << endl;
	cout << "X���������" << rmse_x * 100 << "����" << endl;
	cout << "Y���������" << rmse_y * 100 << "����" << endl;
	cout << "Z���������" << rmse_z * 100 << "����" << endl;

	return 0;
}
