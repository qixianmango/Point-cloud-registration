#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>//**********
#include <pcl/common/utils.h>
using namespace std;
using PointT = pcl::PointXYZ;
/*   ����������Ƶ�ƽ�����ܶ�
  cloud:ָ��������Ƶ�ָ��;
  max_dist:����Ϊ����ĵ��������;
  nr_threads:Ҫʹ�õ��߳���(Ĭ��ֵ=1����������OpenMP��־ʱʹ��)*/
double getMeanPointDensity(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, float max_dist, int nr_threads)
{
    const float max_dist_sqr = max_dist * max_dist;
    const std::size_t s = cloud->points.size();

    pcl::search::KdTree <PointT> tree;
    tree.setInputCloud(cloud);

    float mean_dist = 0.f;
    int num = 0;
    std::vector <int> ids(2);
    std::vector <float> dists_sqr(2);


    //--------------���̼߳��ٿ�ʼ-------------
    pcl::utils::ignore(nr_threads);
#pragma omp parallel for \
  default(none) \
  shared(tree, cloud) \
  firstprivate(ids, dists_sqr) \
  reduction(+:mean_dist, num) \
  firstprivate(s, max_dist_sqr) \
  num_threads(nr_threads)

    //--------------���̼߳��ٽ���--------------
    for (int i = 0; i < 1000; i++)
    {
        tree.nearestKSearch((*cloud)[rand() % s], 2, ids, dists_sqr);
        if (dists_sqr[1] < max_dist_sqr)
        {
            mean_dist += std::sqrt(dists_sqr[1]);
            num++;
        }
    }

    return (mean_dist / num);
};

int
main(int argc, char** argv)
{

    // --------------------����Դ����-----------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\data\\rabbit\\rabbit_1.pcd", *source);

    double Density = getMeanPointDensity(source, 0.2, 1);
    cout << "���Ƶ�ƽ���ܶ�Ϊ��" << Density << endl;
    system("pause");
    return (0);
}
