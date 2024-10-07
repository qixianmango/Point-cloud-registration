#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h> // icp�㷨
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <chrono>

using namespace std;

void cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals)
{
    //-----------------ƴ�ӵ��������뷨����Ϣ---------------------
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

void visualize_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& target, pcl::PointCloud<pcl::PointXYZ>::Ptr& icp)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("��׼���"));
    viewer->setBackgroundColor(255, 255, 255); // ���ñ���ɫΪ��ɫ

    // ���ԭʼ���ƣ���ɫ��
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source, 0, 255, 0);
    viewer->addPointCloud(source, src_h, "source_cloud");

    // ���Ŀ����ƣ���ɫ��
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target, 255, 0, 0);
    viewer->addPointCloud(target, tgt_h, "target_cloud");

    // ���ICP��׼��ĵ��ƣ���ɫ��
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transe(icp, 0, 0, 255);
    viewer->addPointCloud(icp, transe, "icp_cloud");

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
    //----------------�㵽���icp������棩-----------------
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>p_icp;


    p_icp.setInputSource(source_with_normals);
    p_icp.setInputTarget(target_with_normals);
    p_icp.setTransformationEpsilon(1e-10);    // Ϊ��ֹ����������Сת������
    p_icp.setMaxCorrespondenceDistance(10);   // ���ö�Ӧ���֮��������루��ֵ����׼���Ӱ��ϴ󣩡�
    p_icp.setEuclideanFitnessEpsilon(0.001);  // �������������Ǿ�������С����ֵ�� ֹͣ������
    //p_icp.setUseSymmetricObjective(true);   // ����Ϊtrue���Ϊ��һ���㷨
    p_icp.setMaximumIterations(35);           // ����������
    pcl::PointCloud<pcl::PointNormal>::Ptr p_icp_cloud(new pcl::PointCloud<pcl::PointNormal>);
    p_icp.align(*p_icp_cloud);
   //cout << "\nICP has converged, score is " << p_icp.getFitnessScore() << endl;
        // ����������ʱ��
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "��ʱ�䣺 = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
    cout << "�任����\n" << p_icp.getFinalTransformation() << endl;
    // ʹ�ô����ı任��Ϊ�����Դ���ƽ��б任
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source, *out_cloud, p_icp.getFinalTransformation());
    //pcl::io::savePCDFileASCII("667.pcd", *out_cloud);

    visualize_registration(source, target, out_cloud);

    system("pause");
    return (0);
}

