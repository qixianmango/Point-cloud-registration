#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h> // icp�㷨
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

using namespace std;

int
main(int argc, char** argv)
{

    // --------------------����Դ����-----------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\p1.pcd", *source);

    
    // -------------------����Ŀ�����----------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\p2.pcd", *target);


    // ������ʱ��
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    
    //--------------------��ʼ��ICP����--------------------
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //----------------------icp���Ĵ���--------------------
    icp.setInputSource(source);            // Դ����
    icp.setInputTarget(target);            // Ŀ�����
    icp.setTransformationEpsilon(1e-10);   // Ϊ��ֹ����������Сת������
    icp.setMaxCorrespondenceDistance(0.05);  // ���ö�Ӧ���֮��������루��ֵ����׼���Ӱ��ϴ󣩡�
    icp.setEuclideanFitnessEpsilon(0.05);  // �������������Ǿ�������С����ֵ�� ֹͣ������
    icp.setMaximumIterations(100);           // ����������
    icp.setUseReciprocalCorrespondences(true);//����Ϊtrue,��ʹ���໥��Ӧ��ϵ
    // ������Ҫ�ĸ���任�Ա㽫�����Դ����ƥ�䵽Ŀ�����
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*icp_cloud);

    // ����������ʱ��
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "��ʱ�䣺 = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
    //cout << "\nICP has converged, score is " << icp.getFitnessScore() << endl;
    cout << "�任����\n" << icp.getFinalTransformation() << endl;
    // ʹ�ô����ı任��Ϊ����Դ���ƽ��б任
    pcl::transformPointCloud(*source, *icp_cloud, icp.getFinalTransformation());

    // ----------------���ƿ��ӻ�----------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("ICP��׼���"));
    viewer->setBackgroundColor(1.0, 1.0, 1.0);  // ���ñ�����ɫΪ��ɫ
    // ��Ŀ�������ɫ���ӻ� (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(target, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(target, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    // ��Դ������ɫ���ӻ� (blue).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>source_color(source, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(source, source_color, "source cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");

    // ��ת�����Դ������ɫ (green)���ӻ�.
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>icp_color(icp_cloud, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(icp_cloud, icp_color, "icp cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "icp cloud");

    //pcl::io::savePCDFile("D:\\LaTex\\article\\High-precision point calculation method\\pic\\ICP\\n1n2\\n1n2.pcd", *icp_cloud);
    // �������ӻ�

    viewer->initCameraParameters();   //��ʼ������ͷ����
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    system("pause");

    return (0);
}