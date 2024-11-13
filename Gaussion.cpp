#include <iostream>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/convolution_3d.h>  // ��˹�˲�
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

int main()
{
    // �������ݲ���ʼ��ʱ
    auto start_time = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("D:\\PCLProjects\\data\\rabbit\\rabbit_whole.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file pcd\n");
        return -1;
    }

    auto end_time_load = std::chrono::high_resolution_clock::now();
    auto load_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_load - start_time).count();

    // ���ڸ�˹�˺����ľ���˲�ʵ��
    pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> kernel;
    kernel.setSigma(1);                    // ��˹�����ı�׼������������Ŀ��
    kernel.setThresholdRelativeToSigma(5); // �������Sigma�����ľ�����ֵ
    kernel.setThreshold(5);             // ���þ�����ֵ���������������ֵ���迼��

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
    convolution.setKernel(kernel); // ���þ����
    convolution.setInputCloud(cloud);
    convolution.setNumberOfThreads(8);
    convolution.setSearchMethod(tree);
    convolution.setRadiusSearch(0.01);

    auto start_time_convolution = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr gaussianFilter(new pcl::PointCloud<pcl::PointXYZ>);
    convolution.convolve(*gaussianFilter);
    auto end_time_convolution = std::chrono::high_resolution_clock::now();
    auto convolution_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_convolution - start_time_convolution).count();

    // ���������Ϣ
    std::cout << "�˲�ǰ��������: " << cloud->size() << " ����" << std::endl;
    std::cout << "�˲����������: " << gaussianFilter->size() << " ����" << std::endl;
    std::cout << "��������ʱ��: " << convolution_duration << " ����" << std::endl;

    // ��ʾ����
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ShowCloud"));
    viewer->setWindowName("��˹�˲�");
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);
    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
    viewer->addText("Filtered point clouds", 10, 10, "v2_text", v2);

    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
    viewer->addPointCloud<pcl::PointXYZ>(gaussianFilter, "gaussianFilter", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "sample cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "gaussianFilter", v2);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}