#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

int main()
{
    // ��¼����ʼʱ��
    auto program_start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // ��PCD�ļ����ص�������
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\data\\rabbit\\rabbit_whole.pcd", *cloud) == -1)
    {
        PCL_ERROR("�޷���ȡ�ļ�\n");
        return (-1);
    }

    // �����˲�������
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.005f, 0.005f, 0.005f);
    sor.filter(*cloud_filtered);

    // ��¼�������ʱ��
    auto program_end = std::chrono::high_resolution_clock::now();

    // �����������ʱ��
    auto program_duration = std::chrono::duration_cast<std::chrono::milliseconds>(program_end - program_start).count();

    // ���������Ϣ
    std::cout << "�˲�ǰ��������: " << cloud->size() << " ����" << std::endl;
    std::cout << "�˲����������: " << cloud_filtered->size() << " ����" << std::endl;
    std::cout << "��������ʱ��: " << program_duration << " ����" << std::endl;

    // �������ӻ�����
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    // ���������ӿڣ������ʾԭʼ���ƣ��ұ���ʾ�˲���ĵ���
    int v1, v2;
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    // �����ӿ�����
    viewer.setBackgroundColor(0.0, 0.0, 0.0, v1);
    viewer.setBackgroundColor(0.0, 0.0, 0.0, v2);

    // ��ʾԭʼ����
    viewer.addPointCloud(cloud, "ԭʼ����", v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "ԭʼ����", v1);

    // ��ʾ�˲���ĵ���
    viewer.addPointCloud(cloud_filtered, "�˲���ĵ���", v2);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "�˲���ĵ���", v2);

    // ��ʾ���ӻ�����
    viewer.spin();

    return 0;
}