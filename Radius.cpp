#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/time.h>

int main() {
    auto program_start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\data\\rabbit\\rabbit_whole.pcd", *cloud);

    // �����뾶�˲�������
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(0.01);// ���ð뾶Ϊ0.01m��Χ�����ٽ���
    ror.setMinNeighborsInRadius(170);// ���ò�ѯ�������㼯��С��170ɾ��
    ror.filter(*cloud_filtered);

    auto program_end = std::chrono::high_resolution_clock::now();
    auto program_duration = std::chrono::duration_cast<std::chrono::milliseconds>(program_end - program_start).count();

    // ����˲�ǰ������Ϣ
    std::cout << "�˲�ǰ��������: " << cloud->size() << " ����" << std::endl;
    // ����˲��������Ϣ
    std::cout << "�˲����������: " << cloud_filtered->size() << " ����" << std::endl;
    // �����������ʱ��
    std::cout << "��������ʱ��: " << program_duration << " ����" << std::endl;

    // �������ӻ�����
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    // ���������ӿ�
    int v1, v2;
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    // �����ӿ�����
    viewer.setBackgroundColor(0.0, 0.0, 0.0, v1);
    viewer.setBackgroundColor(0.0, 0.0, 0.0, v2);

    // ��ʾԭʼ����
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "ԭʼ����", v1);

    // ��ʾ�˲���ĵ���
    viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, "�˲���ĵ���", v2);

    // ��ʾ���ӻ�����
    viewer.spin();

    return 0;
}