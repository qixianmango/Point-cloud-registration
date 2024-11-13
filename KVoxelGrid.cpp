#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

int main(int argc, char** argv)
{
    // ��¼����ʼʱ��
    auto program_start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\data\\rabbit\\rabbit_whole.pcd", *cloud);

    // ���������Ϣ
    std::cout << "�˲�ǰ��������: " << cloud->size() << " ����" << std::endl;

    // �����˲�
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.003f, 0.003f, 0.003f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*voxel_filtered);

    // K���������
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    pcl::PointIndicesPtr inds = std::make_shared<pcl::PointIndices>();

    for (size_t i = 0; i < voxel_filtered->points.size(); i++) {
        pcl::PointXYZ searchPoint;
        searchPoint.x = voxel_filtered->points[i].x;
        searchPoint.y = voxel_filtered->points[i].y;
        searchPoint.z = voxel_filtered->points[i].z;

        int K = 1;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            inds->indices.push_back(pointIdxNKNSearch[0]);
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr final_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, inds->indices, *final_filtered);

    // ����˲��������Ϣ
    std::cout << "�˲����������: " << final_filtered->size() << " ����" << std::endl;

    // ��¼�������ʱ��
    auto program_end = std::chrono::high_resolution_clock::now();

    // �����������ʱ��
    auto program_duration = std::chrono::duration_cast<std::chrono::milliseconds>(program_end - program_start).count();

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
    viewer.addPointCloud<pcl::PointXYZ>(final_filtered, "�˲���ĵ���", v2);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "�˲���ĵ���", v2);

    // ��ʾ���ӻ�����
    viewer.spin();

    return 0;
}