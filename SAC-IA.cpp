#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>//�����²����˲�
#include <pcl/features/normal_3d_omp.h>//ʹ��OMP��Ҫ��ӵ�ͷ�ļ�
#include <pcl/features/fpfh_omp.h> //fpfh���ټ����omp(��˲��м���)
#include <pcl/registration/ia_ransac.h>//sac_ia�㷨
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
    //-------------------------����������-----------------------
    pointnormal::Ptr normals(new pointnormal);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    n.setInputCloud(input_cloud);
    n.setNumberOfThreads(8);//����openMP���߳���
    n.setSearchMethod(tree);
    n.setKSearch(10);
    n.compute(*normals);
    //------------------FPFH����-------------------------------
    fpfhFeature::Ptr fpfh(new fpfhFeature);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
    f.setNumberOfThreads(8); //ָ��8�˼���
    f.setInputCloud(input_cloud);
    f.setInputNormals(normals);
    f.setSearchMethod(tree);
    f.setKSearch(10);
    f.compute(*fpfh);

    return fpfh;

}

void visualize_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& target, pcl::PointCloud<pcl::PointXYZ>::Ptr& rotated)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("��׼���"));
    viewer->setBackgroundColor(255, 255, 255); // ���ñ���ɫΪ��ɫ

    // ���ԭʼ���ƣ���
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source, 0, 255, 0);
    viewer->addPointCloud(source, src_h, "source_cloud");

    // ���Ŀ����ƣ���
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target, 255, 0, 0);
    viewer->addPointCloud(target, tgt_h, "target_cloud");

    // ��Ӿ�����ת�ĵ��ƣ���
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rotated_h(rotated, 0, 0, 255);
    viewer->addPointCloud(rotated, rotated_h, "rotated_cloud");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
}


int main(int argc, char** argv)
{

    clock_t start, end, time;
    start = clock();
    pointcloud::Ptr source_cloud(new pointcloud);
    pointcloud::Ptr target_cloud(new pointcloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\d1.pcd", *source_cloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\pointcloud\\d2.pcd", *target_cloud);
    //---------------------------ȥ��Դ���Ƶ�NAN��------------------------
    vector<int> indices_src; //����ȥ���ĵ������
    pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices_src);
    cout << "remove *source_cloud nan" << endl;
    //-------------------------Դ�����²����˲�-------------------------
    pcl::VoxelGrid<pcl::PointXYZ> vs;
    vs.setLeafSize(0.05, 0.05, 0.05);
    vs.setInputCloud(source_cloud);
    pointcloud::Ptr source(new pointcloud);
    vs.filter(*source);
    cout << "down size *source_cloud from " << source_cloud->size() << " to " << source->size() << endl;

    //--------------------------ȥ��Ŀ����Ƶ�NAN��--------------------
    vector<int> indices_tgt; //����ȥ���ĵ������
    pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, indices_tgt);
    cout << "remove *target_cloud nan" << endl;
    //----------------------Ŀ������²����˲�-------------------------
    pcl::VoxelGrid<pcl::PointXYZ> vt;
    vt.setLeafSize(0.05, 0.05, 0.05);
    vt.setInputCloud(target_cloud);
    pointcloud::Ptr target(new pointcloud);
    vt.filter(*target);
    cout << "down size *target_cloud from " << target_cloud->size() << " to " << target->size() << endl;
    //---------------����Դ���ƺ�Ŀ����Ƶ�FPFH------------------------
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source, tree);
    fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target, tree);

    //--------------����һ����SAC_IA��ʼ��׼----------------------------
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(source);
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(target);
    sac_ia.setTargetFeatures(target_fpfh);
    sac_ia.setMinSampleDistance(0.05);//��������֮�����С����
    //sac_ia.setNumberOfSamples(200);  //����ÿ�ε���������ʹ�õ�������������ʡ��,�ɽ�ʡʱ��
    sac_ia.setCorrespondenceRandomness(4); //��ѡ�����������Ӧʱ������Ҫʹ�õ��ھӵ�����;
    //Ҳ���Ǽ���Э����ʱѡ��Ľ��ڵ��������ֵԽ��Э����Խ��ȷ�����Ǽ���Ч��Խ��.(��ʡ)
    //sac_ia.setErrorFunction();//��������ǿ�ѡ��
    pointcloud::Ptr align(new pointcloud);
    sac_ia.align(*align);
    end = clock();
    pcl::transformPointCloud(*source_cloud, *align, sac_ia.getFinalTransformation());
    // pcl::io::savePCDFile("crou_output.pcd", *align);
    cout << "��ʱ��: " << float(end - start) * 1000 / CLOCKS_PER_SEC << "ms" << endl;
    //cout << "\nSAC_IA has converged, score is " << sac_ia.getFitnessScore() << endl;
    cout << "�任����\n" << sac_ia.getFinalTransformation() << endl;
    //-------------------���ӻ�------------------------------------
    visualize_registration(source_cloud, target_cloud, align);
    return 0;
}
