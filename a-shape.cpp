#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("D://PCLProjects//pointcloud//p2.pcd", *cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConcaveHull<pcl::PointXYZ> cavehull;
	cavehull.setInputCloud(cloud);
	cavehull.setAlpha(0.003);
	vector<pcl::Vertices> polygons;
	cavehull.reconstruct(*surface_hull, polygons);// �ؽ���Ҫ�ص�����

	pcl::PolygonMesh mesh;
	cavehull.reconstruct(mesh);// �ؽ���Ҫ�ص�mesh
	//pcl::io::saveOBJFile("object_mesh.obj", mesh);
	cerr << "Concave hull has: " << surface_hull->points.size()
		<< " data points." << endl;

	pcl::PCDWriter writer;
	writer.write("hull.pcd", *surface_hull, false);
	// ���ӻ�
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("hull"));
	viewer->setWindowName("alshape�����ع�");
	viewer->addPolygonMesh<pcl::PointXYZ>(surface_hull, polygons, "polyline");
	viewer->spin();

	return (0);
}

