#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

using namespace std;
#pragma region
void CauculateMeanStd(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointXYZ& Mean, pcl::PointXYZ& Std)
{
    Mean = { 0,0,0 }; Std = { 0,0,0 };
    // --------------------计算x,y,z坐标各字段的均值------------------------------
    float X = 0.0, Y = 0.0, Z = 0.0;
    for (auto iter = cloud->begin(); iter != cloud->end(); ++iter)
    {
        X += (*iter).x;
        Y += (*iter).y;
        Z += (*iter).z;
    }
    Mean.x = X / cloud->size();
    Mean.y = Y / cloud->size();
    Mean.z = Z / cloud->size();
    // -------------------计算x,y,z坐标各字段的标准差------------------------------
    pcl::PointXYZ SquareData{ 0,0,0 };
    for (auto iter = cloud->begin(); iter != cloud->end(); iter++)
    {
        SquareData.x += pow((Mean.x - (*iter).x), 2);
        SquareData.y += pow((Mean.y - (*iter).y), 2);
        SquareData.z += pow((Mean.z - (*iter).z), 2);
    }

    Std.x = sqrt((SquareData.x) / cloud->size());
    Std.y = sqrt((SquareData.y) / cloud->size());
    Std.z = sqrt((SquareData.z) / cloud->size());

}
#pragma endregion

int main(int argc, char** argv)
{
    // --------------------------------加载点云数据-------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProjects\\data\\dragon\\dragon_2.pcd", *cloud) == -1)
    {
        PCL_ERROR("Could not read file\n");
    }
    // ------------------------------计算均值和标准差----------------------------

    pcl::PointXYZ Mean, Std;
    CauculateMeanStd(cloud, Mean, Std);

    cout << "点云坐标各字段的均值为：" << Mean << "标准差为：" << Std << endl;

    return 0;
}