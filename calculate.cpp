#include <iostream>
#include <Eigen/Core>
#include <cmath> // Include cmath header for M_PI constant

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 由旋转平移矩阵计算旋转角度
void matrix2angle(const Eigen::Matrix4f& result_trans, Eigen::Vector3f& result_angle) {
    double ax, ay, az;
    if (result_trans(2, 0) == 1 || result_trans(2, 0) == -1) {
        az = 0;
        double dlta;
        dlta = atan2(result_trans(0, 1), result_trans(0, 2));
        if (result_trans(2, 0) == -1) {
            ay = M_PI / 2;
            ax = az + dlta;
        }
        else {
            ay = -M_PI / 2;
            ax = -az + dlta;
        }
    }
    else {
        ay = -asin(result_trans(2, 0));
        ax = atan2(result_trans(2, 1) / cos(ay), result_trans(2, 2) / cos(ay));
        az = atan2(result_trans(1, 0) / cos(ay), result_trans(0, 0) / cos(ay));
    }
    result_angle << ax, ay, az;

    std::cout << "x轴旋转角度：" << ax << std::endl;
    std::cout << "y轴旋转角度：" << ay << std::endl;
    std::cout << "z轴旋转角度：" << az << std::endl;
}

int main() {
    // 输入实际旋转平移参数
    Eigen::Vector3f ANGLE_origin; // 实际旋转角度
    Eigen::Vector3f TRANS_origin; // 实际平移距离

    // 假设输入实际旋转角度和平移距离
    ANGLE_origin << 10.0 * M_PI / 180, 20.0 * M_PI / 180,0.0 * M_PI / 180; // x轴0弧度，y轴0弧度，z轴45度
    TRANS_origin << 0.02, 0.02, 0.01; 

    // 假设经过算法配准后的旋转平移矩阵
    Eigen::Matrix4f icp_trans;
    icp_trans <<
        0.999198, 0.00527703, 0.0397136, 0.00433571,
        -0.00533001, 0.999985, 0.00126362, -0.00965065,
        -0.0397067, -0.00147347, 0.999212, 0.0142141,
        0, 0, 0, 1;

    Eigen::Vector3f ANGLE_result; // 由IMU得出的变换矩阵
    matrix2angle(icp_trans, ANGLE_result);

    // 计算误差
    double a_error_x = fabs(ANGLE_result(0)) - fabs(ANGLE_origin(0));
    double a_error_y = fabs(ANGLE_result(1)) - fabs(ANGLE_origin(1));
    double a_error_z = fabs(ANGLE_result(2)) - fabs(ANGLE_origin(2));

    double t_error_x = fabs(icp_trans(0, 3)) - fabs(TRANS_origin(0));
    double t_error_y = fabs(icp_trans(1, 3)) - fabs(TRANS_origin(1));
    double t_error_z = fabs(icp_trans(2, 3)) - fabs(TRANS_origin(2));

    // 输出结果

    std::cout << "点云实际旋转角度:\n" << ANGLE_origin << std::endl;
    std::cout << "x轴旋转误差 : " << a_error_x << "  y轴旋转误差 : " << a_error_y << "  z轴旋转误差 : " << a_error_z << std::endl;

    std::cout << "点云实际平移距离:\n" << TRANS_origin << std::endl;
    std::cout << "计算得到的平移距禎" << std::endl
        << "x轴平移" << icp_trans(0, 3) << std::endl
        << "y轴平移" << icp_trans(1, 3) << std::endl
        << "z轴平移" << icp_trans(2, 3) << std::endl
        << "x轴平移误差 : " << t_error_x << "  y轴平移误差 : " << t_error_y << "  z轴平移误差 : " << t_error_z << std::endl;

    return 0;
}
