#include <iostream>
#include <Eigen/Core>
#include <cmath> // Include cmath header for M_PI constant

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ����תƽ�ƾ��������ת�Ƕ�
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

    std::cout << "x����ת�Ƕȣ�" << ax << std::endl;
    std::cout << "y����ת�Ƕȣ�" << ay << std::endl;
    std::cout << "z����ת�Ƕȣ�" << az << std::endl;
}

int main() {
    // ����ʵ����תƽ�Ʋ���
    Eigen::Vector3f ANGLE_origin; // ʵ����ת�Ƕ�
    Eigen::Vector3f TRANS_origin; // ʵ��ƽ�ƾ���

    // ��������ʵ����ת�ǶȺ�ƽ�ƾ���
    ANGLE_origin << 10.0 * M_PI / 180, 20.0 * M_PI / 180,0.0 * M_PI / 180; // x��0���ȣ�y��0���ȣ�z��45��
    TRANS_origin << 0.02, 0.02, 0.01; 

    // ���辭���㷨��׼�����תƽ�ƾ���
    Eigen::Matrix4f icp_trans;
    icp_trans <<
        0.999198, 0.00527703, 0.0397136, 0.00433571,
        -0.00533001, 0.999985, 0.00126362, -0.00965065,
        -0.0397067, -0.00147347, 0.999212, 0.0142141,
        0, 0, 0, 1;

    Eigen::Vector3f ANGLE_result; // ��IMU�ó��ı任����
    matrix2angle(icp_trans, ANGLE_result);

    // �������
    double a_error_x = fabs(ANGLE_result(0)) - fabs(ANGLE_origin(0));
    double a_error_y = fabs(ANGLE_result(1)) - fabs(ANGLE_origin(1));
    double a_error_z = fabs(ANGLE_result(2)) - fabs(ANGLE_origin(2));

    double t_error_x = fabs(icp_trans(0, 3)) - fabs(TRANS_origin(0));
    double t_error_y = fabs(icp_trans(1, 3)) - fabs(TRANS_origin(1));
    double t_error_z = fabs(icp_trans(2, 3)) - fabs(TRANS_origin(2));

    // ������

    std::cout << "����ʵ����ת�Ƕ�:\n" << ANGLE_origin << std::endl;
    std::cout << "x����ת��� : " << a_error_x << "  y����ת��� : " << a_error_y << "  z����ת��� : " << a_error_z << std::endl;

    std::cout << "����ʵ��ƽ�ƾ���:\n" << TRANS_origin << std::endl;
    std::cout << "����õ���ƽ�ƾ൝" << std::endl
        << "x��ƽ��" << icp_trans(0, 3) << std::endl
        << "y��ƽ��" << icp_trans(1, 3) << std::endl
        << "z��ƽ��" << icp_trans(2, 3) << std::endl
        << "x��ƽ����� : " << t_error_x << "  y��ƽ����� : " << t_error_y << "  z��ƽ����� : " << t_error_z << std::endl;

    return 0;
}
