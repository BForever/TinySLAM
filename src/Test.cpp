//
// Created by 范宏昌 on 2020/1/14.
//

#include "Test.h"

#include <ctime>
#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.h>

using namespace Eigen;
using namespace std;

#define DIM 30
/// 本程序演示sophus的基本用法
namespace TinySLAM {
int Test::useSophus() {

    // 沿Z轴转90度的旋转矩阵
    Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
    // 或者四元数
    Quaterniond q(R);
    Sophus::SO3 SO3_R(R);              // Sophus::SO3d可以直接从旋转矩阵构造
    Sophus::SO3 SO3_q(q);              // 也可以通过四元数构造
    // 二者是等价的
    cout << "SO(3) from matrix:\n" << SO3_R.matrix() << endl;
    cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << endl;
    cout << "they are equal" << endl;

    // 使用对数映射获得它的李代数
    Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl;
    // hat 为向量到反对称矩阵
    cout << "so3 hat=\n" << Sophus::SO3::hat(so3) << endl;
    // 相对的，vee为反对称到向量
    cout << "so3 hat vee= " << Sophus::SO3::vee(Sophus::SO3::hat(so3)).transpose() << endl;

    // 增量扰动模型的更新
    Vector3d update_so3(1e-4, 0, 0); //假设更新量为这么多
    Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3) * SO3_R;
    cout << "SO3 updated = \n" << SO3_updated.matrix() << endl;

    cout << "*******************************" << endl;
    // 对SE(3)操作大同小异
    Vector3d t(1, 0, 0);           // 沿X轴平移1
    Sophus::SE3 SE3_Rt(R, t);           // 从R,t构造SE(3)
    Sophus::SE3 SE3_qt(q, t);            // 从q,t构造SE(3)
    cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << endl;
    cout << "SE3 from q,t= \n" << SE3_qt.matrix() << endl;
    // 李代数se(3) 是一个六维向量，方便起见先typedef一下
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    cout << "se3 = " << se3.transpose() << endl;
    // 观察输出，会发现在Sophus中，se(3)的平移在前，旋转在后.
    // 同样的，有hat和vee两个算符
    cout << "se3 hat = \n" << Sophus::SE3::hat(se3) << endl;
    cout << "se3 hat vee = " << Sophus::SE3::vee(Sophus::SE3::hat(se3)).transpose() << endl;

    // 最后，演示一下更新
    Vector6d update_se3; //更新量
    update_se3.setZero();
    update_se3(0, 0) = 1e-4;
    Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3) * SE3_Rt;
    cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;

    return 0;
}

void Test::useGeo() {

    // Eigen/Geometry 模块提供了各种旋转和平移的表示
    // 3D 旋转矩阵直接使用 Matrix3d 或 Matrix3f
    Matrix3d rotation_matrix = Matrix3d::Identity();
    // 旋转向量使用 AngleAxis, 它底层不直接是Matrix，但运算可以当作矩阵（因为重载了运算符）
    AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));     //沿 Z 轴旋转 45 度
    cout.precision(3);
    cout << "rotation matrix =\n" << rotation_vector.matrix() << endl;   //用matrix()转换成矩阵
    // 也可以直接赋值
    rotation_matrix = rotation_vector.toRotationMatrix();
    // 用 AngleAxis 可以进行坐标变换
    Vector3d v(1, 0, 0);
    Vector3d v_rotated = rotation_vector * v;
    cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << endl;
    // 或者用旋转矩阵
    v_rotated = rotation_matrix * v;
    cout << "(1,0,0) after rotation (by matrix) = " << v_rotated.transpose() << endl;

    // 欧拉角: 可以将旋转矩阵直接转换成欧拉角
    Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX顺序，即yaw-pitch-roll顺序
    cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

    // 欧氏变换矩阵使用 Eigen::Isometry
    Isometry3d T = Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
    T.rotate(rotation_vector);                                     // 按照rotation_vector进行旋转
    T.pretranslate(Vector3d(1, 3, 4));                     // 把平移向量设成(1,3,4)
    cout << "Transform matrix = \n" << T.matrix() << endl;

    // 用变换矩阵进行坐标变换
    Vector3d v_transformed = T * v;                              // 相当于R*v+t
    cout << "v tranformed = " << v_transformed.transpose() << endl;

    // 对于仿射和射影变换，使用 Eigen::Affine3d 和 Eigen::Projective3d 即可，略

    // 四元数
    // 可以直接把AngleAxis赋值给四元数，反之亦然
    Quaterniond q = Quaterniond(rotation_vector);
    cout << "quaternion from rotation vector = " << q.coeffs().transpose()
         << endl;   // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部
    // 也可以把旋转矩阵赋给它
    q = Quaterniond(rotation_matrix);
    cout << "quaternion from rotation matrix = " << q.coeffs().transpose() << endl;
    // 使用四元数旋转一个向量，使用重载的乘法即可
    v_rotated = q * v; // 注意数学上是qvq^{-1}
    cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;
    // 用常规向量乘法表示，则应该如下计算
    cout << "should be equal to " << (q * Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << endl;

    return;
}

void Test::test() {
    Matrix<double, DIM, DIM> mat;
    mat = Matrix<double, DIM, DIM>::Random();
    mat = mat.transpose() * mat;
    Matrix<double, DIM, 1> vec;
    vec = Matrix<double, DIM, 1>::Random();

    Matrix<double, DIM, 1> x;
    double duration;
    clock_t start = clock();
    x = mat.inverse() * vec;
    clock_t end = clock();
//    std::cout<<x<<std::endl;
    duration = 1000 * (end - start) / double(CLOCKS_PER_SEC);
    std::cout << "duration of inverse:" << duration << std::endl;

    start = clock();
    x = mat.colPivHouseholderQr().solve(vec);
    end = clock();
//    std::cout<<x<<std::endl;
    duration = 1000 * (end - start) / double(CLOCKS_PER_SEC);
    std::cout << "duration of QR:" << duration << std::endl;

    //cholesky分解
    start = clock();
    // 使得NN成为正定矩阵，才能分解

    x = mat.partialPivLu().solve(vec);
    end = clock();
//    std::cout<<x<<std::endl;
    std::cout << "time use in cholesky composition is " << 1000 * (end - start) / double(CLOCKS_PER_SEC) << "ms"
              << std::endl;

    //LU分解
    start = clock();
    x = mat.partialPivLu().solve(vec);
    end = clock();
//    std::cout<<x<<std::endl;
    std::cout << "time use in LU composition is       " << 1000 * (end - start) / double(CLOCKS_PER_SEC) << "ms"
              << std::endl;
}
}