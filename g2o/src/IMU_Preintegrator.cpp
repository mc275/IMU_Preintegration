//
// Created by mc on 19-5-8.
//

#include "IMU_Preintegrator.h"
#include <fstream>

IMUPreintegrator::IMUPreintegrator()
{
    delta_p_.setZero();
    delta_v_.setZero();
    delta_rot_.setIdentity();

    jacobian_p_bias_gyr_.setZero();
    jacobian_p_bias_acc_.setZero();

    jacobian_v_bias_gyr_.setZero();
    jacobian_v_bias_acc_.setZero();

    jacobian_rot_bias_gyr_.setZero();

    cov_p_v_rot_.setZero();

    delta_time_ = 0.0;

}



IMUPreintegrator::IMUPreintegrator(const IMUPreintegrator &preintegrator) :
        delta_p_(preintegrator.delta_p_), delta_v_(preintegrator.delta_v_), delta_rot_(preintegrator.delta_rot_),
        jacobian_p_bias_gyr_(preintegrator.jacobian_p_bias_gyr_),
        jacobian_p_bias_acc_(preintegrator.jacobian_p_bias_acc_),
        jacobian_v_bias_gyr_(preintegrator.jacobian_v_bias_gyr_),
        jacobian_v_bias_acc_(preintegrator.jacobian_v_bias_acc_),
        jacobian_rot_bias_gyr_(preintegrator.jacobian_rot_bias_gyr_),
        cov_p_v_rot_(preintegrator.cov_p_v_rot_), delta_time_(preintegrator.delta_time_)
{

}



void IMUPreintegrator::Update(const Eigen::Vector3d &omega, const Eigen::Vector3d &acc, const double &delta_t)
{
    double dt2 = delta_t*delta_t;

    // 步骤1 计算IMU预积分旋转增量和Jacobian
    Eigen::Matrix3d dR = Sophus::SO3::exp(omega*delta_t).matrix();
    Eigen::Matrix3d Jr_R = Sophus::SO3::JacobianR(omega*delta_t);


    // 步骤2 计算IMU预积分观测方程噪声误差的不确定度方差
    // 参考文献1补充材料的公式A.7-A.9, 文中误差项顺序为RVP, 代码中为PVR
    Eigen::Matrix3d I3x3;
    I3x3.setIdentity();

    Matrix9d A;
    A.setIdentity();
    // block大小是3x3，起始位置(6,6)
    A.block<3,3>(6,6) = dR.transpose();

    A.block<3,3>(3,6) = -delta_rot_ * Sophus::SO3::hat(acc) * delta_t;

    A.block<3,3>(0,6) = -0.5 * delta_rot_ * Sophus::SO3::hat(acc) * dt2;

    A.block<3,3>(0,3) = I3x3 * delta_t;

    // 将论文中的B拆成两部分
    Eigen::Matrix<double, 9, 3> Bg;
    Bg.setZero();
    Bg.block<3,3>(6,0) = Jr_R * delta_t;

    Eigen::Matrix<double, 9, 3> Ca;
    Ca.setZero();
    Ca.block<3,3>(3,0) = delta_rot_ * delta_t;
    Ca.block<3,3>(0,0) = 0.5*delta_rot_*dt2;


    cov_p_v_rot_ = A * cov_p_v_rot_ * A.transpose() +
                   Bg * IMUData::gyr_meas_cov_ * Bg.transpose() +
                   Ca * IMUData::acc_meas_cov_ * Ca.transpose();

    // 步骤3 计算Jacobian, 用于矫正bias更新后的IMU预积分测量值
    // 补充材料的A.20,计算为IMU帧间数据增量信息

    // 在补充材料公式基础上进行递推
    jacobian_p_bias_acc_ += jacobian_v_bias_acc_ * delta_t - 0.5 * delta_rot_ * dt2;
    jacobian_p_bias_gyr_ += jacobian_v_bias_gyr_ * delta_t - 0.5 * delta_rot_ * Sophus::SO3::hat(acc) * jacobian_rot_bias_gyr_ * dt2;

    jacobian_v_bias_acc_ += - delta_rot_ * delta_t;
    jacobian_v_bias_gyr_ += -delta_rot_ * Sophus::SO3::hat(acc) * jacobian_rot_bias_gyr_ * delta_t;

    jacobian_rot_bias_gyr_ = dR.transpose() * jacobian_rot_bias_gyr_ - Jr_R * delta_t;          // 在补充材料公式基础上进行递推




    // 步骤4 计算IMU预积分测量值
    delta_p_ += delta_v_*delta_t + 0.5 * delta_rot_ * acc *dt2;
    delta_v_ += delta_rot_ * acc *delta_t;
    delta_rot_ = NormalizeRotationM(delta_rot_ * dR);
    delta_time_ += delta_t;


}



void IMUPreintegrator::reset()
{
    delta_p_.setZero();
    delta_v_.setZero();
    delta_rot_.setZero();

    jacobian_p_bias_gyr_.setZero();
    jacobian_p_bias_acc_.setZero();

    jacobian_v_bias_gyr_.setZero();
    jacobian_p_bias_acc_.setZero();

    jacobian_rot_bias_gyr_.setZero();

    cov_p_v_rot_.setZero();

    delta_time_ = 0.0;
}



