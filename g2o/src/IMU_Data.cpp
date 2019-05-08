#include "IMU_Data.h"

// 初始化static 成员变量
// 设置IMU传感器参数，continue parameter（默认使用EUROC数据集）
double IMUData::sigma_gyr_ = 1.6968e-4;
double IMUData::sigma_acc_ = 2.0e-3;

double IMUData::sigma_gyr_walk_ = 1.9393e-5;
double IMUData::sigma_acc_walk_ = 3.0e-3;

double IMUData::delta_t = 0.005;


// 计算IMU离散传感器参数
double IMUData::gyr_noise_rw2_ = IMUData::sigma_gyr_ * IMUData::sigma_gyr_ / IMUData::delta_t;
double IMUData::acc_noise_rw2_ = IMUData::sigma_acc_ * IMUData::sigma_acc_ / IMUData::delta_t;

Eigen::Matrix3d IMUData::gyr_meas_cov_ = Eigen::Matrix3d::Identity() * IMUData::gyr_noise_rw2_;
Eigen::Matrix3d IMUData::acc_meas_cov_ = Eigen::Matrix3d::Identity() * IMUData::acc_noise_rw2_;

double IMUData::gyr_bias_rw2_ = IMUData::sigma_gyr_walk_ * IMUData::sigma_gyr_walk_ * IMUData::delta_t;
double IMUData::acc_bias_rw2_ = IMUData::sigma_acc_walk_ * IMUData::sigma_acc_walk_ * IMUData::delta_t;

Eigen::Matrix3d IMUData::gyr_bias_rw_cov_ = Eigen::Matrix3d::Identity() * IMUData::gyr_bias_rw2_;
Eigen::Matrix3d IMUData::acc_bias_rw_cov_ = Eigen::Matrix3d::Identity() * IMUData::acc_bias_rw2_;


IMUData::IMUData(const double &gx, const double &gy, const double &gz, const double &ax, const double &ay,
                 const double &az, const double &t) : gyr_(gx, gy, gz), acc_(ax, ay, az), timestamp_(t)
{

}



