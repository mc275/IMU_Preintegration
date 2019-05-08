#ifndef IMU_PREINTEGRATION_IMUDATA_H
#define IMU_PREINTEGRATION_IMUDATA_H

#include <Eigen/Core>

// IMU model can find in IMG/IMU_Modelling.png

/**
* IMU model https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
* the web IMU Parameter[sigma_g, sigma_gw, sigma_a, sigma_aw]
* For EuRoc dataset, according to V1_01_easy/imu0/sensor.yaml
* The params:
* sigma_g: 1.6968e-4       rad / s / sqrt(Hz)    n_g(gyroscope_noise_density)           // 连续时间gyr白噪声
* sigma_gw: 1.9393e-5      rad / s^2 / sqrt(Hz)  n_bg(gyroscope_random_walk)            // 连续时间gyr随机游走
* sigma_a: 2.0e-3          m / s^2 / sqrt(Hz)    na(accelerometer_noise_density)        // 连续时间acc白噪声
* sigma_aw: 3.0e-3         m / s^3 / sqrt(Hz)    n_ba(accelerometer_random_walk)        // 连续时间acc随机游走
*/

// 保存IMU传感器参数和原始数据
class IMUData
{

public:
    // 成员变量中包含Eigen对象时，需要使用以下宏重载new保证指针对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IMUData() = default;

    IMUData(const double &gx, const double &gy, const double &gz,
            const double &ax, const double &ay, const double &az,
            const double &t);

public:
    // IMU原始数据
    Eigen::Vector3d gyr_;
    Eigen::Vector3d acc_;
    // 数据时间戳
    double timestamp_;


    /**IMU传感器参数，无需多次设置 **/
    // IMU传感器连续高斯白噪声
    static double sigma_gyr_;
    static double sigma_acc_;

    // IMU传感器连续随机游走
    static double sigma_gyr_walk_;
    static double sigma_acc_walk_;

    // IMU传感器数据采集间隔
    static double delta_t;


    // 离散参数和连续参数对应关系见IMU模型介绍
    // IMU传感器的离散随机游走的平方
    static double gyr_noise_rw2_;
    static double acc_noise_rw2_;

    // IMU传感器的离散噪声的平方
    static double gyr_bias_rw2_;
    static double acc_bias_rw2_;


    // IMU传感器的离散高斯白噪声矩阵，认为三轴各向同性
    static Eigen::Matrix3d gyr_meas_cov_;
    static Eigen::Matrix3d acc_meas_cov_;

    // IMU传感器的离散随机游走矩阵，认为三轴各向同性
    static Eigen::Matrix3d gyr_bias_rw_cov_;
    static Eigen::Matrix3d acc_bias_rw_cov_;

};




#endif //IMU_PREINTEGRATION_IMUDATA_H
