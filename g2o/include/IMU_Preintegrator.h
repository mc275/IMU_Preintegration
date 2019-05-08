#ifndef IMU_PREINTEGRATION_IMUPREINTEGRATION_H
#define IMU_PREINTEGRATION_IMUPREINTEGRATION_H

#include "so3.h"
#include "IMU_Data.h"

typedef Eigen::Matrix<double, 9, 9> Matrix9d;

class IMUPreintegrator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 默认构造函数
    IMUPreintegrator();

    // 构造函数
    IMUPreintegrator(const IMUPreintegrator &preintegrator);


    /**
     * @brief 增量更新IMU预积分测量值
     *
     * @param omega     扣除bias后的陀螺仪数据
     * @param omega     扣除bias后的加速度计数据
     * @param delta_t   IMU数据周期
     * @return 无
        * @retval
     */
    void Update(const Eigen::Vector3d &omega, const Eigen::Vector3d &acc, const double &delta_t);



    /**
     * @brief 重置所有状态量，用于下次预积分
     *
     * @param 无
     * @return 无
        * @retval
     */
    void reset();



    /**
     * @brief 获取IMU预积分位置测量值
     *
     * @param 无
     * @return 无
        * @retval
     */
    inline Eigen::Vector3d GetDeltaP() const
    {
        return delta_p_;
    }

    /**
     * @brief 获取IMU预积分速度测量值
     *
     * @param 无
     * @return 无
        * @retval
     */
    inline Eigen::Vector3d GetDeltaV() const
    {
        return delta_v_;
    }

    /**
     * @brief 获取IMU预积分姿态测量值
     *
     * @param 无
     * @return 无
        * @retval
     */
    inline Eigen::Matrix3d GetDeltaRot() const
    {
        return delta_rot_;
    }

    /**
     * @brief 获取IMU预积分观测方程噪声误差不确定性
     *
     * @param 无
     * @return 无
        * @retval
     */
    inline Matrix9d GetCovPVRot() const
    {
        return cov_p_v_rot_;
    }

    /**
     * @brief 获取IMU预积分位置测量值相对于陀螺仪bias的jacobian
     *
     * @param 无
     * @return 无
        * @retval
     */
    inline Eigen::Matrix3d GetPJacoBiasgyr() const
    {
        return jacobian_p_bias_gyr_;
    }

    /**
     * @brief 获取IMU预积分位置测量值相对于加速度计bias的jacobian
     *
     * @param 无
     * @return 无
        * @retval
     */
    inline Eigen::Matrix3d GetPJacoBiasacc() const
    {
        return jacobian_p_bias_acc_;
    }

    /**
     * @brief 获取IMU预积分速度测量值相对于陀螺仪bias的jacobian
     *
     * @param 无
     * @return 无
        * @retval
     */
    inline Eigen::Matrix3d GetVJacoBiasgyr() const
    {
        return jacobian_v_bias_gyr_;
    }

    /**
     * @brief 获取IMU预积分速度测量值相对于加速度计bias的jacobian
     *
     * @param 无
     * @return 无
        * @retval
     */
    inline Eigen::Matrix3d GetVJacoBiasacc() const
    {
        return jacobian_v_bias_acc_;
    }

    /**
     * @brief 获取IMU预积分姿态测量值相对于陀螺仪bias的jacobian
     *
     * @param 无
     * @return 无
        * @retval
     */
    inline Eigen::Matrix3d GetRotJacoBiasgyr() const
    {
        return jacobian_rot_bias_gyr_;
    }


    /**
     * @brief 归一化四元数(保证同方向的归一化)
     *
     * @param q 输入四元数
     * @return 归一化的四元数
        * @retval
     */
    inline Eigen::Quaterniond NormalizeRotationQ(const Eigen::Quaterniond &q)
    {
        Eigen::Quaterniond quat(q);

        // 两个互为相反数的四元数表示同一个旋转, 统一方向
        if(quat.w() < 0)
        {
            quat.coeffs() *= -1;
        }

        return quat.normalized();
    }


    /**
     * @brief 归一化旋转矩阵保证正交性
     *
     * @param R 输入旋转矩阵
     * @return 归一化后的旋转矩阵
        * @retval
     */
    inline Eigen::Matrix3d NormalizeRotationM(const Eigen::Matrix3d &R)
    {
        Eigen::Quaterniond q(R);

        return NormalizeRotationQ(q).toRotationMatrix();
    }


private:

    // 关键帧间IMU的位置、速度和旋转预积分测量值，参考文献1的公式35,36,37
    Eigen::Vector3d delta_p_;           // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
    Eigen::Vector3d delta_v_;           // V_k+1 = V_k + R_k*a_k*dt
    Eigen::Matrix3d delta_rot_;         // R_k+1 = R_k*exp(w_k*dt). note: Rwc, Rwc'=Rwc*[w_body]x

    // 用于矫正bias更新后的IMU预积分测量值的Jacobian, 参考文献1的补充材料A20
    Eigen::Matrix3d jacobian_p_bias_gyr_;
    Eigen::Matrix3d jacobian_p_bias_acc_;
    Eigen::Matrix3d jacobian_v_bias_gyr_;
    Eigen::Matrix3d jacobian_v_bias_acc_;
    Eigen::Matrix3d jacobian_rot_bias_gyr_;

    // IMU预积分观测方程噪声误差的不确定度方差，参考文献1的补充材料A9
    // 误差项顺序为PVR
    Matrix9d cov_p_v_rot_;

    // 预积分数据的时间长度
    double delta_time_;

};


#endif //IMU_PREINTEGRATION_IMUPREINTEGRATION_H
