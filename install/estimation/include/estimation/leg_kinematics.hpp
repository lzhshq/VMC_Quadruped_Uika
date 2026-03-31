#ifndef ESTIMATION_LEG_KINEMATICS_HPP
#define ESTIMATION_LEG_KINEMATICS_HPP

#include <Eigen/Dense>
#include "types.hpp"

namespace estimation {

/**
 * @brief 单腿运动学计算
 *
 * 计算3-DOF腿(hip, thigh, calf)的:
 * - 惯性矩阵 M(q)
 * - Coriolis向量 C^T(q,q̇)·q̇
 * - 重力向量 g(q)
 */
class LegKinematics {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LegKinematics();

    /**
     * @brief 计算关节空间惯性矩阵
     * @param q 关节角度 [hip, thigh, calf] (rad)
     * @return 3x3 惯性矩阵
     */
    Eigen::Matrix3d computeInertiaMatrix(const Eigen::Vector3d& q) const;

    /**
     * @brief 计算 Coriolis 项: C^T(q,q̇)·q̇
     * @param q 关节角度 (rad)
     * @param q_dot 关节速度 (rad/s)
     * @return 3x1 Coriolis向量
     */
    Eigen::Vector3d computeCoriolisTerm(const Eigen::Vector3d& q,
                                         const Eigen::Vector3d& q_dot) const;

    /**
     * @brief 计算重力项 g(q)
     * @param q 关节角度 (rad)
     * @return 3x1 重力向量
     */
    Eigen::Vector3d computeGravityTerm(const Eigen::Vector3d& q) const;

private:
    // 惯性矩阵元素的偏导数 (用于Christoffel符号)
    double dM11_dq1(double q2) const;
    double dM22_dq1(double q2) const;
    double dM22_dq2(double q2) const;
    double dM23_dq1(double q2) const;
    double dM23_dq2(double q2) const;
    double dM33_dq2() const;
};

}  // namespace estimation

#endif  // ESTIMATION_LEG_KINEMATICS_HPP
