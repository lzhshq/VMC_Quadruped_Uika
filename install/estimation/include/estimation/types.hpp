/**
 * @file types.hpp
 * @brief 四足机器人腿部参数定义
 *
 * 本文件定义了进行广义动量(GM)触地检测所需的物理参数。
 *
 * 【GM触地检测原理】
 * 通过广义动量观测器估计外部作用力:
 *   τ_K = τ_measured + C^T·q̇ - g
 * 当 |τ_K| 超过阈值时,判定为触地
 *
 * 【参数来源】
 * - 几何参数(L_*)：从CAD模型或测量获取
 * - 质量参数(M_*, COM)：从CAD模型或估计
 * - 惯性矩(I_*)：从CAD模型或近似圆柱模型计算
 * - 触地阈值：需通过实验标定
 *
 * 【标定建议】
 * 1. 静止站立时4条腿都应触地,检查阈值是否足够小
 * 2. 抬一条腿时该腿应检测为不触地,检查阈值是否足够大
 * 3. 行走时摆动相不应误报,支撑相不应漏报
 */

#ifndef ESTIMATION_TYPES_HPP
#define ESTIMATION_TYPES_HPP

#include <cmath>

namespace estimation {

/**
 * @brief 单腿物理参数结构体
 *
 * 包含进行动力学计算(惯性矩阵、Coriolis项、重力项)所需的全部参数。
 * 参数值为估算值,实际使用时需要根据具体机器人标定。
 */
struct LegConstants {

    //============================================================================
    // 【几何参数】- 腿部长度 (单位: 米)
    //============================================================================
    /** Hip关节相对于身体中心的偏移距离(横向) */
    static constexpr double L_HIP = 0.085;

    /** 大腿连杆长度(髋关节到膝关节) */
    static constexpr double L_THIGH = 0.320;

    /** 小腿连杆长度(膝关节到踝关节) */
    static constexpr double L_CALF = 0.320;

    //============================================================================
    // 【质量参数】- 连杆质量 (单位: 千克)
    //============================================================================
    /** Hip连杆质量(髋关节侧连杆) */
    static constexpr double M_HIP = 0.5;

    /** 大腿连杆质量 */
    static constexpr double M_THIGH = 1.0;

    /** 小腿连杆质量 */
    static constexpr double M_CALF = 0.5;

    //============================================================================
    // 【质心位置】- 相对于近端关节的距离 (单位: 米)
    //============================================================================
    /** Hip连杆质心到髋关节的距离 */
    static constexpr double HIP_COM = 0.025;

    /** 大腿连杆质心到髋关节的距离(取长度一半) */
    static constexpr double THIGH_COM = 0.160;  // = 0.5 * L_THIGH

    /** 小腿连杆质心到膝关节的距离(取长度一半) */
    static constexpr double CALF_COM = 0.160;   // = 0.5 * L_CALF

    //============================================================================
    // 【转动惯量】- 绕质心主轴的惯性矩 (单位: kg·m²)
    //
    // 近似计算方法(圆柱体模型):
    //   I_xx = I_yy = (1/12) * m * (3r² + h²)
    //   I_zz = (1/12) * m * (12r² + h²) ≈ (1/2) * m * r²  (细长杆近似)
    // 其中 r 为半径, h 为长度
    //============================================================================
    /** Hip连杆绕质心x轴惯性矩 */
    static constexpr double I_HIP_XX = 0.0005;
    /** Hip连杆绕质心y轴惯性矩 */
    static constexpr double I_HIP_YY = 0.0005;
    /** Hip连杆绕质心z轴惯性矩 */
    static constexpr double I_HIP_ZZ = 0.0001;

    /** 大腿连杆绕质心x轴惯性矩 */
    static constexpr double I_THIGH_XX = 0.001;
    /** 大腿连杆绕质心y轴惯性矩 */
    static constexpr double I_THIGH_YY = 0.010;
    /** 大腿连杆绕质心z轴惯性矩 */
    static constexpr double I_THIGH_ZZ = 0.010;

    /** 小腿连杆绕质心x轴惯性矩 */
    static constexpr double I_CALF_XX = 0.001;
    /** 小腿连杆绕质心y轴惯性矩 */
    static constexpr double I_CALF_YY = 0.005;
    /** 小腿连杆绕质心z轴惯性矩 */
    static constexpr double I_CALF_ZZ = 0.005;

    //============================================================================
    // 【物理常数】
    //============================================================================
    /** 重力加速度 (单位: m/s²) */
    static constexpr double G = 9.81;

    //============================================================================
    // 【电机参数】
    //============================================================================
    /** ROBSTRIDE_02电机最大扭矩 (单位: Nm) */
    static constexpr double TORQUE_LIMIT = 17.0;

    //============================================================================
    // 【触地检测参数】- 最关键的标定参数!
    //============================================================================
    /**
     * 触地检测默认阈值 (单位: Nm)
     *
     * 【选择依据】
     * - 阈值太小: 会产生误报(空中检测为触地)
     * - 阈值太大: 会产生漏报(触地检测为空中)
     *
     * 【推荐标定范围】
     * - 小型四足(10kg以下): 2-4 Nm
     * - 中型四足(10-30kg): 4-8 Nm
     * - 大型四足(30kg以上): 8-15 Nm
     * 
     *
     * 【标定方法】
     * 1. 从较大值开始(如10Nm)
     * 2. 机器人静止站立,逐渐减小阈值直到出现误报
     * 3. 记录当前值,再加0.5Nm作为安全裕度
     */
    static constexpr double CONTACT_THRESHOLD_DEFAULT = 5.0;

    /** 每条腿的关节数量(hip, thigh, calf) */
    static constexpr int JOINTS_PER_LEG = 3;
};

/**
 * @brief 腿部索引枚举
 *
 * 定义4条腿的编号,对应电机在MotorFeedback12消息中的顺序:
 * - FL (Front-Left):  motors 0,1,2   → can0
 * - FR (Front-Right): motors 3,4,5   → can1
 * - RL (Rear-Left):   motors 6,7,8   → can2
 * - RR (Rear-Right):  motors 9,10,11 → can3
 */
enum class LegIndex : int {
    FL = 0,  /**< 左前腿 (Front-Left) */
    FR = 1,  /**< 右前腿 (Front-Right) */
    RL = 2,  /**< 左后腿 (Rear-Left) */
    RR = 3   /**< 右后腿 (Rear-Right) */
};

/**
 * @brief 获取指定腿的第一个电机索引
 * @param leg 腿部索引
 * @return 电机编号(0-11)
 *
 * 例如 FL 返回 0, FR 返回 3, RL 返回 6, RR 返回 9
 */
inline int get_motor_offset(LegIndex leg) {
    return static_cast<int>(leg) * LegConstants::JOINTS_PER_LEG;
}

}  // namespace estimation

#endif  // ESTIMATION_TYPES_HPP
