/**
 ******************************************************************************
 * @file    qekf.h
 * @brief   四元数 EKF 姿态解算（自包含移植，无 CMSIS-DSP / 无 malloc）
 *
 * 算法等价移植自 Wang Hongxi 的 QuaternionEKF（RoboMaster-C-Board-INS-Example）：
 *   - 6 状态 EKF：x = [q0 q1 q2 q3 | bx by]，bx/by 为陀螺 x/y 轴零偏在线估计
 *   - z 轴(yaw)零偏不可观(重力无法约束航向)，恒为 0，yaw 由去偏后的陀螺纯积分
 *   - 量测 = 单位化加速度(重力方向)，仅修正 roll/pitch 与 x/y 零偏
 *   - 卡方检验剔除运动期的加速度，自适应增益 + 方向余弦门控 + 零偏限幅
 * 与原版差异：用定长矩阵运算替换 arm_mat_*，去掉动态分配；行为一致。
 *
 * 单位：gyro rad/s，accel m/s²，dt s；输出欧拉角 deg。
 ******************************************************************************
 */
#ifndef QEKF_H
#define QEKF_H

#include <stdint.h>

typedef struct
{
    uint8_t  inited;
    uint8_t  ConvergeFlag;     /* 滤波器已收敛 */
    uint8_t  StableFlag;       /* 载体静止(角速度小且加速度≈g) */
    uint32_t ErrorCount;       /* 静止却连续未过卡方检验的次数(判发散) */
    uint32_t UpdateCount;

    float q[4];                /* 姿态四元数 [w x y z] */
    float GyroBias[3];         /* 陀螺零偏在线估计 rad/s (z 恒为0) */
    float Gyro[3];             /* 去零偏后的角速度 rad/s */
    float Accel[3];            /* (低通后)加速度 m/s² */

    float Roll, Pitch, Yaw;    /* 欧拉角 deg */
    float YawTotalAngle;       /* 连续航向角 deg(累计不跳变) */
    int16_t YawRoundCount;
    float YawAngleLast;

    /* 调参量 */
    float Q1;                  /* 四元数过程噪声 */
    float Q2;                  /* 陀螺零偏过程噪声 */
    float R;                   /* 加速度量测噪声 */
    float lambda;              /* 渐消因子(1=关闭) */
    float accLPFcoef;          /* 加速度一阶低通系数(0=直通) */

    float dt;
    float ChiSquare;
    float ChiSquareTestThreshold;
    float AdaptiveGainScale;
    float OrientationCosine[3];
    float gyro_norm, accl_norm;

    /* 滤波器矩阵存储(6 状态 / 3 量测，行主序) */
    float xhat[6], xhatminus[6];
    float P[36], Pminus[36];
    float F[36], FT[36], Q[36];
    float H[18], HT[18];
    float R_mat[9];
    float K[18];
    float z[3];
} QEKF_t;

extern QEKF_t QEKF;

/**
 * @brief 初始化 EKF
 * @param Q1     四元数过程噪声(参考 10)
 * @param Q2     陀螺零偏过程噪声(参考 0.001)
 * @param R      加速度量测噪声(参考 1e7)
 * @param lambda 渐消因子(参考 1，即关闭)
 * @param lpf    加速度低通系数(参考 0，即直通)
 */
void QEKF_Init(float Q1, float Q2, float R, float lambda, float lpf);

/**
 * @brief EKF 一步更新
 * @param gx,gy,gz 角速度 rad/s
 * @param ax,ay,az 加速度 m/s²
 * @param dt       距上次更新的时间 s
 */
void QEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

#endif /* QEKF_H */
