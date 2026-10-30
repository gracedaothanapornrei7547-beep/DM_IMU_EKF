/**
 ******************************************************************************
 * @file    qekf.c
 * @brief   四元数 EKF 姿态解算（自包含实现，详见 qekf.h）
 ******************************************************************************
 */
#include "qekf.h"
#include <string.h>
#include <math.h>

QEKF_t QEKF;

#define RAD2DEG 57.295779513f
#define HALF_PI 1.5707963268f

/* ============ 定长矩阵运算(行主序) ============ */

/* C[ar×bc] = A[ar×ac] · B[ac×bc] */
static void m_mul(const float *A, int ar, int ac, const float *B, int bc, float *C)
{
    for (int r = 0; r < ar; r++)
        for (int c = 0; c < bc; c++)
        {
            float s = 0.0f;
            for (int k = 0; k < ac; k++)
                s += A[r * ac + k] * B[k * bc + c];
            C[r * bc + c] = s;
        }
}

/* AT[ac×ar] = (A[ar×ac])^T */
static void m_trans(const float *A, int ar, int ac, float *AT)
{
    for (int r = 0; r < ar; r++)
        for (int c = 0; c < ac; c++)
            AT[c * ar + r] = A[r * ac + c];
}

static void m_add(const float *A, const float *B, float *C, int n)
{
    for (int i = 0; i < n; i++) C[i] = A[i] + B[i];
}
static void m_sub(const float *A, const float *B, float *C, int n)
{
    for (int i = 0; i < n; i++) C[i] = A[i] - B[i];
}

/* 3×3 求逆(闭式),返回 0 成功,非0 表示奇异 */
static int m3_inv(const float *a, float *inv)
{
    float det = a[0] * (a[4] * a[8] - a[5] * a[7]) -
                a[1] * (a[3] * a[8] - a[5] * a[6]) +
                a[2] * (a[3] * a[7] - a[4] * a[6]);
    if (fabsf(det) < 1e-12f) return 1;
    float id = 1.0f / det;
    inv[0] =  (a[4] * a[8] - a[5] * a[7]) * id;
    inv[1] = -(a[1] * a[8] - a[2] * a[7]) * id;
    inv[2] =  (a[1] * a[5] - a[2] * a[4]) * id;
    inv[3] = -(a[3] * a[8] - a[5] * a[6]) * id;
    inv[4] =  (a[0] * a[8] - a[2] * a[6]) * id;
    inv[5] = -(a[0] * a[5] - a[2] * a[3]) * id;
    inv[6] =  (a[3] * a[7] - a[4] * a[6]) * id;
    inv[7] = -(a[0] * a[7] - a[1] * a[6]) * id;
    inv[8] =  (a[0] * a[4] - a[1] * a[3]) * id;
    return 0;
}

static float invSqrtf(float x) { return (x > 0.0f) ? 1.0f / sqrtf(x) : 0.0f; }

/* 初始协方差:四元数对角 1e5(初值不确定),零偏对角 100,其余弱耦合 0.1 */
static const float P_INIT[36] = {
    100000, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.1, 100000, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 100000, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 100000, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 100, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 100};

void QEKF_Init(float Q1, float Q2, float R, float lambda, float lpf)
{
    memset(&QEKF, 0, sizeof(QEKF));
    QEKF.inited = 1;
    QEKF.Q1 = Q1;
    QEKF.Q2 = Q2;
    QEKF.R = R;
    QEKF.lambda = (lambda > 1.0f) ? 1.0f : (lambda <= 0.0f ? 1.0f : lambda);
    QEKF.accLPFcoef = lpf;
    QEKF.ChiSquareTestThreshold = 1e-8f;
    QEKF.AdaptiveGainScale = 1.0f;

    QEKF.xhat[0] = 1.0f; /* q = [1,0,0,0] */
    QEKF.q[0] = 1.0f;
    memcpy(QEKF.P, P_INIT, sizeof(P_INIT));
}

void QEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    if (!QEKF.inited)
        QEKF_Init(10.0f, 0.001f, 10000000.0f, 1.0f, 0.0f);
    QEKF.dt = dt;

    /* ---- 1. 去(已估计的)零偏 ---- */
    QEKF.Gyro[0] = gx - QEKF.GyroBias[0];
    QEKF.Gyro[1] = gy - QEKF.GyroBias[1];
    QEKF.Gyro[2] = gz - QEKF.GyroBias[2];

    /* ---- 2. 状态转移 F:左上 4×4 四元数传播, 右下 2×2 单位阵; 右上 4×2 先置0 ---- */
    float hx = 0.5f * QEKF.Gyro[0] * dt;
    float hy = 0.5f * QEKF.Gyro[1] * dt;
    float hz = 0.5f * QEKF.Gyro[2] * dt;
    memset(QEKF.F, 0, sizeof(QEKF.F));
    QEKF.F[0] = 1; QEKF.F[7] = 1; QEKF.F[14] = 1; QEKF.F[21] = 1; QEKF.F[28] = 1; QEKF.F[35] = 1;
    QEKF.F[1] = -hx; QEKF.F[2] = -hy; QEKF.F[3] = -hz;
    QEKF.F[6] =  hx; QEKF.F[8] =  hz; QEKF.F[9] = -hy;
    QEKF.F[12] = hy; QEKF.F[13] = -hz; QEKF.F[15] = hx;
    QEKF.F[18] = hz; QEKF.F[19] = hy; QEKF.F[20] = -hx;

    /* ---- 3. 加速度一阶低通(lpf=0 时为直通)---- */
    if (QEKF.UpdateCount == 0)
    {
        QEKF.Accel[0] = ax; QEKF.Accel[1] = ay; QEKF.Accel[2] = az;
    }
    float c = QEKF.accLPFcoef;
    QEKF.Accel[0] = QEKF.Accel[0] * c / (dt + c) + ax * dt / (dt + c);
    QEKF.Accel[1] = QEKF.Accel[1] * c / (dt + c) + ay * dt / (dt + c);
    QEKF.Accel[2] = QEKF.Accel[2] * c / (dt + c) + az * dt / (dt + c);

    /* 量测 z = 单位化加速度(重力方向) */
    float aInv = invSqrtf(QEKF.Accel[0] * QEKF.Accel[0] + QEKF.Accel[1] * QEKF.Accel[1] + QEKF.Accel[2] * QEKF.Accel[2]);
    QEKF.z[0] = QEKF.Accel[0] * aInv;
    QEKF.z[1] = QEKF.Accel[1] * aInv;
    QEKF.z[2] = QEKF.Accel[2] * aInv;

    QEKF.gyro_norm = sqrtf(QEKF.Gyro[0] * QEKF.Gyro[0] + QEKF.Gyro[1] * QEKF.Gyro[1] + QEKF.Gyro[2] * QEKF.Gyro[2]);
    QEKF.accl_norm = sqrtf(QEKF.Accel[0] * QEKF.Accel[0] + QEKF.Accel[1] * QEKF.Accel[1] + QEKF.Accel[2] * QEKF.Accel[2]);
    QEKF.StableFlag = (QEKF.gyro_norm < 0.3f && QEKF.accl_norm > 9.3f && QEKF.accl_norm < 10.3f) ? 1 : 0;

    /* ---- 4. 先验状态:xhatminus = F · xhat(此时 F 右上为0,bias 不直接改 q) ---- */
    m_mul(QEKF.F, 6, 6, QEKF.xhat, 1, QEKF.xhatminus);

    /* 归一化先验四元数 */
    float qn = invSqrtf(QEKF.xhatminus[0] * QEKF.xhatminus[0] + QEKF.xhatminus[1] * QEKF.xhatminus[1] +
                        QEKF.xhatminus[2] * QEKF.xhatminus[2] + QEKF.xhatminus[3] * QEKF.xhatminus[3]);
    for (int i = 0; i < 4; i++) QEKF.xhatminus[i] *= qn;
    float q0 = QEKF.xhatminus[0], q1 = QEKF.xhatminus[1], q2 = QEKF.xhatminus[2], q3 = QEKF.xhatminus[3];

    /* 设置 F 右上 4×2 块(∂q/∂bias 雅可比),用于协方差传播 */
    QEKF.F[4]  =  q1 * dt / 2; QEKF.F[5]  =  q2 * dt / 2;
    QEKF.F[10] = -q0 * dt / 2; QEKF.F[11] =  q3 * dt / 2;
    QEKF.F[16] = -q3 * dt / 2; QEKF.F[17] = -q0 * dt / 2;
    QEKF.F[22] =  q2 * dt / 2; QEKF.F[23] = -q1 * dt / 2;

    /* 渐消:抑制零偏协方差过度收敛(lambda=1 时为恒等) + 限幅防发散 */
    QEKF.P[28] /= QEKF.lambda;
    QEKF.P[35] /= QEKF.lambda;
    if (QEKF.P[28] > 10000) QEKF.P[28] = 10000;
    if (QEKF.P[35] > 10000) QEKF.P[35] = 10000;

    /* 过程噪声 Q / 量测噪声 R */
    memset(QEKF.Q, 0, sizeof(QEKF.Q));
    QEKF.Q[0] = QEKF.Q[7] = QEKF.Q[14] = QEKF.Q[21] = QEKF.Q1 * dt;
    QEKF.Q[28] = QEKF.Q[35] = QEKF.Q2 * dt;
    memset(QEKF.R_mat, 0, sizeof(QEKF.R_mat));
    QEKF.R_mat[0] = QEKF.R_mat[4] = QEKF.R_mat[8] = QEKF.R;

    /* ---- 5. 先验协方差:Pminus = F P F^T + Q ---- */
    float FP[36], FPFT[36];
    m_mul(QEKF.F, 6, 6, QEKF.P, 6, FP);
    m_trans(QEKF.F, 6, 6, QEKF.FT);
    m_mul(FP, 6, 6, QEKF.FT, 6, FPFT);
    m_add(FPFT, QEKF.Q, QEKF.Pminus, 36);

    /* ---- 6. 观测雅可比 H(3×6):∂(重力方向)/∂q,对 bias 两列为0 ---- */
    float dq0 = 2 * q0, dq1 = 2 * q1, dq2 = 2 * q2, dq3 = 2 * q3;
    memset(QEKF.H, 0, sizeof(QEKF.H));
    QEKF.H[0]  = -dq2; QEKF.H[1]  =  dq3; QEKF.H[2]  = -dq0; QEKF.H[3]  = dq1;
    QEKF.H[6]  =  dq1; QEKF.H[7]  =  dq0; QEKF.H[8]  =  dq3; QEKF.H[9]  = dq2;
    QEKF.H[12] =  dq0; QEKF.H[13] = -dq1; QEKF.H[14] = -dq2; QEKF.H[15] = dq3;

    /* ---- 7. 量测更新(含卡方检验) ---- */
    m_trans(QEKF.H, 3, 6, QEKF.HT);
    float HP[18], HPHT[9], S[9], Sinv[9];
    m_mul(QEKF.H, 3, 6, QEKF.Pminus, 6, HP);      /* HP   = H·Pminus  (3×6) */
    m_mul(HP, 3, 6, QEKF.HT, 3, HPHT);            /* HPHT = H·Pminus·H^T (3×3) */
    m_add(HPHT, QEKF.R_mat, S, 9);                /* S    = HPHT + R */

    if (m3_inv(S, Sinv) != 0)
    {
        /* S 奇异(几乎不会发生),退化为仅预测 */
        memcpy(QEKF.xhat, QEKF.xhatminus, sizeof(QEKF.xhat));
        memcpy(QEKF.P, QEKF.Pminus, sizeof(QEKF.P));
    }
    else
    {
        /* 预测的重力方向 h(xhatminus) 与方向余弦 */
        float hvec[3];
        hvec[0] = 2 * (q1 * q3 - q0 * q2);
        hvec[1] = 2 * (q0 * q1 + q2 * q3);
        hvec[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        QEKF.OrientationCosine[0] = acosf(fabsf(hvec[0]));
        QEKF.OrientationCosine[1] = acosf(fabsf(hvec[1]));
        QEKF.OrientationCosine[2] = acosf(fabsf(hvec[2]));

        /* 残差 e = z - h */
        float e[3];
        m_sub(QEKF.z, hvec, e, 3);

        /* 卡方检验量 chi = e^T · S^-1 · e */
        float Sinv_e[3];
        m_mul(Sinv, 3, 3, e, 1, Sinv_e);
        QEKF.ChiSquare = e[0] * Sinv_e[0] + e[1] * Sinv_e[1] + e[2] * Sinv_e[2];

        float thr = QEKF.ChiSquareTestThreshold;
        int reject = 0;

        if (QEKF.ChiSquare < 0.5f * thr)
            QEKF.ConvergeFlag = 1;

        if (QEKF.ChiSquare > thr && QEKF.ConvergeFlag)
        {
            if (QEKF.StableFlag)
                QEKF.ErrorCount++;     /* 静止却过不了卡方,累计判发散 */
            else
                QEKF.ErrorCount = 0;

            if (QEKF.ErrorCount > 50)
            {
                /* 判定发散:强制重新量测更新 */
                QEKF.ConvergeFlag = 0;
                QEKF.AdaptiveGainScale = 1.0f;
            }
            else
            {
                reject = 1;            /* 运动期残差异常,本次仅预测 */
            }
        }
        else
        {
            /* 残差越小增益越大,否则更信预测 */
            if (QEKF.ChiSquare > 0.1f * thr && QEKF.ConvergeFlag)
                QEKF.AdaptiveGainScale = (thr - QEKF.ChiSquare) / (0.9f * thr);
            else
                QEKF.AdaptiveGainScale = 1.0f;
            QEKF.ErrorCount = 0;
        }

        if (reject)
        {
            memcpy(QEKF.xhat, QEKF.xhatminus, sizeof(QEKF.xhat));
            memcpy(QEKF.P, QEKF.Pminus, sizeof(QEKF.P));
        }
        else
        {
            /* 增益 K = Pminus·H^T·S^-1 (6×3) */
            float PHT[18];
            m_mul(QEKF.Pminus, 6, 6, QEKF.HT, 3, PHT);
            m_mul(PHT, 6, 3, Sinv, 3, QEKF.K);

            /* 自适应增益缩放 */
            for (int i = 0; i < 18; i++)
                QEKF.K[i] *= QEKF.AdaptiveGainScale;
            /* 零偏行(4,5)按方向余弦门控:对应轴越接近水平越可观测 */
            for (int i = 4; i < 6; i++)
                for (int j = 0; j < 3; j++)
                    QEKF.K[i * 3 + j] *= QEKF.OrientationCosine[i - 4] / HALF_PI;

            /* 修正量 corr = K·e */
            float corr[6];
            m_mul(QEKF.K, 6, 3, e, 1, corr);

            /* 零偏修正限幅,防异常跳变 */
            if (QEKF.ConvergeFlag)
            {
                float lim = 1e-2f * dt;
                for (int i = 4; i < 6; i++)
                {
                    if (corr[i] > lim) corr[i] = lim;
                    if (corr[i] < -lim) corr[i] = -lim;
                }
            }
            corr[3] = 0; /* 不修正 yaw(q3):重力观测不到航向 */

            m_add(QEKF.xhatminus, corr, QEKF.xhat, 6);

            /* 后验协方差 P = Pminus - K·H·Pminus */
            float KH[36], KHP[36];
            m_mul(QEKF.K, 6, 3, QEKF.H, 6, KH);
            m_mul(KH, 6, 6, QEKF.Pminus, 6, KHP);
            m_sub(QEKF.Pminus, KHP, QEKF.P, 36);
        }
    }

    /* ---- 8. 输出:归一化四元数 + 反解欧拉角 + 连续航向 ---- */
    float on = invSqrtf(QEKF.xhat[0] * QEKF.xhat[0] + QEKF.xhat[1] * QEKF.xhat[1] +
                        QEKF.xhat[2] * QEKF.xhat[2] + QEKF.xhat[3] * QEKF.xhat[3]);
    for (int i = 0; i < 4; i++) QEKF.xhat[i] *= on;

    QEKF.q[0] = QEKF.xhat[0];
    QEKF.q[1] = QEKF.xhat[1];
    QEKF.q[2] = QEKF.xhat[2];
    QEKF.q[3] = QEKF.xhat[3];
    QEKF.GyroBias[0] = QEKF.xhat[4];
    QEKF.GyroBias[1] = QEKF.xhat[5];
    QEKF.GyroBias[2] = 0.0f; /* z 轴零偏不可观 */

    float w = QEKF.q[0], x = QEKF.q[1], y = QEKF.q[2], zq = QEKF.q[3];
    /* 标准 ZYX 欧拉角:Roll(纵轴)=atan2 ±180、Pitch(横轴)=asin ±90、Yaw=atan2 ±180。
     * 干净独立:单轴转动互不串扰。代价是 pitch 锁死 ±90(欧拉角铁律,只能选一个轴折叠)。
     * 想三轴都满程会互相串(试过 dual-atan2),要任意姿态既满程又不串只能用四元数 imu_q。*/
    QEKF.Yaw   = atan2f(2.0f * (w * zq + x * y), 2.0f * (w * w + x * x) - 1.0f) * RAD2DEG;
    QEKF.Roll  = atan2f(2.0f * (w * x + y * zq), 2.0f * (w * w + zq * zq) - 1.0f) * RAD2DEG;
    QEKF.Pitch = asinf(-2.0f * (x * zq - w * y)) * RAD2DEG;

    if (QEKF.Yaw - QEKF.YawAngleLast > 180.0f)
        QEKF.YawRoundCount--;
    else if (QEKF.Yaw - QEKF.YawAngleLast < -180.0f)
        QEKF.YawRoundCount++;
    QEKF.YawTotalAngle = 360.0f * QEKF.YawRoundCount + QEKF.Yaw;
    QEKF.YawAngleLast = QEKF.Yaw;

    QEKF.UpdateCount++;
}
