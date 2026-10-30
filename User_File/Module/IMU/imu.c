/**
 ******************************************************************************
 * @file    imu.c
 * @brief   BMI088 裸机集成层（数据读取 + 恒温控制）
 *
 * 移植自达妙官方例程 CtrBoard-H7_IMU_TempCtrl 的 App/imu_temp_ctrl.c，
 * 由 FreeRTOS 任务 + 信号量改写为裸机：ACC_INT 外部中断置位数据就绪
 * 标志，主循环里调用 IMU_Task() 完成一次读取与温控 PID。
 ******************************************************************************
 */
#include "imu.h"
#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "spi.h"
#include "BMI088driver.h"
#include "qekf.h"
#include "bsp_dwt.h"
#include "bsp_usb.h"
#include <math.h>

/* ===== 恒温 PID 参数（与官方一致） ===== */
#define DES_TEMP   40.0f      /* 目标温度 °C */
#define KP         100.0f
#define KI         50.0f
#define KD         10.0f
#define MAX_OUT    500        /* 加热 PWM 输出上限（对应 TIM3 ARR=10000 时约 5% 占空比） */

/* ===== 对外数据 ===== */
float   imu_gyro[3]     = {0};
float   imu_accel[3]    = {0};
float   imu_temp        = 0.0f;
uint8_t imu_temp_ready  = 0;
uint8_t imu_init_status = 0xFF;        /* 0=成功；上电默认 0xFF=未初始化 */

/* ===== 姿态解算输出（四元数 EKF）===== */
float imu_yaw       = 0.0f;            /* 航向角 deg（无磁力计，长期缓慢漂移） */
float imu_pitch     = 0.0f;            /* 俯仰角 deg */
float imu_roll      = 0.0f;            /* 横滚角 deg */
float imu_yaw_total = 0.0f;            /* 连续航向角 deg（累计、不在±180跳变） */
float imu_q[4]      = {1, 0, 0, 0};    /* 姿态四元数 */

/* ===== 开机静止陀螺零偏标定结果 ===== */
float   gyro_offset[3] = {0, 0, 0};    /* 三轴静态零偏 rad/s，IMU_Update 里从原始角速度减去 */
uint8_t gyro_cal_ok    = 0;            /* 1=标定成功；0=开机时检测到运动，未标定(仅靠EKF在线估计) */
float   imu_dt         = 0.0f;         /* 最近一次 IMU_Update 的 dt(s)，供离线调参采集用 */

static uint32_t ekf_dwt_cnt = 0;       /* DWT 计时游标 */

/* ===== 内部状态 ===== */
volatile uint8_t imu_data_ready = 0;   /* 由 ACC_INT EXTI 置位 */
uint8_t  forceStop              = 0;   /* 置 1 可强制停止加热 */

static float out    = 0.0f;
static float err    = 0.0f;
static float err_l  = 0.0f;
static float err_ll = 0.0f;

/* ===== 静止零偏标定参数 ===== */
#define CAL_SAMPLES   1000     /* 采样数(每个约1ms) -> 约1秒标定窗口 */
#define CAL_PP_THRESH 0.05f    /* 峰峰阈值 rad/s(约2.9°/s)，超过判定为在运动 */
#define CAL_ATTEMPTS  5        /* 检测到运动时最多重试次数 */

/**
 * @brief 开机静止陀螺零偏标定：板子静止时采一段角速度求平均当作零偏。
 *        无磁力计时这是压低 yaw 漂移最关键的一步(EKF 只能在线估计 x/y 零偏，
 *        z/yaw 零偏不可观，必须靠这里量出来减掉)。
 * @note  采样期间持续调用 usb_task() 维持 CDC 活动。检测到运动则重试，
 *        多次仍失败则零偏置 0、gyro_cal_ok=0，交给 EKF 在线估计兜底。
 */
static void IMU_GyroCalibrate(void)
{
    float g[3], a[3], t;

    for (int attempt = 0; attempt < CAL_ATTEMPTS; attempt++)
    {
        float sum[3] = {0, 0, 0};
        float mn[3], mx[3];
        uint8_t moved = 0;

        BMI088_read(g, a, &t);
        for (int k = 0; k < 3; k++) { mn[k] = mx[k] = g[k]; }

        for (int i = 0; i < CAL_SAMPLES; i++)
        {
            BMI088_read(g, a, &t);
            for (int k = 0; k < 3; k++)
            {
                sum[k] += g[k];
                if (g[k] < mn[k]) mn[k] = g[k];
                if (g[k] > mx[k]) mx[k] = g[k];
            }
            usb_task();          /* 标定耗时约1秒，期间维持 USB CDC 不掉线 */
            HAL_Delay(1);
        }

        for (int k = 0; k < 3; k++)
            if (mx[k] - mn[k] > CAL_PP_THRESH) moved = 1;

        if (!moved)
        {
            for (int k = 0; k < 3; k++)
                gyro_offset[k] = sum[k] / (float)CAL_SAMPLES;
            gyro_cal_ok = 1;
            return;
        }
    }

    /* 多次都在动：放弃静态标定，零偏置0，由 EKF 在线估计兜底 */
    gyro_offset[0] = gyro_offset[1] = gyro_offset[2] = 0.0f;
    gyro_cal_ok = 0;
}

/* ===== 运行时陀螺零偏校正(等价移植自 x-io Fusion 的 FusionOffset) =====
 * 检测到“静止”(三轴角速度均 < 阈值)持续超过 TIMEOUT 秒后，以极低截止频率
 * 把残余角速度慢慢并入零偏，从而持续跟踪温度引起的零偏漂移——尤其是 EKF
 * 观测不到的 z/yaw 零偏。开机静态标定只给初值，这里负责长期“保鲜”。 */
#define OFFSET_RATE_HZ    500.0f        /* IMU_Update 调用频率 = TIM6 500Hz(每2ms一次) */
#define OFFSET_CUTOFF_HZ  0.1f          /* 零偏跟踪截止频率(时间常数~1.6s)。离线扫参:0.1比0.02把yaw稳态漂移从~0.07降到~0.02°/min、峰峰减半 */
#define OFFSET_TIMEOUT_S  3.0f          /* 静止多久后才开始更新零偏(停下后更快恢复校正) */
#define OFFSET_THRESH     0.05236f      /* 静止判定阈值 = 3°/s。注:gx陀螺噪声std~0.9°/s,阈值不能再低否则被噪声峰值误判成"运动" */

float           runtime_gyro_offset[3] = {0, 0, 0};  /* 运行时跟踪的零偏 rad/s(对外可观察) */
static float    offset_filt_coef = 0.0f;             /* = 2π·fc/fs */
static uint32_t offset_timeout   = 0;                /* = TIMEOUT·fs (样本数) */
static uint32_t offset_timer     = 0;

/**
 * @brief 初始化运行时零偏校正，用开机静态标定结果做种子(立刻就有好初值)
 */
static void GyroOffset_Init(const float seed[3])
{
    offset_filt_coef = 2.0f * 3.14159265f * OFFSET_CUTOFF_HZ / OFFSET_RATE_HZ;
    offset_timeout   = (uint32_t)(OFFSET_TIMEOUT_S * OFFSET_RATE_HZ);
    offset_timer     = 0;
    runtime_gyro_offset[0] = seed[0];
    runtime_gyro_offset[1] = seed[1];
    runtime_gyro_offset[2] = seed[2];
}

/**
 * @brief 对一帧角速度做零偏校正并在静止时缓慢更新零偏(原地修改 g[3])
 */
static void GyroOffset_Apply(float g[3])
{
    g[0] -= runtime_gyro_offset[0];
    g[1] -= runtime_gyro_offset[1];
    g[2] -= runtime_gyro_offset[2];

    /* 任一轴超阈值 -> 运动中, 复位计时, 不更新零偏 */
    if (fabsf(g[0]) > OFFSET_THRESH || fabsf(g[1]) > OFFSET_THRESH || fabsf(g[2]) > OFFSET_THRESH)
    {
        offset_timer = 0;
        return;
    }
    /* 静止但还没满 TIMEOUT -> 仅累加计时 */
    if (offset_timer < offset_timeout)
    {
        offset_timer++;
        return;
    }
    /* 静止够久 -> 把残余角速度以极慢速度并入零偏(含 z 轴) */
    runtime_gyro_offset[0] += g[0] * offset_filt_coef;
    runtime_gyro_offset[1] += g[1] * offset_filt_coef;
    runtime_gyro_offset[2] += g[2] * offset_filt_coef;
}

/**
 * @brief IMU 初始化：上电、启动加热 PWM、初始化 BMI088、零偏标定、EKF 初始化
 * @note  需在 MX_GPIO_Init / MX_SPI2_Init / MX_TIM3_Init 之后调用
 * @return 0=成功，非0=最后一次 BMI088_init 的错误码
 */
uint8_t IMU_Init(void)
{
    /* 给 IMU/外设 5V 供电使能 */
    HAL_GPIO_WritePin(POWER_5V_GPIO_Port, POWER_5V_Pin, GPIO_PIN_SET);
    HAL_Delay(100);

    /* 启动恒温加热 PWM（TIM3 CH4 -> PB1） */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    /* ★先初始化 DWT 周期计数器：BMI088_delay_us 要用它(且在调度器启动前调用,
       不能用 SysTick——FreeRTOS 下 SysTick 此时还没配置) */
    DWT_Init(240);

    /* 初始化 BMI088：有限次重试后返回，不再死等 */
    for (int i = 0; i < 10; i++)
    {
        imu_init_status = BMI088_init();
        if (imu_init_status == 0)
        {
            break;          /* 成功 */
        }
        HAL_Delay(10);
    }

    /* 开机静止三轴零偏标定(请保持板子静止约1秒) */
    IMU_GyroCalibrate();

    /* 用标定结果做种子，启动运行时零偏校正(长期跟踪温漂，含 z/yaw) */
    GyroOffset_Init(gyro_offset);

    /* 初始化四元数 EKF 姿态解算(DWT 已在上面 BMI088 之前初始化好)
     * 参数同 Wang Hongxi 原版: Q1=10, Q2=0.001, R=1e7, lambda=1(关渐消), lpf=0(直通) */
    QEKF_Init(10.0f, 0.001f, 10000000.0f, 1.0f, 0.0f);
    DWT_GetDeltaT(&ekf_dwt_cnt);   /* 复位 dt 基准 */

    /* 启动定时器中断驱动节拍：TIM6 2ms(500Hz)->IMU更新, TIM7 1ms计数(每5ms->200Hz发送)。
     * 中断里只置标志，IMU_Update / vofa_send_imu 在主循环消费标志执行。
     * TIM6 的 CubeMX 配置(PSC=240-1,ARR=1000-1 @120MHz定时器钟)已是 2ms，无需改；
     * TIM7 CubeMX 默认也是 2ms，这里运行时把 ARR 覆盖成 1ms(500-1)，regen 安全，
     * 无需改生成代码(若想让 CubeMX 也一致：TIM7 Counter Period 1000-1 -> 500-1)。 */
    __HAL_TIM_SET_AUTORELOAD(&htim7, 500 - 1);   /* TIM7 -> 1ms */
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);

    return imu_init_status;
}

/* Step① 调试:IMU_Update 三段耗时(µs)。定位完瓶颈可删。 */
volatile uint32_t dbg_us_spi = 0, dbg_us_ekf = 0, dbg_us_pid = 0;

/**
 * @brief IMU 一次完整更新：读数据 + EKF 姿态解算 + 恒温 PID
 * @note  定时器(TIM6 1kHz)直接调用，不依赖数据就绪标志，每次都读最新值。
 */
void IMU_Update(void)
{
    uint32_t _t0 = DWT->CYCCNT;
    /* 读取角速度/加速度/温度 */
    BMI088_read(imu_gyro, imu_accel, &imu_temp);
    uint32_t _t1 = DWT->CYCCNT;

    /* ---- 运行时零偏校正(静止时持续跟踪温漂，含 z/yaw) ---- */
    float g[3] = {imu_gyro[0], imu_gyro[1], imu_gyro[2]};
    GyroOffset_Apply(g);

    /* ---- 四元数 EKF 姿态解算 ---- */
    float dt = DWT_GetDeltaT(&ekf_dwt_cnt);
    imu_dt = dt;
    QEKF_Update(g[0], g[1], g[2],
                imu_accel[0], imu_accel[1], imu_accel[2], dt);

    /* 取出 EKF 输出(四元数 / 欧拉角 / 连续航向，EKF 内部已算好) */
    imu_q[0] = QEKF.q[0]; imu_q[1] = QEKF.q[1]; imu_q[2] = QEKF.q[2]; imu_q[3] = QEKF.q[3];
    imu_roll      = QEKF.Roll;
    imu_pitch     = QEKF.Pitch;
    imu_yaw       = QEKF.Yaw;
    imu_yaw_total = QEKF.YawTotalAngle;
    uint32_t _t2 = DWT->CYCCNT;

    /* ---- 恒温 PID（官方公式，位置式 KP + 积分累加 + 微分） ---- */
    err_ll = err_l;
    err_l  = err;
    err    = DES_TEMP - imu_temp;
    out    = KP * err + KI * (err + err_l + err_ll) + KD * (err - err_l);

    if (out > MAX_OUT) out = MAX_OUT;
    if (out < 0)       out = 0.0f;

    if (forceStop)     out = 0.0f;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint16_t)out);

    /* 温度进入目标 ±1°C 视为恒温就绪 */
    if (!imu_temp_ready && err < 1.0f && err > -1.0f)
    {
        imu_temp_ready = 1;
    }

    uint32_t _t3 = DWT->CYCCNT;
    dbg_us_spi = (_t1 - _t0) / 240u;   /* 240 = 240MHz/1MHz,得µs */
    dbg_us_ekf = (_t2 - _t1) / 240u;   /* 含零偏校正+dt+EKF+取值 */
    dbg_us_pid = (_t3 - _t2) / 240u;
    /* 注:IMU_Update 现由 TIM6 中断回调直接调用(500Hz)，不在这里碰 USB；
     *    遥测发送(含 IMU_RAW_LOG 原始流)统一由主循环在 tud_task 上下文执行。 */
}

/**
 * @brief EXTI(数据就绪)驱动版本：保留兼容。定时器架构下不再使用。
 */
void IMU_Task(void)
{
    if (!imu_data_ready)
    {
        return;
    }
    imu_data_ready = 0;
    IMU_Update();
}

/**
 * @brief GPIO 外部中断回调：BMI088 加速度数据就绪
 * @note  HAL 弱定义的重写；若工程别处已实现该回调，请把这里的逻辑合并过去
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ACC_INT_Pin)
    {
        imu_data_ready = 1;
    }
    else if (GPIO_Pin == GYRO_INT_Pin)
    {
        /* 陀螺数据就绪（本例不单独使用） */
    }
}
