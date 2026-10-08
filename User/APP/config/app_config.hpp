#pragma once

#include <cstdint>
#include <limits>
#include <numbers>

// =============================================================================
// 编译期开关：运行模式
// =============================================================================
// 默认模式只决定上电后的初始状态；运行时仍可用 gimbal_mode 0/1 切换。
#define MODE_NORMAL 0  // 普通定点模式编号
#define MODE_CRUISE 1  // 正弦巡航模式编号
#ifndef GIMBAL_MODE
#define GIMBAL_MODE MODE_NORMAL  // 上电默认运行模式
#endif
static_assert(GIMBAL_MODE == MODE_NORMAL || GIMBAL_MODE == MODE_CRUISE);

// =============================================================================
// 编译期开关：反馈控制器类型
// =============================================================================
#define CTRL_PID 0         // 串级 PID：角度环 PID + 速度环 PID
#define CTRL_PID_LADRC1 1  // 角度环 PID + 一阶速度对象 LADRC
#define CTRL_LADRC2 2      // 二阶位置对象 LADRC

#ifndef GIMBAL_CTRL_MODE
#define GIMBAL_CTRL_MODE CTRL_PID  // 未单独指定轴时采用的默认控制器
#endif
#ifndef GIMBAL_YAW_CTRL_MODE
#define GIMBAL_YAW_CTRL_MODE GIMBAL_CTRL_MODE  // Yaw 轴控制器类型
#endif
#ifndef GIMBAL_PITCH_CTRL_MODE
#define GIMBAL_PITCH_CTRL_MODE CTRL_PID  // Pitch 轴控制器类型
#endif

static_assert(GIMBAL_YAW_CTRL_MODE == CTRL_PID ||
              GIMBAL_YAW_CTRL_MODE == CTRL_PID_LADRC1 ||
              GIMBAL_YAW_CTRL_MODE == CTRL_LADRC2);
static_assert(GIMBAL_PITCH_CTRL_MODE == CTRL_PID ||
              GIMBAL_PITCH_CTRL_MODE == CTRL_PID_LADRC1 ||
              GIMBAL_PITCH_CTRL_MODE == CTRL_LADRC2);

// =============================================================================
// 编译期开关：系统辨识
// =============================================================================
// 启用后，所选轴的最终电机命令会被一次性开环 Chirp 扫频信号替代。
#define SYS_ID_AXIS_YAW 0    // 系统辨识轴编号：Yaw
#define SYS_ID_AXIS_PITCH 1  // 系统辨识轴编号：Pitch
#ifndef ENABLE_GIMBAL_SYS_ID
#define ENABLE_GIMBAL_SYS_ID 0  // 是否启用开环扫频辨识：0 关闭，1 开启
#endif
#ifndef GIMBAL_SYS_ID_AXIS
#define GIMBAL_SYS_ID_AXIS SYS_ID_AXIS_YAW  // 执行系统辨识的轴
#endif
#ifndef ENABLE_SYS_ID_RLS
#define ENABLE_SYS_ID_RLS 0  // 在线 RLS 开关；当前只允许为 0，以保留原始数据
#endif

static_assert(ENABLE_GIMBAL_SYS_ID == 0 || ENABLE_GIMBAL_SYS_ID == 1);
static_assert(GIMBAL_SYS_ID_AXIS == SYS_ID_AXIS_YAW ||
              GIMBAL_SYS_ID_AXIS == SYS_ID_AXIS_PITCH);
static_assert(ENABLE_SYS_ID_RLS == 0,
              "Online RLS is intentionally disabled for raw data capture");

namespace AppConfig {

namespace Gimbal {

// =============================================================================
// 参数结构体
// =============================================================================

/// 单轴开环 Chirp 系统辨识参数。
struct SysIdParam {
  float start_freq_hz;    ///< 扫频起始频率，单位 Hz
  float end_freq_hz;      ///< 扫频终止频率，单位 Hz
  float duration_s;       ///< 单次扫频持续时间，单位 s
  float amplitude_nm;     ///< 扫频信号幅值，单位 N·m
  float output_limit_nm;  ///< 扫频输出绝对值上限，单位 N·m
  float max_angle_rad;    ///< 相对实验起点的最大安全偏角，单位 rad
};

/// LADRC 参数；观测器和控制器带宽由 ts 在算法内部计算。
struct LadrcParam {
  float b0;            ///< 被控对象的名义输入增益
  float ts;            ///< 期望调节时间，单位 s
  float output_limit;  ///< 输出绝对值上限，本工程中单位为 N·m
  float r;             ///< 跟踪微分器速度因子；0 表示不启用
};

// =============================================================================
// 控制任务与通信超时
// =============================================================================
inline constexpr bool kControllerEnable = true;  // 是否创建并启动云台控制器
inline constexpr bool kControlEnabledAtStartup = true;  // 上电后是否请求启用电机控制
inline constexpr uint32_t kControlPeriodMs = 1;  // 控制循环周期，单位 ms
inline constexpr uint32_t kFeedbackTimeoutMs = 20;  // 电机反馈有效期，单位 ms
inline constexpr uint32_t kAttitudeTimeoutMs = 20;  // IMU/VQF 姿态数据有效期，单位 ms

// =============================================================================
// 正弦巡航轨迹
// =============================================================================
// 轨迹：中心角 + 幅值*sin(2*pi*频率*时间)，中心角相对回零后的机械中心。
inline constexpr bool kCruiseEnabledAtStartup =
    GIMBAL_MODE == MODE_CRUISE;  // 上电是否直接进入巡航模式
inline constexpr float kCruiseYawCenterRad = 0.0F;  // Yaw 巡航中心角，单位 rad
inline constexpr float kCruisePitchCenterRad = 0.0F;  // Pitch 巡航中心角，单位 rad
inline constexpr float kCruiseYawAmplitudeRad = 2.0F;  // Yaw 正弦幅值，单位 rad
inline constexpr float kCruiseYawFrequencyHz = 0.15F;  // Yaw 巡航频率，单位 Hz
inline constexpr float kCruisePitchAmplitudeRad = 0.8F;  // Pitch 正弦幅值，单位 rad
inline constexpr float kCruisePitchFrequencyHz = 0.15F;  // Pitch 巡航频率，单位 Hz
inline constexpr float kCruiseMaximumFrequencyHz = 1.00F;  // 命令允许的最大巡航频率，单位 Hz
inline constexpr float kYawPidCruiseVelocityFeedforwardGain =
    0.5F;  // Yaw 串级 PID 模式的巡航目标速度前馈系数
inline constexpr float kYawPidLadrc1CruiseVelocityFeedforwardGain =
    0.5F;  // Yaw PID+LADRC1 模式的巡航目标速度前馈系数
static_assert(kYawPidCruiseVelocityFeedforwardGain >= 0.0F);
static_assert(kYawPidLadrc1CruiseVelocityFeedforwardGain >= 0.0F);

// =============================================================================
// IMU 坐标方向
// =============================================================================
// 当前安装旋转为单位四元数：Z 轴角速度对应 Yaw，Y 轴对应 Pitch。
inline constexpr float kYawGyroSign = 1.0F;  // Yaw 陀螺仪角速度方向系数
inline constexpr float kPitchGyroSign = 1.0F;  // Pitch 陀螺仪角速度方向系数

// =============================================================================
// Yaw：串级 PID 参数
// =============================================================================
inline constexpr float kYawAngleKp = 2.80F;  // Yaw 角度环比例增益
inline constexpr float kYawAngleKi = 0.025F;  // Yaw 角度环积分增益
inline constexpr float kYawAngleKd = 0.001F;  // Yaw 角度环微分增益
inline constexpr float kYawAngleDeadbandRad =
    0.5F * std::numbers::pi_v<float> / 180.0F;  // Yaw 角度环死区，当前为 0.5°
inline constexpr float kYawAngleIntegralLimit = 1000.0F;  // Yaw 角度环积分限幅；0 表示禁用积分累积
inline constexpr float kYawSpeedRefLimitRadPerSec =
    std::numeric_limits<float>::infinity();  // Yaw 目标角速度不限幅
inline constexpr float kYawSpeedKp = 0.20F;  // Yaw 速度环比例增益
inline constexpr float kYawSpeedKi = 0.0F;  // Yaw 速度环积分增益
inline constexpr float kYawSpeedKd = 0.0F;  // Yaw 速度环微分增益
inline constexpr float kYawSpeedDeadbandRadPerSec = 0.008F;  // Yaw 速度环死区，单位 rad/s
inline constexpr float kYawSpeedIntegralLimit = 1000.0F;  // Yaw 速度环积分限幅；0 表示禁用积分累积
inline constexpr float kYawVelocityPidOutputLimitNm =
    std::numeric_limits<float>::infinity();  // Yaw 速度环 PID 输出不限幅

// =============================================================================
// Yaw：电机输出、静摩擦与 LADRC2 脱困
// =============================================================================
inline constexpr float kYawMotorOutputLimitNm = 1.0F;  // Yaw 最终电机扭矩绝对值上限，单位 N·m
inline constexpr float kYawMotorCommandSign = 1.0F;  // Yaw 控制正方向到电机命令正方向的映射系数
inline constexpr float kYawStaticFrictionCompensationNm = 0.0F;  // 关闭非 LADRC2 模式静摩擦补偿
inline constexpr float kYawLadrc2BreakawayTorqueNm = 0.08F;  // LADRC2 初始脱困扭矩，单位 N·m
inline constexpr float kYawLadrc2BreakawayMaximumTorqueNm = 0.10F;  // LADRC2 脱困扭矩上限，单位 N·m
inline constexpr float kYawLadrc2BreakawayRampRateNmPerSec = 0.02F;  // 脱困扭矩爬升率，单位 N·m/s
inline constexpr float kYawLadrc2BreakawaySpeedThresholdRadPerSec = 0.05F;  // 触发脱困的速度阈值，单位 rad/s
inline constexpr float kYawLadrc2DisturbanceCompensationLimitNm = 0.07F;  // ESO 扰动补偿限幅，单位 N·m
inline constexpr float kYawLadrc2BreakawayReleaseGyroRadPerSec = 0.08F;  // 释放脱困的 IMU 速度阈值，单位 rad/s
inline constexpr float kYawLadrc2BreakawayReleaseMotorRadPerSec = 0.08F;  // 释放脱困的电机速度阈值，单位 rad/s
inline constexpr uint32_t kYawLadrc2BreakawayMotionConfirmMs = 40;  // 确认持续运动的时间，单位 ms

// =============================================================================
// Yaw：堵转保护与诊断
// =============================================================================
inline constexpr uint32_t kYawStallTimeMs = 1200;  // 堵转条件持续多久后触发保护，单位 ms
inline constexpr float kYawStallCommandThresholdNm = 0.10F;  // 进入堵转判断的最小扭矩，单位 N·m
inline constexpr float kYawStallGyroThresholdRadPerSec = 0.02F;  // 堵转判断最大 IMU 角速度，单位 rad/s
inline constexpr float kYawStallMotorSpeedThresholdRadPerSec = 0.02F;  // 堵转判断最大电机速度，单位 rad/s
inline constexpr float kYawDiagnosticTorqueLimitNm = 0.10F;  // yaw_torque 诊断扭矩上限，单位 N·m

// =============================================================================
// Yaw：编码器标定、机械限位与中心
// =============================================================================
// 2026-09-25 固定底座标定：vqf_yaw ~= 1.00776*motor_encoder-2.97724 rad。
// 6D VQF 没有绝对航向观测，因此控制主要使用标定斜率和编码器机械边界。
inline constexpr float kYawEncoderToVqfScale = 1.00776F;  // 编码器角到 VQF Yaw 的标定比例
inline constexpr float kYawEncoderToVqfOffsetRad = -2.97724F;  // 标定截距，单位 rad
inline constexpr bool kEnableYawEncoderLimit = true;  // 是否启用编码器机械限位保护
inline constexpr float kYawEncoderNegativeStopPhaseRad = 3.0656F;  // 负向硬限位编码器相位，单位 rad
inline constexpr float kYawEncoderPositiveStopPhaseRad = 1.1223F;  // 正向硬限位编码器相位，单位 rad
inline constexpr float kYawEncoderLimitMarginRad = 0.08F;  // 与硬限位预留的安全角度，单位 rad
inline constexpr float kYawEncoderNegativeSafePhaseRad =
    kYawEncoderNegativeStopPhaseRad +
    kYawEncoderLimitMarginRad;  // 加入余量后的负向安全边界，单位 rad
inline constexpr float kYawEncoderPositiveSafePhaseRad =
    kYawEncoderPositiveStopPhaseRad -
    kYawEncoderLimitMarginRad;  // 加入余量后的正向安全边界，单位 rad
inline constexpr float kYawEncoderForbiddenSectorSplitRad =
    (kYawEncoderPositiveStopPhaseRad +
     kYawEncoderNegativeStopPhaseRad) /
    2.0F;  // 不可达扇区中点，用于回绕后的限位方向判定，单位 rad
inline constexpr float kYawEncoderCenterPhaseRad =
    (kYawEncoderNegativeStopPhaseRad +
     (kYawEncoderPositiveStopPhaseRad +
      2.0F * std::numbers::pi_v<float>)) /
    2.0F;  // Yaw 有效机械行程的编码器中心相位，单位 rad

// =============================================================================
// 自动回零：使用与正常控制相同的串级 PID，仅保留完成判据
// =============================================================================
// 上电后两轴移动到绝对编码器机械中心，稳定后将当前位置设为控制零点。
inline constexpr float kHomingEncoderToleranceRad =
    5.0F * std::numbers::pi_v<float> / 180.0F;  // 回零中心允许误差，当前为 5°
inline constexpr float kHomingMotorSpeedThresholdRadPerSec = 0.20F;  // 判定回零稳定的最大电机速度，单位 rad/s
inline constexpr uint32_t kHomingSettleTimeMs = 200;  // 两轴在中心稳定后需保持的时间，单位 ms

// =============================================================================
// Pitch：串级 PID 参数
// =============================================================================
inline constexpr float kPitchSpeedKp = 1.0F;  // Pitch 速度环比例增益
inline constexpr float kPitchSpeedKi = 0.0F;  // Pitch 速度环积分增益
inline constexpr float kPitchSpeedKd = 0.0001F;  // Pitch 速度环微分增益
inline constexpr float kPitchSpeedDeadbandRadPerSec = 0.005F;  // Pitch 速度环死区，单位 rad/s
inline constexpr float kPitchSpeedIntegralLimit = 0.0F;  // Pitch 速度环积分限幅；0 表示禁用积分累积
inline constexpr float kPitchAngleKp = 3.0F;  // Pitch 角度环比例增益
inline constexpr float kPitchAngleKi = 0.01F;  // Pitch 角度环积分增益
inline constexpr float kPitchAngleKd = 0.0F;  // Pitch 角度环微分增益
inline constexpr float kPitchAngleDeadbandRad =
    0.5F * std::numbers::pi_v<float> / 180.0F;  // Pitch 角度环死区，当前为 0.5°
inline constexpr float kPitchAngleIntegralLimit = 1000.0F;  // Pitch 角度环积分限幅；0 表示禁用积分累积
inline constexpr float kPitchSpeedRefLimitRadPerSec =
    std::numeric_limits<float>::infinity();  // Pitch 目标角速度不限幅
inline constexpr float kPitchVelocityPidOutputLimitNm =
    std::numeric_limits<float>::infinity();  // Pitch 速度环 PID 输出不限幅
inline constexpr float kPitchMotorPositiveOutputLimitNm = 0.18F;  // Pitch 正方向最终扭矩上限，单位 N·m
inline constexpr float kPitchMotorNegativeOutputLimitNm = 0.30F;  // Pitch 负方向最终扭矩绝对值上限，单位 N·m

// =============================================================================
// 系统辨识扫频参数
// =============================================================================
// Yaw 低频扫频容易快速触及 15° 安全边界，当前改用 3~8 Hz 的保守激励。
inline constexpr SysIdParam kYawSysIdParam{
    .start_freq_hz = 3.0F,  // Yaw 扫频起始频率，单位 Hz
    .end_freq_hz = 8.0F,  // Yaw 扫频终止频率，单位 Hz
    .duration_s = 15.0F,  // Yaw 扫频持续时间，单位 s
    .amplitude_nm = 0.05F,  // Yaw Chirp 扭矩幅值，单位 N·m
    .output_limit_nm = 0.06F,  // Yaw Chirp 输出绝对值上限，单位 N·m
    .max_angle_rad = 15.0F * std::numbers::pi_v<float> /
                     180.0F,  // Yaw 相对起点最大安全偏角，当前为 15°
};
inline constexpr SysIdParam kPitchSysIdParam{
    .start_freq_hz = 0.2F,  // Pitch 扫频起始频率，单位 Hz
    .end_freq_hz = 5.0F,  // Pitch 扫频终止频率，单位 Hz
    .duration_s = 20.0F,  // Pitch 扫频持续时间，单位 s
    .amplitude_nm = 0.02F,  // Pitch Chirp 扭矩幅值，单位 N·m
    .output_limit_nm = 0.03F,  // Pitch Chirp 输出绝对值上限，单位 N·m
    .max_angle_rad = 5.0F * std::numbers::pi_v<float> /
                     180.0F,  // Pitch 相对起点最大安全偏角，当前为 5°
};

static_assert(kYawSysIdParam.output_limit_nm <= kYawMotorOutputLimitNm);
static_assert(kYawSysIdParam.start_freq_hz > 0.0F &&
              kYawSysIdParam.end_freq_hz >=
                  kYawSysIdParam.start_freq_hz &&
              kYawSysIdParam.duration_s > 0.0F &&
              kYawSysIdParam.max_angle_rad > 0.0F);
static_assert(kYawSysIdParam.amplitude_nm <=
              kYawSysIdParam.output_limit_nm);
static_assert(kPitchSysIdParam.output_limit_nm <=
              kPitchMotorPositiveOutputLimitNm);
static_assert(kPitchSysIdParam.output_limit_nm <=
              kPitchMotorNegativeOutputLimitNm);
static_assert(kPitchSysIdParam.amplitude_nm <=
              kPitchSysIdParam.output_limit_nm);
static_assert(kPitchSysIdParam.start_freq_hz > 0.0F &&
              kPitchSysIdParam.end_freq_hz >=
                  kPitchSysIdParam.start_freq_hz &&
              kPitchSysIdParam.duration_s > 0.0F &&
              kPitchSysIdParam.max_angle_rad > 0.0F);
// =============================================================================
// Pitch：电机方向、静摩擦与命令整形
// =============================================================================
inline constexpr float kPitchMotorCommandSign = -1.0F;  // 姿态正方向到电机命令正方向的映射系数
inline constexpr float kPitchEncoderCommandSign = 1.0F;  // 编码器误差到电机命令方向的映射系数
inline constexpr float kPitchStaticFrictionCompensationNm = 0.0F;  // 关闭 Pitch 运动静摩擦补偿















inline constexpr float kYawPositionFeedforwardGain = 0.0F;  // 关闭 Yaw 目标位置微分前馈
inline constexpr float kPitchPositionFeedforwardGain = 0.0F;  // 关闭 Pitch 目标位置微分前馈


















// =============================================================================
// Pitch：堵转保护
// =============================================================================
inline constexpr uint32_t kPitchStallTimeMs = 300;  // 堵转条件持续多久后触发保护，单位 ms
inline constexpr float kPitchStallCommandThresholdNm = 0.16F;  // 进入堵转判断的最小扭矩，单位 N·m
inline constexpr float kPitchStallGyroThresholdRadPerSec = 0.02F;  // 堵转判断最大 IMU 角速度，单位 rad/s
inline constexpr float kPitchStallMotorSpeedThresholdRadPerSec = 0.11F;  // 堵转判断最大电机速度，单位 rad/s

// =============================================================================
// Pitch：编码器标定与机械限位
// =============================================================================
// 2026-10-05 固定底座标定：vqf_pitch ~= -0.99428*motor_encoder+2.05908 rad。
// 三次往返实测编码器硬限位约为 [1.58, 3.43] rad，安全边界各内缩 0.08 rad。
inline constexpr float kPitchEncoderToVqfScale = -0.99428F;  // 编码器角到 VQF Pitch 的标定比例
inline constexpr float kPitchEncoderToVqfOffsetRad = 2.05908F;  // 标定截距，单位 rad
inline constexpr bool kEnablePitchEncoderLimit = true;  // 是否启用编码器机械限位保护
inline constexpr float kPitchEncoderHardMinRad = 1.58F;  // 编码器最小硬限位，单位 rad
inline constexpr float kPitchEncoderHardMaxRad = 3.43F;  // 编码器最大硬限位，单位 rad
inline constexpr float kPitchEncoderLimitMarginRad = 0.08F;  // 硬限位安全余量，单位 rad
inline constexpr float kPitchEncoderMinRad =
    kPitchEncoderHardMinRad + kPitchEncoderLimitMarginRad;
inline constexpr float kPitchEncoderMaxRad =
    kPitchEncoderHardMaxRad - kPitchEncoderLimitMarginRad;
inline constexpr float kPitchEncoderCenterRad =
    (kPitchEncoderMinRad + kPitchEncoderMaxRad) /
    2.0F;  // Pitch 编码器机械中心角，单位 rad

// =============================================================================
// 重力补偿、软件角度限制与 Pitch 诊断
// =============================================================================
// 重力前馈：W1*sin(pitch)+W2*cos(pitch)+W3+回零保持扭矩；当前 W1~W3 为 0。
inline constexpr float kPitchGravityW1 = 0.330F;  // sin(theta) 项系数，单位 N·m
inline constexpr float kPitchGravityW2 = 0.001F;  // cos(theta) 项系数，单位 N·m
inline constexpr float kPitchGravityW3 = -0.018F;  // 常值项，单位 N·m
inline constexpr bool kEnablePitchLimit = true;  // 是否启用 Pitch 目标角软件限位
inline constexpr float kPitchMinRad = -0.80F;  // 回零后 Pitch 最小目标角，单位 rad
inline constexpr float kPitchMaxRad = 0.80F;  // 回零后 Pitch 最大目标角，单位 rad
inline constexpr bool kEnableYawLimit = true;  // 是否启用 Yaw 目标角软件限位
inline constexpr float kYawMinRad = -2.08F;  // 回零后 Yaw 最小目标角，单位 rad
inline constexpr float kYawMaxRad = 2.08F;  // 回零后 Yaw 最大目标角，单位 rad
inline constexpr float kPitchDiagnosticTorqueLimitNm = 0.30F;  // pitch_torque 诊断扭矩上限，单位 N·m

// LADRC<1> (speed plant) and LADRC<2> (position plant) intentionally have
// independent settling-time configurations. The LADRC class derives wc and
// w0 from ts; application code must not duplicate that calculation.
inline constexpr LadrcParam kYawLadrc1Param{
    .b0 = 2350.0F,
    .ts = 0.10F,
    .output_limit = 0.10F,
    .r = 0.0F,
};
inline constexpr LadrcParam kYawLadrc2Param{
    .b0 = 2350.0F,
    .ts = 0.35F,
    .output_limit = 0.10F,
    .r = 0.0F,
};

// TODO: Pitch system identification required. A zero b0 is an explicit
// unusable placeholder and must never be replaced with Yaw's identified b0.
inline constexpr LadrcParam kPitchLadrc1Param{
    .b0 = 0.0F,
    .ts = 0.0F,
    .output_limit = 0.0F,
    .r = 0.0F,
};
inline constexpr LadrcParam kPitchLadrc2Param{
    .b0 = 0.0F,
    .ts = 0.0F,
    .output_limit = 0.0F,
    .r = 0.0F,
};

static_assert(kYawLadrc1Param.b0 > 0.0F && kYawLadrc1Param.ts > 0.0F &&
              kYawLadrc1Param.output_limit > 0.0F);
static_assert(kYawLadrc2Param.b0 > 0.0F && kYawLadrc2Param.ts > 0.0F &&
              kYawLadrc2Param.output_limit > 0.0F);
static_assert(kYawLadrc1Param.output_limit <= kYawMotorOutputLimitNm);
static_assert(kYawLadrc2Param.output_limit <= kYawMotorOutputLimitNm);
static_assert(kYawLadrc2BreakawayTorqueNm > 0.0F &&
              kYawLadrc2BreakawayTorqueNm <= kYawMotorOutputLimitNm);
static_assert(kYawLadrc2BreakawayMaximumTorqueNm >
                  kYawLadrc2BreakawayTorqueNm &&
              kYawLadrc2BreakawayMaximumTorqueNm <=
                  kYawMotorOutputLimitNm);
static_assert(kYawLadrc2BreakawayRampRateNmPerSec > 0.0F);
static_assert(kYawLadrc2DisturbanceCompensationLimitNm > 0.0F &&
              kYawLadrc2DisturbanceCompensationLimitNm <
                  kYawLadrc2BreakawayTorqueNm);
static_assert(kYawLadrc2BreakawaySpeedThresholdRadPerSec > 0.0F);
static_assert(kYawLadrc2BreakawayReleaseGyroRadPerSec >
              kYawLadrc2BreakawaySpeedThresholdRadPerSec);
static_assert(kYawLadrc2BreakawayReleaseMotorRadPerSec > 0.0F);
static_assert(kYawLadrc2BreakawayMotionConfirmMs > 0U);
#if GIMBAL_PITCH_CTRL_MODE == CTRL_PID_LADRC1
static_assert(kPitchLadrc1Param.b0 > 0.0F &&
                  kPitchLadrc1Param.ts > 0.0F &&
                  kPitchLadrc1Param.output_limit > 0.0F,
              "Pitch LADRC<1> requires identified and valid parameters");
#elif GIMBAL_PITCH_CTRL_MODE == CTRL_LADRC2
static_assert(kPitchLadrc2Param.b0 > 0.0F &&
                  kPitchLadrc2Param.ts > 0.0F &&
                  kPitchLadrc2Param.output_limit > 0.0F,
              "Pitch LADRC<2> requires identified and valid parameters");
#endif

// After automatic homing, serial angle commands are center-relative.
inline constexpr float kPitchAngleCommandLimitRad = kPitchMaxRad;
inline constexpr float kYawAngleCommandLimitRad = kYawMaxRad;
}  // namespace Gimbal

namespace Telemetry {
enum class Mode : uint8_t {
  kImu,
  kAttitude,
  kMotor,
  kAll,
  kControl,
  kYawCalibration,
  kPitchCalibration
};

inline constexpr bool kEnable = true;
inline constexpr uint32_t kPeriodMs = 20;
inline constexpr uint32_t kSysIdPeriodMs = 5;
inline constexpr Mode kMode = Mode::kControl;
}  // namespace Telemetry

}  // namespace AppConfig
