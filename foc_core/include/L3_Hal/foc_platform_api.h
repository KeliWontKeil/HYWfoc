#ifndef FOC_PLATFORM_API_H
#define FOC_PLATFORM_API_H

#include <stdint.h>

/*
 * ============================================================================
 * 平台 API 契约（Platform API Contract）
 * ============================================================================
 *
 * 本头文件是 L1/L2/L3 访问硬件的唯一平台抽象面（foc_pal）。
 *
 * 契约模型：
 *   1) 接口面稳定：全部接口无条件声明，不随配置宏裁剪。宏组合只改变
 *      实现行为，不改变接口面。
 *   2) 行为自适应：条件功能由实现内部按配置宏 #if 适配；条件关闭时
 *      安全退化为 no-op / 返回 0。
 *   3) 参数分类：编译期固定配置（PWM 频率、采样频率、调度节拍等）由
 *      实现内部从 LS 配置宏（foc_cfg_*.h）读取，不进入接口签名；
 *      仅运行时可变参数显式传参。
 *
 * 契约档位：
 *   【必须】恒定实现，所有平台必须提供完整功能。
 *   【按需】依赖配置宏组合；条件成立时须实现，否则可安全退化为
 *           no-op / 返回 0。
 *   【可选】允许空实现，不提供该功能时恒返回 0 / 不动作。
 *
 * 宏组合行为矩阵：
 *   FOC_CURRENT_LOOP_ISR_MODE == 2ISR  → AuxTimer* 可为 no-op（L1 不调用）
 *   FOC_CURRENT_LOOP_ISR_MODE == 3ISR  → AuxTimer* 必须实现
 *   FOC_CURRENT_SENSE_PHASES == NONE   → SensorInputInit 仅初始化存活路径；
 *                                         ReadPhaseCurrent 恒返回 0；
 *                                         SetSensorSampleOffsetPercent no-op
 *   FOC_CURRENT_SENSE_PHASES == 2/3    → 电流采样相关必须实现（C 相指针
 *                                         按相数传 NULL / 有效指针）
 *   无角度反馈硬件                       → ReadMechanicalAngleRad 恒返回 0
 *   FOC_FEATURE_UNDERVOLTAGE_PROTECTION 关闭 → ReadVbusVoltage 可空实现返回 0
 *   未支持的通信源 ID                   → CommSource_ReadFrame 恒返回 0
 * ============================================================================
 */

/* 平台无参中断回调（PWM ISR / 控制节拍 / 辅助定时器共用） */
typedef void (*FOC_Platform_IsrCallback_t)(void);

/* 通信源 ID */
typedef enum {
    FOC_COMM_SOURCE_0 = 0,
    FOC_COMM_SOURCE_1,
    FOC_COMM_SOURCE_2,
    FOC_COMM_SOURCE_3,
    FOC_COMM_SOURCE_COUNT
} FOC_Platform_CommSourceId_t;

/* 辅助定时器用途 ID */
typedef enum {
    FOC_AUX_TIMER_CURRENT_LOOP = 0,  /* 三 ISR 模式电流环定时器 */
    FOC_AUX_TIMER_COUNT
} FOC_Platform_AuxTimerId_t;

/* ===== Runtime & Clock（运行时/时钟） ===== */

/** @brief 平台基础运行时初始化（系统时钟、中断分组等平台固定配置）。【必须】 */
void FOC_Platform_RuntimeInit(void);

/** @brief 绑定 PWM 更新中断回调（高速调制更新路径）。【必须】 */
void FOC_Platform_SetPwmUpdateCallback(FOC_Platform_IsrCallback_t callback);

/** @brief 初始化控制节拍源（频率取 FOC_SCHEDULER_TICK_HZ）。【必须】 */
void FOC_Platform_ControlTickSourceInit(void);

/** @brief 绑定控制节拍回调。【必须】 */
void FOC_Platform_SetControlTickCallback(FOC_Platform_IsrCallback_t callback);

/** @brief 启动控制节拍源。【必须】 */
void FOC_Platform_StartControlTickSource(void);

/** @brief 使能/禁用控制相关外设更新中断（控制节拍/PWM/辅助定时器）。
 *  @param enable 0=禁用, 1=使能 【必须】 */
void FOC_Platform_SetControlInterruptsEnabled(uint8_t enable);

/** @brief 初始化辅助自由运行定时器（按需，见宏组合矩阵）。
 *  @param id       辅助定时器用途 ID
 *  @param freq_hz  中断频率 [Hz]
 *  @param callback 更新中断回调 【按需】 */
void FOC_Platform_AuxTimerInit(FOC_Platform_AuxTimerId_t id,
                               uint32_t freq_hz,
                               FOC_Platform_IsrCallback_t callback);

/** @brief 启动辅助定时器中断。【按需】 */
void FOC_Platform_AuxTimerStart(FOC_Platform_AuxTimerId_t id);

/** @brief 停止辅助定时器中断。【按需】 */
void FOC_Platform_AuxTimerStop(FOC_Platform_AuxTimerId_t id);

/** @brief 设置辅助定时器回调。【按需】 */
void FOC_Platform_SetAuxTimerCallback(FOC_Platform_AuxTimerId_t id,
                                      FOC_Platform_IsrCallback_t callback);

/** @brief 阻塞等待指定毫秒数（仅初始化/标定使用，禁止在 ISR 中调用）。【必须】 */
void FOC_Platform_WaitMs(uint32_t ms);

/** @brief 数据内存屏障（多 ISR 共享数据写序保证，如 SVPWM pending commit）。【必须】 */
void FOC_Platform_MemoryBarrier(void);

/* ===== Indicator（指示灯） ===== */

/** @brief 初始化板载指示灯。【必须】 */
void FOC_Platform_IndicatorInit(void);

/** @brief 按逻辑索引设置指示灯状态（索引值取 LS 层 FOC_LED_*_INDEX）。【必须】
 *  @param led_index 指示灯逻辑索引
 *  @param on        0=灭, 1=亮 */
void FOC_Platform_SetIndicator(uint8_t led_index, uint8_t on);

/* ===== Communication（通信） ===== */

/** @brief 初始化通信外设。【必须】 */
void FOC_Platform_CommInit(void);

/** @brief 从指定通信源读取一帧完整数据。
 *  @param id      通信源 ID
 *  @param buffer  输出缓冲区（存放原始帧字节）
 *  @param max_len 缓冲区最大长度
 *  @return 实际读取字节数；0 表示无帧或该源未实现
 *  【必须：源 0/1；可选：源 2/3（未支持恒返回 0）】 */
uint16_t FOC_Platform_CommSource_ReadFrame(FOC_Platform_CommSourceId_t id,
                                           uint8_t *buffer,
                                           uint16_t max_len);

/** @brief 慢路径输出调试文本（主循环阻塞式 DMA，禁止 ISR 调用）。【必须】 */
void FOC_Platform_WriteDebugText(const char *str);

/** @brief 快路径输出调试文本（ISR-safe 非阻塞，环形缓冲+发送中断）。【必须】 */
void FOC_Platform_WriteDebugFast(const char *str);

/** @brief 输出单字节状态码（ISR-safe 非阻塞）。【必须】 */
void FOC_Platform_WriteStatusByte(uint8_t status_code);

/* ===== Sensor / Acquisition（传感器/采集） ===== */

/** @brief 初始化传感器输入（电流/角度采样；频率取 FOC_SENSOR_SAMPLE_FREQ_KHZ）。
 *  【按需：FOC_CURRENT_SENSE_PHASES == NONE 时仅初始化存活路径（编码器等）】 */
void FOC_Platform_SensorInputInit(void);

/** @brief 设置 ADC 采样触发偏移（相对 PWM 周期的百分比）。【按需】
 *  @param percent 偏移百分比 [0.0, 100.0] */
void FOC_Platform_SetSensorSampleOffsetPercent(float percent);

/** @brief 读取相电流（两相采样时 C 相指针传 NULL）。【按需】
 *  @return 1=成功, 0=失败（无电流采样恒返回 0） */
uint8_t FOC_Platform_ReadPhaseCurrent(float *phase_current_a,
                                      float *phase_current_b,
                                      float *phase_current_c);

/** @brief 读取机械角度 [rad]（范围 0~2PI）。【按需】
 *  @return 1=成功, 0=失败（无角度反馈恒返回 0） */
uint8_t FOC_Platform_ReadMechanicalAngleRad(float *angle_rad);

/** @brief 读取母线电压 [V]（用于欠压保护与电压前馈）。【按需】
 *  @return 1=成功, 0=失败/不支持 */
uint8_t FOC_Platform_ReadVbusVoltage(float *vbus_v);

/* ===== PWM / Actuation（PWM/驱动输出） ===== */

/** @brief 初始化 PWM 输出（频率/死区取 FOC_PWM_FREQ_KHZ 与
 *  FOC_SVPWM_DEADTIME_PERCENT_DEFAULT）。【必须】 */
void FOC_Platform_PWMInit(void);

/** @brief 启动 PWM 输出。【必须】 */
void FOC_Platform_PWMStart(void);

/** @brief 写入三相占空比（ISR 高频调用，须极低延迟）。【必须】
 *  @param duty_a A 相占空比 [0.0~1.0]（0.5 为中点）
 *  @param duty_b B 相占空比 [0.0~1.0]
 *  @param duty_c C 相占空比 [0.0~1.0] */
void FOC_Platform_PWMSetDutyCycleTripleFloat(float duty_a, float duty_b, float duty_c);

/* ===== Diagnostics / Profiler（诊断/性能分析） ===== */

/** @brief 使能执行时间性能计数器（如 DWT_CYCCNT）。【可选】 */
void FOC_Platform_EnableCycleCounter(void);

/** @brief 读取性能计数器值（不支持时恒返回 0）。【可选】 */
uint32_t FOC_Platform_ReadCycleCounter(void);

#endif /* FOC_PLATFORM_API_H */
