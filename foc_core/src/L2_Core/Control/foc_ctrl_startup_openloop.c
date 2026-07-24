#include "L2_Core/Control/foc_ctrl_startup.h"

#include <math.h>

#include "L2_Core/Control/foc_ctrl_estim.h"
#include "LS_Config/foc_config.h"

#if (FOC_STARTUP_OPENLOOP_ENABLE == FOC_CFG_ENABLE)

void FOC_StartupOpenLoop_RunStep(foc_motor_t *motor, float dt_sec)
{
    float vs;
    float virtual_angle_elec;
    float virtual_angle_mech;
    float est_speed;

    if (motor == 0) return;
    if (motor->startup_openloop_state.phase != FOC_STARTUP_PHASE_RUNNING) return;
    if (motor->pole_pairs == 0U) return;

    /* 速度 ramping：累加 ramp_rate * dt → 限幅到目标速度 */
    motor->startup_openloop_state.virtual_speed_rad_s +=
        motor->startup_openloop_state.ramp_rate_rad_s2 * dt_sec;
    vs = motor->startup_openloop_state.virtual_speed_rad_s;
    if (vs > motor->startup_openloop_state.target_speed_rad_s)
    {
        vs = motor->startup_openloop_state.target_speed_rad_s;
    }

    /* 角度从当前角度累积 */
    virtual_angle_elec = motor->startup_openloop_state.virtual_angle_rad + vs * dt_sec;
    while (virtual_angle_elec > FOC_MATH_TWO_PI) virtual_angle_elec -= FOC_MATH_TWO_PI;
    while (virtual_angle_elec < 0.0f)            virtual_angle_elec += FOC_MATH_TWO_PI;
    motor->startup_openloop_state.virtual_angle_rad = virtual_angle_elec;

    /* 同步电角度到电流环（I/F 开环强拖需要旋转 dq 系） */
    motor->electrical_phase_angle = virtual_angle_elec;

    /* 写入 ctrl_input */
    virtual_angle_mech = virtual_angle_elec / (float)motor->pole_pairs;
    motor->ctrl_input.mech_angle_rad = virtual_angle_mech;
    motor->ctrl_input.valid = 1U;
    motor->ctrl_input.source = FOC_ESTIMATOR_TYPE_NONE;

    /* 设置开环电流 */
    motor->iq_target = motor->startup_openloop_state.current_ref_a;

    /* 收敛检测：使用累加后的 vs（电角速度） */
    est_speed = motor->est_state.elec_speed_rad_s;
    if ((motor->est_state.state >= FOC_ESTIMATOR_STATE_CONVERGING) &&
        (fabsf(est_speed - vs) < FOC_STARTUP_OPENLOOP_SPEED_THRESHOLD_RAD_S))
    {
        motor->startup_openloop_state.phase = FOC_STARTUP_PHASE_DONE;
    }
}

#endif
