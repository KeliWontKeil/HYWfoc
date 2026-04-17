#include "L3_Algorithm/foc_control_c01_entry.h"

#include "L3_Algorithm/foc_control_c03_outer_loop.h"
#include "L3_Algorithm/foc_control_c05_actuation.h"
#include "L41_Math/math_transforms.h"
#include "LS_Config/foc_config.h"

#define FOC_CONTROL_DT_DEFAULT_SEC FOC_CONTROL_DT_SEC

static void FOC_ResetPIDState(foc_pid_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

void FOC_OpenLoopStep(foc_motor_t *motor, float voltage, float turn_speed)
{
    if (motor == 0)
    {
        return;
    }

    motor->electrical_phase_angle = Math_WrapRad(
        motor->electrical_phase_angle +
        FOC_MATH_TWO_PI * turn_speed * motor->pole_pairs * FOC_CONTROL_DT_DEFAULT_SEC * motor->direction);

    motor->ud = 0.0f;
    motor->uq = Math_ClampFloat(voltage, 0.0f, motor->set_voltage);

    FOC_ControlApplyElectricalAngleRuntime(motor, motor->electrical_phase_angle);
}

uint8_t FOC_ControlOuterLoopStep(foc_motor_t *motor,
                                 foc_pid_t *current_pid,
                                 foc_pid_t *speed_pid,
                                 foc_pid_t *angle_hold_pid,
                                 const sensor_data_t *sensor,
                                 uint8_t control_mode,
                                 float speed_only_rad_s,
                                 float target_angle_rad,
                                 float angle_position_speed_rad_s,
                                 float dt_sec)
{
#if (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_FULL)
    static uint8_t g_prev_control_mode = COMMAND_MANAGER_DEFAULT_CONTROL_MODE;
    static uint8_t g_prev_control_mode_valid = 0U;
#endif

    if ((motor == 0) || (speed_pid == 0) || (sensor == 0))
    {
        return 0U;
    }

#if (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_SPEED_ONLY)
    (void)current_pid;
    (void)angle_hold_pid;
    (void)control_mode;
    (void)target_angle_rad;
    (void)angle_position_speed_rad_s;
    FOC_SpeedOuterLoopStep(motor,
                           speed_pid,
                           speed_only_rad_s,
                           sensor,
                           dt_sec);
    return 1U;
#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_SPEED_ANGLE_ONLY)
    (void)current_pid;
    (void)control_mode;
    (void)speed_only_rad_s;
    if (angle_hold_pid == 0)
    {
        return 0U;
    }
    FOC_SpeedAngleOuterLoopStep(motor,
                                speed_pid,
                                angle_hold_pid,
                                target_angle_rad,
                                angle_position_speed_rad_s,
                                sensor,
                                dt_sec);
    return 1U;
#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_FULL)
    if (angle_hold_pid == 0)
    {
        return 0U;
    }

    if ((control_mode != COMMAND_MANAGER_CONTROL_MODE_SPEED_ONLY) &&
        (control_mode != COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE))
    {
        return 0U;
    }

    if (g_prev_control_mode_valid == 0U)
    {
        g_prev_control_mode = control_mode;
        g_prev_control_mode_valid = 1U;
    }

    if (control_mode != g_prev_control_mode)
    {
        if (control_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE)
        {
            FOC_ControlRebaseMechanicalAngleAccum(motor, sensor->mech_angle_rad.output_value);
        }

        FOC_ResetPIDState(current_pid);
        FOC_ResetPIDState(speed_pid);
        FOC_ResetPIDState(angle_hold_pid);
        FOC_ControlResetSpeedLoopState();
        g_prev_control_mode = control_mode;
    }

    if (control_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ONLY)
    {
        FOC_SpeedOuterLoopStep(motor,
                               speed_pid,
                               speed_only_rad_s,
                               sensor,
                               dt_sec);
    }
    else
    {
        FOC_SpeedAngleOuterLoopStep(motor,
                                    speed_pid,
                                    angle_hold_pid,
                                    target_angle_rad,
                                    angle_position_speed_rad_s,
                                    sensor,
                                    dt_sec);
    }

    return 1U;
#else
#error "Unsupported FOC_BUILD_CONTROL_ALGO_SET"
#endif
}
