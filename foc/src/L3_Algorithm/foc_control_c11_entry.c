#include "L3_Algorithm/foc_control_c11_entry.h"

#include "LS_Config/foc_config.h"
#include "L3_Algorithm/foc_control_c21_outer_loop.h"
#include "L3_Algorithm/foc_control_c22_current_loop.h"
#include "L3_Algorithm/foc_control_c24_compensation.h"
#include "L41_Math/math_transforms.h"

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

void FOC_ControlApplyElectricalAngleInitBridge(foc_motor_t *motor, float electrical_angle)
{
    FOC_CurrentControlApplyElectricalAngleDirect(motor, electrical_angle);
}

uint8_t FOC_ControlCurrentLoopRequiresSample(void)
{
    return FOC_ControlRequiresCurrentSample();
}

void FOC_ControlCurrentLoopStep(foc_motor_t *motor,
                                foc_pid_t *current_pid,
                                const sensor_data_t *sensor,
                                float electrical_angle,
                                float dt_sec)
{
    FOC_CurrentControlStep(motor,
                           current_pid,
                           sensor,
                           electrical_angle,
                           dt_sec);
}

void FOC_ControlCompensationStep(foc_motor_t *motor, const sensor_data_t *sensor)
{
    if ((motor == 0) || (sensor == 0))
    {
        return;
    }

#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    FOC_ControlApplyCoggingCompensation(motor,
                                        sensor->mech_angle_rad.output_value,
                                        motor->cogging_speed_ref_rad_s);
#endif
}

void FOC_ControlOpenLoopStep(foc_motor_t *motor, float voltage, float turn_speed)
{
    FOC_CurrentControlOpenLoopStep(motor, voltage, turn_speed, FOC_CONTROL_DT_DEFAULT_SEC);
}

void FOC_OpenLoopStep(foc_motor_t *motor, float voltage, float turn_speed)
{
    FOC_ControlOpenLoopStep(motor, voltage, turn_speed);
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

#if (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_SPEED_ONLY)
		
    FOC_SpeedOuterLoopStep(motor,
                           speed_pid,
                           speed_only_rad_s,
                           sensor,
                           dt_sec);
    return 1U;
#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_SPEED_ANGLE_ONLY)

    FOC_SpeedAngleOuterLoopStep(motor,
                                speed_pid,
                                angle_hold_pid,
                                target_angle_rad,
                                angle_position_speed_rad_s,
                                sensor,
                                dt_sec);
    return 1U;
#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_FULL)

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
