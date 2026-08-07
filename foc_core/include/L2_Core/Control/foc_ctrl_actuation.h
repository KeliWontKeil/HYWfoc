#ifndef FOC_CTRL_ACTUATION_H

#define FOC_CTRL_ACTUATION_H


#include "L2_Core/foc_ctrl_types.h"
#include "L2_Core/Control/foc_ctrl_outer_loop.h"
#include "L3_Hal/foc_svpwm.h"

float FOC_ControlMechanicalToElectricalAngle(const foc_motor_params_t *params,
                                             float fallback_elec_angle_rad,
                                             float mech_angle_rad);
void FOC_ControlRecordPhaseOutputDqAngle(foc_phase_output_state_t *phase_output,
                                         foc_control_runtime_t *ctrl,
                                         uint8_t phase,
                                         uint8_t state_id,
                                         float electrical_angle,
                                         float ud,
                                         float uq);
void FOC_ControlRecordPhaseOutputZero(foc_phase_output_state_t *phase_output,
                                      foc_control_runtime_t *ctrl,
                                      foc_outer_loop_private_t *outer_loop,
                                      uint8_t phase,
                                      uint8_t state_id);
void FOC_ControlApplyPhaseOutputRuntime(foc_control_runtime_t *ctrl,
                                        svpwm_interp_state_t *svpwm,
                                        foc_applied_output_state_t *applied,
                                        foc_alpha_beta_phase_t *alpha_beta,
                                        foc_phase_output_state_t *phase_output,
                                        const foc_motor_params_t *params);
void FOC_ControlApplyElectricalAngleRuntime(foc_control_runtime_t *ctrl,
                                            svpwm_interp_state_t *svpwm,
                                            foc_applied_output_state_t *applied,
                                            foc_alpha_beta_phase_t *alpha_beta,
                                            const foc_motor_params_t *params,
                                            float electrical_angle);
void FOC_ControlApplyElectricalAngleDirect(foc_control_runtime_t *ctrl,
                                           svpwm_interp_state_t *svpwm,
                                           foc_applied_output_state_t *applied,
                                           foc_alpha_beta_phase_t *alpha_beta,
                                           const foc_motor_params_t *params,
                                           float electrical_angle);
uint8_t FOC_SampleLockedMechanicalAngle(foc_control_runtime_t *ctrl,
                                        svpwm_interp_state_t *svpwm,
                                        foc_applied_output_state_t *applied,
                                        foc_alpha_beta_phase_t *alpha_beta,
                                        const foc_motor_params_t *params,
                                        float electrical_angle,
                                        uint16_t settle_ms,
                                        uint16_t sample_count,
                                        float *mech_angle_rad);

#endif /* FOC_CONTROL_C31_ACTUATION_H */
