#ifndef DEBUG_STREAM_H
#define DEBUG_STREAM_H

#include <stdint.h>

#include "foc_shared_types.h"
#include "foc_config.h"

void DebugStream_Init(void);
void DebugStream_Process(const sensor_data_t *sensor, const foc_motor_t *motor);

#endif /* DEBUG_STREAM_H */
