#include "L2_Core/Control/foc_ctrl_source_openloop.h"

#include "LS_Config/foc_config.h"

#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)

/* OpenLoop Source 的 snapshot 由 LowSpeedPolicy 在 Control ISR 中写入，
 * 不再需要独立的 Step 函数。本文件保留以维持构建结构。 */

#endif