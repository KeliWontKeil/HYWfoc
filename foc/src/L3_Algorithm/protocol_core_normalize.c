#include "L3_Algorithm/protocol_core_normalize.h"

#include "LS_Config/foc_config.h"

uint8_t ProtocolCore_ParseStateValue(float value, uint8_t *state_out)
{
    if (state_out == 0)
    {
        return 0U;
    }

    if (value == 0.0f)
    {
        *state_out = COMMAND_MANAGER_ENABLED_DISABLE;
        return 1U;
    }

    if (value == 1.0f)
    {
        *state_out = COMMAND_MANAGER_ENABLED_ENABLE;
        return 1U;
    }

    return 0U;
}
