#include "foc_app.h"

int main(void)
{
    FOC_App_Init();
    FOC_App_Start();

    while (1)
    {
        FOC_App_Loop();
        /* User application logic can be inserted here without changing FOC app internals. */
    }
}
