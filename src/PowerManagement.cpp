#include "PowerManagement.hpp"
#include "hal.h"

using namespace GoodBot;

void GoodBot::InitializePowerManagement()
{
    palWritePad(GPIOA, 3, false);
    palSetPadMode(GPIOA, 3, PAL_MODE_OUTPUT_PUSHPULL);
    palWritePad(GPIOA, 3, false);
}

void GoodBot::SetMainPower(bool on)
{
    palWritePad(GPIOA, 3, on);
}
