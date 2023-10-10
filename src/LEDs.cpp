#include "LEDs.hpp"
#include "hal.h"

using namespace GoodBot;

std::array<bool, 4> GoodBot::LED_States = {false, false, false, false};

void GoodBot::InitializeLEDs()
{
    palSetPadMode(GPIOF, 9, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOF, 10, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOF, 11, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOF, 12, PAL_MODE_OUTPUT_PUSHPULL);
    
    SetLED(1, false);
    SetLED(2, false);
    SetLED(3, false);
    SetLED(4, false);
}

void GoodBot::SetLED(int8_t ledNumber, bool light)
{
    if((ledNumber < 1) || (ledNumber > 4))
    {
        return;
    }

    switch(ledNumber)
    {
        case 1:
            palWritePad(GPIOF, 12, light);
            break;
        case 2:
            palWritePad(GPIOF, 11, light);
            break;
        case 3:
            palWritePad(GPIOF, 10, light);
            break;
        case 4:
            palWritePad(GPIOF, 9, light);
            break;

    }
    
    LED_States[ledNumber-1] = light;
}

void GoodBot::ToggleLED(int8_t ledNumber)
{
    if((ledNumber < 1) || (ledNumber > 4))
    {
        return;
    }
    
    SetLED(ledNumber, !LED_States[ledNumber-1]);
}
