#include "PowerManagement.hpp"
#include "hal.h"

using namespace GoodBot;

PWMController* GoodBot::ChargerPWMController = nullptr;

void GoodBot::InitializePowerManagement()
{
    palWritePad(GPIOA, 3, false);
    palSetPadMode(GPIOA, 3, PAL_MODE_OUTPUT_PUSHPULL);
    palWritePad(GPIOA, 3, false);
    
    SetBatteryChargerPower(0);
    palSetPadMode(GPIOA, 4, PAL_MODE_ALTERNATE(4));
    SetBatteryChargerPower(0);
}

void GoodBot::SetMainPower(bool on)
{
    palWritePad(GPIOA, 3, on);
}

void GoodBot::SetBatteryChargerPower(int dutyCycle) //Duty cycle range 0-100
{
    if(dutyCycle < 0)
    {
       return;
    }
    else if(dutyCycle > 100)
    {
        dutyCycle = 100;
    }
    
    ChargerPWMController->SetDutyCycleIntegerPercentage(0, (uint32_t) dutyCycle);
}
