#pragma once

#include "PWMController.hpp"

namespace GoodBot
{

const int Charger_PWM_FREQUENCY = 1000; //Hertz
const int HundredthsOfMicroSecondsPerSecond = 100000000;
const int Charger_PWM_Period_Number = HundredthsOfMicroSecondsPerSecond/Charger_PWM_FREQUENCY; 

void InitializePowerManagement();

void SetMainPower(bool on);

void SetBatteryChargerPower(int dutyCycle);

extern PWMController* ChargerPWMController;
















}
