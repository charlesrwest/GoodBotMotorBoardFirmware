#pragma once

#include<cstdint>
#include<array>

namespace GoodBot
{

extern std::array<bool, 4> LED_States;

void InitializeLEDs();

void SetLED(int8_t ledNumber, bool light);

void ToggleLED(int8_t ledNumber);

}
