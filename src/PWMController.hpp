#pragma once

#include "ch.h"
#include "hal.h"
#include "mcuconf.h"
#include "halconf.h"
#include "chconf.h"

#include "chprintf.h"

namespace GoodBot
{


enum class Timer : uint8_t
{
#if STM32_PWM_USE_TIM1
TIMER1
#endif

#if STM32_PWM_USE_TIM1 && (STM32_PWM_USE_TIM2 || STM32_PWM_USE_TIM3)
,
#endif

#if STM32_PWM_USE_TIM2
//TIMER2
#endif

#if STM32_PWM_USE_TIM2 && STM32_PWM_USE_TIM3
,
#endif

#if STM32_PWM_USE_TIM3
//TIMER3
#endif

#if STM32_PWM_USE_TIM14
, TIMER14
#endif
};

PWMDriver& GetDriver(Timer inputTimer);

/**
Note: Timer setup
Settable:
Frequency in ticks/sec
how many "ticks" per PWM consideration period 
(duty cycle is set by # ticks high per period)
Frequency of PWM signal is (tick frequency/consideration period)
The number of possible duty cycle settings is (consideration period)
*/

/**
This class takes control of one of the timer peripherals and uses it to generate a PWM signal.
*/
class PWMController
{
public:
PWMController(Timer inputTimerToClaim, uint32_t inputPeriodInMicroseconds);

bool SetDutyCycle(uint32_t inputChannel, double inputDutyCycle);

bool SetDutyCycle(uint32_t inputChannel, uint32_t inputMicroSecondsHighPerCycle);

bool SetDutyCycleIntegerPercentage(uint32_t inputChannel, uint32_t percentage);

uint32_t GetNumberOfChannels() const;

static constexpr uint32_t DEFAULT_PWM_TIMER_FREQUENCY = 1000000;

PWMConfig config; 
uint32_t periodInMicroseconds;
PWMDriver *driver;
};
















}
