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

uint32_t GetNumberOfChannels() const;

static constexpr uint32_t DEFAULT_PWM_TIMER_FREQUENCY = 1000000;

PWMConfig config; 
uint32_t periodInMicroseconds;
PWMDriver *driver;
};

PWMController::PWMController(Timer inputTimerToClaim, uint32_t inputPeriodInMicroseconds) : 
config
{
   DEFAULT_PWM_TIMER_FREQUENCY,
   inputPeriodInMicroseconds,
   NULL,
   {
      {PWM_OUTPUT_ACTIVE_HIGH, NULL},
      {PWM_OUTPUT_ACTIVE_HIGH, NULL},
      {PWM_OUTPUT_ACTIVE_HIGH, NULL},
      {PWM_OUTPUT_ACTIVE_HIGH, NULL}
   },
   0,
   0,
   0
}, periodInMicroseconds(inputPeriodInMicroseconds), driver(&GetDriver(inputTimerToClaim))
{
pwmStart(driver, &config);
chprintf((BaseSequentialStream*)&SD1, "Started pwm driver\r\n");
}

bool PWMController::SetDutyCycle(uint32_t inputChannel, double inputDutyCycle)
{
if((inputChannel+1) > GetNumberOfChannels())
{
return false;
}



pwmEnableChannel(driver, inputChannel, PWM_PERCENTAGE_TO_WIDTH(driver, 10000*inputDutyCycle));
return true;
}

bool PWMController::SetDutyCycle(uint32_t inputChannel, uint32_t inputMicroSecondsHighPerCycle)
{
if((inputChannel+1) > GetNumberOfChannels())
{
return false;
}

ToggleLED(1);
//chprintf((BaseSequentialStream*)&SD1, "Channel %d Microseconds high %d\r\n", inputChannel, inputMicroSecondsHighPerCycle);
pwmEnableChannel(driver, inputChannel, inputMicroSecondsHighPerCycle);
return true;
}

uint32_t PWMController::GetNumberOfChannels() const
{
return PWM_CHANNELS;
}

PWMDriver& GetDriver(Timer inputTimer)
{
switch(inputTimer)
{
#if STM32_PWM_USE_TIM1
case Timer::TIMER1:
return PWMD1;
break;
#endif

#if STM32_PWM_USE_TIM2
//case Timer::TIMER2:
//return PWMD2;
//break;
#endif

#if STM32_PWM_USE_TIM3
//case Timer::TIMER3:
//return PWMD3;
//break;
#endif

default:
#if STM32_PWM_USE_TIM1
return PWMD1;
#elif STM32_PWM_USE_TIM2
//return PWMD2;
#elif STM32_PWM_USE_TIM3
//return PWMD3;
#endif
}
}
















}
