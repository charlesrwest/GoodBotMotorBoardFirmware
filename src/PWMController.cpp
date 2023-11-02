#include "PWMController.hpp"

using namespace GoodBot;


PWMController::PWMController(Timer inputTimerToClaim, uint32_t inputPeriodInMicroseconds) : periodInMicroseconds(inputPeriodInMicroseconds), driver(&GetDriver(inputTimerToClaim))
{
    if((inputTimerToClaim != Timer::TIMER14))
    {
        config = {
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
    };
        pwmStart(driver, &config);
    }
    else
    {
        config = {
   DEFAULT_PWM_TIMER_FREQUENCY,
   inputPeriodInMicroseconds,
   NULL,
   {
      {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   },
   0,
   0,
   0
    };
        pwmStart(driver, &config);
    }
//chprintf((BaseSequentialStream*)&SD1, "Started pwm driver\r\n");
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

//chprintf((BaseSequentialStream*)&SD1, "Channel %d Microseconds high %d\r\n", inputChannel, inputMicroSecondsHighPerCycle);
pwmEnableChannelI(driver, inputChannel, inputMicroSecondsHighPerCycle);
return true;
}

bool PWMController::SetDutyCycleIntegerPercentage(uint32_t inputChannel, uint32_t percentage)
{
    if((inputChannel+1) > GetNumberOfChannels())
    {
        return false;
    }

    //chprintf((BaseSequentialStream*)&SD1, "Channel %d Microseconds high %d\r\n", inputChannel, inputMicroSecondsHighPerCycle);
    pwmEnableChannelI(driver, inputChannel, PWM_PERCENTAGE_TO_WIDTH(driver, 100*percentage));
    return true;
}

uint32_t PWMController::GetNumberOfChannels() const
{
return PWM_CHANNELS;
}

PWMDriver& GoodBot::GetDriver(Timer inputTimer)
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

#if STM32_PWM_USE_TIM14
case Timer::TIMER14:
return PWMD14;
break;
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

