#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "LEDs.hpp"
#include "PowerManagement.hpp"
#include "PWMController.hpp"
#include "MotorStateEstimation.hpp"
#include "MotorPhasePowerControl.hpp"

using namespace GoodBot;

/**
TODO:
1. Investigate setting input to pull up causing a crash.  See if reproducable in dev boards.
2. Investigate strange spikes in estimated speed
*/

//LED blinking thread
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg)
{
  (void)arg;
  chRegSetThreadName("blinker");
  int led_to_blink = 1;
  while (true)
  {
    ToggleLED(led_to_blink);
    chThdSleepMilliseconds(500);
    ToggleLED(led_to_blink);
    chThdSleepMilliseconds(500);
    
    led_to_blink++;
    if(led_to_blink > 4)
    {
        led_to_blink = 1;
    }
  }
}


bool MotorHallStateChanged(int motorIndex)
{
    if(motorIndex < 0 || motorIndex > 4)
    {
        return false;
    }
    
    const auto& hall_states = Hall_Pin_States[motorIndex];
    const auto& last_hall_states = Last_Hall_Pin_States[motorIndex];
    
    bool change_occurred = false;
    for(int32_t hall_index = 0; hall_index < (int) hall_states.size(); hall_index++)
    {
        if(hall_states[hall_index] != last_hall_states[hall_index])
        {
            change_occurred = true;
            break;
        }
    }
    
    return change_occurred;
}

static THD_WORKING_AREA(MotorStateEstimatorThreadWorkingArea, 2048);
static THD_FUNCTION(MotorStateEstimatorThread, arg)
{
  (void)arg;
  chRegSetThreadName("MotorStateEstimatorThread");
  
  systime_t last_time = chVTGetSystemTime();
  
  //Test motor 2
  MotorSettings[2].DutyCycle = 0;
  MotorSettings[2].Mode = MotorMode::CW;
  
  while(true)
  {
    UpdateHallStates();
    UpdateVelocityEstimates();
    
    for(int32_t motor_index = 0; motor_index < (int) Hall_Pin_States.size(); motor_index++)
    {
        //if(MotorHallStateChanged(motor_index) && motor_index == 2)
        if(motor_index == 2)
        {
            UpdateMotorHalfBridges(motor_index, MotorSettings[motor_index].Mode, MotorSettings[motor_index].DutyCycle, Hall_Pin_States[motor_index][0], Hall_Pin_States[motor_index][1], Hall_Pin_States[motor_index][2]);
        }
    }
    
    //Update at 10 kilohertz
    last_time = chThdSleepUntilWindowed(last_time, chTimeAddX(last_time, TIME_US2I(100)));
  }
}


static SerialConfig NANO_UART_CONFIG = {
    115200,
    0,
    0,
    0,
};

const char hello_world_string[] = "Hello world!\r\n";

int main(void)
{
    halInit();
    chSysInit();

    InitializeLEDs();
    InitializePowerManagement();
    InitializeMotorStateEstimation();
    
    palSetPadMode(GPIOB, 12,  PAL_MODE_INPUT_PULLUP);
    palSetPadMode(GPIOB, 13,  PAL_MODE_INPUT_PULLUP);
    palSetPadMode(GPIOB, 14,  PAL_MODE_INPUT_PULLUP);
    
    //palSetPadMode(GPIOF, 3,  PAL_MODE_INPUT_PULLUP); // <- causes crashes but can be set to pulldown.  However, crashes if set high
    //palSetPadMode(GPIOF, 4,  PAL_MODE_INPUT_PULLUP); // <- causes crashes but can read if pulldown
    //palSetPadMode(GPIOF, 5,  PAL_MODE_INPUT_PULLUP); // <- works as expected

//{{{GPIOF, 3}, {GPIOF, 4}, {GPIOF, 5}}}, {{{GPIOC, 6}, {GPIOC, 7}, {GPIOD, 8}}},  {{{GPIOB, 12}, {GPIOB, 13}, {GPIOB, 14}}},  {{{GPIOF, 7}, {GPIOE, 7}, {GPIOE, 8}}}

    chThdSleepMilliseconds(500);
    SetMainPower(true);

    //Setup pins for nano UART
    sdStart(&SD1, &NANO_UART_CONFIG);
    palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(0)); //USART1_TX
    palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(0)); //USART1_RX

    //Setup pins for motor control
    MotorsPWMManagerClass motor_pwm_manager;
    MotorsPWMManager = &motor_pwm_manager;
    SetupMotorControllerPins();

/*
    static const int Manager_PWM_FREQUENCY = 6000; //Hertz
    static const int HundredthsOfMicroSecondsPerSecond = 100000000;
    static const int PWM_Period_Number = HundredthsOfMicroSecondsPerSecond/Manager_PWM_FREQUENCY; 

    PWMDriver *driver = &PWMD1;

    PWMConfig config = {
   1000000,
   PWM_Period_Number,
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
    */

    //palSetPadMode(GPIOE, 14, PAL_MODE_ALTERNATE(1));
    //pwmEnableChannel(driver, 4-1, PWM_Period_Number/2);

    



    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
    chThdCreateStatic(MotorStateEstimatorThreadWorkingArea, sizeof(MotorStateEstimatorThreadWorkingArea), NORMALPRIO, MotorStateEstimatorThread, NULL);

    int elapsed_time = 0;
    while(true)
    {
        chThdSleepMilliseconds(50);
        elapsed_time += 50;
        
        for(int motor_index = 0; motor_index < (int) Hall_Pin_States.size(); motor_index++)
        {
            std::array<int, 3> state = {{Hall_Pin_States[motor_index][0], Hall_Pin_States[motor_index][1], Hall_Pin_States[motor_index][2]}};
            //chprintf((BaseSequentialStream*)&SD1, "Motor state %d: %d %d %d\r\n", motor_index, state[0], state[1], state[2]);//MotorVelocityEstimateMilliRPM[2]);
            chprintf((BaseSequentialStream*)&SD1, "Motor State %d\r\n", MotorChangeHistory[2][0].ValidStateIndexCW);
            //chprintf((BaseSequentialStream*)&SD1, "Motor State %d\r\n", MotorChangeHistory[2][0].ValidStateIndexCW);
        }
        //chprintf((BaseSequentialStream*)&SD1, "Motor velocity %d\r\n", MotorVelocityEstimateMilliRPM[2]/1000);
        
        if(elapsed_time > 10000)
        {
            MotorSettings[2].DutyCycle = 0;
            MotorSettings[2].Mode = MotorMode::BRAKES_ON;
        }
        else
        {
            MotorSettings[2].DutyCycle = elapsed_time/100;
        }

        
        //chnWrite(&SD1, (uint8_t*) hello_world_string, 14);
    }
}
