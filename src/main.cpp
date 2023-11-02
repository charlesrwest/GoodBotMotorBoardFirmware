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

void adcCalibrate(ADCDriver *adcp)
{
    ADC_TypeDef *adcreg = adcp->adc; // Get the ADC registers for the given ADC driver

    // Ensure ADEN is 0 (ADC is disabled)
    adcreg->CR &= ~ADC_CR_ADEN;

    // Ensure ADVREGEN = 1 (ADC voltage regulator is enabled)
    adcreg->CR |= ADC_CR_ADVREGEN;

    // A delay might be needed here for tADCVREG_SETUP time. Check datasheet for specific value.
    chThdSleepMilliseconds(1); // For example, if tADCVREG_SETUP is 1ms.

    // Start the calibration by setting ADCAL bit
    adcreg->CR |= ADC_CR_ADCAL;

    // Wait for calibration to complete by polling ADCAL or EOCAL bit
    while (adcreg->CR & ADC_CR_ADCAL) {
        // Optionally, you can add a timeout here to prevent endless waiting
    }

    // At this point, the calibration is complete. You can read the calibration factor
    // from the ADC_DR or ADC_CALFACT registers if you need it.
    // uint8_t calibration_factor = adcreg->DR & 0x7F;  // Using ADC_DR as an example.
}

#define ADC_GRP1_NUM_CHANNELS   1
#define ADC_GRP1_BUF_DEPTH      256

static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

//BATTERY ADCS READINGS:
//Battery 13.28 volts
//Inside divider 1.08 volts
//Behind diode .82 volts -> Measuring this seems to drop it
//ADC readings: .94 volts when not measuring behind diode, .80 to .81 when I am
//So when not measuring, diode drop is .14 currently

//Power supply 0: 12.28 volts
//Inside divider: 1.00 volts
//Behind diode: .75
//ADC readings: .92 volts when not measuring behind diode, .74 when I am
//So when not measuring, diode drop is .08 currently

//Power supply 1: 15.44 volts
//Inside divider: 1.25 volts
//Behind diode: .96
//ADC readings: .105 volts when not measuring behind diode, .95 volts when I am
//So when not measuring, diode drop is .2 currently

//My main power diodes have effectively 0 voltage drop right now when nothing is drawing much current.

//560k between battery and reading point
//52.3 between reading point and ground. So reading roughly 8.54 % of voltage.  Would expect 1.134 volts

//Remove the rectifier diode and double the voltage divider values.  The resistance should be enough to protect in the case of reverse insertion.

/**
Charging testing.
13.33 volt start voltage
15.44 voltage power supply
.7 A current at 30% duty cycle
.9 A current at 32%  duty cycle
1.0 A current at 33%  duty cycle
1.2 A current at 34%  duty cycle
3.8 A current at 35% duty cycle
3.8 A current at 40% duty cycle
3.8 A current at 50% duty cycle
 
Probably need to implement current monitoring, since the duty cycle/current relationship seems to be strongly non-linear.
*/


/**
TODO:
Get data reading up and running on the Power In.  See if the connection is still good.
Hook a lightbulb up the the battery connections and see how well driving it with PWM goes.
*/

volatile int32_t Battery_Voltage_Average_ADC_Reading = -1;
static void adccallback(ADCDriver *adcp) 
{
    int64_t average = 0;
    if(adcIsBufferComplete(adcp)) 
    {
        //Second half of buffer is full
        average = 0;
        for(int16_t buffer_index = 128; buffer_index < 256; buffer_index++)
        {
            average += samples1[buffer_index];
        } 
        average = average / 128;
    }
    else 
    {
        //First half of buffer is full
        average = 0;
        for(int16_t buffer_index = 0; buffer_index < 128; buffer_index++)
        {
            average += samples1[buffer_index];
        } 
        average = average / 128;
    }

    Battery_Voltage_Average_ADC_Reading = average;
    
    //Convert to hundreths of a volt
    Battery_Voltage_Average_ADC_Reading = (Battery_Voltage_Average_ADC_Reading*330)/4095;
}

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) 
{
  (void)adcp;
  (void)err;
}

static ADCConversionGroup adcgrpcfg1;



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
  MotorSettings[2].Mode = MotorMode::CCW;
  
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
    last_time = chThdSleepUntilWindowed(last_time, chTimeAddX(last_time, TIME_US2I(1000)));
  }
}


static SerialConfig NANO_UART_CONFIG = {
    115200,
    0,
    0,
    0,
};

const char hello_world_string[] = "Hello world!\r\n";

static void callback_function(void *arg)
{
    (void)arg; // Unused variable
    ToggleLED((int) arg);
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
}

int main(void)
{
    halInit();
    chSysInit();

     

    InitializeLEDs();
    
    PWMController charger_pwm(Timer::TIMER14, Charger_PWM_Period_Number);
    ChargerPWMController = &charger_pwm;
    
    InitializePowerManagement();
    InitializeMotorStateEstimation();
    
    palSetPadCallback(GPIOB, 12, callback_function, (void*) 1); 
    palSetPadCallback(GPIOB, 13, callback_function, (void*) 2);
    palSetPadCallback(GPIOB, 14, callback_function, (void*) 3);
    
    palSetPadMode(GPIOB, 12,  PAL_MODE_INPUT_PULLUP);
    palSetPadMode(GPIOB, 13,  PAL_MODE_INPUT_PULLUP);
    palSetPadMode(GPIOB, 14,  PAL_MODE_INPUT_PULLUP);
    
    palEnablePadEvent(GPIOB, 12, PAL_EVENT_MODE_BOTH_EDGES);
    palEnablePadEvent(GPIOB, 13, PAL_EVENT_MODE_BOTH_EDGES);
    palEnablePadEvent(GPIOB, 14, PAL_EVENT_MODE_BOTH_EDGES);
    
    //Setup ADC
    adcgrpcfg1.circular = true;
    adcgrpcfg1.num_channels = ADC_GRP1_NUM_CHANNELS;
    adcgrpcfg1.end_cb = adccallback;
    adcgrpcfg1.error_cb = adcerrorcallback;
    adcgrpcfg1.cfgr1 = ADC_CFGR1_CONT | ADC_CFGR1_RES_12BIT;
    adcgrpcfg1.tr1 = ADC_TR(0, 0);
    adcgrpcfg1.smpr = ADC_SMPR_SMP_160P5;
    adcgrpcfg1.chselr = ADC_CHSELR_CHSEL2;
      
    
    palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
    adcStart(&ADCD1, NULL);
    
    adcCalibrate(&ADCD1);
    
    adcStartConversion(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
    
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
            //chprintf((BaseSequentialStream*)&SD1, "Motor State %d\r\n", MotorChangeHistory[2][0].ValidStateIndexCW);
            //chprintf((BaseSequentialStream*)&SD1, "Motor State %d\r\n", MotorChangeHistory[2][0].ValidStateIndexCW);
            chprintf((BaseSequentialStream*)&SD1, "Battery ADC %d\r\n", Battery_Voltage_Average_ADC_Reading);
        }
        //chprintf((BaseSequentialStream*)&SD1, "Motor velocity %d\r\n", MotorVelocityEstimateMilliRPM[2]/1000);
        
        SetBatteryChargerPower(0);
        if(elapsed_time > 5000)
        {
            MotorSettings[2].DutyCycle = 0;
            MotorSettings[2].Mode = MotorMode::BRAKES_ON;
            //SetBatteryChargerPower(34);
        }
        else
        {
            //SetBatteryChargerPower(0);
            //MotorSettings[2].DutyCycle = elapsed_time/400;
            //MotorSettings[2].DutyCycle = 25;
        }

        
        //chnWrite(&SD1, (uint8_t*) hello_world_string, 14);
    }
}
