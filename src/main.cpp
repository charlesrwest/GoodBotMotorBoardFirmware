#include <array>
#include <algorithm>

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

#define ADC_GRP1_NUM_CHANNELS   3
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
New measures for ADC:
//bat, charger, current

Battery
Ground: 4
15.45 volts -> 1309
13.3 volts -> 1130
13.5 volts -> 1145
13.55 volts -> 1149
13.62 -> 1153
13.69 -> 1159
14.67 -> 1241

DC power:
Ground: 7
15.4 volts -> 1321

Current resistor:
15.4 volts -> 1321

*/

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
 
13.30 volt start voltage
15.46 voltage power supply
.3 A current at 20% duty cycle
1.05 A current at 30% duty cycle
1.15 A current at 32% duty cycle
1.35 A current at 34% duty cycle
4.0 A current at 35% duty cycle
 
11.18 volt start voltage (complete discharge followed by ~48 hours of inactivity).
15.45 voltage power supply
0.0 A current at 10% duty cycle
0.0 A current at 15% duty cycle
0.2 A current at 20% duty cycle
0.65 A current at 25% duty cycle
1.2 A current at 30% duty cycle
1.45 A current at 32% duty cycle
1.7 A current at 34% duty cycle
4.5 A current at 35% duty cycle
4.5 A current at 36% duty cycle

 
Probably need to implement current monitoring, since the duty cycle/current relationship seems to be strongly non-linear.
*/


/**
TODO:
Bypass the ADC diodes and see if the readings are still good. <- much better
Check duty cycle vs current relationship with less charged battery.
Determine pin for current draw monitoring -> PA5 looks good

*/

volatile int32_t Battery_Voltage_Average_ADC_Reading = -1;
volatile int32_t Charger_Voltage_Average_ADC_Reading = -1;
volatile int32_t Charger_Current_Average_ADC_Reading = -1;
static void adccallback(ADCDriver *adcp) 
{
    int64_t average1 = 0; // For PA1
    int64_t average2 = 0; // For PA2
    int64_t average5 = 0; // For PA5

    if(adcIsBufferComplete(adcp)) 
    {
        //Second half of buffer is full
        for(int16_t buffer_index = (ADC_GRP1_NUM_CHANNELS*ADC_GRP1_BUF_DEPTH/2); buffer_index < ADC_GRP1_NUM_CHANNELS*ADC_GRP1_BUF_DEPTH; buffer_index+=3) 
        {
            average1 += samples1[buffer_index];
            average2 += samples1[buffer_index+1];
            average5 += samples1[buffer_index+2];
        } 
        average2 = average2 / 128;
        average1 = average1 / 128;
        average5 = average5 / 128;
    }
    else 
    {
        //First half of buffer is full
        for(int16_t buffer_index = 0; buffer_index < (ADC_GRP1_NUM_CHANNELS*ADC_GRP1_BUF_DEPTH/2); buffer_index+=3)
        {
            average1 += samples1[buffer_index];
            average2 += samples1[buffer_index+1];
            average5 += samples1[buffer_index+2];
        } 
        average2 = average2 / 128;
        average1 = average1 / 128;
        average5 = average5 / 128;
    }

    Battery_Voltage_Average_ADC_Reading = average2;
    Charger_Voltage_Average_ADC_Reading = average1;
    Charger_Current_Average_ADC_Reading = average5;
    
    //Convert to hundreths of a volt
    Battery_Voltage_Average_ADC_Reading = (Battery_Voltage_Average_ADC_Reading*3300)/4095;
    Charger_Voltage_Average_ADC_Reading = (Charger_Voltage_Average_ADC_Reading*3300)/4095;
    Charger_Current_Average_ADC_Reading = (Charger_Current_Average_ADC_Reading*3300)/4095;
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

std::array<int, 4> MotorVelocityTargetMilliRPM = {{0,0,0,0}};
std::array<bool, 4> SpeedControlDisabled = {{true, true, true, true}};
std::array<int, 4> LastPowerSettingScaled = {{0,0,0,0}};
const int PowerFixedPointScalingFactor = 1000;

void SetMotorTargetVelocity(int motorIndex, int TargetForwardVelocityMilliRPM)
{
    if((motorIndex < 0) || (motorIndex > MotorVelocityTargetMilliRPM.size()))
    {
        return;
    }

    int sign = 1;
    if((motorIndex == 0) || (motorIndex == 3))
    {
        sign = -1;
    }
    else
    {
        sign = 1;
    }
    
    MotorVelocityTargetMilliRPM[motorIndex] = sign*TargetForwardVelocityMilliRPM;
    SpeedControlDisabled[motorIndex] = false;
}

void StopMotor(int motorIndex)
{
    if((motorIndex < 0) || (motorIndex > MotorVelocityTargetMilliRPM.size()))
    {
        return;
    }
    
    DisableMotor(motorIndex);
    SpeedControlDisabled[motorIndex] = true;
    LastPowerSettingScaled[motorIndex] = 0;
}

/*
void UpdateMotorPowerForTargetVelocity()
{
    for(int motor_index = 0; motor_index < MotorVelocityTargetMilliRPM.size(); motor_index++)
    {
        if(SpeedControlDisabled[motorIndex])
        {
            continue;
        }
        
        int target_velocity = MotorVelocityTargetMilliRPM[motor_index];
        
        MotorMode mode = MotorMode::CCW;
        if(target_velocity >= 0)
        {
            mode = MotorMode::CW;
        }
        
        int current_velocity = MotorVelocityEstimateMilliRPM[motor_index];
        
        
        int velocity_difference = target_velocity - current_velocity;
        int power_increment = abs(velocity_difference/100);
        
    }

    MotorMode
    MotorVelocityTargetMilliRPM
}
*/

int last_error = 0;
int last_adjustment = 0;

void ControlMotors(int32_t deltaTimeMilliseconds)
{
    const int fixed_scale = 10000;
    const int max_speed = 230000; //200 rpm

    const int use_break_threshold = 100000;

    for (int motor_index = 0; motor_index < SpeedControlDisabled.size(); motor_index++)
    {
        if(SpeedControlDisabled[motor_index])
        {
            continue;
        }
        
        int currentVelocity = MotorVelocityEstimateMilliRPM[motor_index];
        int desiredVelocity = MotorVelocityTargetMilliRPM[motor_index];
        int average_velocity_scale = abs(desiredVelocity);
        int desired_velocity_scaling_factor = (average_velocity_scale*100)/max_speed;
        int power_adjustment_constant = 16*desired_velocity_scaling_factor;
    

        int desired_change = desiredVelocity - currentVelocity;
        last_error = desired_change;
        int power_adjustment = desired_change*power_adjustment_constant*deltaTimeMilliseconds/fixed_scale;
        last_adjustment = power_adjustment;

        
        int power_scaled = std::clamp<int>(power_adjustment+LastPowerSettingScaled[motor_index], -PowerFixedPointScalingFactor*FULL_POWER_SETTING, PowerFixedPointScalingFactor*FULL_POWER_SETTING); // Scale back to 0-100 range
        
        LastPowerSettingScaled[motor_index] = power_scaled;
        int actual_power = abs(power_scaled/PowerFixedPointScalingFactor);
        MotorMode mode = power_scaled > 0 ? MotorMode::CW : MotorMode::CCW;

        if((abs(currentVelocity) > use_break_threshold) && ((currentVelocity < 0) != (desiredVelocity < 0)))
        {
            //Need to reverse direction but doing so while powered will cause a big voltage spike.  Use the brake to slow down and then apply power
            DisableMotor(motor_index);
        }
        else
        {
            SetMotorPower(motor_index, mode, actual_power);
        }
    }
}

static THD_WORKING_AREA(MotorStateEstimatorThreadWorkingArea, 2048);
static THD_FUNCTION(MotorStateEstimatorThread, arg)
{
  (void)arg;
  chRegSetThreadName("MotorStateEstimatorThread");
  
  systime_t last_time = chVTGetSystemTime();
  

  for(int motor_index = 0; motor_index < MotorSettings.size(); motor_index++)
  {
    DisableMotor(motor_index);
  }
  
  while(true)
  {
    chSysLock();
    UpdateHallStates();
    chSysUnlock();
    
    chSysLock();
    UpdateVelocityEstimates();
    chSysUnlock();


    for(int32_t motor_index = 0; motor_index < (int) Hall_Pin_States.size(); motor_index++)
    {
        chSysLock();
        UpdateMotorHalfBridges(motor_index, MotorSettings[motor_index].Mode, MotorSettings[motor_index].DutyCycle, Hall_Pin_States[motor_index][0], Hall_Pin_States[motor_index][1], Hall_Pin_States[motor_index][2]);
        chSysUnlock();
    }
    
    
    //Update at 200 hz
    last_time = chThdSleepUntilWindowed(last_time, chTimeAddX(last_time, TIME_US2I(5000)));
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
        UpdateMotorHalfBridges(motor_index, MotorSettings[motor_index].Mode, MotorSettings[motor_index].DutyCycle, Hall_Pin_States[motor_index][0], Hall_Pin_States[motor_index][1], Hall_Pin_States[motor_index][2]);
    }
}



// Call ControlMotors in a loop at 1000 Hz with desired velocities
static THD_WORKING_AREA(MotorPIDThreadWorkingArea, 2048);
static THD_FUNCTION(MotorPIDThread, arg)
{
  (void)arg;
  chRegSetThreadName("MotorPIDThread");
  
  systime_t last_time = chVTGetSystemTime();
  while(true)
  {
    ControlMotors(1);
  
    last_time = chThdSleepUntilWindowed(last_time, chTimeAddX(last_time, TIME_US2I(100)));
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
    
    //Motor 0: PF3, PF4, PF5
    palSetPadCallback(GPIOF, 3, callback_function, (void*) 1); 
    palSetPadCallback(GPIOF, 4, callback_function, (void*) 2);
    palSetPadCallback(GPIOF, 5, callback_function, (void*) 3);
    
    palSetPadMode(GPIOF, 3,  PAL_MODE_INPUT_PULLUP);
    palSetPadMode(GPIOF, 4,  PAL_MODE_INPUT_PULLUP);
    palSetPadMode(GPIOF, 5,  PAL_MODE_INPUT_PULLUP);
    
    palEnablePadEvent(GPIOF, 3, PAL_EVENT_MODE_BOTH_EDGES);
    palEnablePadEvent(GPIOF, 4, PAL_EVENT_MODE_BOTH_EDGES);
    palEnablePadEvent(GPIOF, 5, PAL_EVENT_MODE_BOTH_EDGES);
    
    //Motor 1: PC6, PC7, PD8
    palSetPadCallback(GPIOC, 6, callback_function, (void*) 1); 
    palSetPadCallback(GPIOC, 7, callback_function, (void*) 2);
    palSetPadCallback(GPIOD, 8, callback_function, (void*) 3);
    
    palSetPadMode(GPIOC, 6,  PAL_MODE_INPUT_PULLUP);
    palSetPadMode(GPIOC, 7,  PAL_MODE_INPUT_PULLUP);
    palSetPadMode(GPIOD, 8,  PAL_MODE_INPUT_PULLUP);
    
    palEnablePadEvent(GPIOC, 6, PAL_EVENT_MODE_BOTH_EDGES);
    palEnablePadEvent(GPIOC, 7, PAL_EVENT_MODE_BOTH_EDGES);
    palEnablePadEvent(GPIOD, 8, PAL_EVENT_MODE_BOTH_EDGES);
    
    //Motor 2: PB12, PB13, PB14
    palSetPadCallback(GPIOB, 12, callback_function, (void*) 1); 
    palSetPadCallback(GPIOB, 13, callback_function, (void*) 2);
    palSetPadCallback(GPIOB, 14, callback_function, (void*) 3);
    
    palSetPadMode(GPIOB, 12,  PAL_MODE_INPUT_PULLUP);
    palSetPadMode(GPIOB, 13,  PAL_MODE_INPUT_PULLUP);
    palSetPadMode(GPIOB, 14,  PAL_MODE_INPUT_PULLUP);
    
    palEnablePadEvent(GPIOB, 12, PAL_EVENT_MODE_BOTH_EDGES);
    palEnablePadEvent(GPIOB, 13, PAL_EVENT_MODE_BOTH_EDGES);
    palEnablePadEvent(GPIOB, 14, PAL_EVENT_MODE_BOTH_EDGES);
    
    //Motor 3: PF7, PE7, PE8
    palSetPadCallback(GPIOF, 7, callback_function, (void*) 1); 
    palSetPadCallback(GPIOE, 7, callback_function, (void*) 2);
    palSetPadCallback(GPIOE, 8, callback_function, (void*) 3);
    
    palSetPadMode(GPIOF, 7,  PAL_MODE_INPUT_PULLUP);
    palSetPadMode(GPIOE, 7,  PAL_MODE_INPUT_PULLUP);
    palSetPadMode(GPIOE, 8,  PAL_MODE_INPUT_PULLUP);
    
    palEnablePadEvent(GPIOF, 7, PAL_EVENT_MODE_BOTH_EDGES);
    palEnablePadEvent(GPIOE, 7, PAL_EVENT_MODE_BOTH_EDGES);
    palEnablePadEvent(GPIOE, 8, PAL_EVENT_MODE_BOTH_EDGES);
    
    
    
    //Setup ADC
    adcgrpcfg1.circular = true;
    adcgrpcfg1.num_channels = ADC_GRP1_NUM_CHANNELS;
    adcgrpcfg1.end_cb = adccallback;
    adcgrpcfg1.error_cb = adcerrorcallback;
    adcgrpcfg1.cfgr1 = ADC_CFGR1_CONT | ADC_CFGR1_RES_12BIT;
    adcgrpcfg1.tr1 = ADC_TR(0, 0);
    adcgrpcfg1.smpr = ADC_SMPR_SMP1_160P5;
    adcgrpcfg1.chselr = ADC_CHSELR_CHSEL2 | ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL5;
      
    
    palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
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
    chThdCreateStatic(MotorPIDThreadWorkingArea, sizeof(MotorPIDThreadWorkingArea), NORMALPRIO, MotorPIDThread, NULL);

    int elapsed_time = 0;
    while(true)
    {
        chThdSleepMilliseconds(50);
        elapsed_time += 50;
        
        for(int motor_index = 0; motor_index < (int) Hall_Pin_States.size(); motor_index++)
        {
            std::array<int, 3> state = {{Hall_Pin_States[motor_index][0], Hall_Pin_States[motor_index][1], Hall_Pin_States[motor_index][2]}};
            chprintf((BaseSequentialStream*)&SD1, "Motor state %d: %d %d %d\r\n", motor_index, state[0], state[1], state[2]);//MotorVelocityEstimateMilliRPM[2]);
            //chprintf((BaseSequentialStream*)&SD1, "Motor State %d\r\n", MotorChangeHistory[2][0].ValidStateIndexCW);
            //chprintf((BaseSequentialStream*)&SD1, "Motor State %d\r\n", MotorChangeHistory[2][0].ValidStateIndexCW);
            //chprintf((BaseSequentialStream*)&SD1, "ADCs %d %d\r\n", Battery_Voltage_Average_ADC_Reading, Charger_Voltage_Average_ADC_Reading);
        }
        //chprintf((BaseSequentialStream*)&SD1, "Motor velocity %d\r\n", MotorVelocityEstimateMilliRPM[2]/1000);
        
        SetBatteryChargerPower(0);
        //chprintf((BaseSequentialStream*)&SD1, "ADC: %d %d %d\r\n" , Battery_Voltage_Average_ADC_Reading, Charger_Voltage_Average_ADC_Reading, Charger_Current_Average_ADC_Reading);
        for(int motor_index = 0; motor_index < Hall_Pin_States.size(); motor_index++)//MotorSettings.size()
        {   
            if(elapsed_time < 20000)
            {
                SetMotorTargetVelocity(motor_index, 80000);
                
                //SetMotorPower(motor_index, MotorDirection::BACKWARD, FULL_POWER_SETTING/4);
                chprintf((BaseSequentialStream*)&SD1, "Motor speed %d: %d %d %d %d\r\n", motor_index, MotorVelocityEstimateMilliRPM[motor_index], LastPowerSettingScaled[motor_index]/PowerFixedPointScalingFactor, last_error, last_adjustment);
                //StopMotor(motor_index);
                //DisableMotor(motor_index);
            }
            else if(elapsed_time < 40000)
            {
                SetMotorTargetVelocity(motor_index, -12000);
                chprintf((BaseSequentialStream*)&SD1, "Motor speed %d: %d %d %d %d\r\n", motor_index, MotorVelocityEstimateMilliRPM[motor_index], LastPowerSettingScaled[motor_index]/PowerFixedPointScalingFactor, last_error, last_adjustment);
            }
            else if(elapsed_time < 60000)
            {
                StopMotor(motor_index);
                //SetMotorPower(motor_index, MotorDirection::BACKWARD, FULL_POWER_SETTING/2);
                chprintf((BaseSequentialStream*)&SD1, "Motor speed %d: %d %d %d %d\r\n", motor_index, MotorVelocityEstimateMilliRPM[motor_index], LastPowerSettingScaled[motor_index]/PowerFixedPointScalingFactor, last_error, last_adjustment);
            }
            else
            {
                StopMotor(motor_index);
                //SetMotorPower(motor_index, MotorMode::DISABLED, 0);
                chprintf((BaseSequentialStream*)&SD1, "Motor speed %d: %d\r\n", motor_index, MotorVelocityEstimateMilliRPM[motor_index]);
                DisableMotor(motor_index);
            }
        }
        
        /*
        int active_motor_index = elapsed_time / 5000;
        for(int motor_index = 0; motor_index < MotorSettings.size(); motor_index++)
        {
            if(motor_index == active_motor_index)
            {
                SetMotorPower(motor_index, MotorMode::CW, 30);
            }
            else
            {
                DisableMotor(motor_index);
            }
        }
        */

        
        //chnWrite(&SD1, (uint8_t*) hello_world_string, 14);
    }
}
