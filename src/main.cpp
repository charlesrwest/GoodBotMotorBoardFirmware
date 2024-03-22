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
#include "Messages.hpp"
#include "MessageReader.hpp"
#include "AtomicNanoMessageWriter.hpp"
#include "MessageQueue.hpp"


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
0 volts -> 4
13.3 volts -> 1130
13.5 volts -> 1145
13.55 volts -> 1149
13.62 -> 1153
13.69 -> 1159
14.67 -> 1241
15.45 volts -> 1309

linear regression with integer math:
microvolts = 11840*(ADCValue)−49580

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


bool AllStop = true;
bool CurrentlyCharging = false;


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
static THD_WORKING_AREA(waThread1, 256);
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
    if((motorIndex < 0) || (motorIndex > (int) MotorVelocityTargetMilliRPM.size()))
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
    if((motorIndex < 0) || (motorIndex > (int) MotorVelocityTargetMilliRPM.size()))
    {
        return;
    }
    
    DisableMotor(motorIndex);
    SpeedControlDisabled[motorIndex] = true;
    LastPowerSettingScaled[motorIndex] = 0;
}

void StopMotors()
{
    for(int motor_index = 0; motor_index < ((int) MotorVelocityTargetMilliRPM.size()); motor_index++)
    {
        StopMotor(motor_index);
    }
}


int last_error = 0;
int last_adjustment = 0;

void ControlMotors(int32_t deltaTimeMilliseconds)
{
    const int fixed_scale = 10000;
    const int max_speed = 230000; //200 rpm

    const int use_break_threshold = 100000;

    for (int motor_index = 0; motor_index < (int) SpeedControlDisabled.size(); motor_index++)
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
  
  for(int motor_index = 0; motor_index < (int) MotorSettings.size(); motor_index++)
  {
    DisableMotor(motor_index);
  }
  
  while(true)
  {
    UpdateHallStates();
    UpdateVelocityEstimates();


    for(int32_t motor_index = 0; motor_index < (int) Hall_Pin_States.size(); motor_index++)
    {
        UpdateMotorHalfBridges(motor_index, MotorSettings[motor_index].Mode, MotorSettings[motor_index].DutyCycle, Hall_Pin_States[motor_index][0], Hall_Pin_States[motor_index][1], Hall_Pin_States[motor_index][2]);
    }
    
    chBSemWaitTimeout(&HallInterruptTriggeredBinarySemaphore, TIME_US2I(5000));
  }
}


const char hello_world_string[] = "Hello world!\r\n";

static void callback_function(void *arg)
{
    (void)arg; // Unused variable
    chSysLockFromISR();
    chBSemSignalI(&HallInterruptTriggeredBinarySemaphore);
    chSysUnlockFromISR();
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
    if(AllStop || CurrentlyCharging)
    {
        StopMotors();
    }
    else
    {
        ControlMotors(10);
    }
    
    last_time = chThdSleepUntilWindowed(last_time, chTimeAddX(last_time, TIME_US2I(1000)));
  }
}

static SerialConfig GPS_SERIAL_CONFIG = {
    38400,
    0,
    0,
    0,
};

const static std::array<uint8_t, 44> UBX_CFG_NAV5_STRING = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x82};

static THD_WORKING_AREA(GPS_UART_ThreadWorkingArea, 1024);
static THD_FUNCTION(GPS_UART_Thread, arg) 
{
    (void)arg;
    chRegSetThreadName("GPS_UART_Thread");

    //Setup pins for GPS UART
    sdStart(&SD2, &GPS_SERIAL_CONFIG);
    palSetPadMode(GPIOD, 5, PAL_MODE_ALTERNATE(0)); //USART2_TX
    palSetPadMode(GPIOD, 6, PAL_MODE_ALTERNATE(0)); //USART2_RX

    chThdSleepMilliseconds(2000); //Give the GPS time to wake up and be ready to receive

    //Set to pedestrian
    for(int8_t try_index = 0; try_index < 10; try_index++)
    {
        chnWrite(&SD2, (uint8_t*) UBX_CFG_NAV5_STRING.data(), UBX_CFG_NAV5_STRING.size());
        chThdSleepMilliseconds(20);
    }

    AtomicNanoMessageWriter uart_writer;

    std::array<char, 32> receive_buffer; //Change to 32 to fit max nano UART passing message size

    while(true)
    {
        //Long timeout, get a single character
        int32_t number_of_bytes_read = chnReadTimeout(&SD2, (uint8_t*) receive_buffer.data(), receive_buffer.size(), TIME_MS2I(50));

        //Forward received message
        //Construct and send message to USB
        if(number_of_bytes_read > 0)
        {
            uart_writer.BeginTransaction();
            uart_writer.WriteMessageHeader(MessageType::REPORT_NMEA_STRING_PART, 0);
            uart_writer.WriteBinary((const char *) receive_buffer.data(), number_of_bytes_read);
            uart_writer.CommitTransaction();
        }
    }

}


static SerialConfig NANO_UART_CONFIG = {
    115200,
    0,
    0,
    0,
};

static THD_WORKING_AREA(Nano_Write_UART_ThreadWorkingArea, 3072);
static THD_FUNCTION(Nano_Write_UART_Thread, arg) 
{
    (void)arg;
    chRegSetThreadName("Nano_Write_UART_Thread");

//Maintain 2 buffers.  Buffer 1 is mutex protected and for other threads to write to.  Buffer 2 is internal to the UART write thread and is used for writes.  When a write to buffer 1 is completed, the UART write thread is woken up to claim Buffer 1 and copy its contents to buffer 2.  It then writes the contents of buffer 2 to UART.

    MessageQueue<OUTGOING_NANO_MESSAGE_QUEUE_MAX_MESSAGE_COUNT, MAX_OUTGOING_NANO_MESSAGE_BUFFER_MESSAGE_SIZE> global_usb_write_queue;
    GLOBAL_OUTGOING_NANO_MESSAGE_QUEUE = &global_usb_write_queue; //Make queue globally accessable

    std::array<char, 512> internal_buffer;
    PPPEncoder encoder;

    //Setup pins for GPS UART
    sdStart(&SD1, &NANO_UART_CONFIG);
    palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(0)); //USART1_TX
    palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(0)); //USART1_RX

    while(true)
    {   
        int32_t number_of_messages = GLOBAL_OUTGOING_NANO_MESSAGE_QUEUE->NumberOfMessagesAvailable();
        number_of_messages = std::max<int32_t>(1, number_of_messages); //Do a blocking read if there aren't any ready to be read

        int32_t buffer_offset = 0;
        for(int32_t message_index = 0; message_index < number_of_messages; message_index++)
        {
            int32_t bytes_retrieved = GLOBAL_OUTGOING_NANO_MESSAGE_QUEUE->RetrieveMessage((uint8_t*) (internal_buffer.data() + buffer_offset), internal_buffer.size() - buffer_offset, 100);
            if(bytes_retrieved < 0)
            {
                break; //Out of messages to process
            }

            buffer_offset += bytes_retrieved;
        }

        //Frame and write to nano
        if(buffer_offset > 0)
        {
            char start_byte = encoder.StartFrame();
            chnWrite(&SD1, (uint8_t*) &start_byte, sizeof(start_byte));
            for(int32_t buffer_index = 0; buffer_index < buffer_offset; buffer_index++)
            {
                std::array<char, 2> encoded_bytes = encoder.EncodeCharacter(internal_buffer[buffer_index]);
                chnWrite(&SD1, (uint8_t*) encoded_bytes.data(), encoder.NumberOfEncodedCharactersToUse());
            }

            std::array<char, MaxEndFrameLength> encoded_bytes = encoder.EndFrame();
            chnWrite(&SD1, (uint8_t*) encoded_bytes.data(), encoder.NumberOfEncodedCharactersToUse());
        }
    }
}

static THD_WORKING_AREA(Nano_Read_UART_ThreadWorkingArea, 1024);
static THD_FUNCTION(Nano_Read_UART_Thread, arg) 
{
    (void)arg;
    chRegSetThreadName("Nano_Read_UART_Thread");

    auto SetPWMToStop = [&]()
    {
        //STOP
    };

    //Set chassis to start out stopped and wheels pointed center
    SetPWMToStop();

    uint32_t milliseconds_to_wait_without_command_before_shutdown = 300;
    uint32_t time_of_last_update = chVTGetSystemTime() - TIME_MS2I(milliseconds_to_wait_without_command_before_shutdown*2);
 
    MessageReader<256> reader;
    while(true)
    {
        uint32_t current_time_minus_stop_delay = chVTGetSystemTime() - TIME_MS2I(milliseconds_to_wait_without_command_before_shutdown);
        if(current_time_minus_stop_delay > time_of_last_update)
        {
            SetPWMToStop(); //Haven't gotten update in a while, so switch to safe state
        }
        int32_t frames_decoded = reader.BlockingRead(SD2, 500); //Wait for 500 milliseconds

        while(reader.PeekNextMessageType() != MessageType::INVALID)
        {
            switch(reader.PeekNextMessageType())
            {
                case MessageType::SET_PWM:
                {
                    SetPWMMessage message;
                    if(!reader.GetMessage(message))
                    {
                       continue;
                    }

                    if(message.Channel == 0)
                    {
                    /*
                        bool got_lock = chMtxTryLock(&pwm_mutex);
                        if(got_lock)
                        {
                            ToggleBlueLED();
                            pwm_controller->SetDutyCycle(STEERING_CHANNEL, (uint32_t) message.OnTime);
                            chMtxUnlock(&pwm_mutex);
                        }
                        */
                        time_of_last_update = chVTGetSystemTime();
                    }
                    else if(message.Channel == 1)
                    {
                        //Set target for speed/direction
                        /*
                        bool got_lock = chMtxTryLock(&pwm_mutex);
                        if(got_lock)
                        {
                            //ToggleBlueLED();
                            pwm_controller->SetDutyCycle(SPEED_CHANNEL, (uint32_t) message.OnTime);
                            chMtxUnlock(&pwm_mutex);
                        }
                        */
                        time_of_last_update = chVTGetSystemTime();
                    }
                }
                break;

                default:
                chThdSleepMilliseconds(2);
                break;
            }
        }
    }

}

bool IsBetween(int value, int lowerBound, int upperBound)
{
    return (value > lowerBound) && (value < upperBound);
}

int ConvertBatteryADCToMicrovolts(int adcValue)
{
    return 11840*(adcValue)-49580;
}

static THD_WORKING_AREA(Battery_Management_ThreadWorkingArea, 1024);
static THD_FUNCTION(Battery_Management_Thread, arg) 
{
    /**
    Charging plan for now:
    1. Define a target charge voltage Vt = 14.2 volts.  This is the voltage that we want the battery to reach during the charging cycle.
    2. Define a recharge voltage Vc = 12.4 volts.  This is the maximum voltage the battery can be to start a recharge cycle.
    3. Define an expected voltage window for both the battery and the charger.  If the battery or the charger are outside the window, no charging will occur (battery damaged, battery disconnected, charger disconnected, etc).
        *For battery: (11.5 volts, 14.4 volts) both when connected to charger and without
        *For charger: (14.4 volts, 15.8 volts) when disconnected from battery, battery range when connected
    4. If the battery voltage and the charger are inside the expected voltage window range and the battery voltage is < Vc, the charging cycle will be initiated.
    5. During the charging cycle, the PWM duty cycle will be set to 32% until either a maximum time or the battery voltage reaches Vt.  If either the battery or the charge voltage moves ouside the expected voltage window, then charging is disabled.
    6. Set a minimum voltage for motor operations Vm = 12.2 volts.  If the battery goes below 12.2 volts, disable main system power.


    Implementation loop:
    State: CurrentlyCharging, AllStop.

    Every 10 ms, check if we should be charging.  PWM set to 32% if CurrentlyCharging is set to true and 0% otherwise.

    1. If battery and charger voltages are outside of the acceptable windows, CurrentlyCharging = false.
    2. else if battery voltage >= Vt, CurrentlyCharging = false.
    3. else if battery voltage < Vc, CurrentlyCharging = true.

    if CurrentlyCharging == true, battery voltage is outside of the acceptable window or battery voltage is < Vm, AllStop = true.

    Set PWM value according to CurrentlyCharging, wait for next 10 ms checkin.  Also, force motors into BRAKE mode and kill main power if AllStop is triggered.
    */
    (void)arg;
    chRegSetThreadName("Battery_Management_Thread");
    
    const static int target_voltage_micro_volts = 14200000;
    const static int recharge_voltage_micro_volts = 13800000;
    const static int min_batter_voltage_micro_volts = 12200000;
    const static std::array<const int, 2> battery_voltage_window = {{11500000, 14400000}};
    const static std::array<const int, 2> charger_voltage_window = {{14400000, 15800000}};

    chThdSleepMilliseconds(500);

    systime_t last_time = chVTGetSystemTime();
    while(true)
    {
        int battery_voltage_estimate_microvolts = ConvertBatteryADCToMicrovolts(Battery_Voltage_Average_ADC_Reading);
        int charger_voltage_estimate_microvolts = ConvertBatteryADCToMicrovolts(Charger_Voltage_Average_ADC_Reading);
    
        bool battery_voltage_in_window = IsBetween(battery_voltage_estimate_microvolts, battery_voltage_window[0], battery_voltage_window[1]);
        bool charger_voltage_in_window = false;
        charger_voltage_in_window = IsBetween(charger_voltage_estimate_microvolts, charger_voltage_window[0], charger_voltage_window[1]);

        
        //1. If battery and charger voltages are outside of the acceptable windows, CurrentlyCharging = false.
        if((!battery_voltage_in_window) || (!charger_voltage_in_window))
        {
            CurrentlyCharging = false;
        }
        //2. else if battery voltage >= Vt, CurrentlyCharging = false.
        else if(battery_voltage_estimate_microvolts > target_voltage_micro_volts)
        {
            CurrentlyCharging = false;
        }
        else if(battery_voltage_estimate_microvolts < recharge_voltage_micro_volts)
        {
            CurrentlyCharging = true;
        }
        
        //chprintf((BaseSequentialStream*)&SD1, "battery: %d %d %d %d %d %d\r\n" , battery_voltage_estimate_microvolts, charger_voltage_estimate_microvolts, battery_voltage_in_window, charger_voltage_in_window, (battery_voltage_estimate_microvolts < min_batter_voltage_micro_volts), CurrentlyCharging);
        
        
        //if CurrentlyCharging == true, battery voltage is outside of the acceptable window or battery voltage is < Vm, AllStop = true.
        if(CurrentlyCharging || (!battery_voltage_in_window) || (battery_voltage_estimate_microvolts < min_batter_voltage_micro_volts) || charger_voltage_in_window)
        {
            AllStop = true;
            if(!CurrentlyCharging)
            {
                SetMainPower(false);
            }
        }
        else
        {
            AllStop = false;
            SetMainPower(true);
        }
        
        if(CurrentlyCharging)
        {
            SetBatteryChargerPower(34);
        }
        else
        {
            SetBatteryChargerPower(0);
        }
        
        //SetLED(1, CurrentlyCharging);
        //SetLED(2, AllStop);
        
        last_time = chThdSleepUntilWindowed(last_time, chTimeAddX(last_time, TIME_US2I(100000)));
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

    chThdSleepMilliseconds(5000);
    SetMainPower(true);

    //Setup pins for motor control
    MotorsPWMManagerClass motor_pwm_manager;
    MotorsPWMManager = &motor_pwm_manager;
    SetupMotorControllerPins();


    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
    chThdCreateStatic(MotorStateEstimatorThreadWorkingArea, sizeof(MotorStateEstimatorThreadWorkingArea), NORMALPRIO, MotorStateEstimatorThread, NULL);
    chThdCreateStatic(MotorPIDThreadWorkingArea, sizeof(MotorPIDThreadWorkingArea), NORMALPRIO, MotorPIDThread, NULL);
    chThdCreateStatic(GPS_UART_ThreadWorkingArea, sizeof(GPS_UART_ThreadWorkingArea), NORMALPRIO, GPS_UART_Thread, NULL);
    chThdCreateStatic(Battery_Management_ThreadWorkingArea, sizeof(Battery_Management_ThreadWorkingArea), NORMALPRIO, Battery_Management_Thread, NULL);

    //Start the Nano UART writer/reader threads
    chThdCreateStatic(Nano_Write_UART_ThreadWorkingArea, sizeof(Nano_Write_UART_ThreadWorkingArea), NORMALPRIO, Nano_Write_UART_Thread, NULL);
    //chThdCreateStatic(Nano_Read_UART_ThreadWorkingArea, sizeof(Nano_Read_UART_ThreadWorkingArea), NORMALPRIO, Nano_Read_UART_Thread, NULL);


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
            //chprintf((BaseSequentialStream*)&SD1, "ADCs %d %d\r\n", Battery_Voltage_Average_ADC_Reading, Charger_Voltage_Average_ADC_Reading);
        }
        //chprintf((BaseSequentialStream*)&SD1, "Motor velocity %d\r\n", MotorVelocityEstimateMilliRPM[2]/1000);
        
        //SetBatteryChargerPower(0);
        //chprintf((BaseSequentialStream*)&SD1, "ADC: %d %d %d\r\n" , Battery_Voltage_Average_ADC_Reading, Charger_Voltage_Average_ADC_Reading, Charger_Current_Average_ADC_Reading);
        for(int motor_index = 4; motor_index < 4/*(int) Hall_Pin_States.size()*/; motor_index++)//MotorSettings.size()
        {   
        SetMotorTargetVelocity(motor_index, 80000);
        /*
            if(elapsed_time < 20000)
            {
                SetMotorTargetVelocity(motor_index, 80000);
                
                //SetMotorPower(motor_index, MotorDirection::BACKWARD, FULL_POWER_SETTING/4);
                //chprintf((BaseSequentialStream*)&SD1, "Motor speed %d: %d %d %d %d\r\n", motor_index, MotorVelocityEstimateMilliRPM[motor_index], LastPowerSettingScaled[motor_index]/PowerFixedPointScalingFactor, last_error, last_adjustment);
                //StopMotor(motor_index);
                //DisableMotor(motor_index);
            }
            else if(elapsed_time < 40000)
            {
                SetMotorTargetVelocity(motor_index, -12000);
                //chprintf((BaseSequentialStream*)&SD1, "Motor speed %d: %d %d %d %d\r\n", motor_index, MotorVelocityEstimateMilliRPM[motor_index], LastPowerSettingScaled[motor_index]/PowerFixedPointScalingFactor, last_error, last_adjustment);
            }
            else if(elapsed_time < 60000)
            {
                StopMotor(motor_index);
                //SetMotorPower(motor_index, MotorDirection::BACKWARD, FULL_POWER_SETTING/2);
                //chprintf((BaseSequentialStream*)&SD1, "Motor speed %d: %d %d %d %d\r\n", motor_index, MotorVelocityEstimateMilliRPM[motor_index], LastPowerSettingScaled[motor_index]/PowerFixedPointScalingFactor, last_error, last_adjustment);
            }
            else
            {
                StopMotor(motor_index);
                //SetMotorPower(motor_index, MotorMode::DISABLED, 0);
                //chprintf((BaseSequentialStream*)&SD1, "Motor speed %d: %d\r\n", motor_index, MotorVelocityEstimateMilliRPM[motor_index]);
                DisableMotor(motor_index);
            }
            */
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
