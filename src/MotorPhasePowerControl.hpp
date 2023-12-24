#pragma once

#include<array>
#include<tuple>

#include "LEDs.hpp"
#include "PWMController.hpp"
#include "chprintf.h"

namespace GoodBot
{

struct MotorControllerParameters
{
  std::array<std::pair<stm32_gpio_t*, int>, 3> INPins; //UVW
  std::array<std::tuple<stm32_gpio_t*, int, int>, 3> ENPins;
  std::array<std::pair<int, int>, 3> ENTimerIndexAndChannelIndex;
};

std::array<MotorControllerParameters, 4> MotorControllerParametersList;

enum class MotorMode : uint8_t
{
  CW = 0,
  CCW = 1,
  BRAKES_ON = 3,
  DISABLED = 4
};

struct MotorSetting
{
  MotorMode Mode = MotorMode::BRAKES_ON;
  int DutyCycle = 0; //0 to FULL_POWER_SETTING for no power to full power
};

//Power level/mode to set a given motor to
std::array<MotorSetting, 4> MotorSettings;
const int FULL_POWER_SETTING = 10000;

enum class HalfBridgeId
{
  U = 0,
  V = 1, 
  W = 2
};

enum class HalfBridgeState
{
  HIGH_SIDE = 0,
  LOW_SIDE = 1, 
  DISABLED = 2
};


void SetMotorPower(int motorIndex, MotorMode mode, int power)
{
    if((motorIndex < 0) || (motorIndex >= (int) MotorSettings.size()))
    {
        return; //Invalid motor
    }

    if(power < 0)
    {
        power = 0;
    }
    if(power > FULL_POWER_SETTING)
    {
        power = FULL_POWER_SETTING;
    }
    
    MotorSettings[motorIndex].Mode = mode;
    MotorSettings[motorIndex].DutyCycle = power;
}

void DisableMotor(int motorIndex)
{
    SetMotorPower(motorIndex, MotorMode::BRAKES_ON, 0);
}

enum class MotorDirection : uint8_t
{
  FORWARD = 0,
  BACKWARD = 1,
  BRAKES_ON = 3,
  DISABLED = 4
};

MotorMode Translate(int motorIndex, MotorDirection mode)
{
    if(mode == MotorDirection::BRAKES_ON)
    {
        return MotorMode::BRAKES_ON;
    }
    
    if(mode == MotorDirection::DISABLED)
    {
        return MotorMode::DISABLED;
    }
    
    if(mode == MotorDirection::FORWARD)
    {
        if((motorIndex == 0) || (motorIndex == 3))
        {
            return MotorMode::CCW;
        }
        else
        {
            return MotorMode::CW;
        }
    }
    else if(mode == MotorDirection::BACKWARD)
    {
        if((motorIndex == 0) || (motorIndex == 3))
        {
            return MotorMode::CW;
        }
        else
        {
            return MotorMode::CCW;
        }
    }
    
    return MotorMode::BRAKES_ON;
}

void SetMotorPower(int motorIndex, MotorDirection mode, int power)
{
    if((motorIndex < 0) || (motorIndex >= (int) MotorSettings.size()))
    {
        return; //Invalid motor
    }

    MotorMode motor_mode = Translate(motorIndex, mode);
    
    SetMotorPower(motorIndex, motor_mode, power);
}

/*
//EN for motors
Motor 0: PB9 (TIM4_CH4), PC11 (TIM1_CH1), PE6 (TIM3_CH4)
Motor 1: PD12 (TIM4_CH1), PD13 (TIM4_CH2), PD14 (TIM4_CH3)
Motor 2: PE11 (TIM1_CH2), PE13 (TIM1_CH3), PE14 (TIM1_CH4)
Motor 3: PA6 (TIM3_CH1), PA7 (TIM3_CH2), PB0 (TIM3_CH3)

//IN for motors
Motor 0: PC10, PE4, PE5
Motor 1: PD9, PD10, PD11
Motor 2: PE15, PB10, PB11
Motor 3: PB1, PB2, PF6

//HALL for motors
Motor 0: PF3, PF4, PF5
Motor 1: PC6, PC7, PD8
Motor 2: PB12, PB13, PB14
Motor 3: PF7, PE7, PE8
*/

class MotorsPWMManagerClass
{
    static const int Manager_PWM_FREQUENCY = 30000; //Hertz
    static const int HundredthsOfMicroSecondsPerSecond = 100000000;
    static const int PWM_Period_Number = PWMController::DEFAULT_PWM_TIMER_FREQUENCY/Manager_PWM_FREQUENCY; 

    public:
    MotorsPWMManagerClass() : TimerPWMControllers({{{Timer::TIMER1, PWM_Period_Number}, {Timer::TIMER3, PWM_Period_Number}, {Timer::TIMER4, PWM_Period_Number}}})
    {
    }
    
    void StopMotorPWM(int motorIndex)
    {
        for(const auto& en_timer_index_and_channel : MotorControllerParametersList[motorIndex].ENTimerIndexAndChannelIndex)
        {
            TimerPWMControllers[en_timer_index_and_channel.first].SetDutyCycle(en_timer_index_and_channel.second, (uint32_t) 0);
        }
    }
    
    void ActivateMotorPWM(int motorIndex, int enIndex, int dutyCycle) //Duty cycle range 0-FULL_POWER_SETTING
    {
        const auto& en_timer_index_and_channel = MotorControllerParametersList[motorIndex].ENTimerIndexAndChannelIndex[enIndex];
    
        if(dutyCycle < 0)
        {
            return;
        }
        else if(dutyCycle > FULL_POWER_SETTING)
        {
            dutyCycle = FULL_POWER_SETTING;
        }
    
        int duty_cycle_time = (PWM_Period_Number*dutyCycle)/FULL_POWER_SETTING;
        
        //chprintf((BaseSequentialStream*)&SD1, "ActivateMotorPWM %d %d %d\r\n", motorIndex, (int) enIndex, dutyCycle);
        TimerPWMControllers[en_timer_index_and_channel.first].SetDutyCycle(en_timer_index_and_channel.second, (uint32_t) duty_cycle_time);
    }

    std::array<PWMController, 3> TimerPWMControllers;
    protected:
    
};

MotorsPWMManagerClass* MotorsPWMManager = nullptr;


void SetHalfBridge(int motorIndex, HalfBridgeId bridge, HalfBridgeState state, int dutyCycle)
{
    if((motorIndex < 0) || (motorIndex >= (int) MotorControllerParametersList.size()))
    {
        return;
    }

    if(dutyCycle < 0)
    {
        return;
    }
  
    dutyCycle = dutyCycle > FULL_POWER_SETTING ? FULL_POWER_SETTING : dutyCycle; //Cap magnitudes

    const MotorControllerParameters& parameters = MotorControllerParametersList[motorIndex];
    int bridge_number = (int) bridge;
    const auto& in_pin_info = parameters.INPins[bridge_number];

    //chprintf((BaseSequentialStream*)&SD1, "set half bridge %d %d %d %d\r\n", motorIndex, (int) bridge, (int) state, dutyCycle);
  
    switch(state)
    {
        case HalfBridgeState::HIGH_SIDE:
            palWritePad(in_pin_info.first, in_pin_info.second, 1);
            MotorsPWMManager->ActivateMotorPWM(motorIndex, bridge_number, dutyCycle);
            break;
      
        case HalfBridgeState::LOW_SIDE:
            palWritePad(in_pin_info.first, in_pin_info.second, 0);
            MotorsPWMManager->ActivateMotorPWM(motorIndex, bridge_number, FULL_POWER_SETTING);
            break;
      
        case HalfBridgeState::DISABLED:
            palWritePad(in_pin_info.first, in_pin_info.second, 0);
            MotorsPWMManager->ActivateMotorPWM(motorIndex, bridge_number, 0);
            break;
    }
};

//Which phases need to be active given a particular hall state
std::array<std::array<HalfBridgeState, 3>, 6> HallStateIndexToCWBridgeStates;
std::array<std::array<HalfBridgeState, 3>, 6> HallStateIndexToCCWBridgeStates;
std::array<int, 6> HallStateIndexToCWHighBridgeIndex;
std::array<int, 6> HallStateIndexToCCWHighBridgeIndex;

void UpdateMotorHalfBridges(int motorIndex, MotorMode mode, int dutyCycle, bool hallA, bool hallB, bool hallC)
{
    if(dutyCycle < 0)
    {
        return;
    }

    dutyCycle = dutyCycle > FULL_POWER_SETTING ? FULL_POWER_SETTING : dutyCycle; //Cap magnitudes

    int motor_state_index = HallStatesToValidIndex(hallA, hallB, hallC);
    if(motor_state_index > 5)
    {
        return;
    }

    if((mode == MotorMode::BRAKES_ON) || (motor_state_index < 0))
    {
        //Brakes on or something up with hall input
        //Serial.print("Brakes on\n");
        //MotorsPWMManager->StopMotorPWM(motorIndex);
        SetHalfBridge(motorIndex, HalfBridgeId::U, HalfBridgeState::LOW_SIDE, 0);
        SetHalfBridge(motorIndex, HalfBridgeId::V, HalfBridgeState::LOW_SIDE, 0);
        SetHalfBridge(motorIndex, HalfBridgeId::W, HalfBridgeState::LOW_SIDE, 0);
        return;
    }
  
    if(mode == MotorMode::DISABLED)
    {
        SetHalfBridge(motorIndex, HalfBridgeId::U, HalfBridgeState::DISABLED, 0);
        SetHalfBridge(motorIndex, HalfBridgeId::V, HalfBridgeState::DISABLED, 0);
        SetHalfBridge(motorIndex, HalfBridgeId::W, HalfBridgeState::DISABLED, 0);
        return;
    }

    std::array<HalfBridgeState, 3> target_states;
    if(mode == MotorMode::CW)
    {
        target_states = HallStateIndexToCWBridgeStates[motor_state_index];
    }
    else if(mode == MotorMode::CCW)
    {
        target_states = HallStateIndexToCCWBridgeStates[motor_state_index];
    }
    else
    {
        return;
    }

    //chprintf((BaseSequentialStream*)&SD1, "Target states: %d %d %d\r\n", (int) target_states[0], (int) target_states[1], (int) target_states[2]);

/*
    if(mode == MotorMode::CW)
    {
        MotorsPWMManager->ActivateMotorPWM(motorIndex, HallStateIndexToCWHighBridgeIndex[motor_state_index], dutyCycle);
    }
    else if(mode == MotorMode::CCW)
    {
        MotorsPWMManager->ActivateMotorPWM(motorIndex, HallStateIndexToCCWHighBridgeIndex[motor_state_index], dutyCycle);
    }
    else
    {
        return;
    }
*/

    for(int phase_index = 0; phase_index < (int) target_states.size(); phase_index++)
    {
        SetHalfBridge(motorIndex, (HalfBridgeId) phase_index, target_states[phase_index], dutyCycle);
    }
    
}

void SetupHallStateToPhaseStateMapping()
{
    HallStateIndexToCWBridgeStates[0][0] = HalfBridgeState::DISABLED; //HalfBridgeState::DISABLED;
    HallStateIndexToCWBridgeStates[0][1] = HalfBridgeState::LOW_SIDE;
    HallStateIndexToCWBridgeStates[0][2] = HalfBridgeState::HIGH_SIDE;

    HallStateIndexToCWBridgeStates[1][0] = HalfBridgeState::HIGH_SIDE;
    HallStateIndexToCWBridgeStates[1][1] = HalfBridgeState::LOW_SIDE;
    HallStateIndexToCWBridgeStates[1][2] = HalfBridgeState::DISABLED;

    HallStateIndexToCWBridgeStates[2][0] = HalfBridgeState::HIGH_SIDE;
    HallStateIndexToCWBridgeStates[2][1] = HalfBridgeState::DISABLED;
    HallStateIndexToCWBridgeStates[2][2] = HalfBridgeState::LOW_SIDE;

    HallStateIndexToCWBridgeStates[3][0] = HalfBridgeState::DISABLED;
    HallStateIndexToCWBridgeStates[3][1] = HalfBridgeState::HIGH_SIDE;
    HallStateIndexToCWBridgeStates[3][2] = HalfBridgeState::LOW_SIDE;

    HallStateIndexToCWBridgeStates[4][0] = HalfBridgeState::LOW_SIDE;
    HallStateIndexToCWBridgeStates[4][1] = HalfBridgeState::HIGH_SIDE;
    HallStateIndexToCWBridgeStates[4][2] = HalfBridgeState::DISABLED;

    HallStateIndexToCWBridgeStates[5][0] = HalfBridgeState::LOW_SIDE;
    HallStateIndexToCWBridgeStates[5][1] = HalfBridgeState::DISABLED;
    HallStateIndexToCWBridgeStates[5][2] = HalfBridgeState::HIGH_SIDE;

    HallStateIndexToCWHighBridgeIndex = {2, 0, 0, 1, 1, 2};
    HallStateIndexToCCWHighBridgeIndex = {1, 1, 2, 2, 0, 0};

    for(int hall_index = 0; hall_index < (int) HallStateIndexToCWBridgeStates.size(); hall_index++)
    {
        for(int phase_index = 0; phase_index < (int) HallStateIndexToCWBridgeStates[hall_index].size(); phase_index++)
        {
            HallStateIndexToCCWBridgeStates[hall_index][phase_index] = HallStateIndexToCWBridgeStates[hall_index][phase_index];
            if(HallStateIndexToCWBridgeStates[hall_index][phase_index] == HalfBridgeState::HIGH_SIDE)
            {
                HallStateIndexToCCWBridgeStates[hall_index][phase_index] = HalfBridgeState::LOW_SIDE;
            }
            else if(HallStateIndexToCWBridgeStates[hall_index][phase_index] == HalfBridgeState::LOW_SIDE)
            {
                HallStateIndexToCCWBridgeStates[hall_index][phase_index] = HalfBridgeState::HIGH_SIDE;
            }
        }
    }
}

void SetupMotorControllerPins()
{ 
/**
//EN for motors
Motor 0: PB9 (TIM4_CH4, 9), PC11 (TIM1_CH1 <- wrong.  Change to PC8, 2), PE6 (TIM3_CH4, 1)
Motor 1: PD12 (TIM4_CH1, 2), PD13 (TIM4_CH2, 2), PD14 (TIM4_CH3, 2)
Motor 2: PE11 (TIM1_CH2, 1), PE13 (TIM1_CH3, 1), PE14 (TIM1_CH4, 1)
Motor 3: PA6 (TIM3_CH1, 1), PA7 (TIM3_CH2, 1), PB0 (TIM3_CH3, 1)
*/
    SetupHallStateToPhaseStateMapping();

    MotorControllerParametersList[0].INPins = {{{GPIOC, 10}, {GPIOE, 4}, {GPIOE, 5}}}; 
    MotorControllerParametersList[1].INPins = {{{GPIOD, 9}, {GPIOD, 10}, {GPIOD, 11}}}; 
    MotorControllerParametersList[2].INPins = {{{GPIOE, 15}, {GPIOB,10}, {GPIOB, 11}}}; 
    MotorControllerParametersList[3].INPins = {{{GPIOB, 1}, {GPIOB, 2}, {GPIOF, 6}}};  

    MotorControllerParametersList[0].ENPins = {{{GPIOB, 9, 9}, {GPIOC, 8, 2}, {GPIOE, 6, 1}}};
    MotorControllerParametersList[1].ENPins = {{{GPIOD, 12, 2}, {GPIOD, 13, 2}, {GPIOD, 14, 2}}};
    MotorControllerParametersList[2].ENPins = {{{GPIOE, 11, 1}, {GPIOE, 13, 1}, {GPIOE, 14, 1}}};
    MotorControllerParametersList[3].ENPins = {{{GPIOA, 6, 1}, {GPIOA, 7, 1}, {GPIOB, 0, 1}}};

    MotorControllerParametersList[0].ENTimerIndexAndChannelIndex = {{{2, 4-1}, {0, 1-1}, {1, 4-1}}};
    MotorControllerParametersList[1].ENTimerIndexAndChannelIndex = {{{2, 1-1}, {2, 2-1}, {2, 3-1}}};
    MotorControllerParametersList[2].ENTimerIndexAndChannelIndex = {{{0, 2-1}, {0, 3-1}, {0, 4-1}}};
    MotorControllerParametersList[3].ENTimerIndexAndChannelIndex = {{{1, 1-1}, {1, 2-1}, {1, 3-1}}};

    for(int motorIndex = 0; motorIndex < (int) MotorControllerParametersList.size(); motorIndex++)
    {       
        for(const auto& pin_info : MotorControllerParametersList[motorIndex].INPins)
        {
            palSetPadMode(pin_info.first, pin_info.second, PAL_MODE_OUTPUT_PUSHPULL);
            palWritePad(pin_info.first, pin_info.second, 0);
        }

        for(const auto& pin_info : MotorControllerParametersList[motorIndex].ENPins)
        {
            palSetPadMode(std::get<0>(pin_info), std::get<1>(pin_info), PAL_MODE_ALTERNATE(std::get<2>(pin_info)));
        }
  }
}








}
