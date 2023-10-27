#pragma once

#include "ch.h"
#include "hal.h"

#include<array>

/*
Observed hall sequence:
ccw:
101 -> 5 -> 4
100 -> 4 -> 3
110 -> 6 -> 5
010 -> 2 -> 1
011 -> 3 -> 2
001 -> 1 -> 0

90 state changes per revolution = 15 state number revolutions per real revolution

Motor:
V, hall A -> green wires
U, hall B -> blue wires
W, hall C -> yellow wires

//HALL for motors
Motor 0: PF3, PF4, PF5
Motor 1: PC6, PC7, PD8
Motor 2: PB12, PB13, PB14
Motor 3: PF7, PE7, PE8

*/
//Let's make a sequential state system so that it's easier to tell which way things are going.  001 will be 0.

//Returns negative if in invalid state
//Valid state sequence 001 (0) -> 011 (1) -> 010 (2) -> 110 (3) -> 100 (4) -> 101 (5)
const int NumberOfStateCyclesPerRevolution = 15;
const int NumberOfPulsesPerStateCycle = 6;
const int NumberOfPulsesPerRevolution = NumberOfStateCyclesPerRevolution*NumberOfPulsesPerStateCycle;

const int SecondsPerMinute = 60;
const int MicroSecondsPerSecond = 1000000;
const int MicroSecondsPerMinute = SecondsPerMinute*MicroSecondsPerSecond;
const int MilliPerWhole = 1000;

std::array<int, 6> BasicHallIndexToValidStateIndexCW = {0, 2, 1, 4, 5, 3}; 
int HallStatesToValidIndex(bool A, bool B, bool C)
{ 
  if((A && B && C) || ((!A) && (!B) && (!C)))
  {
    return -1;
  }
  
  int basic_index = A*4+B*2+C*1-1;
  
  return BasicHallIndexToValidStateIndexCW[basic_index];
}

struct MotorStateChangeEvent
{
  systime_t EventTime = 0;
  int ValidStateIndexCW = 0;
};

const std::array<std::array<std::pair<stm32_gpio_t*, int>, 3>, 4> Hall_Pins = {{
{{{GPIOF, 3}, {GPIOF, 4}, {GPIOF, 5}}},
 {{{GPIOC, 6}, {GPIOC, 7}, {GPIOD, 8}}}, 
 {{{GPIOB, 12}, {GPIOB, 13}, {GPIOB, 14}}}, 
 {{{GPIOF, 7}, {GPIOE, 7}, {GPIOE, 8}}}
}};
std::array<std::array<bool, 3>, 4> Last_Hall_Pin_States = {{{{false, false, false}}, {{false, false, false}}, {{false, false, false}}, {{false, false, false}}}};
std::array<std::array<bool, 3>, 4> Hall_Pin_States = {{{{false, false, false}}, {{false, false, false}}, {{false, false, false}}, {{false, false, false}}}};
std::array<systime_t, 4> LastUpdateTime = {{0, 0, 0, 0}};
std::array<systime_t, 4> LastChangeTime = {{0, 0, 0, 0}};
std::array<std::array<MotorStateChangeEvent, 2>, 4> MotorChangeHistory; //Most recent first
std::array<int32_t, 4> LastMotorVelocityEventEstimateMilliRPM = {{0,0,0,0}};
std::array<int32_t, 4> MotorVelocityEventEstimateMilliRPM = {{0,0,0,0}};
std::array<int32_t, 4> MotorVelocityEstimateMilliRPM = {{0,0,0,0}};

void UpdateHallStates()
{
    Last_Hall_Pin_States = Hall_Pin_States;

    for(int motor_index = 0; motor_index < (int) Hall_Pins.size(); motor_index++)
    {
        const auto& motor_pins = Hall_Pins[motor_index];
        for(int pin_index = 0; pin_index < (int) Hall_Pins.size(); pin_index++)
        {
            const auto& pin_info = motor_pins[pin_index];
        
            Hall_Pin_States[motor_index][pin_index] = palReadPad(pin_info.first, pin_info.second);
        }
        
        auto time = chVTGetSystemTime();
        LastUpdateTime[motor_index] = time;
        
        if(Hall_Pin_States[motor_index] != Last_Hall_Pin_States[motor_index])
        {
            //Updating motor change history, so move the old entry back one
            MotorChangeHistory[motor_index][1] = MotorChangeHistory[motor_index][0];
            MotorChangeHistory[motor_index][0].ValidStateIndexCW = HallStatesToValidIndex(Hall_Pin_States[motor_index][0], Hall_Pin_States[motor_index][1], Hall_Pin_States[motor_index][2]);
            MotorChangeHistory[motor_index][0].EventTime = time;
            
            LastChangeTime[motor_index] = time;
        }
    }
}


void InitializeMotorStateEstimation()
{
    for(const auto& motor_pins : Hall_Pins)
    {
        for(const std::pair<stm32_gpio_t*, int>& pin_info : motor_pins)
        {
            palSetPadMode(pin_info.first, pin_info.second,  PAL_MODE_INPUT_PULLDOWN);
        }
    }
    
    UpdateHallStates();
    for(int motor_index = 0; motor_index < (int) Hall_Pins.size(); motor_index++)
    {
        MotorChangeHistory[motor_index][1] = MotorChangeHistory[motor_index][0]; //Set both histories to the same since not moving yet
    }
}

int GetModuloDistance(int left, int right, int moduloNumber)
{
  if(left >= right)
  {
    return left - right;
  }
  
  int distance_to_modulo = moduloNumber - right;
  return left + distance_to_modulo;
}



int GetMotorVelocityMilliRPM(const struct MotorStateChangeEvent& lastEvent, const struct MotorStateChangeEvent& currentEvent)
{
  if(currentEvent.ValidStateIndexCW == lastEvent.ValidStateIndexCW)
  {
    //Same state, not moving
    return 0;
  }
  
  int elapsed_time = chTimeI2US(chTimeDiffX(lastEvent.EventTime, currentEvent.EventTime));

  int cw_difference = GetModuloDistance(currentEvent.ValidStateIndexCW, lastEvent.ValidStateIndexCW, NumberOfPulsesPerStateCycle);
  int ccw_difference = GetModuloDistance(lastEvent.ValidStateIndexCW, currentEvent.ValidStateIndexCW, NumberOfPulsesPerStateCycle); 

  int velocity = 0;
  if(cw_difference <= ccw_difference)
  {
    velocity = (cw_difference*((long long int) SecondsPerMinute)*MicroSecondsPerSecond*MilliPerWhole)/(elapsed_time*((long long int )NumberOfPulsesPerRevolution));
  }
  else
  {
    velocity = (-ccw_difference*((long long int) SecondsPerMinute)*MicroSecondsPerSecond*MilliPerWhole)/(elapsed_time*((long long int )NumberOfPulsesPerRevolution));
  }
  
  return velocity;
}

int32_t abs(int32_t value)
{
    if(value >= 0)
    {
        return value;
    }
    
    return -value;
}

void UpdateVelocityEstimates()
{
    for(int motor_index = 0; motor_index < (int) Hall_Pins.size(); motor_index++)
    {
        int event_based_estimate = GetMotorVelocityMilliRPM(MotorChangeHistory[motor_index][1], MotorChangeHistory[motor_index][0]);
        
        if(MotorVelocityEventEstimateMilliRPM[motor_index] != event_based_estimate)
        {
            //An event happened, so update all estimates based on actual data we just got
            LastMotorVelocityEventEstimateMilliRPM[motor_index] = MotorVelocityEventEstimateMilliRPM[motor_index];
            MotorVelocityEventEstimateMilliRPM[motor_index] = event_based_estimate;
            MotorVelocityEstimateMilliRPM[motor_index] = event_based_estimate;
        }
        else if(event_based_estimate != 0)
        {
            //Can't interpolate based on time if the velocity is zero, so only try if not that case
            int32_t time_since_last_event_us = chTimeI2US(chTimeDiffX(MotorChangeHistory[motor_index][0].EventTime, chVTGetSystemTime()));
            int32_t number_of_pulse_per_min_expected = NumberOfPulsesPerRevolution*MotorVelocityEventEstimateMilliRPM[motor_index]*MilliPerWhole;
            int32_t us_per_pulse_expected = MicroSecondsPerMinute / number_of_pulse_per_min_expected;
            
            if(time_since_last_event_us > us_per_pulse_expected)
            {
                //We are going slower than expected, so update the estimate accordingly
                int32_t sign = -1;
                if(event_based_estimate > 0)
                {
                    sign = 1;
                }
                
                int32_t time_based_estimate = (sign*((long long int) SecondsPerMinute)*MicroSecondsPerSecond*MilliPerWhole)/(time_since_last_event_us*((long long int )NumberOfPulsesPerRevolution));
                 
                 if(abs(time_based_estimate) < abs(event_based_estimate))
                 {
                    MotorVelocityEstimateMilliRPM[motor_index] = time_based_estimate;
                 }
            }
        }
    }
}




