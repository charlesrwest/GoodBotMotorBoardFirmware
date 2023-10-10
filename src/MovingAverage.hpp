#pragma once
#include<array>

namespace GoodBot
{

/**
The purpose of this class is to make a reasonably efficient way to calculate moving averages with an arbitrary size window.
*/
template<class type, int32_t window_size>
class MovingAverage
{
public:
/**
Create zeroed out moving average window.
*/
MovingAverage();

/**
Create window initialized to a particular value.
@param inputInitialValue: The value to set the window to initially
*/
MovingAverage(type inputInitialValue);

/**
Place a new value into the circular window buffer.
@param inputNewValue: One of the values to consider for the moving average
*/
void AddValue(type inputNewValue);

/**
Returns sum of window/size of window.
@return: The current value of the moving average.
*/
type GetWindowAverage() const;

/**
Returns the size of the window.
@return: The size of the window
*/
int64_t size() const;

/**
Retrieve value from the circular buffer offset by a certain amount from the current position.
@param inputOffset: How much to offset.
@return: The value at that location
*/
type GetWindowValue(int32_t inputOffset) const;

/**
Gets the sum of the window values.
@return: The window sum
*/
type GetSum() const;

private:
int32_t arrayPosition; //Current position to set values at
type sum;
std::array<type, window_size> values;
};
 
/**
Create zeroed out moving average window.
*/
template<class type, int32_t window_size>
MovingAverage<type, window_size>::MovingAverage() : arrayPosition(0), sum(0)
{
values.fill(0);
}

/**
Create window initialized to a particular value.
@param inputInitialValue: The value to set the window to initially
*/
template<class type, int32_t window_size>
MovingAverage<type, window_size>::MovingAverage(type inputInitialValue) : sum(inputInitialValue*values.size()), arrayPosition(0)
{
values.fill(inputInitialValue);
}

/**
Place a new value into the circular window buffer.
@param inputNewValue: One of the values to consider for the moving average
*/
template<class type, int32_t window_size> 
void MovingAverage<type, window_size>::AddValue(type inputNewValue)
{
sum = sum - values[arrayPosition] + inputNewValue;
values[arrayPosition] = inputNewValue;
arrayPosition = (arrayPosition+1) % values.size();
}

/**
Returns sum of window/size of window.
@return: The current value of the moving average.
*/
template<class type, int32_t window_size> 
type MovingAverage<type, window_size>::GetWindowAverage() const
{
return sum / values.size();
}

/**
Returns the size of the window.
@return: The size of the window
*/
template<class type, int32_t window_size> 
int64_t MovingAverage<type, window_size>::size() const
{
return values.size();
}

/**
Retrieve value from the circular buffer offset by a certain amount from the current position.
@param inputOffset: How much to offset.
@return: The value at that location
*/
template<class type, int32_t window_size> 
type MovingAverage<type, window_size>::GetWindowValue(int32_t inputRelativeOffset) const
{
int32_t absoluteOffset = (arrayPosition + inputRelativeOffset) % size();
return values[absoluteOffset >= 0 ? absoluteOffset : (absoluteOffset + (int32_t) size())];
}

/**
Gets the sum of the window values.
@return: The window sum
*/
template<class type, int32_t window_size> 
type MovingAverage<type, window_size>::GetSum() const 
{
return sum;
}

}
