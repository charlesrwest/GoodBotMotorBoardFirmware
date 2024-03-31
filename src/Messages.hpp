#pragma once

namespace GoodBot
{

/**
Planned functionality:
Multiple commands can be sent in a single transmit

General: Transmit commands via with format:
msgpack_int message type
msgpack_int message version
fields

Message type -1 reserved for invalid

Message type 0 (Set PWM, frequency set to 60 Hz):
msgpack_int 0
msgpack_int 0
msgpack_int pwmChannel (0 for steering, 1 for speed/acceleration)
msgpack_int onTimeInMicroseconds

Message type 1 (Reporting ranges):
msgpack_int 1
msgpack_int 0
msgpack_int_array ranges (negative if invalid, with an entry for each ranger the MCU is tracking, units in mm)

Message type 2 (Reporting battery voltage):
msgpack_int 2
msgpack_int 0
msgpack_float estimated_battery_voltage

Message type 3 (Report Calibrated Acceleration):
msgpack_int 3
msgpack_int 0
msgpack_float_array calibrated_acceleration

Message type 4 (Report Linear Acceleration):
msgpack_int 4
msgpack_int 0
msgpack_float_array acceleration

Message type 5 (Report Gravity Acceleration):
msgpack_int 5
msgpack_int 0
msgpack_float_array gravity_acceleration

Message type 6 (Report angular velocity):
msgpack_int 6
msgpack_int 0
msgpack_float_array angular_velocity

Message type 7 (Report magnetic_field_calibrated):
msgpack_int 7
msgpack_int 0
msgpack_float_array magnetic_field_calibrated

Message type 8 (Report Orientation):
msgpack_int 8
msgpack_int 0
msgpack_float_array orientationAndAccuracy //I, J, K, Real, Accuracy

Message type 9 (Report Error, used for debugging):
msgpack_int 9
msgpack_int 0
msgpack_int errorSubtype
msgpack_v4_str errorString
msgpack_int_array integerInfo
msgpack_float_array floatInfo

Message type 10 (GPS NMEA string portion)
msgpack_int 10
msgpack_int 0
msgpack_bin NMEA string portion

Message type 11 (Speed estimated from hall effect sensor on motor)
msgpack_int 11
msgpack_int 0
msgpack_double Speed in m/s

Message type 12 (Target velocity to try to achieve)
msgpack_int 12
msgpack_int 0
msgpack_double Velocity in m/s (positive forward)

Message type 13 (motor rotation velocity estimate)
msgpack_int 13
msgpack_int 0
msgpack_int motor_id
msgpack_int motor_velocity_milli_rpm (forward+)

Message type 14 (motor rotation velocity target)
msgpack_int 14
msgpack_int 0
msgpack_int left_target_motor_velocity_milli_rpm (forward+)
msgpack_int right_target_motor_velocity_milli_rpm (forward+)
msgpack_int brakes_on (1 if true, 0 otherwise)
*/

struct Vector3D
{
    double X = 0.0;
    double Y = 0.0;
    double Z = 0.0;
};

struct Quat
{
    double X = 0.0;
    double Y = 0.0;
    double Z = 0.0;
    double W = 0.0;
};

enum class MessageType : int8_t
{
INVALID = -1,
SET_PWM = 0,
REPORT_RANGES = 1,
REPORT_BATTERY_VOLTAGE = 2,
REPORT_CALIBRATED_ACCELERATION = 3,
REPORT_LINEAR_ACCELERATION = 4, 
REPORT_GRAVITY_ACCELERATION = 5, 
REPORT_ANGULAR_VELOCITY = 6, 
REPORT_MAGNETIC_FIELD_CALIBRATED = 7,
REPORT_ORIENTATION = 8,
REPORT_ERROR = 9,
REPORT_NMEA_STRING_PART = 10,
REPORT_ESTIMATED_SPEED = 11,
SET_TARGET_VELOCITY = 12,
SET_MOTOR_ROTATION_VELOCITY_TARGET = 14
};

struct SetPWMMessage
{
    MessageType Type;
    int64_t Version;
    int64_t Channel; //0 for steering, 1 for speed (speed not directly set this way anymore)
    int64_t OnTime;
};

struct SetTargetVelocityMessage
{
    MessageType Type;
    int64_t Version;
    double Velocity; //m/s
};

struct SetMotorRotationVelocityTarget
{
    MessageType Type;
    int64_t Version;
    int64_t TargetMotorVelocityMilliRPMLeft;
    int64_t TargetMotorVelocityMilliRPMRight;
    int64_t BrakesOn;
};









}
