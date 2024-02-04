#pragma once
#include <iostream>
// https://www.waveshare.com/wiki/WAVE_ROVER

enum class WAVE_ROVER_COMMAND_TYPE {
    BUS_SERVO_MID=-5,
    PWM_SERVO_MID=-4,
    OLED_DEFAULT=-3,
    EMERGENCY_STOP=0,
    SPEED_INPUT=1,
    PID_SET=2,
    OLED_SET=3,
    PWM_SERVO_CONTROL=40,
    BUS_SERVO_CONTROL=50,
    BUS_SERVO_INFO=53,
    BUS_SERVO_ID_SET=54,
    BUS_SERVO_TORQUE_LOCK=55,
    BUS_SERVO_TORQUE_LIMIT=56,
    BUS_SERVO_MODE=57,
    WIFI_SCAN=60,
    WIFI_TRY_STA=61,
    WIFI_AP_DEFAULT=62,
    WIFI_INFO=65,
    WIFI_OFF=66,
    INA219_INFO=70,
    IMU_INFO=71,
    ENCODER_INFO=73,
    DEVICE_INFO=74,
    IO_IR_CUT=80,
    SET_SPD_RATE=901,
    GET_SPD_RATE=902,
    SPD_RATE_SAVE=903,
    GET_NVS_SPACE=904,
    NVS_CLEAR=905
};

enum class INFO_TYPE {
    DEVICE = 0,
    IMU,
    WIFI,
    INA219
};

inline std::ostream& operator<<(std::ostream& os, WAVE_ROVER_COMMAND_TYPE command) {
    switch (command) {
    case WAVE_ROVER_COMMAND_TYPE::BUS_SERVO_MID:
        return os << "BUS_SERVO_MID";
    case WAVE_ROVER_COMMAND_TYPE::PWM_SERVO_MID:
        return os << "PWM_SERVO_MID";
    case WAVE_ROVER_COMMAND_TYPE::OLED_DEFAULT:
        return os << "OLED_DEFAULT";
    case WAVE_ROVER_COMMAND_TYPE::EMERGENCY_STOP:
        return os << "EMERGENCY_STOP";
    case WAVE_ROVER_COMMAND_TYPE::SPEED_INPUT:
        return os << "SPEED_INPUT";
    case WAVE_ROVER_COMMAND_TYPE::PID_SET:
        return os << "PID_SET";
    case WAVE_ROVER_COMMAND_TYPE::OLED_SET:
        return os << "OLED_SET";
    case WAVE_ROVER_COMMAND_TYPE::PWM_SERVO_CONTROL:
        return os << "PWM_SERVO_CONTROL";
    case WAVE_ROVER_COMMAND_TYPE::BUS_SERVO_CONTROL:
        return os << "BUS_SERVO_CONTROL";
    case WAVE_ROVER_COMMAND_TYPE::BUS_SERVO_INFO:
        return os << "BUS_SERVO_INFO";
    case WAVE_ROVER_COMMAND_TYPE::BUS_SERVO_ID_SET:
        return os << "BUS_SERVO_ID_SET";
    case WAVE_ROVER_COMMAND_TYPE::BUS_SERVO_TORQUE_LOCK:
        return os << "BUS_SERVO_TORQUE_LOCK";
    case WAVE_ROVER_COMMAND_TYPE::BUS_SERVO_TORQUE_LIMIT:
        return os << "BUS_SERVO_TORQUE_LIMIT";
    case WAVE_ROVER_COMMAND_TYPE::BUS_SERVO_MODE:
        return os << "BUS_SERVO_MODE";
    case WAVE_ROVER_COMMAND_TYPE::WIFI_SCAN:
        return os << "WIFI_SCAN";
    case WAVE_ROVER_COMMAND_TYPE::WIFI_TRY_STA:
        return os << "WIFI_TRY_STA";
    case WAVE_ROVER_COMMAND_TYPE::WIFI_AP_DEFAULT:
        return os << "WIFI_AP_DEFAULT";
    case WAVE_ROVER_COMMAND_TYPE::WIFI_OFF:
        return os << "WIFI_OFF";
    case WAVE_ROVER_COMMAND_TYPE::WIFI_INFO:
        return os << "WIFI_INFO";
    case WAVE_ROVER_COMMAND_TYPE::INA219_INFO:
        return os << "INA219_INFO";
    case WAVE_ROVER_COMMAND_TYPE::IMU_INFO:
        return os << "IMU_INFO";
    case WAVE_ROVER_COMMAND_TYPE::ENCODER_INFO:
        return os << "ENCODER_INFO";
    case WAVE_ROVER_COMMAND_TYPE::DEVICE_INFO:
        return os << "DEVICE_INFO";
    case WAVE_ROVER_COMMAND_TYPE::IO_IR_CUT:
        return os << "IO_IR_CUT";
    case WAVE_ROVER_COMMAND_TYPE::SET_SPD_RATE:
        return os << "SET_SPD_RATE";
    case WAVE_ROVER_COMMAND_TYPE::GET_SPD_RATE:
        return os << "GET_SPD_RATE";
    case WAVE_ROVER_COMMAND_TYPE::SPD_RATE_SAVE:
        return os << "SPD_RATE_SAVE";
    case WAVE_ROVER_COMMAND_TYPE::GET_NVS_SPACE:
        return os << "GET_NVS_SPACE";
    case WAVE_ROVER_COMMAND_TYPE::NVS_CLEAR:
        return os << "NVS_CLEAR";
    }
}
