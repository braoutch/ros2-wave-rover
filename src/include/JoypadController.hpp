#pragma once

#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <linux/joystick.h>
#include <stdio.h>
#include <thread>
#include <unistd.h>
#include <mutex>
#include <QObject>

// this class handles betaflight virtual HID as a joypad, because I had one for spare and did not want to buy anything else.

enum STATE {
    MANUAL_MODE_ENABLED,
    MANUAL_MODE_DISABLED,
    IGNORE_OBSTACLES_ENABLED,
    IGNORE_OBSTACLES_DISABLED,
};

enum BUTTON {

};

struct TimestampedDouble {
    double value = 0;
    unsigned long long timestamp;
};

Q_DECLARE_METATYPE(BUTTON)
Q_DECLARE_METATYPE(TimestampedDouble);

class JoypadController : public QObject {
    Q_OBJECT
public:
    JoypadController();
    ~JoypadController();

    int start();
    int stop();
    int joypad_connection_loop();
    int retrieve_command_loop();

    TimestampedDouble GetCurrentLinerarVelocity();
    TimestampedDouble GetCurrentAngularVelocity();

signals:
    void JoypadCommandAvailable(TimestampedDouble, TimestampedDouble);
    void JoypadCommandAvailable(BUTTON);

private:
    static float scaleIntToFloat(int value, int minInt, int maxInt, float minFloat, float maxFloat);
    static float inputToVelocity(int value);
    static float inputToAngularSpeed(int value);
    void changeManualMode(bool mode);

    void sendLinearAndAngularVelocityToBridge(const float v, const float r);

    bool _verbose = false;
    bool _is_started = false;
    bool _has_exited = false;
    std::thread _joypad_thread;
    std::thread _joypad_connection_loop_thread;

    bool _is_manual_mode = false;
    bool _ignore_obstacles = false;

    std::mutex _command_mutex;
    TimestampedDouble _current_velocity;
    TimestampedDouble _current_angle;

    const int joystick_sensitivity_threshold = 7000;

    static constexpr int _inputVelMin = -32767;
    static constexpr int _inputVelMax = 32767;
    static constexpr float _velMin = -1;
    static constexpr float _velMax = 1;
    static constexpr int _inputAngularVelMin = -32767;
    static constexpr int _inputAngularVelMax = 32767;
    static constexpr float _angularVelMin = -1.5;
    static constexpr float _angularVelMax = 1.5;
};
