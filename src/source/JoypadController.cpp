#include <include/JoypadController.hpp>
#include <QDebug>
#include <chrono>

/**
 * Reads a joystick event from the joystick device.
 *
 * Returns 0 on success. Otherwise -1 is returned.
 */
int read_event(int fd, struct js_event *event)
{
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    /* Error, could not read full event. */
    return -1;
}

/**
 * Returns the number of axes on the controller or 0 if an error occurs.
 */
size_t get_axis_count(int fd)
{
    __u8 axes;

    if (ioctl(fd, JSIOCGAXES, &axes) == -1)
        return 0;

    return axes;
}

/**
 * Returns the number of buttons on the controller or 0 if an error occurs.
 */
size_t get_button_count(int fd)
{
    __u8 buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
        return 0;

    return buttons;
}

/**
 * Current state of an axis.
 */
struct axis_state
{
    short x, y;
};

/**
 * Keeps track of the current axis state.
 *
 * NOTE: This function assumes that axes are numbered starting from 0, and that
 * the X axis is an even number, and the Y axis is an odd number. However, this
 * is usually a safe assumption.
 *
 * Returns the axis that the event indicated.
 */
size_t get_axis_state(struct js_event *event, struct axis_state axes[3])
{
    size_t axis = event->number / 2;

    if (axis < 3)
    {
        if (event->number % 2 == 0)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }

    return axis;
}

JoypadController::JoypadController() {
    _current_angle.value = 0;
    _current_velocity.value = 0;

    _current_angle.timestamp = 0;
    _current_velocity.timestamp = 0;

    qRegisterMetaType<BUTTON>("JoypadButton");
    qRegisterMetaType<TimestampedDouble>("TimestampedDouble");

    qDebug() << "Joyad initialized.";
}

JoypadController::~JoypadController()
{

}

int JoypadController::start()
{
    _is_started = true;
    emit JoypadCommandAvailable(TimestampedDouble(), TimestampedDouble());
    _joypad_connection_loop_thread = std::thread(&JoypadController::joypad_connection_loop, this);
    return 0;
}

int JoypadController::stop()
{
    _is_started = false;
    if (_joypad_connection_loop_thread.joinable())
    {
        _joypad_connection_loop_thread.join();
    }
    if (_joypad_thread.joinable())
    {
        _joypad_thread.join();
    }
    return 0;
}

int JoypadController::joypad_connection_loop()
{
    _joypad_thread = std::thread(&JoypadController::retrieve_command_loop, this);
    while (_is_started)
    {
        if (_has_exited)
        {
            qDebug() << "Trying to restart the joypad thread...";
            emit JoypadCommandAvailable(TimestampedDouble(), TimestampedDouble());
            if (_joypad_thread.joinable())
            {
                _joypad_thread.join();
            }
            _joypad_thread = std::thread(&JoypadController::retrieve_command_loop, this);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
    return 0;
}

int JoypadController::retrieve_command_loop()
{
    _has_exited = false;
    const char *device;
    int js;
    struct js_event event;
    struct axis_state axes[3] = {0};
    size_t axis;

    device = "/dev/input/js0";

    if (access(device, F_OK) == -1)
    {
        _has_exited = true;
        return 1;
    }

    js = open(device, O_RDONLY);

    if (js == -1)
    {
        perror("Could not open joystick");
        _has_exited = true;
        return 1;
    }
    /* This loop will exit if the controller is unplugged. */
    while (read_event(js, &event) == 0)
    {
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        _current_velocity.value = 0;
        _current_angle.value = 0;

        switch (event.type)
        {
        case JS_EVENT_BUTTON:
            // printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
            // if (event.number == 1 && event.value) // B
            // {

            // }
            // if (event.number == 1 && event.value) // X
            // {

            // }
            if (event.number == 0 && event.value) // A
            {
                if (!_is_manual_mode)
                {
                    std::cout << "Manual mode enabled" << std::endl;
                    changeManualMode(true);
                }
            }
            if (event.number == 3 && event.value) // Y
            {
                if (_is_manual_mode)
                {
                    std::cout << "Manual mode disabled" << std::endl;
                    changeManualMode(false);
                }
            }
            if (event.number == 5) // RB
            {
                if (event.value)
                {
                    std::cout << "Ignoring obstacles..." << std::endl;
                    _ignore_obstacles = true;
                }
                else
                {
                    std::cout << "Stop ignoring obstacles." << std::endl;
                    _ignore_obstacles = false;
                }
            }

            break;

        case JS_EVENT_AXIS:
            axis = get_axis_state(&event, axes);
            if (axis < 3)
            {
                _command_mutex.lock();
                // printf("Axis %zu at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);
                if (axis == 0) // left stick
                {
                    if (abs(axes[axis].y) >= joystick_sensitivity_threshold)
                    {
                        // printf("Axis %zu at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);
                        // if(axes[axis].y < 0)
                        //     std::cout << "move forward" << std::endl;
                        // else
                        //     std::cout << "move backward" << std::endl;
                        _current_velocity.value = axes[axis].y;
                        _current_velocity.timestamp = timestamp;
                    }
                    else
                        _current_velocity.value = 0;
                }
                if (axis == 1) // right stick
                {
                    if (abs(axes[axis].y) >= joystick_sensitivity_threshold)
                    {
                        // printf("Axis %zu at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);
                        // if(axes[axis].y < 0)
                        //     std::cout << "move left" << std::endl;
                        // else
                        //     std::cout << "move right" << std::endl;
                        _current_angle.value = axes[axis].y;
                        _current_angle.timestamp = timestamp;
                    }
                    else
                        _current_angle.value = 0;
                }
                _command_mutex.unlock();
            }
            break;
        default:
            /* Ignore init events. */
            break;
        }

        fflush(stdout);
        emit JoypadCommandAvailable(_current_velocity, _current_angle);
    }

    {
        std::lock_guard<std::mutex> lock(_command_mutex);
        _current_velocity.value = 0;
        _current_angle.value = 0;
        changeManualMode(false);
    }

    close(js);
    _has_exited = true;
    return 0;
}

void JoypadController::sendLinearAndAngularVelocityToBridge(const float v, const float r)
{
#ifdef ENABLE_NAV2BRIDGE
    _perception_controller->getBridge()->sendVelocityCmd(v, r);
#endif
}

float JoypadController::scaleIntToFloat(int value, int minInt, int maxInt, float minFloat, float maxFloat)
{
    float floatRange = maxFloat - minFloat;
    int intRange = maxInt - minInt;
    float scale = floatRange / intRange;
    int offset = minInt;
    return ((value - offset) * scale) + minFloat;
}

float JoypadController::inputToVelocity(int value)
{
    return -1 * scaleIntToFloat(value, _inputVelMin, _inputVelMax, _velMin, _velMax);
}

float JoypadController::inputToAngularSpeed(int value)
{
    return -1 * scaleIntToFloat(value, _inputAngularVelMin, _inputAngularVelMax, _angularVelMin, _angularVelMax);
}

void JoypadController::changeManualMode(bool mode)
{

    _is_manual_mode = mode;
    // disable the stop and go when using the joypad, so that we can still override it with `_ignore_obstacles`
    qDebug() << "Switched manual mode : " << mode;
}

TimestampedDouble JoypadController::GetCurrentLinerarVelocity() {
    return _current_velocity;
}

TimestampedDouble JoypadController::GetCurrentAngularVelocity() {
    return _current_angle;
}

