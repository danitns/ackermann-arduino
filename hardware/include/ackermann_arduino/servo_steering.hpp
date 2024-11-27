#ifndef ACKERMANN_ARDUINO_STEERING_HPP
#define ACKERMANN_ARDUINO_STEERING_HPP

#include <string>
#include <math.h>

class ServoSteering
{
private:
    double minControlValue = -0.5;
    double maxControlValue = 0.5;
    double minArduinoValue = 45;
    double maxArduinoValue = 135;
    double controlToArduinoSlope = (maxArduinoValue - minArduinoValue) / (maxControlValue - minControlValue);
    double arduinoToControlSlope = (maxControlValue - minControlValue) / (maxArduinoValue - minArduinoValue);

    double clamp(double value, double min, double max) const
    {
        return std::max(min, std::min(value, max));
    }

public:
    std::string name_l = "";
    std::string name_r = "";
    double cmd_l = 0;
    double cmd_r = 0;
    double cmd_raw = 0;
    double pos_l = 0;
    double pos_r = 0;
    double pos_raw = 0;

    ServoSteering() = default;

    ServoSteering(const std::string &joint_name_l, const std::string &joint_name_r)
    {
        name_l = joint_name_l;
        name_r = joint_name_r;
    }

    void map_cmd_to_arduino()
    {
        cmd_raw = cmd_r * 1000;
    }

    void map_pos_to_control()
    {
        // TO DO: sample more values of the servo and wheel angles -> find a function
        this->pos_r = this->pos_raw / 1000;
        if (this->pos_r > 0)
        {
            this->pos_l = this->pos_r + (this->pos_r * 0.15);
        }
        else
        {
            this->pos_l = this->pos_r - (this->pos_r * 0.08);
        }
    }
};

#endif // ACKERMANN_ARDUINO_STEERING_HPP