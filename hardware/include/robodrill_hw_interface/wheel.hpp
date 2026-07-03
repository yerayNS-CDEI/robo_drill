#ifndef ROBODRILL_HW_INTERFACE__WHEEL_HPP
#define ROBODRILL_HW_INTERFACE__WHEEL_HPP

#include <string>
#include <cmath>

namespace robodrill_hw_interface
{
  namespace motor_states
  {
    typedef struct
    {
      int encoder_ticks_l = 0;
      float motor_vel_l = 0;
      int encoder_ticks_r = 0;
      float motor_vel_r = 0;
      int encoder_ticks_turret = 0;
      float turret_vel = 0;
    } MotorStates;
  }

  class Wheel
  {
  public:
    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;

    Wheel() = default;

    explicit Wheel(const std::string &wheel_name)
    {
      setup(wheel_name);
    }

    void setup(const std::string &wheel_name)
    {
      name = wheel_name;
    }
  };

} // namespace robodrill_hw_interface

#endif // ROBODRILL_HW_INTERFACE__WHEEL_HPP
