#ifndef STOP_CONDITION_H
#define STOP_CONDITION_H

struct StopCondition
{
  double measured_velocity_min;
  double desired_velocity_min;
  bool contact;
  double activation_distance;
};

#endif // STOP_CONDITION_H