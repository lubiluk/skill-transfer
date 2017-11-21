#include "skill_transfer/twist_log.h"
#include <algorithm>
#include <cmath>

TwistLog::TwistLog(unsigned int size) : size_(size)
{
}

void TwistLog::push(geometry_msgs::Twist twist)
{
  // Keep the log size fixed by removing the oldest entry
  if (log_.size() >= size_)
    log_.pop_front();

  // Save twist to log
  log_.push_back(twist);
}

void TwistLog::clear()
{
  log_.clear();
}

bool TwistLog::allFilledAndBelowThreshold(double threshold)
{
  // Log has to be filled up
  if (log_.size() < size_)
    return false;

  return std::all_of(log_.begin(), log_.end(),
                     [threshold](const geometry_msgs::Twist &t) {
                       return (std::abs(t.linear.x) < threshold) &&
                              (std::abs(t.linear.y) < threshold) &&
                              (std::abs(t.linear.z) < threshold) &&
                              (std::abs(t.angular.x) < threshold) &&
                              (std::abs(t.angular.y) < threshold) &&
                              (std::abs(t.angular.z) < threshold);
                     });
}
