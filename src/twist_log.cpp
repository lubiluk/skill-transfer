#include "skill_transfer/twist_log.h"
#include <algorithm>

TwistLog::TwistLog(unsigned int size, double threshold) : size_(size),
                                                          threshold_(threshold)
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

bool TwistLog::allFilledAndBelowThreshold()
{
  // Log has to be filled up
  if (log_.size() < size_)
    return false;

  return std::all_of(log_.begin(), log_.end(),
                     [this](const geometry_msgs::Twist &t) {
                       return (t.linear.x < this->threshold_) &&
                              (t.linear.y < this->threshold_) &&
                              (t.linear.z < this->threshold_);
                     });
}