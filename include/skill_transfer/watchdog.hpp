/*
* Copyright (C) 2016-2017 Georg Bartels <georg.bartels@cs.uni-bremen.de>
*
* This file is part of giskard.
*
* giskard is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#ifndef __GISKARD_WATCHDOG_HPP__
#define __GISKARD_WATCHDOG_HPP__

namespace giskard_ros
{
  template<class Time, class Duration>
  class Watchdog
  {
    public:
      bool barking(const Time& now)
      {
        return (now - last_kick_) > period_;
      }

      void setPeriod(const Duration& period)
      {
        period_ = period;
      }

      const Duration& getPeriod() const
      {
        return period_;
      }

      void kick(const Time& now)
      {
        last_kick_ = now;
      }

      const Time& getLastPetTime() const
      {
        return last_kick_;
      }
    private:
      Duration period_;
      Time last_kick_;
  };
}

#endif // __GISKARD_WATCHDOG__HPP