/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Param/AP_Param.h>

#ifndef AP_LIGHTSENSOR_ENABLED
#define AP_LIGHTSENSOR_ENABLED 1
#endif

class AP_LightSensor_Backend;

class LightSensor
{
    friend class AP_LightSensor_Backend;
public:
    LightSensor(const LightSensor &other) = delete;
    LightSensor &operator=(const LightSensor&) = delete;

    enum class Type{
        NONE    = 0,
        BH1750  = 1,
        SIM     =100,
    };
    enum class Status {
        NotConnected = 0,
        NoData,
        OutOfRangeLow,
        OutOfRangeHigh,
        Good
    };
    struct LightSensor_State {
        uint32_t luminance_lux = 0;      //Measured luminance
        enum LightSensor::Status status = Status::NotConnected; // sensor status
        uint32 last_reading_ms;          // system time of last successful update 
    }
}