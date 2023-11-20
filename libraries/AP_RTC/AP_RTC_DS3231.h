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

#include "AP_RTC_config.h"

/* 
    backend driver for DS3231 RTC clock connected to I2C bus
*/

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

#include "AP_RTC_Backend.h"


class AP_RTC_DS3231 : public AP_RTC_Backend
{
public:
    AP_RTC_DS3231(AP_RTC &frontend);
    ~AP_RTC_DS3231(void) {}

    //Tries to init the sensor
    bool init(void) override;

    // ... more to come

private:
   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

};

