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

/*
  backend driver for RTC Clock DS3231 connected to I2C
 */

#include "AP_RTC_DS3231.h"

#include <stdio.h>
#include <utility>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <GCS_MAVLink/GCS.h>


extern const AP_HAL::HAL &hal;


#define DS3231_I2C_ADDR 0x68

#define DS3231_REG_STATUS 0x0f
#define DS3231_REG_CONTROL 0x0e


AP_RTC_DS3231::AP_RTC_DS3231(AP_RTC &frontend) : AP_RTC_Backend(frontend)
{

}


bool AP_RTC_DS3231::init(void)
{
    // check if it is on address 
    // check if it is stopped
    // if stopped clear the osf bit and make the time/date invalid, waiting to update it.

    dev = hal.i2c_mgr->get_device(get_bus(), DS3231_I2C_ADDR);

    if (!dev)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RTC DS3231: cannot allicate bus %u addr 0x%02x", get_bus(), DS3231_I2C_ADDR);
        return false;
    }
    
    WITH_SEMAPHORE(dev->get_semaphore());
    dev->set_retries(5);
    uint8_t val;

    if (!dev->read_registers(DS3231_REG_CONTROL, &val, 1))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "RTC DS3231: unable to read status");
        return false;
    }

    //The magic number to check is 28, this is the default powerup value for the control register.
    //And this will not be changes by the RTC lib, to reset it do default value, remove backup battery

    if (val == 28)
    {
        //it is a DS3231, now read the status register to check for valid time
        WITH_SEMAPHORE(dev->get_semaphore());
        if (!dev->read_registers(DS3231_REG_STATUS, &val, 1))
            {
              GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "RTC DS3231: unable to read status");
              return false;
            }

        if ((val & 0x80) == 0)
        {
            _frontend._hw_rtc_valid = true;
        }
        else
        {
            _frontend._hw_rtc_valid = false;
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "RTC DS3231: Clock lost, need gps or gcs");
        }
    }
    return true;
}