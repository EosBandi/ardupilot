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

#include "AP_RTC.h"

class AP_RTC_Backend
{
public:
    // constructor. This incorporates initialization as well.
    AP_RTC_Backend(AP_RTC &frontend);

    // we declare a virtual destructor so that WindVane drivers can
    // override with a custom destructor if need be
    virtual ~AP_RTC_Backend() {}

    // initialization
    virtual bool init(void) { return false;};

    // update the state structure

protected:

    AP_RTC &_frontend;
    uint8_t get_bus(void);

};