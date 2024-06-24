-- Script to dynamycally adjust the wpnav_speed during auto flight, based on the generator status
-- The overall objective of this is to fly the vehicle as fast as possible to from A to B by utilising the maximum amount of fuel cell power available in flight 
-- and while staying within safe flight control parameter limits. 
-- Based on the case ; If the maximum power output of the fuel cell is reached and more current is still drawn by the drone, 
-- the fuel cell voltage will decrease to match the hybrid battery voltage and current will then be drawn from the hybrid batteries as well.

--  Author : Andras "EosBandi" Schaffer
--  Date : 2024.06.24
--  Version : 1.0

local PARAM_TABLE_KEY = 75              -- 97 randomly selected, make sure other scripts do not use the same key
local PARAM_TABLE_PREFIX = 'DYNSPD_'
local FLIGHMODE_AUTO = 3
local FLIGHTMODE_RTL = 6


local lowcount = 0
local midcount = 0
local highcount = 0

-- bind a parameter to a variable given
function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

function get_time_sec()
    return millis():tofloat() * 0.001
end

function clear_counters()
    lowcount = 0
    midcount = 0
    highcount = 0
end

-- start message
gcs:send_text(5, "Dynamic Speed Control 1.0 script started")

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add param table')

local DYNSPD_ENABLE = bind_add_param('ENABLE',  1, 1)  -- Enable/Disable the dynamic speed control
local DYNSPD_SPEED = bind_add_param('SPEED',  2, 1000) -- Default NAV speed  
local DYNSPD_MAXSPEED = bind_add_param('MAXSPEED',  3, 2000) -- Maximum NAV speed
local DYNSPD_MINSPEED = bind_add_param('MINSPEED',  4, 300) -- Minimum NAV speed
local DYNSPD_LOWVOLT = bind_add_param('LOWVOLT',  5, 49.0) -- Low voltage threshold
local DYNSPD_HIGHVOLT = bind_add_param('HIGHVOLT',  6, 49.5) -- High voltage threshold
local DYNSPD_AUXSW = bind_add_param('AUXSW',  7, 302) -- Aux switch to enable/disable the script
local DYNSPD_CYLES = bind_add_param('CYCLES',  8, 8) -- Number of cycles to adjust the speed (1 cycle = 500ms)

local prev_flightmode = vehicle:get_mode()
local speed_to_set = DYNSPD_SPEED:get() -- keeping track what speed we want to set


local RTL_SPEED = Parameter()
if not RTL_SPEED:init('RTL_SPEED') then
  gcs:send_text(6, 'DYNSPD: No parameter: RTL_SPEED')
  return
end

function reset_speed()

    -- If we are in RTL mode and the RTL_SPEED is not zero 
    -- then do not reset the speed, since it will override the RTL_SPEED
    if vehicle:get_mode() == FLIGHTMODE_RTL and RTL_SPEED:get() ~= 0 then
        gcs:send_text(5,"DYNSPD:RTL Speed set as WPNAV_SPEED " .. RTL_SPEED:get()/100 .. "m/s")
        return
    end
    -- Otherwise RTL uses WPNAV_SPEED to we have to reset it
    speed_to_set = DYNSPD_SPEED:get()
    vehicle:set_desired_speed(speed_to_set/100)
    gcs:send_text(5,"DYNSPD:Speed reset to WPNAV_SPEED " .. speed_to_set/100 .. "m/s")
end

-- check if the generator class is available
if not generator then
    gcs:send_text(0, "DYNSPD:Generator class not available")
    return
end


local last_aux_pos = -1

function update()

    --We don't run in the script is disabled
    if DYNSPD_ENABLE:get() == 0 then
        gcs:send_text(5, "DYNSPD:Script is disabled")   
        reset_speed()
        return
    end

    -- if the SUXSW is not zero then handle switch changes
    if DYNSPD_AUXSW:get() ~= 0 then
        local aux_pos = rc:get_aux_cached(DYNSPD_AUXSW:get())
        if aux_pos ~= last_aux_pos then
            last_aux_pos = aux_pos
            if aux_pos == 0 then
                gcs:send_text(5, "DYNSPD:Aux switch is off")
                reset_speed()
                clear_counters()
                return update, 500
            else
                gcs:send_text(5, "DYNSPD:Aux switch is on")
            end
        end
        -- force the script to stay stopped if the aux switch is off
        if (aux_pos == 0) then
            return update, 500
        end

    end

    if not arming:is_armed() then
        return update, 500
    end

    -- If we are not in auto mode and was in auto mode previously, reset the speed to the default speed 
    if vehicle:get_mode() ~= FLIGHMODE_AUTO and prev_flightmode == FLIGHMODE_AUTO then
        reset_speed()
        prev_flightmode = vehicle:get_mode()
        return update, 500
    end

    -- if we are not in auto mode, reset the counters and return
    if vehicle:get_mode() ~= FLIGHMODE_AUTO then
        clear_counters()
        prev_flightmode = vehicle:get_mode()
        return update, 500
    end
    
    prev_flightmode = vehicle:get_mode()

    -- Normally code 21 will cause a failsafe, so previous check will catch this case and reset the speed to default 
    -- This is for safety only
    local errorcode = generator:get_errorcode()
    if errorcode > 20 then
        reset_speed()
        clear_counters()
        return update, 500  
    end

    prev_flightmode = vehicle:get_mode()

    -- We are in auto mode and generator is working, so update battery status at every 500ms

    local battery_voltage = battery:voltage(0)
    -- set the counter based on the battery voltage status
    if battery_voltage < DYNSPD_LOWVOLT:get() then
        lowcount = lowcount + 1
    end
    if battery_voltage > DYNSPD_HIGHVOLT:get() then
        highcount = highcount + 1
    end
    if battery_voltage >= DYNSPD_LOWVOLT:get() and battery_voltage <= DYNSPD_HIGHVOLT:get() then
        midcount = midcount + 1
    end

    -- we have 8 meassurements in a row, so we can adjust the speed if needed
    if (lowcount + midcount + highcount) == DYNSPD_CYLES:get() then
        if lowcount == DYNSPD_CYLES:get() then
            if (speed_to_set > DYNSPD_MINSPEED:get()) then
                speed_to_set = speed_to_set - 50
                vehicle:set_desired_speed(speed_to_set/100)
                gcs:send_text(5,"DYNSPD:Speed decreased to " .. speed_to_set/100 .."m/s")
            end
        end
        if highcount == DYNSPD_CYLES:get() then
            if (speed_to_set < DYNSPD_MAXSPEED:get()) and gps:ground_speed(0)*100 >= speed_to_set-10 then
                speed_to_set = speed_to_set + 25
                vehicle:set_desired_speed(speed_to_set/100)
                gcs:send_text(5,"DYNSPD:Speed increased to " .. speed_to_set/100 .."m/s")
            end
        end
        clear_counters()
    end

return update, 500
end

return update()
