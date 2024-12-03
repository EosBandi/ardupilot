-- Time tracker device driver for Ardupilot
-- Version 1.1 2024-11-27
-- Set the I2C bus number, where the eeprom os connected. 
-- Warning I2C bus number not always corresponding to the connector labeling on the board.

local I2CBUS = 0

---@diagnostic disable: need-check-nil

local mavlink_msgs = require("MAVLink/mavlink_msgs")

local COMMAND_ACK_ID = mavlink_msgs.get_msgid("COMMAND_ACK")
local COMMAND_LONG_ID = mavlink_msgs.get_msgid("COMMAND_LONG")

local msg_map = {}
msg_map[COMMAND_ACK_ID] = "COMMAND_ACK"
msg_map[COMMAND_LONG_ID] = "COMMAND_LONG"

mavlink:init(1, 10)
mavlink:register_rx_msgid(COMMAND_LONG_ID)

-- this will be used for incoming communication from the GCS
-- 

local MAV_CMD_WAYPOINT_USER_1 = 31000
mavlink:block_command(MAV_CMD_WAYPOINT_USER_1)

-- add a pram to check if we the system installed
local PARAM_TABLE_KEY = 79
assert(param:add_table(PARAM_TABLE_KEY, "TRCKNG_", 1), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'ENABLE', 1), 'could not add TRCKNG_ENABLE')

-- eeprom memory structure definition
local BANK0 = 0
local BANK1 = 1
local BANK2 = 2
local BANK3 = 3
local BANK4 = 4
local BANK5 = 5
local BANK6 = 6
local BANK7 = 7

local INIT_STEP0 = 0
local INIT_STEP1 = 1
local INIT_STEP2 = 2
local INIT_STEP3 = 3
local INIT_STEP4 = 4
local INIT_STEP5 = 5
local INIT_STEP6 = 6
local INIT_STEP7 = 7
local INIT_STEP8 = 8
local INIT_STEP9 = 9

-- multi step initialisation
local init_step = INIT_STEP0

local EEPROM_VERSION = 0xAA04

-- BANK0, i2C ADDRESS 80
local ADDR0_EEPROM_VERSION =  0     -- 2 bytes See EEPROM_VERSION
local ADDR0_VIN =             2     -- 10 bytes string vehicle identification text
local ADDR0_SERIAL =          12    -- 2 bytes serial number
local ADDR0_SYSID =           22    -- 1 bytes system id 
local ADDR0_TIMERNUMBERS =    23    -- 1 bytes number of timers
local ADDR0_COMISSION_TIME =  24    -- 4 bytes comission time in utc time !!!!!! *24*60*60
local ADDR0_NEXT_LOG_ENTRY =  28    -- 1 bytes next log entry
local ADDR0_TIMER0 =          29    -- 4 bytes timer 0
local ADDR0_TIMER31 =         153   -- 4 bytes timer 31
local ADDR0_COMP0_SRV =       157   -- 2 bytes service interval for component 0
local ADDR0_COMP31_SRV =      219   -- 2 bytes service interval for component 31
-- BANK1; I2C ADDRESS 81
local ADDR1_COMPONENT0 =      0     -- 8 bytes component name for entry 0
local ADDR1_COMPONENT31 =     248   -- 8 bytes component name for entry 31

--BANK2; I2C ADDRESS 82
local ADDR2_LOG0 = 0         -- 10 bytes log entry 0
-- 24 log entries per BANK, starting at BANK2 all goes all to BANK7 total 144 log entries
-- from the log entry number (0-143) you get the bank number by dividing 24
-- bank = entry_number // 24
-- position in bank = entry number % 24

local address = 0
local found = 0

local eeprom_ok = false

-- number of timers
local counters = 0
-- timer values
local timers = {}
-- service intervals
local service_intervals = {}
-- component names
local component_names = {}
-- vin
local vin = ""
-- serial
local serial = ""
-- sysid
local sysid = 0
-- comission time
local comission_time = 0
-- next log entry
local next_log_entry = 0
local arm_time = 0

-- for debugging
local fake_arm = false




function message(msg)
  gcs:send_text(5, "FL: ".. msg)
end

local i2c_bus = i2c:get_device(I2CBUS,0)
i2c_bus:set_retries(10)
i2c_bus:set_address(80)

if i2c_bus == nil then
  message("Log EEPROM not found")
  return
end

-- BANK0 to BANK7
function setBank(bank)
  if (bank < 0 or bank > 7) then
    message("Invalid bank number " .. bank)
    return
  end
  i2c_bus:set_address(80 + bank)
end

-- Write Queue to eeprom, since we have a 5ms Tw write cycle between writes
local initialQueueSize = 1000  -- Example preallocation size

local writeQueue = {}
 writeQueue.first = 0
 writeQueue.last = -1
 writeQueue.data = {}

function queueInsert(q, val)
  q.last = q.last + 1
  q.data[q.last] = val
end

function queueIsEmpty(q)
    if (q.first > q.last) then 
     return true
    end
    return false
end

function queueResetCounters(q)
    if queueIsEmpty(q) then
        q.first = 0
        q.last = -1
    end
end

function queueRemove(q)
    if queueIsEmpty(q) then 
        rval = -1
    else
      rval = q.data[q.first]
      q.data[q.first] = nil        -- to allow garbage collection
      q.first = q.first + 1
    end
    queueResetCounters(q)
    return rval
end

function queueEEPROM(bank,addr, buffer, length)
    for i = 1,length
    do
        address = addr + i - 1
        data = {0,0,0}
        data[1] = bank
        data[2] = address
        data[3] = string.byte(buffer,i)
        queueInsert(writeQueue,data)
    end
end

function queueEEPROMByte(bank,addr, value)
    data = {0,0,0}
    data[1] = bank
    data[2] = addr
    data[3] = value
    queueInsert(writeQueue,data)
end

--- End of Write Queue  

-- Convert a 2-byte string to a float number
-- call as flt1 = to_number(str:sub(1, 2)) && flt2 = to_number(str:sub(3, 4))
-- to get two float numbers that can send and receive thourgh script command
local function to_number(bytes)
  local b1, b2 = string.byte(bytes, 1, 2) -- Get all bytes in one call
  return (b1 << 8) + b2
end

local function to_string(flt)
  local b1 = (flt >> 8) & 0xff
  local b2 = flt & 0xff
  return string.char(b1, b2) -- Combine in one call
end


-- Check if the first two bytes are 0x55AA that means an initialised EEPROM
function checkEEPROMSignature()
  local data = readEEPROM(BANK0,ADDR0_EEPROM_VERSION, 2)
  if data == nil then
    return false
  end
  local magic = string.unpack("<H", data)
  if magic == EEPROM_VERSION then
    return true
  end
  return false
end

function readEEPROM(bank, addr, length)
  local data = ""
  setBank(bank)
  local retval = i2c_bus:read_registers(addr,length) 
  for key,value in pairs(retval) do 
    data = data .. string.char(retval[key])
  end
return data
end

-----------------------------------------
function getComponentName(index)
  if index < 0 or index > 31 then
    message("Invalid component index")
    return
  end
  local data = readEEPROM(BANK1,ADDR1_COMPONENT0 + index * 8, 8)
  if data == nil then
    message("Error reading eeprom (component name)")
    return
  end
  return data
end

function setComponentName(index, name)
  if index < 0 or index > 31 then
    message("Invalid component index")
    return
  end
  component_names[index] = name
  queueEEPROM(BANK1,ADDR1_COMPONENT0 + index * 8, name, 8)
end

----------------------------------------------------------------

function initEEPROM(components, vin, serial, sysid, comission_time)
  -- set up device ---------------------------------------------
  if components >32 or components < 1 then
    message("Invalid number of components")
    return
  end


   queueEEPROM(BANK0,ADDR0_EEPROM_VERSION, string.pack("<H", EEPROM_VERSION), 2)
   queueEEPROM(BANK0, ADDR0_VIN, vin, 10)
   queueEEPROM(BANK0, ADDR0_SERIAL, string.pack("<H", serial),2)
   queueEEPROMByte(BANK0, ADDR0_SYSID, sysid)
   queueEEPROMByte(BANK0,ADDR0_TIMERNUMBERS, components)
   queueEEPROM(BANK0,ADDR0_COMISSION_TIME,string.pack("<H",comission_time) , 2)

end

function addLogEntry(type, time, mask, uid)
  -- check if we are reached the 144 log entries
  if next_log_entry > 143 then
    next_log_entry = 0
    message("Log full, rewrite oldest entries")
    return
  end

  local bank = math.floor(next_log_entry / 24)
  local position = next_log_entry % 24
  local data = string.pack("<BI4I4B", type, time, mask, uid) 
  queueEEPROM(bank + 2, position * 10, data, 10)
  next_log_entry = next_log_entry + 1
  queueEEPROMByte(BANK0, ADDR0_NEXT_LOG_ENTRY, next_log_entry)
end

function getLogEntry(entry)
  if entry > 143 then
    message("Invalid log entry number")
    return
  end
  local bank = math.floor(entry / 24)
  local position = entry % 24
  local data = readEEPROM(bank + 2, position * 10, 10)
  if data == nil then
    message("Error reading eeprom (log entry)")
    return
  end
  return string.unpack("<BI4I4B", data)
end

function handle_command_long(cmd)
    if (cmd.command == MAV_CMD_DO_SET_MODE) then
        gcs:send_text(0, "Got mode change")

    elseif (cmd.command == MAV_CMD_WAYPOINT_USER_1) then
      if cmd.param7 == 1 then
        -- param1-4 name, param5 service life, param6 component number
        local name = to_string(cmd.param1) .. to_string(cmd.param2) .. to_string(cmd.param3) .. to_string(cmd.param4)
        local index = math.floor(cmd.param6)
        local servicelife = math.floor(cmd.param5)
        service_intervals[index] = servicelife
        timers[index] = 0 -- reset timer
        setComponentName(index, name)
        local data = string.pack("<H", servicelife)
        queueEEPROM(BANK0,ADDR0_COMP0_SRV + index * 2, data, 2)
        --message("set ".. name .. " " .. index .. " " .. servicelife .. " ")
        return 0 -- Accepted
      elseif cmd.param7 == 2 then
        send_base_data()
        return 0 -- Accepted
      elseif cmd.param7 == 3 then
        if (cmd.param6 < 0 or cmd.param6 > 32) then
          return 0
        end
        send_component_data(cmd.param6) 
        return 0
      elseif cmd.param7 == 4 then
        local bitmask = math.floor(cmd.param5) + math.floor(cmd.param4) * 65536
        local time = math.floor(cmd.param3) + math.floor(cmd.param2) * 65536
        message("Resetting timers for components " .. bitmask)
        -- reset the timer for component number in the bitmask
        for i = 0, 31 do
          if (bitmask & (1 << i)) ~= 0 then
            timers[i] = 0
            queueEEPROM(BANK0, ADDR0_TIMER0 + i * 4, string.pack("I", 0), 4)
          end
        end
        addLogEntry(0, time, bitmask, 0)
        return 0        
        -- reset flight timers
        -- add log entry
      elseif cmd.param7 == 5 then
        -- set up base data 1 (init eeprom)
        queueEEPROM(BANK0,ADDR0_EEPROM_VERSION, string.pack("<H", EEPROM_VERSION), 2)
        sysid = math.floor(cmd.param5)
        queueEEPROMByte(BANK0, ADDR0_SYSID, sysid)
        counters = math.floor(cmd.param4)
        queueEEPROMByte(BANK0, ADDR0_TIMERNUMBERS, counters)
        comission_time = math.floor(cmd.param3)
        queueEEPROM(BANK0,ADDR0_COMISSION_TIME,string.pack("<H",comission_time) , 2)
        queueEEPROM(BANK0, ADDR0_NEXT_LOG_ENTRY, string.pack("<B", 0), 1)
        eeprom_ok = true
        return 0
      elseif cmd.param7 == 6 then
        -- set up base data 2
        vin = to_string(cmd.param1) .. to_string(cmd.param2) .. to_string(cmd.param3) .. to_string(cmd.param4) .. to_string(cmd.param5)
        queueEEPROM(BANK0, ADDR0_VIN, vin, 10)
        return 0
      elseif cmd.param7 == 7 then
        -- set up base data 3
        serial = to_string(cmd.param1) .. to_string(cmd.param2) .. to_string(cmd.param3) .. to_string(cmd.param4) .. to_string(cmd.param5)
        queueEEPROM(BANK0, ADDR0_SERIAL, serial, 10)
        return 0
      elseif cmd.param7 == 8 then
        p1, timestamp, bitmask ,p2 = getLogEntry(cmd.param6)
        send_log_entry(cmd.param6, bitmask, timestamp)
        return 0
      elseif cmd.param7 == 9 then
        message("Log cleared");
        queueEEPROM(BANK0, ADDR0_NEXT_LOG_ENTRY, string.pack("<B", 0), 1)
        next_log_entry = 0
        return 0
--- fake arming for testing
      elseif cmd.param7 == 10 then
        fake_arm = true
        return 0
      elseif cmd.param7 == 11 then
        fake_arm = false
        return 0
      end
      return 0 -- Accepted
    end
    return nil
end

function send_log_entry(id, bitmask, timestamp)
  local data32 = {}
  data32.type = 3
  data32.len = 16
  data32.id = id
  data32.bitmask = bitmask
  data32.timestamp = timestamp
  mavlink:send_chan(0, mavlink_msgs.encode("DATA32_3", data32))
end


function send_component_data(component_id)
  local data32 = {}
  data32.type = 2
  data32.len = 16
  data32.comp_no = component_id
  data32.comp_name = component_names[component_id]
  data32.comp_timer = timers[component_id]
  data32.comp_life = service_intervals[component_id]
  data32.comp_status = 1
  mavlink:send_chan(0, mavlink_msgs.encode("DATA32_2", data32))
end

function send_base_data()

  local data32 = {}

  if (eeprom_ok == false) then
    data32.type = 1
    data32.len = 16
    data32.vin =  "0000000000"
    data32.serial = "0000000000"
    data32.sysid = 0
    data32.timers = 0
    data32.comission_time = 0
    data32.log_entries = 0
  else
    data32.type = 1
    data32.len = 16
    data32.vin = vin
    data32.serial = serial
    data32.sysid = sysid
    data32.timers = counters
    data32.comission_time = comission_time
    data32.log_entries = next_log_entry
  end


  mavlink:send_chan(0, mavlink_msgs.encode("DATA32_1", data32))
end

local a = 0

local component_name_load_progress = 0
-- initEEPROM(32, "1234567890", 1234, 1, 6473)

function update() -- this is the loop which periodically runs

-- process Write Queue and write the next byte to the EEPROM is it is not empty
-- This is regrdless of the init_step, so it is always processed
  if not queueIsEmpty(writeQueue) then
    local data = queueRemove(writeQueue)
    setBank(data[1])
    i2c_bus:write_register(data[2], data[3])
    return update, 10 -- reschedule for 10 ms for the next byte  (5ms is the Tw write cycle)
  end

  -- INIT with multiple steps, since de number of bytes that can be read from the EEPROM in one go is limited
  if init_step == INIT_STEP0 then
    if checkEEPROMSignature() == false then
      message("EEPROM not initialised")
      eeprom_ok = false
      init_step = INIT_STEP6
      return update, 500
    end
    eeprom_ok = true
    init_step = INIT_STEP1
    return update, 500 -- reschedules the loop 
  end

if init_step == INIT_STEP1 then
    message("Initialisation started")
    setBank(BANK0)
    local counters_t = i2c_bus:read_registers(ADDR0_TIMERNUMBERS, 1)
    counters = counters_t[1]
    serial = readEEPROM(BANK0,ADDR0_SERIAL, 10)
    local sysid_t =  i2c_bus:read_registers(ADDR0_SYSID, 1)
    sysid = sysid_t[1]
    comission_time = string.unpack("<H",readEEPROM(BANK0,ADDR0_COMISSION_TIME, 2))
    vin = readEEPROM(BANK0, ADDR0_VIN, 10)
    next_log_entry = i2c_bus:read_registers(ADDR0_NEXT_LOG_ENTRY, 1)
    next_log_entry = next_log_entry[1]
    -- message("counters: " .. string.format("%i",counters[1]))
    -- message("Serial: " .. string.format("%i", serial) .. "sysid:" .. string.format("%i", sysid[1]) .. " Comission time: " .. string.format("%i", comission_time) .. " VIN: " .. vin)
    if counters == nil then
      message("Error reading eeprom (counters)")
      return
    end
    init_step = INIT_STEP2
    return update, 500 -- reschedules the loop
  end

  if init_step == INIT_STEP2 then
    -- build a table of timers based on the number above
    for i = 0, counters - 1 do
      local data = readEEPROM(BANK0, ADDR0_TIMER0 + i * 4, 4)
      if data == nil then
        message("Error reading eeprom (timer)")
        return
      end
      local value = string.unpack("I", data)
      timers[i] = value
    end
    init_step = INIT_STEP3
    return update, 500 -- reschedules the loop
  end

  if init_step == INIT_STEP3 then
    -- build table of components service intervals
    for i = 0, counters - 1 do
      local data = readEEPROM(BANK0,ADDR0_COMP0_SRV + i * 2, 2)
      if data == nil then
        message("Error reading eeprom (service interval)")
        return
      end
      local value = string.unpack("H", data)
      service_intervals[i] = value
    end
    init_step = INIT_STEP4
    return update, 500 -- reschedules the loop
  end
  if init_step == INIT_STEP4 then
    if component_name_load_progress < (counters)  then
      component_names[component_name_load_progress] = getComponentName(component_name_load_progress)
      component_name_load_progress = component_name_load_progress + 1
      return update, 200 -- reschedules the loop
    end
    init_step = INIT_STEP5
    message("Initialisation complete")
    return update, 200 -- reschedules the loop
  end

  if init_step == INIT_STEP5 then
     for i = 0, counters - 1 do
       message("Component " .. component_names[i] .. ": " .. " " .. string.format("%i",math.floor(timers[i]/3600)) .. "h from "..  string.format("%u", service_intervals[i]).. "h")
     end
    init_step = INIT_STEP6
    return update, 500 -- reschedules the loop
  end

   if init_step == INIT_STEP6 then
     -- check if any compinent reached it's service life
     -- service_intervals is in hours, timers in seconds so we need to converting
     for i = 0, counters - 1 do
       if timers[i] > service_intervals[i] * 3600 then
          gcs:send_text(2, "FL: Component " .. component_names[i] .. " reached it's service life")
       end
     end
     init_step = INIT_STEP7
     return update, 1000 -- reschedules the loop
   end

  -- we passed init steps so this will be the normal operation loop

  -- check if we are armed and start the timers
  if (arming:is_armed() == true and arm_time == 0) then
    arm_time = millis()
  end


  -- if we are armed, then skip the mavlink message part, to avoid any inflight issues
  if (arming:is_armed() == true) then
    return update, 500
  end

  -- Handle MAVLINK incoming messages
  local msg, chan = mavlink:receive_chan()
  if (msg ~= nil) then
      local parsed_msg = mavlink_msgs.decode(msg, msg_map)
      if (parsed_msg ~= nil) then

          local result
          if parsed_msg.msgid == COMMAND_LONG_ID then
              result = handle_command_long(parsed_msg)
          end

          if (result ~= nil) then
              -- Send ack if the command is one were intrested in
              local ack = {}
              ack.command = parsed_msg.command
              ack.result = 0
              ack.progress = 0
              ack.result_param2 = 0
              ack.target_system = parsed_msg.sysid
              ack.target_component = parsed_msg.compid
              mavlink:send_chan(chan, mavlink_msgs.encode("COMMAND_ACK", ack))
          end
      end
  end
  

  if (arming:is_armed() == false and arm_time ~= 0) then
    local elapsed = millis() - arm_time
    elapsed = elapsed / 1000
    for i = 0, counters - 1 do
      timers[i] = timers[i] + elapsed:toint()
      queueEEPROM(BANK0, ADDR0_TIMER0 + i * 4, string.pack("<I4", timers[i]), 4)
    end
    arm_time = 0
    message("Component timers updated")
  end

  -- end of normal operation loop
  return update, 500 -- reschedules the loop
end

return update(),1000 -- run immediately before starting to reschedule
