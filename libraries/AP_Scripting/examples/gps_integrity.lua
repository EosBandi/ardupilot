--[[
   example of reading the GNSS integrity (jamming/spoofing) state via
   gps:gnss_integrity(). States follow the MAVLink GNSS_INTEGRITY message:
   0 UNKNOWN (no data or detector disabled), 1 OK, 2 MITIGATED, 3 DETECTED
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local STATE_NAME = {[0]="UNKNOWN", [1]="OK", [2]="MITIGATED", [3]="DETECTED"}

local last_jam = {}
local last_spoof = {}

local function severity_for(state)
   if state >= 3 then
      return MAV_SEVERITY.CRITICAL
   elseif state == 2 then
      return MAV_SEVERITY.WARNING
   end
   return MAV_SEVERITY.INFO
end

local function update()
   for instance = 0, gps:num_sensors() - 1 do
      local jam, spoof, auth, syserr = gps:gnss_integrity(instance)
      if jam ~= nil then
         if jam ~= last_jam[instance] then
            gcs:send_text(severity_for(jam), string.format("GPS %u jamming: %s", instance + 1, STATE_NAME[jam] or tostring(jam)))
            last_jam[instance] = jam
         end
         if spoof ~= last_spoof[instance] then
            gcs:send_text(severity_for(spoof), string.format("GPS %u spoofing: %s", instance + 1, STATE_NAME[spoof] or tostring(spoof)))
            last_spoof[instance] = spoof
         end
      end
   end
   return update, 1000
end

gcs:send_text(MAV_SEVERITY.INFO, "gps_integrity.lua loaded")

return update, 1000
