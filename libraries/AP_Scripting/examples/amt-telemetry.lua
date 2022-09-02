-- Get AMT telemetry from the first serial port marked with function 28
-- Serial speed at the engine telemetry is 2400 baud

-- Variables to store actual data
local motor_rpm = 0
local motor_egt = 0
local motor_thr = 0
local telemetry_status = true

local motor_status = 0
local motor_error = 0
local bptr = 0
local last_packet_out = 0

-- input buffer
local inBuffer = {}
local lastpacket = {}

-- find the serial first (0) scripting serial port instance
-- SERIALx_PROTOCOL 28
local port = assert(serial:find_serial(0),"Could not find Scripting Serial Port")


-- read avail bytes from the engine interface and put latest packet in lastpacket
function Do_engine_comm()

    local n_bytes = port:available()
    local c = 0

    while n_bytes > 0 do
        c = port:read()
        if c == 255 and bptr == 6 then
            if (inBuffer[1] ~= 6) then
                lastpacket[0] = inBuffer[0]
                lastpacket[1] = inBuffer[1]
                lastpacket[2] = inBuffer[2]
                lastpacket[3] = inBuffer[3]
                lastpacket[4] = inBuffer[4]
                lastpacket[5] = inBuffer[5]
            end
            bptr = 0
            inBuffer[bptr] = c
            bprt = bptr + 1
        elseif (c == 255) then
            bptr = 0
            inBuffer[bptr] = c
            bptr = bptr + 1
        elseif (bptr < 6) then
            inBuffer[bptr] = c
            bptr = bptr + 1
        end
    end
end

function Process_engine_packets()

    if lastpacket[0] == 255 and lastpacket[1] ~= 0 then
    -- no error message
        motor_rpm = lastpacket[2] * 500
        motor_egt = lastpacket[3] * 4.6 - 50
        motor_thr = lastpacket[4] / 2
        motor_status = lastpacket[1]
        motor_error = 0
    elseif lastpacket[1] == 0 then
    -- we have an error message
        motor_egt = lastpacket[3] * 4.6 - 50
        motor_thr = lastpacket[4] /2 
        motor_error = lastpacket[2]
    end

end



function update()

    Do_engine_comm()
    Process_engine_packets()

    if (last_packet_out + 500 < millis()) then
        -- send out EFI packet
        
    end


    -- restart in 100ms
    return update, 100
end


--Start script when boot
return update()
