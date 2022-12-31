-- add new param MOT_STOP_BITMASK
local PARAM_TABLE_KEY = 1
assert(param:add_table(PARAM_TABLE_KEY, "LIGHTS_", 1), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "ON", 1), "could not add param")

local lights_on = Parameter()
assert(lights_on:init("LIGHTS_ON"), "could not find param")

local LED_BRIGHT = Parameter()
if not LED_BRIGHT:init('NTF_LED_BRIGHT') then
   gcs:send_text(6, 'init NTF_LED_BRIGHT failed')
end

-- toggle a relay at 50Hz

local RELAY_NUM = 0
local loop_time = 500 -- number of ms between runs
local flash_step = 0
local flash_length = 10
local flash_gap = 250
local flash_period = 1500

--gcs:send_text(0, "Starting LUA")
function update() -- this is the loop which periodically runs
   local home = ahrs:get_home()
   local position = ahrs:get_location()
   if lights_on:get() <= 0.5 then
      relay:off(RELAY_NUM)
      loop_time= 1000;
      --gcs:send_text(6, 'Arm LEDs off')
      if lights_on:get() <= -0.5 and LED_BRIGHT:get() ~= 0 then
         --gcs:send_text(6, 'Status LED off')
         LED_BRIGHT:set(0)
      elseif lights_on:get() >= -0.5 and LED_BRIGHT:get() ~= 3 then
         --gcs:send_text(6, 'Status LED on')
         LED_BRIGHT:set(3)
      end
   else
      if LED_BRIGHT:get() ~= 3 then
         --gcs:send_text(6, 'Status LED on')
         LED_BRIGHT:set(3)
      end
      if not arming:is_armed() then
         flash_length = 1
      elseif home and position then
         flash_length = 1 + math.min((position:get_distance(home)), 124)
         --gcs:send_text(0, "Distance to home: " .. tostring(position:get_distance(home)) .. " Flash Time: " .. tostring(flash_length) .. " ms")
      else
         flash_length = 1
      end
      if flash_step == 0 then
         relay:on(RELAY_NUM)
         loop_time = flash_length
         flash_step = flash_step + 1
      elseif flash_step == 1 then
         relay:off(RELAY_NUM)
         loop_time = flash_gap - flash_length
         flash_step = flash_step + 1
      elseif flash_step == 2 then
         relay:on(RELAY_NUM)
         loop_time = flash_length
         flash_step = flash_step + 1
      else
         relay:off(RELAY_NUM)
         loop_time = flash_period - (flash_gap + 2 * flash_length)
         flash_step = 0
      end
   end
   return update, loop_time
end

return update() -- run immediately before starting to reschedule
