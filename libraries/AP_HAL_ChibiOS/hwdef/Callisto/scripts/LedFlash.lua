
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
   return update, loop_time -- reschedules the loop at 50Hz
end

return update() -- run immediately before starting to reschedule
