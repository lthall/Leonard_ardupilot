-- This script is an example of reading a analog pin, PWM in and GPIO

-- for these examples BRD_PWM_COUNT must be 0

gpio:pinMode(50,1) -- set AUX 2 to output, gpio:pinMode(51,0) would be input
gpio:pinMode(51,1) -- set AUX 2 to output, gpio:pinMode(51,0) would be input
gpio:pinMode(52,1) -- set AUX 2 to output, gpio:pinMode(51,0) would be input

local color = 1

function update()
  color = 1
  if ahrs:healthy()  then
    gcs:send_text(0, "healthy")
    color = 4
    if arming:pre_arm_checks() then
      gcs:send_text(0, "pre_arm_checks")
      color = 5
    end
  end
  if color == 0 then
    -- Off
    gpio:write(50, 0)
    gpio:write(51, 0)
    gpio:write(52, 0)
  elseif color == 1 then
    -- Red
    gpio:write(50, 1)
    gpio:write(51, 0)
    gpio:write(52, 0)
  elseif color == 2 then
    -- Green
    gpio:write(50, 0)
    gpio:write(51, 1)
    gpio:write(52, 0)
  elseif color == 3 then
    -- Yellow
    gpio:write(50, 1)
    gpio:write(51, 1)
    gpio:write(52, 0)
  elseif color == 4 then
    -- Blue
    gpio:write(50, 0)
    gpio:write(51, 0)
    gpio:write(52, 1)
  elseif color == 5 then
    -- Magenta
    gpio:write(50, 1)
    gpio:write(51, 0)
    gpio:write(52, 1)
  elseif color == 6 then
    -- Cyan
    gpio:write(50, 0)
    gpio:write(51, 1)
    gpio:write(52, 1)
  elseif color == 7 then
    -- White
    gpio:write(50, 1)
    gpio:write(51, 1)
    gpio:write(52, 1)
  end
  return update, 1000
end

return update() -- run immediately before starting to reschedule
