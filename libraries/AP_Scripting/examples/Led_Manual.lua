-- This script is an example of reading a analog pin, PWM in and GPIO

-- for these examples BRD_PWM_COUNT must be 0

gpio:pinMode(50,1) -- set AUX 2 to output, gpio:pinMode(51,0) would be input
gpio:pinMode(51,1) -- set AUX 2 to output, gpio:pinMode(51,0) would be input
gpio:pinMode(52,1) -- set AUX 2 to output, gpio:pinMode(51,0) would be input

local color = 1

function update()
  color = 1
  if ahrs:healthy()  then
    --gcs:send_text(0, "healthy")
    if gps:num_sensors() == 2 and gps:status(0) >= gps.GPS_OK_FIX_3D and gps:status(1) >= gps.GPS_OK_FIX_3D_RTK_FLOAT then
      --gcs:send_text(0, "GPS Lock")
      color = 2
    else
      --gcs:send_text(0, "Waiting for GPS Lock")
      color = 3
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
