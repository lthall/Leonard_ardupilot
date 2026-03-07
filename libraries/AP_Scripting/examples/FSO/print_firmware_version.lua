--[[
   Prints the firmware version string to GCS on boot.
   One-shot script - runs once after a short delay then exits.
--]]

-- Returns true if running any Callisto C2.4 build (any rXX revision)
local function is_callisto_c2_4()
   return string.find(FWVersion:string(), "ArduCopter V4%.5%.7%-C2%.4") ~= nil
end

-- Returns true if running any Callisto C2.5 build (any rXX revision)
local function is_callisto_c2_5()
   return string.find(FWVersion:string(), "ArduCopter V4%.5%.7%-C2%.5") ~= nil
end

-- Returns true if running stock ArduCopter 4.5.7
local function is_arducopter_4_5_7()
   return FWVersion:major() == 4 and FWVersion:minor() == 5 and FWVersion:patch() == 7
      and FWVersion:string() == "ArduCopter V4.5.7"
end

function update()
   local now = millis()
   if now < 5000 then
      return update, 500
   end

   gcs:send_text(6, string.format("Firmware: %s (v%d.%d.%d) hash: %s",
      FWVersion:string(),
      FWVersion:major(),
      FWVersion:minor(),
      FWVersion:patch(),
      FWVersion:hash()
   ))
   gcs:send_text(6, string.format("is_callisto_c2_4: %s", tostring(is_callisto_c2_4())))
   gcs:send_text(6, string.format("is_callisto_c2_5: %s", tostring(is_callisto_c2_5())))
   gcs:send_text(6, string.format("is_arducopter_4_5_7: %s", tostring(is_arducopter_4_5_7())))

   return  -- one-shot
end

return update()
