--[[
   This script reads the firmware, calculates a checksum and compares it againsta a fixed known-good checksum.  

   It does this 10 seconds after boot.  And any time millis() wraps.
--]]

local flash_file_path = "@SYS/flash.bin"

function update()
   -- wait until we've been booted 10 seconds before sending:
   local now = millis()
   if now < 10000 then
      -- not time yet
      return update, 100
   end

   local expected_checksum = 0x253F9506

   local result = fs:crc32(flash_file_path):toint()
   if result == nil then
      gcs:send_text(1, string.format("checksum_firmware error: checksumming failed"))
   elseif result - expected_checksum ~= 0 then
      gcs:send_text(1, string.format("checksum_firmware: BAD: calculated=0x%x != expected=0x%x", result, expected_checksum))
   else
      gcs:send_text(1, string.format("checksum_firmware: CORRECT: 0x%x", result))
   end

   return  -- this script is one-shot 
end

return update()
