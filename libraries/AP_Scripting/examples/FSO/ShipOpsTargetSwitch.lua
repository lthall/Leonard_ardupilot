-- add new param SHP1_
local ship_selected = 0
local PARAM_TABLE_KEY1 = 20
local PARAM_TABLE_PREFIX1 = "SHP1_"
local PARAM_TABLE_KEY2 = 21
local PARAM_TABLE_PREFIX2 = "SHP2_"

-- bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value, key, prefix)
   assert(param:add_param(key, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(prefix .. name)
end

assert(param:add_table(PARAM_TABLE_KEY1, PARAM_TABLE_PREFIX1, 11), 'could not add param table')
assert(param:add_table(PARAM_TABLE_KEY2, PARAM_TABLE_PREFIX2, 11), 'could not add param table')

local shp1_pch_ang = bind_add_param('PCH_ANG', 1, 180, PARAM_TABLE_KEY1, PARAM_TABLE_PREFIX1)
local shp1_pch_rad = bind_add_param('PCH_RAD', 2, 25, PARAM_TABLE_KEY1, PARAM_TABLE_PREFIX1)
local shp1_pch_alt = bind_add_param('PCH_ALT', 3, 25, PARAM_TABLE_KEY1, PARAM_TABLE_PREFIX1)
local shp1_koz_cw = bind_add_param('KOZ_CW', 4, 90, PARAM_TABLE_KEY1, PARAM_TABLE_PREFIX1)
local shp1_koz_ccw = bind_add_param('KOZ_CCW', 5, -90, PARAM_TABLE_KEY1, PARAM_TABLE_PREFIX1)
local shp1_koz_rad = bind_add_param('KOZ_RAD', 6, 250, PARAM_TABLE_KEY1, PARAM_TABLE_PREFIX1)
local shp1_koz_dkr = bind_add_param('KOZ_DKR', 7, 10, PARAM_TABLE_KEY1, PARAM_TABLE_PREFIX1)
local shp1_ofs_x = bind_add_param('OFS_X', 8, 0, PARAM_TABLE_KEY1, PARAM_TABLE_PREFIX1)
local shp1_ofs_y = bind_add_param('OFS_Y', 9, 0, PARAM_TABLE_KEY1, PARAM_TABLE_PREFIX1)
local shp1_ofs_z = bind_add_param('OFS_Z', 10, 0, PARAM_TABLE_KEY1, PARAM_TABLE_PREFIX1)
local shp1_sysid = bind_add_param('SYSID', 11, 200, PARAM_TABLE_KEY1, PARAM_TABLE_PREFIX1)

local shp2_pch_ang = bind_add_param('PCH_ANG', 1, 180, PARAM_TABLE_KEY2, PARAM_TABLE_PREFIX2)
local shp2_pch_rad = bind_add_param('PCH_RAD', 2, 25, PARAM_TABLE_KEY2, PARAM_TABLE_PREFIX2)
local shp2_pch_alt = bind_add_param('PCH_ALT', 3, 25, PARAM_TABLE_KEY2, PARAM_TABLE_PREFIX2)
local shp2_koz_cw = bind_add_param('KOZ_CW', 4, 90, PARAM_TABLE_KEY2, PARAM_TABLE_PREFIX2)
local shp2_koz_ccw = bind_add_param('KOZ_CCW', 5, -90, PARAM_TABLE_KEY2, PARAM_TABLE_PREFIX2)
local shp2_koz_rad = bind_add_param('KOZ_RAD', 6, 250, PARAM_TABLE_KEY2, PARAM_TABLE_PREFIX2)
local shp2_koz_dkr = bind_add_param('KOZ_DKR', 7, 10, PARAM_TABLE_KEY2, PARAM_TABLE_PREFIX2)
local shp2_ofs_x = bind_add_param('OFS_X', 8, 0, PARAM_TABLE_KEY2, PARAM_TABLE_PREFIX2)
local shp2_ofs_y = bind_add_param('OFS_Y', 9, 0, PARAM_TABLE_KEY2, PARAM_TABLE_PREFIX2)
local shp2_ofs_z = bind_add_param('OFS_Z', 10, 0, PARAM_TABLE_KEY2, PARAM_TABLE_PREFIX2)
local shp2_sysid = bind_add_param('SYSID', 11, 201, PARAM_TABLE_KEY2, PARAM_TABLE_PREFIX2)

local SHIP_PCH_ANG = Parameter()
if not SHIP_PCH_ANG:init('SHIP_PCH_ANG') then
   gcs:send_text(6, 'init SHIP_PCH_ANG failed')
end

local SHIP_PCH_RAD = Parameter()
if not SHIP_PCH_RAD:init('SHIP_PCH_RAD') then
   gcs:send_text(6, 'init SHIP_PCH_RAD failed')
end

local SHIP_PCH_ALT = Parameter()
if not SHIP_PCH_ALT:init('SHIP_PCH_ALT') then
   gcs:send_text(6, 'init SHIP_PCH_ALT failed')
end

local SHIP_KOZ_CW = Parameter()
if not SHIP_KOZ_CW:init('SHIP_KOZ_CW') then
   gcs:send_text(6, 'init SHIP_KOZ_CW failed')
end

local SHIP_KOZ_CCW = Parameter()
if not SHIP_KOZ_CCW:init('SHIP_KOZ_CCW') then
   gcs:send_text(6, 'init SHIP_KOZ_CCW failed')
end

local SHIP_KOZ_RAD = Parameter()
if not SHIP_KOZ_RAD:init('SHIP_KOZ_RAD') then
   gcs:send_text(6, 'init SHIP_KOZ_RAD failed')
end

local SHIP_KOZ_DKR = Parameter()
if not SHIP_KOZ_DKR:init('SHIP_KOZ_DKR') then
   gcs:send_text(6, 'init SHIP_KOZ_DKR failed')
end

local FOLL_OFS_X = Parameter()
if not FOLL_OFS_X:init('FOLL_OFS_X') then
   gcs:send_text(6, 'init FOLL_OFS_X failed')
end

local FOLL_OFS_Y = Parameter()
if not FOLL_OFS_Y:init('FOLL_OFS_Y') then
   gcs:send_text(6, 'init FOLL_OFS_Y failed')
end

local FOLL_OFS_Z = Parameter()
if not FOLL_OFS_Z:init('FOLL_OFS_Z') then
   gcs:send_text(6, 'init FOLL_OFS_Z failed')
end

local FOLL_SYSID = Parameter()
if not FOLL_SYSID:init('FOLL_SYSID') then
   gcs:send_text(6, 'init FOLL_SYSID failed')
end

local interval_ms = 1000     -- update at 1hz


gcs:send_text(0, "Starting SHIP OPS LUA")
function update() -- this is the loop which periodically runs
    
    pwm6 = rc:get_pwm(6)
    if pwm6 and pwm6 < 1250 then    -- check if RC6 input has moved low
      if ship_selected ~= 2 then
         gcs:send_text(0, "Ship 2 for payload place")
         ship_selected = 2
         SHIP_PCH_ANG:set(shp2_pch_ang:get())
         SHIP_PCH_RAD:set(shp2_pch_rad:get())
         SHIP_PCH_ALT:set(shp2_pch_alt:get())
         SHIP_KOZ_CW:set(shp2_koz_cw:get())
         SHIP_KOZ_CCW:set(shp2_koz_ccw:get())
         SHIP_KOZ_RAD:set(shp2_koz_rad:get())
         SHIP_KOZ_DKR:set(shp2_koz_dkr:get())
         FOLL_OFS_X:set(shp2_ofs_x:get())
         FOLL_OFS_Y:set(shp2_ofs_y:get())
         FOLL_OFS_Z:set(shp2_ofs_z:get())
         FOLL_SYSID:set(shp2_sysid:get())
      end
    else
      if ship_selected ~= 1 then
         gcs:send_text(0, "Ship 1 for approach")
         ship_selected = 1
         SHIP_PCH_ANG:set(shp1_pch_ang:get())
         SHIP_PCH_RAD:set(shp1_pch_rad:get())
         SHIP_PCH_ALT:set(shp1_pch_alt:get())
         SHIP_KOZ_CW:set(shp1_koz_cw:get())
         SHIP_KOZ_CCW:set(shp1_koz_ccw:get())
         SHIP_KOZ_RAD:set(shp1_koz_rad:get())
         SHIP_KOZ_DKR:set(shp1_koz_dkr:get())
         FOLL_OFS_X:set(shp1_ofs_x:get())
         FOLL_OFS_Y:set(shp1_ofs_y:get())
         FOLL_OFS_Z:set(shp1_ofs_z:get())
         FOLL_SYSID:set(shp1_sysid:get())
      end
    end
    return update, interval_ms
end

return update() -- run immediately before starting to reschedule