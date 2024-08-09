-- add new param SHP1_
local ship_selected = 0
local PARAM_TABLE_KEY1 = 20
local PARAM_TABLE_PREFIX1 = "SHP1_"
local PARAM_TABLE_KEY2 = 21
local PARAM_TABLE_PREFIX2 = "SHP2_"

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value, key, prefix)
   assert(param:add_param(key, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(prefix .. name)
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

local SHIP_PCH_ANG = Parameter('SHIP_PCH_ANG')
local SHIP_PCH_RAD = Parameter('SHIP_PCH_RAD')
local SHIP_PCH_ALT = Parameter('SHIP_PCH_ALT')
local SHIP_KOZ_CW = Parameter('SHIP_KOZ_CW')
local SHIP_KOZ_CCW = Parameter('SHIP_KOZ_CCW')
local SHIP_KOZ_RAD = Parameter('SHIP_KOZ_RAD')
local SHIP_KOZ_DKR = Parameter('SHIP_KOZ_DKR')

local FOLL_OFS_X = Parameter('FOLL_OFS_X')
local FOLL_OFS_Y = Parameter('FOLL_OFS_Y')
local FOLL_OFS_Z = Parameter('FOLL_OFS_Z')

local FOLL_SYSID = Parameter('FOLL_SYSID')

-- Use RCx_OPTION 300: Scripting1
local AuxFunScripting1 = 300
local AuxSwitchPos = { LOW = 0, MIDDLE = 1, HIGH = 2 }

local interval_ms = 1000     -- update at 1hz
gcs:send_text(0, "Starting SHIP OPS LUA")
function update() -- this is the loop which periodically runs

   local aux = rc:get_aux_cached(AuxFunScripting1)
   if (aux == nil) or (aux == AuxSwitchPos.LOW) then
      -- If aux has never been set, or aux is low then use ship 2
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
