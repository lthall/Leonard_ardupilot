-- A Lua script for switching between parameter banks with checking and runtime updates
-- Select bank with RCx_OPTION 300

local bank_target = {
   { name = "HTL_ANG", value = Parameter('SHIP_HTL_ANG'), bank_defualt = 180 },
   { name = "HTL_RAD", value = Parameter('SHIP_HTL_RAD'), bank_defualt = 25 },
   { name = "HTL_ALT", value = Parameter('SHIP_HTL_ALT'), bank_defualt = 25 },
   { name = "KOZ_CW", value = Parameter('SHIP_KOZ_CW'), bank_defualt = 90 },
   { name = "KOZ_CCW", value = Parameter('SHIP_KOZ_CCW'), bank_defualt = -90 },
   { name = "KOZ_RAD", value = Parameter('SHIP_KOZ_RAD'), bank_defualt = 250 },
   { name = "KOZ_DKR", value = Parameter('SHIP_KOZ_DKR'), bank_defualt = 10 },

   { name = "OFS_X", value = Parameter('FOLL_OFS_X'), bank_defualt = 0 },
   { name = "OFS_Y", value = Parameter('FOLL_OFS_Y'), bank_defualt = 0 },
   { name = "OFS_Z", value = Parameter('FOLL_OFS_Z'), bank_defualt = 0 },

   { name = "SYSID", value = Parameter('FOLL_SYSID'), bank_defualt = 200 }
}

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value, key, prefix)
   assert(param:add_param(key, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(prefix .. name)
end

local function add_param_bank(target_params, table_key, prefix, bank_name, extra_msg, defualt_override)

   if extra_msg == nil then
      extra_msg = ""
   end

   local self = {
      params = {},
      active = false,
      checked = false,
   }

   -- Add params table with the given key and prefix
   assert(param:add_table(table_key, prefix, #target_params), 'could not add param \"' .. prefix .. '\" table for ' .. bank_name .. 'param bank')

   -- Add copy of each param
   for i, param in pairs(target_params) do
      local default_value = param.bank_defualt
      if (defualt_override ~= nil) and (defualt_override[param.name] ~= nil) then
         default_value = defualt_override[param.name]
      end

      self.params[i] = {}
      self.params[i].value = bind_add_param(param.name, i, default_value, table_key, prefix)
      self.params[i].name = prefix .. param.name
      self.params[i].last_set_value = self.params[i].value:get()
   end

   -- Set target param to bank value for given index
   local function set(i)
      local new_value = self.params[i].value:get()
      target_params[i].value:set(new_value)
      self.params[i].last_set_value = new_value
   end

   -- Bank update function, pass true if selected
   function self.update(b)
      if not b then
         -- De-select
         self.active = false
         self.checked = false
         return
      end

      if not self.active then
         -- Initial set
         for i = 1,#target_params do
            set(i)
         end
         gcs:send_text(0, bank_name .. " parameters selected" .. extra_msg)

         self.active = true
         self.checked = false
         return
      end

      if not self.checked then
         local all_checked = true
         -- Check each param value is correct
         for i = 1,#target_params do
            if target_params[i].value:get() ~= self.params[i].value:get() then
               -- Value not set, try again
               all_checked = false
               set(i)
            end
         end
         if all_checked then
            -- Everything set correctly
            gcs:send_text(0, bank_name .. " parameters checked")
            self.checked = true
         end
         return
      end

      -- Update target param for runtime changes of bank
      for i = 1,#target_params do
         if self.params[i].last_set_value ~= self.params[i].value:get() then
            set(i)
            gcs:send_text(0, bank_name .. " parameters " .. self.params[i].name .. " updated")
         end
      end

   end

   return self
end

local banks = {
   add_param_bank(bank_target, 20, "SHP1_", "Ship 1", ""),
   add_param_bank(bank_target, 21, "SHP2_", "Ship 2", "", { SYSID = 201 })
}

-- Use RCx_OPTION 300: Scripting1
-- local AuxFunChOption = 300
-- Use RCx_OPTION 180: SHIP_OPS_MODE
local AuxFunChOption = 180
local AuxSwitchPos = { LOW = 0, MIDDLE = 1, HIGH = 2 }

local interval_ms = 1000     -- update at 1hz
gcs:send_text(0, "Starting SHIP OPS LUA")

local function update() -- this is the loop which periodically runs
   local bank_selected = 1

   local aux = rc:get_aux_cached(AuxFunChOption)
   if (aux ~= nil) and (aux ~= AuxSwitchPos.MIDDLE) then
      -- If aux has never been set, or aux is middle then do nothing
      if (aux == AuxSwitchPos.LOW) then
         bank_selected = 2
      end
      for i = 1,#banks do
         banks[i].update(i == bank_selected)
      end
   end

   return update, interval_ms
end

return update() -- run immediately before starting to reschedule
