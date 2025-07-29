-- config_profiles.lua
-- Continuously watches CFG_* parameters and applies associated parameter sets

-- all_params must contain default values for any parameter present in
-- the parameter lists.  When switching between domain selections
-- these values are used if the domain doesn't have one.  The same
-- parameter must not exist in multiple domains!


-- TODO:
-- emit profiles being used once every 60 seconds or so

-- roll in the parameter whitelist script (so one script not two)
-- Backport to the Callisto Branch: https://github.com/lthall/Leonard_ardupilot/commits/20250702_Callisto_4.5.7-9_dev/

-- far flung future - change parameters on peripherals too

gcs:send_text(6, string.format("CFG: config_profiles v0.2 starting"))

local SEL_APPLY_DEFAULTS = 0
local SEL_DO_NOTHING = -1

-- This is a marker for the start of the config_domains; it is used to swap these out for CI testing
local must_be_set = "must be set"

local config_domains = {
   JOB = {
      param_name = "JOB",
      param_sel_index = 6,
      all_param_defaults = {
         ["FLTMODE1"] = must_be_set,
         ["FLTMODE2"] = must_be_set,
         ["FLTMODE3"] = must_be_set,
         ["FLTMODE4"] = must_be_set,
         ["FLTMODE5"] = must_be_set,
         ["FLTMODE6"] = must_be_set,
         ["LOG_BITMASK"] = 180223,
      },
      default_sel_value = 1,
      profiles = {
         [1] = {
            name = "DeliveryDefault",
            params = {
               ["FLTMODE1"] = 2,
               ["FLTMODE2"] = 2,
               ["FLTMODE3"] = 5,
               ["FLTMODE4"] = 5,
               ["FLTMODE5"] = 6,
               ["FLTMODE6"] = 6,
               ["LOG_BITMASK"] = 180222,
            }
         },
         [2] = {
            name = "FSO",
            params = {
               ["FLTMODE1"] = 5,
               ["FLTMODE2"] = 5,
               ["FLTMODE3"] = 3,
               ["FLTMODE4"] = 3,
               ["FLTMODE5"] = 6,
               ["FLTMODE6"] = 6,
            },
         },
         [3] = {
            name = "Testing",
            params = {
               ["FLTMODE1"] = 0,
               ["FLTMODE2"] = 0,
               ["FLTMODE3"] = 2,
               ["FLTMODE4"] = 2,
               ["FLTMODE5"] = 5,
               ["FLTMODE6"] = 5,
            },
         },
      },
   },
   ARMS = {
      param_name = "ARMS",
      param_sel_index = 5,
      all_param_defaults = {  -- all parameters present in the params for each option
         ["ACRO_BAL_PITCH"] = 1,
         ["ACRO_BAL_ROLL"] = 1,
         ["ACRO_OPTIONS"] = 0,
         ["ACRO_RP_EXPO"] = 0.3,
         ["ACRO_RP_RATE"] = 360,
         ["ACRO_RP_RATE_TC"] = 0.25,
         ["ACRO_THR_MID"] = 0,
         ["ACRO_TRAINER"] = 2,
         ["ACRO_Y_EXPO"] = 0,
         ["ACRO_Y_RATE"] = 90,
         ["ACRO_Y_RATE_TC"] = 0.25,
         ["ANGLE_MAX"] = 3000,
         ["ATC_ACCEL_P_MAX"] = 30000,
         ["ATC_ACCEL_R_MAX"] = 30000,
         ["ATC_ACCEL_Y_MAX"] = 6000,
         ["ATC_ANGLE_BOOST"] = 1,
         ["ATC_ANG_LIM_TC"] = 1,
         ["ATC_ANG_PIT_P"] = 7,
         ["ATC_ANG_RLL_P"] = 7,
         ["ATC_ANG_YAW_P"] = 5,
         ["ATC_INPUT_TC"] = 0.25,
         ["ATC_RATE_FF_ENAB"] = 1,
         ["ATC_RATE_P_MAX"] = 150,
         ["ATC_RATE_R_MAX"] = 150,
         ["ATC_RATE_Y_MAX"] = 0,
         ["ATC_RAT_PIT_D"] = 0.005,
         ["ATC_RAT_PIT_FF"] = 0,
         ["ATC_RAT_PIT_FLTD"] = must_be_set,
         ["ATC_RAT_PIT_FLTE"] = 0,
         ["ATC_RAT_PIT_FLTT"] = 20,
         ["ATC_RAT_PIT_I"] = must_be_set,
         ["ATC_RAT_PIT_IMAX"] = 0.5,
         ["ATC_RAT_PIT_P"] = must_be_set,
         ["ATC_RAT_PIT_SMAX"] = 50,
         ["ATC_RAT_RLL_D"] = must_be_set,
         ["ATC_RAT_RLL_FF"] = 0,
         ["ATC_RAT_RLL_FLTD"] = must_be_set,
         ["ATC_RAT_RLL_FLTE"] = 0,
         ["ATC_RAT_RLL_FLTT"] = 20,
         ["ATC_RAT_RLL_I"] = must_be_set,
         ["ATC_RAT_RLL_IMAX"] = 0.5,
         ["ATC_RAT_RLL_P"] = must_be_set,
         ["ATC_RAT_RLL_SMAX"] = 50,
         ["ATC_RAT_YAW_D"] = 0.02,
         ["ATC_RAT_YAW_FF"] = 0,
         ["ATC_RAT_YAW_FLTD"] = 4,
         ["ATC_RAT_YAW_FLTE"] = 2,
         ["ATC_RAT_YAW_FLTT"] = 20,
         ["ATC_RAT_YAW_I"] = 0.05,
         ["ATC_RAT_YAW_IMAX"] = 0.5,
         ["ATC_RAT_YAW_P"] = 0.5,
         ["ATC_RAT_YAW_SMAX"] = 50,
         ["ATC_SLEW_YAW"] = 6000,
--         ["ATC_THR_G_BOOST"] = 0,
         ["ATC_THR_MIX_MAN"] = 0.1,
         ["ATC_THR_MIX_MAX"] = 2,
         ["ATC_THR_MIX_MIN"] = 0.1,
         ["AUTOTUNE_AGGR"] = 0.1,
         ["AUTOTUNE_AXES"] = 3,
         ["AUTOTUNE_MIN_D"] = 0.001,
         ["INS_ACCEL_FILTER"] = 10,
         ["INS_GYRO_FILTER"] = must_be_set,
         ["INS_HNTC2_ATT"] = 80,
         ["INS_HNTC2_BW"] = must_be_set,
         ["INS_HNTC2_ENABLE"] = 1,
         ["INS_HNTC2_FM_RAT"] = 1,
         ["INS_HNTC2_FREQ"] = must_be_set,
         ["INS_HNTC2_HMNCS"] = 1,
         ["INS_HNTC2_MODE"] = 0,
         ["INS_HNTC2_OPTS"] = 16,
         ["INS_HNTC2_REF"] = 0,
         ["INS_HNTCH_ATT"] = must_be_set,
         ["INS_HNTCH_BW"] = must_be_set,
         ["INS_HNTCH_ENABLE"] = 1,
         ["INS_HNTCH_FM_RAT"] = must_be_set,
         ["INS_HNTCH_FREQ"] = must_be_set,
         ["INS_HNTCH_HMNCS"] = must_be_set,
         ["INS_HNTCH_MODE"] = 1,
         ["INS_HNTCH_OPTS"] = 16,
         ["INS_HNTCH_REF"] = must_be_set,
         ["MOT_BAT_CURR_MAX"] = must_be_set,
         ["MOT_BAT_CURR_TC"] = 2,
         ["MOT_BAT_IDX"] = 0,
         ["MOT_BAT_VOLT_MAX"] = 50.4,
         ["MOT_BAT_VOLT_MIN"] = 39.6,
         ["MOT_BOOST_SCALE"] = 0,
         ["MOT_HOVER_LEARN"] = 0,
         ["MOT_PWM_MAX"] = 1900,
         ["MOT_PWM_MIN"] = 1100,
         ["MOT_PWM_TYPE"] = 0,
         ["MOT_SAFE_DISARM"] = 0,
         ["MOT_SLEW_DN_TIME"] = 0,
         ["MOT_SLEW_UP_TIME"] = 0,
         ["MOT_SPIN_ARM"] = 0.075,
         ["MOT_SPIN_MAX"] = 1,
         ["MOT_SPIN_MIN"] = 0.11,
         ["MOT_SPOOL_TIME"] = 0.5,
         ["MOT_THST_EXPO"] = 0.5,
         ["MOT_THST_HOVER"] = must_be_set,
         ["MOT_YAW_HEADROOM"] = 50,
         ["PILOT_ACCEL_Z"] = 250,
         ["PILOT_SPEED_DN"] = 0,
         ["PILOT_SPEED_UP"] = 250,
         ["PILOT_THR_BHV"] = 3,
         ["PILOT_THR_FILT"] = 0,
         ["PILOT_TKOFF_ALT"] = 100,
         ["PILOT_Y_EXPO"] = 0,
         ["PILOT_Y_RATE"] = 60,
         ["PILOT_Y_RATE_TC"] = 0.25,
         ["PSC_ACCZ_D"] = 0,
         ["PSC_ACCZ_FF"] = 0,
         ["PSC_ACCZ_FLTD"] = 10,
         ["PSC_ACCZ_FLTE"] = 0,
         ["PSC_ACCZ_FLTT"] = 10,
         ["PSC_ACCZ_I"] = 0.5,
         ["PSC_ACCZ_IMAX"] = 800,
         ["PSC_ACCZ_P"] = 0.25,
         ["PSC_ACCZ_SMAX"] = 0,
         ["PSC_ANGLE_MAX"] = 45,
         ["PSC_JERK_XY"] = 5,
         ["PSC_JERK_Z"] = 10,
         ["PSC_POSXY_P"] = 1,
         ["PSC_POSZ_P"] = 0.5,
         ["PSC_VELXY_D"] = 0.25,
         ["PSC_VELXY_FF"] = 0,
         ["PSC_VELXY_FLTD"] = 5,
         ["PSC_VELXY_FLTE"] = 5,
         ["PSC_VELXY_I"] = 1,
         ["PSC_VELXY_IMAX"] = 1000,
         ["PSC_VELXY_P"] = 2,
         ["PSC_VELZ_D"] = 0,
         ["PSC_VELZ_FF"] = 0,
         ["PSC_VELZ_FLTD"] = 5,
         ["PSC_VELZ_FLTE"] = 5,
         ["PSC_VELZ_I"] = 0,
         ["PSC_VELZ_IMAX"] = 1000,
         ["PSC_VELZ_P"] = 2.5,
         ["RTL_ALT"] = 2000,
         ["RTL_ALT_FINAL"] = 0,
         ["RTL_ALT_TYPE"] = 0,
         ["RTL_CLIMB_MIN"] = 0,
         ["RTL_CONE_SLOPE"] = 3,
         ["RTL_LOIT_TIME"] = 1000,
         ["RTL_OPTIONS"] = 0,
         ["RTL_SPEED"] = 0,
         ["WPNAV_ACCEL"] = 250,
         ["WPNAV_ACCEL_C"] = 0,
         ["WPNAV_ACCEL_Z"] = 100,
         ["WP_NAVALT_MIN"] = 0,
         ["WPNAV_JERK"] = 1,
         ["WPNAV_RADIUS"] = 200,
         ["WPNAV_RFND_USE"] = 0,
         ["WPNAV_SPEED"] = 1500,
         ["WPNAV_SPEED_DN"] = 150,
         ["WPNAV_SPEED_UP"] = 250,
         ["WPNAV_TER_MARGIN"] = 10,
         ["WP_YAW_BEHAVIOR"] = 1,
         ["WVANE_ENABLE"] = 0,
      },
      default_sel_value = 50,
      profiles = {
         [50] = {
            name = "Callisto 50",
            params = {
               ["ATC_RAT_PIT_D"] = 0.0055,
               ["ATC_RAT_PIT_FLTD"] = 10,
               ["ATC_RAT_PIT_I"] = 0.14,
               ["ATC_RAT_PIT_P"] = 0.14,
               ["ATC_RAT_RLL_D"] = 0.0055,
               ["ATC_RAT_RLL_FLTD"] = 10,
               ["ATC_RAT_RLL_I"] = 0.14,
               ["ATC_RAT_RLL_P"] = 0.14,
               ["INS_GYRO_FILTER"] = 40,
               ["INS_HNTC2_BW"] = 45,
               ["INS_HNTC2_FREQ"] = 45,
               ["INS_HNTCH_ATT"] = 80,
               ["INS_HNTCH_BW"] = 15,
               ["INS_HNTCH_FM_RAT"] = 0.9,
               ["INS_HNTCH_FREQ"] = 30,
               ["INS_HNTCH_HMNCS"] = 11,
               ["INS_HNTCH_REF"] = 0.22,
               ["MOT_BAT_CURR_MAX"] = 240,
               ["MOT_THST_HOVER"] = 0.35,
            },
         },
         [25] = {
            name = "Callisto 25",
            params = {
               ["ATC_RAT_PIT_FLTD"] = 15,
               ["ATC_RAT_PIT_I"] = 0.1,
               ["ATC_RAT_PIT_P"] = 0.1,
               ["ATC_RAT_RLL_D"] = 0.005,
               ["ATC_RAT_RLL_FLTD"] = 15,
               ["ATC_RAT_RLL_I"] = 0.1,
               ["ATC_RAT_RLL_P"] = 0.1,
               ["INS_GYRO_FILTER"] = 30,
               ["INS_HNTC2_FREQ"] = 30,
               ["INS_HNTC2_BW"] = 20,
               ["INS_HNTCH_ATT"] = 40,
               ["INS_HNTCH_BW"] = 33,
               ["INS_HNTCH_FM_RAT"] = 0.75,
               ["INS_HNTCH_FREQ"] = 49.5,
               ["INS_HNTCH_HMNCS"] = 15,
               ["INS_HNTCH_REF"] = 0.21,
               ["MOT_BAT_CURR_MAX"] = 150,
               ["MOT_THST_HOVER"] = 0.2053075,
            },
         },
      },
   },

   BATTERY = {
      param_name = "BAT",
      param_sel_index = 4,
      all_param_defaults = {
         ["BATT_CAPACITY"] = must_be_set,
         ["BATT_CRT_MAH"] = must_be_set,
         ["BATT_CRT_VOLT"] = must_be_set,
         ["BATT_LOW_MAH"] = must_be_set,
         ["BATT_LOW_VOLT"] = must_be_set,
      },
      default_sel_value = 16,
      profiles = {
         [16] = {
            name = "16Ah",
            params = {
               ["BATT_CAPACITY"] = 32000,
               ["BATT_CRT_MAH"] = 6400,
               ["BATT_CRT_VOLT"] = 44.5,
               ["BATT_LOW_MAH"] = 9600,
               ["BATT_LOW_VOLT"] = 45.0,
            },
         },
         [22] = {
            name = "22Ah",
            params = {
               ["BATT_CAPACITY"] = 44000,
               ["BATT_CRT_MAH"] = 8800,
               ["BATT_CRT_VOLT"] = 43.25,
               ["BATT_LOW_MAH"] = 13200,
               ["BATT_LOW_VOLT"] = 43.75,
            },
         },
         [40] = {
            name = "40Ah",
            params = {
               ["BATT_CAPACITY"] = 80000,
               ["BATT_CRT_MAH"] = 16000,
               ["BATT_CRT_VOLT"] = 43.75,
               ["BATT_LOW_MAH"] = 24000,
               ["BATT_LOW_VOLT"] = 44.25,
            },
         },
      },
   },

   PAY = {
      param_name = "PAY",
      param_sel_index = 3,
      all_param_defaults = {
         ["GRIP_ENABLE"] = 0,
         ["GRIP_GRAB"] = 1000,
         ["GRIP_NEUTRAL"] = 1000,
         ["GRIP_REGRAB"] = 0,
         ["GRIP_RELEASE"] = 2000,
         ["GRIP_TYPE"] = 1,
         ["MNT1_DEFLT_MODE"] = 3,     -- 3 = RC Targeting
         ["MNT1_LEAD_PTCH"] = 0,
         ["MNT1_LEAD_RLL"] = 0,
         ["MNT1_NEUTRAL_X"] = 0,
         ["MNT1_NEUTRAL_Y"] = 0,
         ["MNT1_NEUTRAL_Z"] = 0,
         ["MNT1_PITCH_MAX"] = 20,
         ["MNT1_PITCH_MIN"] = -90,
         ["MNT1_RC_RATE"] = 90,
         ["MNT1_RETRACT_X"] = 0,
         ["MNT1_RETRACT_Y"] = 0,
         ["MNT1_RETRACT_Z"] = 0,
         ["MNT1_ROLL_MAX"] = 30,
         ["MNT1_ROLL_MIN"] = -30,
         ["MNT1_TYPE"] = 0,
         ["MNT1_YAW_MAX"] = 180,
         ["MNT1_YAW_MIN"] = -180,
         ["RC13_OPTION"] = 0,
         ["RC13_REVERSED"] = 0,
         ["RC14_OPTION"] = 0,
         ["RC14_REVERSED"] = 0,
         ["RC8_OPTION"] = 0,        -- disabled
         ["SERIAL5_BAUD"] = 115,      -- 115 = 115200 baud
         ["SERIAL5_OPTIONS"] = 0,
         ["SERIAL5_PROTOCOL"] = 2,    -- 2 = MAVLink2
         ["SERVO10_FUNCTION"] = -1,
         ["SERVO10_MAX"] = 2000,
         ["SERVO10_MIN"] = 1000,
         ["SERVO10_REVERSED"] = 0,
         ["SERVO10_TRIM"] = 1500,
         ["SERVO11_FUNCTION"] = -1,
         ["SERVO12_FUNCTION"] = -1,
         ["SERVO13_FUNCTION"] = -1,
         ["SERVO14_FUNCTION"] = -1,
         ["SERVO9_FUNCTION"] = -1,
         ["SERVO9_MAX"] = 2000,
         ["SERVO9_MIN"] = 1000,
         ["SERVO9_REVERSED"] = 0,
         ["SERVO9_TRIM"] = 1500,
      },
      default_sel_value = SEL_APPLY_DEFAULTS,
      profiles = {
         [0] = {
            name = "Gimbal - Hook Slung",
            params = {
               ["GRIP_ENABLE"] = 1,
               ["RC8_OPTION"] = 19,        -- gripper
               ["SERVO9_FUNCTION"] = 146,  -- k_rcin7_mapped?!
               ["SERVO9_REVERSED"] = 1,
               ["SERVO10_FUNCTION"] = 28,  -- gripper
               ["SERVO10_TRIM"] = 2000,
               ["SERVO11_FUNCTION"] = 64,  -- rcin 14
               ["SERVO12_FUNCTION"] = 63,  -- rcin 13
               ["SERVO13_FUNCTION"] = 61,  -- rcin 11
               ["SERVO14_FUNCTION"] = 56,  -- rcin 6
            }
         },
         [1] = {
            name = "HookFixed",
            params = {
               ["GRIP_ENABLE"] = 1,
               ["RC8_OPTION"] = 19,          -- 19 = Gripper (RC switch triggers grab/release)
               ["SERVO9_FUNCTION"] = 28,     -- 28 = gripper
            }
         },
         [2] = {
            name = "HookSlung",
            params = {
               ["GRIP_ENABLE"] = 1,
               ["RC8_OPTION"] = 19,           -- 19 = Gripper (RC switch triggers grab/release)
               ["SERVO9_FUNCTION"] = 146,     -- 146 = k_rcin7_mapped?!
               ["SERVO9_REVERSED"] = 1,
               ["SERVO10_FUNCTION"] = 28,     -- 28 = gripper
            },
         },
         [3] = {
            name = "Gimbal - AVT_CM62",
            params = {
               ["MNT1_TYPE"] = 6,           -- 6 = Mount type MAVLinkv2 (or Custom)
               ["RC13_OPTION"] = 214,       -- 214 = Mount Yaw
               ["RC14_OPTION"] = 213,       -- 213 = Mount Pitch
            },
         },
         [4] = {
            name = "Gimbal - HookFixed",
            params = {
               ["GRIP_ENABLE"] = 1,
               ["RC8_OPTION"] = 19,            -- 19 = Gripper

               ["SERVO9_FUNCTION"] = 28,       -- 28 = Mount Shutter (e.g. camera trigger)
               ["SERVO10_FUNCTION"] = 64,      -- 64 = Landing Gear
               ["SERVO11_FUNCTION"] = 63,      -- 63 = Mount Tilt
               ["SERVO12_FUNCTION"] = 62,      -- 62 = Mount Roll
               ["SERVO13_FUNCTION"] = 58,      -- 58 = Mount Zoom
               ["SERVO14_FUNCTION"] = 57,      -- 57 = Mount Focus
            },
         },
      },
   },
}
-- This is a marker for the end of the config_domains; it is used to swap these out for CI testing

local msg_pfx = "CFG: "

-- set up for denying arming when problems occur:
local auth_id = arming:get_aux_auth_id() or 0

local function set_aux_auth_failed(reason)
   arming:set_aux_auth_failed(auth_id, msg_pfx .. reason)
end

local function set_aux_auth_passed()
   arming:set_aux_auth_passed(auth_id)
end

set_aux_auth_failed("Validation pending")

local function send_text(severity, message)
   gcs:send_text(severity, msg_pfx .. message)
end

local function validate_unique_parameters_across_domains()
   -- ensure that no parameter exists in multiple domains:
   local all_domain_parameters = {}
   for _, domain in pairs(config_domains) do
      for param_name, _ in pairs(domain.all_param_defaults) do
         local first_domain_name = all_domain_parameters[param_name]
         if first_domain_name ~= nil then
            send_text(3, string.format("%s exists in two domains (%s and %s)", param_name, first_domain_name, domain.param_name))
            return false
         end
         all_domain_parameters[param_name] = domain.param_name
      end
   end
   return true
end

local function validate_params_in_param_defaults()
   -- ensure that any parameter in the params tables are also in all_params.
   -- Also ensure that it doesn't have the same value as the default
   -- also ensure all defaults are over-ridden in one of the profiles
   for _, domain in pairs(config_domains) do
--      local default_overridden = {}
      for profile_num, profile in pairs(domain.profiles) do
         for param_name, param_value in pairs(profile.params) do
--            default_overridden[param_name] = true
            local param_default = domain.all_param_defaults[param_name]
            if param_default == nil then
               send_text(3, string.format("%s exists in %s[%f](%s) but not in %s.all_param_defaults", param_name, domain.param_name, profile_num, profile.name, domain.param_name))
               return false
            end
            if param_default == param_value then
               send_text(3, string.format("%s in %s[%f](%s) has the same value as %s.all_param_defaults", param_name, domain.param_name, profile_num, profile.name, domain.param_name))
               return false
            end
         end
      end
-- turns out this is a pain as you need to find values to put into all_param_defaults which is annoying:
--      local failed_defaults_used = false
--      for param_name, _ in pairs(domain.all_param_defaults) do
         --if default_overridden[param_name] == nil then
--            send_text(3, string.format("%s.all_param_defaults contains %s but no profile overrides it", domain.param_name, param_name))
--            failed_defaults_used = true
--         end
--      end
--      if failed_defaults_used then
--         return false
--      end
   end
   return true
end

local function validate_must_be_set_parameters()
   -- ensure that if a parameter has a "must be set" value in the
   -- defaults that every domain defines a value for that default
   -- also assert that at least one profile has a different value to the others
   for _, domain in pairs(config_domains) do
      for param_name, value in pairs(domain.all_param_defaults) do
         if value ~= must_be_set then
            goto vmbsp_next_param
         end
         local first_value = nil
         local at_least_two_values = false
         for profile_num, profile in pairs(domain.profiles) do
            param_value = profile.params[param_name]
            if param_value == nil then
               send_text(3, string.format("%s exists in %s.all_param_defaults with must-be-set-value but profile %s(%s) params does not have a value for it", param_name, domain.param_name, profile_num, profile.name))
               return false
            end
            if first_value == nil then
               first_value = param_value
            else
               if param_value ~= first_value then
                  at_least_two_values = true
               end
            end
         end
         if not at_least_two_values then
            send_text(3, string.format("%s in %s is must-be-set but all profiles have the same value", param_name, domain.param_name))
            return false
         end
         ::vmbsp_next_param::
      end
   end
   return true
end

local function validate_domain_attributes()
   -- check direct domain attributes are valid
   for _, domain in pairs(config_domains) do
      -- check default selection is valid
      default_sel_value = domain.default_sel_value
      if default_sel_value == nil then
         goto good_sel_value
      end
      if default_sel_value == SEL_DO_NOTHING or default_sel_value == SEL_APPLY_DEFAULTS then
         goto good_sel_value
      end
      
      sel = domain.profiles[default_sel_value]
      if sel ~= nil then
         goto good_sel_value
      end

      if true then  -- needed to fool lua checker
         send_text(3, string.format("%s.default_sel_value is invalid", domain.param_name))
         return false
      end

      ::good_sel_value::
   end
   return true
end

-- this is a runtime check:
local function validate_apply_defaults_no_must_be_sets()
   -- ensure that if a parameter is marked as must-be-supplied in the
   -- defaults that the domain isn't in "apply defaults" mode
   for _, domain in pairs(config_domains) do
      local sel_value = domain.sel_param:get()
      if sel_value == nil then
         send_text(3, string.format("Bad sel parameter for %s", domain.param_name))
         return false
      end
      if sel_value ~= SEL_APPLY_DEFAULTS then
         goto vadnn_next_domain
      end
      for param_name, value in pairs(domain.all_param_defaults) do
         if value == must_be_set then
            send_text(3, string.format("%s mode is set-values-from-defaults but %s is marked must-be-set in %s.all_param_defaults", domain.param_name, param_name, domain.param_name))
            return false
         end
      end
      ::vadnn_next_domain::
   end
   return true
end

-- validate_config_domains - step through the domain configuration
-- structure and ensure that no mistake has been made in the setup of
-- the configuration sets
local function validate_config_domains()
   validators = {
      validate_unique_parameters_across_domains,
      validate_params_in_param_defaults,
      validate_must_be_set_parameters,
      validate_apply_defaults_no_must_be_sets,
      validate_domain_attributes,
   }
   for _, validator in pairs(validators) do
      if validator == nil then
         set_aux_auth_failed("Invalid validator")
         return false
      end
      if not validator() then
         set_aux_auth_failed("Domain configuration invalid")
         return false
      end
   end
   return true
end

-- parameter setup
--
local PARAM_TABLE_KEY = 10
local PARAM_TABLE_PREFIX = "CFG_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 16), 'could not add param table')

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: CFG_ENABLE
  // @DisplayName: Enable configuration script
  // @Description: Script immediately exits if this is zero
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local PARAM_SET_ENABLE = bind_add_param("ENABLE", 1, 1)

local function add_param_for_domain(domain, index, name, default)
   pname = domain.param_name .. "_" .. name
   -- send_text(6, string.format("Registering %s at index=%s with default %s", pname, index, default))
   return bind_add_param(pname, index, default)
end

for _, domain in pairs(config_domains) do
   domain_sel_default = domain.default_sel_value
   if domain_sel_default == nil then
      domain_sel_default = 0
   end
   domain.sel_param = add_param_for_domain(domain, domain.param_sel_index, "SEL", domain_sel_default)
end

local a_parameter_was_ever_set = false


-- utility: apply one profile
local function apply_parameters(domain, params)
   for param_name, new_value in pairs(domain.all_param_defaults) do
      local profile_param_value = params[param_name]
      if profile_param_value ~= nil then
         new_value = profile_param_value
      end
      if new_value == nil then
         send_text(3, string.format("Validation code has failed as %s has nil value", param_name))
         return
      end
      if new_value == must_be_set then
         send_text(3, string.format("Validation code has failed as %s is must_be_set", param_name))
         return
      end
      local old_value = param:get(param_name)
      if old_value == nil then
         send_text(3, string.format("Unable to fetch parameter %s", param_name))
         return
      end
      if old_value ~= new_value then
         send_text(6, string.format("Set %s=%s (was %.3f)", param_name, new_value, old_value))
         param:set_and_save(param_name, new_value)
         a_parameter_was_ever_set = true
      end
   end
end

-- check each domain configuration parameter, act if any has changed
local function handle_domains()
   local success = true
   for _, domain in pairs(config_domains) do
      local sel_value = domain.sel_param:get()
      local sel_value_changed = false
      if sel_value ~= domain.last_sel_value then
         domain.last_sel_value = sel_value
         sel_value_changed = true
      end

      if sel_value == SEL_DO_NOTHING then
         if sel_value_changed then
            send_text(6, string.format("Doing nothing for %s", domain.param_name))
         end
         goto cd_next_domain
      end

      if sel_value == SEL_APPLY_DEFAULTS then
         if sel_value_changed then
            send_text(6, string.format("Applying %s defaults", domain.param_name))
         end
         apply_parameters(domain, {})
         goto cd_next_domain
      end

      local profile = domain.profiles[sel_value]
      if profile == nil then
         set_aux_auth_failed(string.format("Invalid profile selected for %s", domain.param_name))
         success = false
         goto cd_next_domain
      end

      if domain.last_sel_value ~= sel_value or sel_value_changed then
         domain.last_sel_value = sel_value
         send_text(6, string.format("Applying %s profile: %s", domain.param_name, profile.name))
      end
      apply_parameters(domain, profile.params)
      goto cd_next_domain

      ::cd_next_domain::
   end
   if a_parameter_was_ever_set then
      set_aux_auth_failed("Reboot needed for config change")
   elseif success then
      set_aux_auth_passed()
   end
end

-- update function
local domains_valid = false
function update()
   if PARAM_SET_ENABLE:get() == 0 then
      -- permanently exit
      send_text(3, string.format("exitting"))
      return
   end

   -- do nothing if armed:
   if arming:is_armed() then
      return update, 1000
   end

   if not domains_valid then
      if validate_config_domains() then
         domains_valid = true
         set_aux_auth_failed("Profile value validation pending")
      else
         return update, 10000  -- rerun periodically to make the problem clear
      end
   end

   handle_domains()
   return update, 1000  -- recheck every second
end

return update()
