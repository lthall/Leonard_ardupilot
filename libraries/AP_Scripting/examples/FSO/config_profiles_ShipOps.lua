-- config_profiles.lua
-- Continuously watches CFG_* parameters and applies associated parameter sets

-- all_params must contain default values for any parameter present in
-- the parameter lists.  When switching between domain selections
-- these values are used if the domain doesn't have one.  The same
-- parameter must not exist in multiple domains!


-- TODO:
--   far flung future - change parameters on peripherals too


gcs:send_text(6, string.format("CFG: Ship Ops config_profiles v1.4 starting"))

local SEL_APPLY_DEFAULTS = 0
local SEL_DO_NOTHING = -1

local must_be_set = "must be set"

-- This is a marker for the start of the config_domains; it is used to swap these out for CI testing

-- a table of param indexes to help avoid annoying conflicts:
local param_index = {
   ["enable"]    =  1,
   ["pm_filter"] =  2,

   ["JOB"]       = 10,
   ["ARMS"]      = 11,
   ["BAT"]       = 12,
   ["PAY"]       = 13,
}

local config_domains = {
   JOB = {
      param_name = "JOB",
      all_param_defaults = {
         -- Flight Modes
         ["FLTMODE_CH"] = 5,
         ["FLTMODE1"] = 2,
         ["FLTMODE2"] = 2,
         ["FLTMODE3"] = 5,
         ["FLTMODE4"] = 5,
         ["FLTMODE5"] = 6,
         ["FLTMODE6"] = 6,
         ["FLTMODE_GCSBLOCK"] = 12232576,

         -- Flight Behavior
         ["ANGLE_MAX"] = 3000,
         ["ATC_SLEW_YAW"] = 6000,

         -- Geofence
         ["FENCE_ACTION"] = 1,
         ["FENCE_ALT_MAX"] = 100,
         ["FENCE_ALT_MIN"] = -10,
         ["FENCE_ENABLE"] = 1,
         ["FENCE_MARGIN"] = 2,
         ["FENCE_RADIUS"] = 300,
         ["FENCE_TOTAL"] = 0,
         ["FENCE_TYPE"] = 7,

         -- Failsafes
         ["FS_CRASH_CHECK"] = 1,
         ["FS_DR_ENABLE"] = 2,
         ["FS_DR_TIMEOUT"] = 30,
         ["FS_EKF_ACTION"] = 1,
         ["FS_EKF_FILT"] = 5,
         ["FS_EKF_THRESH"] = 1,
         ["FS_GCS_ENABLE"] = 1,
         ["FS_GCS_TIMEOUT"] = 5,
         ["FS_OPTIONS"] = 16,
         ["FS_THR_ENABLE"] = 1,
         ["FS_THR_VALUE"] = 975,
         ["FS_VIBE_ENABLE"] = 1,

         -- Battery Failsafes
         ["BATT_FS_CRT_ACT"] = 1,
         ["BATT_FS_LOW_ACT"] = 2,

         -- IMU Logging
         ["INS_FAST_SAMPLE"] = 7,
         ["INS_LOG_BAT_CNT"] = 1024,
         ["INS_LOG_BAT_MASK"] = 0,
         ["INS_LOG_BAT_OPT"] = 0,

         -- Landing
         ["LAND_ALT_LOW"] = 1000,
         ["LAND_REPOSITION"] = 1,
         ["LAND_SPEED"] = 50,
         ["LAND_SPEED_HIGH"] = 0,

         -- Logging
         ["LOG_BITMASK"] = 180222,
         ["LOG_FILE_DSRMROT"] = 0,

         -- Loiter Mode
         ["LOIT_ACC_MAX"] = 500,
         ["LOIT_ANG_MAX"] = 25,
         ["LOIT_BRK_ACCEL"] = 250,
         ["LOIT_BRK_DELAY"] = 1,
         ["LOIT_BRK_JERK"] = 500,
         ["LOIT_SPEED"] = 1250,

         -- Pilot Input Tuning
         ["PHLD_BRAKE_ANGLE"] = 3000,
         ["PHLD_BRAKE_RATE"] = 8,
         ["PILOT_ACCEL_Z"] = 250,
         ["PILOT_SPEED_DN"] = 0,
         ["PILOT_SPEED_UP"] = 250,
         ["PILOT_THR_BHV"] = 3,
         ["PILOT_THR_FILT"] = 0,
         ["PILOT_TKOFF_ALT"] = 300,
         ["PILOT_Y_EXPO"] = 0,
         ["PILOT_Y_RATE"] = 60,
         ["PILOT_Y_RATE_TC"] = 0.25,

         -- Trajectory Shaping
         ["PSC_ANGLE_MAX"] = 45,
         ["PSC_JERK_XY"] = 5,
         ["PSC_JERK_Z"] = 10,

         -- RTL Behavior
         ["RTL_ALT_FINAL"] = 0,
         ["RTL_ALT_TYPE"] = 0,
         ["RTL_CLIMB_MIN"] = 0,
         ["RTL_CONE_SLOPE"] = 3,
         ["RTL_LOIT_TIME"] = 1000,
         ["RTL_OPTIONS"] = 0,
         ["RTL_SPEED"] = 0,
         
         -- Takeoff
         ["TKOFF_SLEW_TIME"] = 4,

         -- Terrain Following
         ["TERRAIN_ENABLE"] = 1,
         ["TERRAIN_MARGIN"] = 0.05,
         ["TERRAIN_OFS_MAX"] = 15,
         ["TERRAIN_OPTIONS"] = 0,
         ["TERRAIN_SPACING"] = 100,

         -- Waypoint Navigation
         ["WP_NAVALT_MIN"] = 0,
         ["WP_YAW_BEHAVIOR"] = 1,
         ["WPNAV_ACCEL"] = 250,
         ["WPNAV_ACCEL_C"] = 0,
         ["WPNAV_ACCEL_Z"] = 100,
         ["WPNAV_JERK"] = 1,
         ["WPNAV_RADIUS"] = 200,
         ["WPNAV_RFND_USE"] = 0,
         ["WPNAV_SPEED"] = 1500,
         ["WPNAV_SPEED_DN"] = 150,
         ["WPNAV_SPEED_UP"] = 250,
         ["WPNAV_TER_MARGIN"] = 10,
      },
      default_sel_value = SEL_APPLY_DEFAULTS,
      profiles = {
         [100] = {
            name = "Testing",
            params = {
               ["ANGLE_MAX"] = 4500,
               ["FLTMODE_CH"] = 5,
               ["FLTMODE1"] = 0,
               ["FLTMODE2"] = 0,
               ["FLTMODE3"] = 2,
               ["FLTMODE4"] = 2,
               ["FLTMODE5"] = 5,
               ["FLTMODE6"] = 5,
               ["LOG_BITMASK"] = 180223,
            },
         },
         [101] = {
            name = "FSO",
            params = {
               ["PSC_ANGLE_MAX"] = 2500,
               ["FLTMODE_CH"] = 5,
               ["FLTMODE1"] = 5,
               ["FLTMODE2"] = 5,
               ["FLTMODE3"] = 3,
               ["FLTMODE4"] = 3,
               ["FLTMODE5"] = 6,
               ["FLTMODE6"] = 6,
            },
         },
      },
   },
   ARMS = {
      param_name = "ARMS",
      all_param_defaults = {  -- all parameters present in the params for each option
         ["ATC_ACCEL_P_MAX"] = 30000,
         ["ATC_ACCEL_R_MAX"] = 30000,
         ["ATC_ACCEL_Y_MAX"] = 6000,
         ["ATC_ANGLE_BOOST"] = 1,
         ["ATC_ANG_LIM_TC"] = 1,
         ["ATC_ANG_PIT_P"] = 7,
         ["ATC_ANG_RLL_P"] = 7,
         ["ATC_ANG_YAW_P"] = 5,
         ["ATC_INPUT_TC"] = 0.25,
         ["ATC_LAND_P_MULT"] = 0.5,
         ["ATC_LAND_R_MULT"] = 0.5,
         ["ATC_LAND_Y_MULT"] = 0.5,
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
         ["ATC_THR_MIX_MAN"] = 0.1,
         ["ATC_THR_MIX_MAX"] = 2,
         ["ATC_THR_MIX_MIN"] = 0.1,
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
         ["PSC_ACCZ_D"] = 0,
         ["PSC_ACCZ_FF"] = 0,
         ["PSC_ACCZ_FLTD"] = 10,
         ["PSC_ACCZ_FLTE"] = 0,
         ["PSC_ACCZ_FLTT"] = 10,
         ["PSC_ACCZ_I"] = 0.5,
         ["PSC_ACCZ_IMAX"] = 800,
         ["PSC_ACCZ_P"] = 0.25,
         ["PSC_ACCZ_SMAX"] = 0,
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
         [75] = {
            name = "Callisto 75",
            params = {
               ["ATC_ACCEL_P_MAX"] = 25000,
               ["ATC_ACCEL_R_MAX"] = 25000,
               ["ATC_ANG_YAW_P"] = 4.0,
               ["ATC_RATE_P_MAX"] = 120,
               ["ATC_RATE_R_MAX"] = 120,
               ["ATC_RAT_PIT_D"] = 0.004,
               ["ATC_RAT_PIT_FLTT"] = 10.0,
               ["ATC_RAT_PIT_FLTD"] = 10.0,
               ["ATC_RAT_PIT_I"] = 0.09,
               ["ATC_RAT_PIT_P"] = 0.09,
               ["ATC_RAT_RLL_D"] = 0.004,
               ["ATC_RAT_RLL_FLTT"] = 10.0,
               ["ATC_RAT_RLL_FLTD"] = 10.0,
               ["ATC_RAT_RLL_I"] = 0.09,
               ["ATC_RAT_RLL_P"] = 0.09,
               ["ATC_RAT_YAW_D"] = 0.01,
               ["ATC_RAT_YAW_I"] = 0.025,
               ["ATC_RAT_YAW_P"] = 0.25,
               ["INS_ACCEL_FILTER"] = 5.0,
               ["INS_GYRO_FILTER"] = 20.0,
               ["INS_HNTC2_BW"] = 10.0,
               ["INS_HNTC2_FREQ"] = 20.0,
               ["INS_HNTC2_HMNCS"] = 3,
               ["INS_HNTC2_OPTS"] = 16,
               ["INS_HNTCH_ATT"] = 80.0,
               ["INS_HNTCH_BW"] = 11.75,
               ["INS_HNTCH_FM_RAT"] = 0.9,
               ["INS_HNTCH_FREQ"] = 23.5,
               ["INS_HNTCH_HMNCS"] = 11.0,
               ["INS_HNTCH_OPTS"] = 20.0,
               ["INS_HNTCH_REF"] = 0.09,
               ["MOT_BAT_CURR_MAX"] = 240.0,
               ["MOT_PWM_MAX"] = 2000.0,
               ["MOT_PWM_MIN"] = 1000.0,
               ["MOT_SPIN_ARM"] = 0.15,
               ["MOT_SPIN_MAX"] = 0.95,
               ["MOT_SPIN_MIN"] = 0.2,
               ["MOT_SPOOL_TIME"] = 2.0,
               ["MOT_THST_EXPO"] = 0.55,
               ["MOT_THST_HOVER"] = 0.2,
               ["PSC_ACCZ_I"] = 0.25,
               ["PSC_ACCZ_P"] = 0.125,
            },
         },
      },
   },

   BAT = {
      param_name = "BAT",
      all_param_defaults = {
         ["BATT_CAPACITY"] = must_be_set,
         ["BATT_CRT_MAH"] = must_be_set,
         ["BATT_CRT_VOLT"] = must_be_set,
         ["BATT_LOW_MAH"] = must_be_set,
         ["BATT_LOW_VOLT"] = must_be_set,
      },
      default_sel_value = 22,
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
         [30] = {
            name = "30Ah",
            params = {
               ["BATT_CAPACITY"] = 60000,
               ["BATT_CRT_MAH"] = 12000,
               ["BATT_CRT_VOLT"] = 43.75,
               ["BATT_LOW_MAH"] = 18000,
               ["BATT_LOW_VOLT"] = 44.25,
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
      all_param_defaults = {
         -- Payload Power supply checks
         ["BATT2_ARM_VOLT"] = 0,
         ["BATT3_ARM_VOLT"] = 0,

         -- Gripper
         ["GRIP_ENABLE"] = 0,
         ["GRIP_GRAB"] = 1000,
         ["GRIP_NEUTRAL"] = 1000,
         ["GRIP_REGRAB"] = 0,
         ["GRIP_RELEASE"] = 2000,
         ["GRIP_TYPE"] = 1,

         -- Mount Control
         ["MNT1_DEFLT_MODE"] = 3,
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

         -- network setup
         ["NET_ENABLE"] = 0,
         ["NET_OPTIONS"] = 0,
         ["NET_P1_IP0"] = 192,
         ["NET_P1_IP1"] = 168,
         ["NET_P1_IP2"] = 111,
         ["NET_P1_IP3"] = 15,
         ["NET_P1_PORT"] = 14550,
         ["NET_P1_PROTOCOL"] = 2,
         ["NET_P1_TYPE"] = 2,
         ["NET_P2_TYPE"] = 0,
         ["NET_P3_TYPE"] = 0,
         ["NET_P4_TYPE"] = 0,

         -- rc input control
         ["RC5_OPTION"] = 0,
         ["RC5_REVERSED"] = 0,
         ["RC6_OPTION"] = 0,
         ["RC6_REVERSED"] = 0,
         ["RC7_OPTION"] = 0,
         ["RC7_REVERSED"] = 0,
         ["RC8_OPTION"] = 0,
         ["RC8_REVERSED"] = 0,
         ["RC9_OPTION"] = 0,
         ["RC9_REVERSED"] = 0,
         ["RC10_OPTION"] = 0,
         ["RC10_REVERSED"] = 0,
         ["RC11_OPTION"] = 0,
         ["RC11_REVERSED"] = 0,
         ["RC12_OPTION"] = 0,
         ["RC12_REVERSED"] = 0,
         ["RC13_OPTION"] = 0,
         ["RC13_REVERSED"] = 0,
         ["RC14_OPTION"] = 0,
         ["RC14_REVERSED"] = 0,
         ["RC15_OPTION"] = 0,
         ["RC15_REVERSED"] = 0,
         ["RC16_OPTION"] = 0,
         ["RC16_REVERSED"] = 0,
         
         -- External Serial Port
         ["SERIAL5_BAUD"] = 115,
         ["SERIAL5_OPTIONS"] = 0,
         ["SERIAL5_PROTOCOL"] = 2,

         -- External PWM Port
         ["SERVO9_FUNCTION"] = -1,
         ["SERVO9_MAX"] = 1900,
         ["SERVO9_MIN"] = 1000,
         ["SERVO9_REVERSED"] = 0,
         ["SERVO9_TRIM"] = 1500,
         ["SERVO10_FUNCTION"] = -1,
         ["SERVO10_MAX"] = 1900,
         ["SERVO10_MIN"] = 1100,
         ["SERVO10_REVERSED"] = 0,
         ["SERVO10_TRIM"] = 1500,
         ["SERVO11_FUNCTION"] = -1,
         ["SERVO11_MAX"] = 1900,
         ["SERVO11_MIN"] = 1100,
         ["SERVO11_REVERSED"] = 0,
         ["SERVO11_TRIM"] = 1500,
         ["SERVO12_FUNCTION"] = -1,
         ["SERVO12_MAX"] = 1900,
         ["SERVO12_MIN"] = 1100,
         ["SERVO12_REVERSED"] = 0,
         ["SERVO12_TRIM"] = 1500,
         ["SERVO13_FUNCTION"] = -1,
         ["SERVO13_MAX"] = 1900,
         ["SERVO13_MIN"] = 1100,
         ["SERVO13_REVERSED"] = 0,
         ["SERVO13_TRIM"] = 1500,
         ["SERVO14_FUNCTION"] = -1,
         ["SERVO14_MAX"] = 1900,
         ["SERVO14_MIN"] = 1100,
         ["SERVO14_REVERSED"] = 0,
         ["SERVO14_TRIM"] = 1500,
         ["SERVO15_FUNCTION"] = -1,
         ["SERVO15_MAX"] = 1900,
         ["SERVO15_MIN"] = 1100,
         ["SERVO15_REVERSED"] = 0,
         ["SERVO15_TRIM"] = 1500,
         ["SERVO16_FUNCTION"] = -1,
         ["SERVO16_MAX"] = 1900,
         ["SERVO16_MIN"] = 1100,
         ["SERVO16_REVERSED"] = 0,
         ["SERVO16_TRIM"] = 1500,
      },
      default_sel_value = SEL_APPLY_DEFAULTS,
      profiles = {
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
         [5] = {
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
         [10] = {
            name = "Silvus",
            params = {
               ["SERIAL5_BAUD"] = 115,      -- 115 = 115200 baud
               ["SERIAL5_PROTOCOL"] = 2,    -- 2 = MAVLink2
            }
         },
         [11] = {
            name = "Skylink",
            params = {
               ["NET_ENABLE"] = 1,
               ["NET_OPTIONS"] = 0,
               ["NET_P1_IP0"] = 192,
               ["NET_P1_IP1"] = 168,
               ["NET_P1_IP2"] = 111,
               ["NET_P1_IP3"] = 15,
               ["NET_P1_PORT"] = 14550,
               ["NET_P1_PROTOCOL"] = 2,
               ["NET_P1_TYPE"] = 2,
               ["NET_P2_TYPE"] = 0,
               ["NET_P3_TYPE"] = 0,
               ["NET_P4_TYPE"] = 0,
            }
         },
      },
   },
}
-- This is a marker for the end of the config_domains; it is used to swap these out for CI testing

-- a list of parameters which can still be set even if we are
-- filtering which parameter values can bet set.  We also permit the
-- configuration paramters to be dynamically set.
local parameters_which_can_be_set = {
   -- Config / Parameter Management
   ["CFG_FLT_PM_SET"] = true,

   -- Aircraft options
   ["COMPASS_USE3"] = true,
   ["LIGHTS_ON"] = true,

   -- Battery failsafes
   ["BATT_ARM_MAH"] = true,
   ["BATT_ARM_VOLT"] = true,
   ["BATT_FS_CRT_ACT"] = true,
   ["BATT_FS_LOW_ACT"] = true,
   ["BATT2_ARM_VOLT"] = true,
   ["BATT3_ARM_VOLT"] = true,

   -- Fence configuration
   ["FENCE_ACTION"] = true,
   ["FENCE_ALT_MAX"] = true,
   ["FENCE_ENABLE"] = true,
   ["FENCE_RADIUS"] = true,
   ["FENCE_TOTAL"] = true,
   ["FENCE_TYPE"] = true,

   -- Failsafes
   ["FLTMODE_CH"] = true,
   ["FLTMODE_GCSBLOCK"] = true,
   ["FLTMODE1"] = true,
   ["FLTMODE2"] = true,
   ["FLTMODE3"] = true,
   ["FLTMODE4"] = true,
   ["FLTMODE5"] = true,
   ["FLTMODE6"] = true,

   -- Failsafes
   ["FS_DR_ENABLE"] = true,
   ["FS_EKF_ACTION"] = true,
   ["FS_GCS_ENABLE"] = true,
   ["FS_GCS_TIMEOUT"] = true,
   ["FS_OPTIONS"] = true,
   ["FS_THR_ENABLE"] = true,
   ["FS_VIBE_ENABLE"] = true,

   -- Follow mode
   ["FOLL_DIST"] = true,
   ["FOLL_ENABLE"] = true,
   ["FOLL_HDG_MODE"] = true,
   ["FOLL_OFS_X"] = true,
   ["FOLL_OFS_Y"] = true,
   ["FOLL_OFS_Z"] = true,
   ["FOLL_SYSID"] = true,

   -- Guided mode timeout
   ["GUID_OPTIONS"] = true,
   ["GUID_TIMEOUT"] = true,

   -- IMU Logging
   ["INS_LOG_BAT_CNT"] = true,
   ["INS_LOG_BAT_MASK"] = true,
   ["INS_LOG_BAT_OPT"] = true,

   -- Landing behavior
   ["LAND_ALT_LOW"] = true,
   ["LAND_REPOSITION"] = true,
   ["LAND_SPEED"] = true,
   ["LAND_SPEED_HIGH"] = true,

   -- Logging
   ["LOG_BITMASK"] = true,
   ["LOG_DISARMED"] = true,
   ["LOG_FILE_DSRMROT"] = true,

   -- Loiter mode
   ["LOIT_ACC_MAX"] = true,
   ["LOIT_ANG_MAX"] = true,
   ["LOIT_BRK_ACCEL"] = true,
   ["LOIT_BRK_DELAY"] = true,
   ["LOIT_BRK_JERK"] = true,
   ["LOIT_SPEED"] = true,

   -- Payload: Mount/gimbal configuration
   ["MNT1_TYPE"] = true,
   ["MNT1_DEFLT_MODE"] = true,
   ["MNT1_PITCH_MIN"] = true,
   ["MNT1_PITCH_MAX"] = true,
   ["MNT1_ROLL_MIN"] = true,
   ["MNT1_ROLL_MAX"] = true,
   ["MNT1_YAW_MIN"] = true,
   ["MNT1_YAW_MAX"] = true,
   ["MNT1_RC_RATE"] = true,
   ["MNT1_LEAD_PTCH"] = true,
   ["MNT1_LEAD_RLL"] = true,
   ["MNT1_NEUTRAL_X"] = true,
   ["MNT1_NEUTRAL_Y"] = true,
   ["MNT1_NEUTRAL_Z"] = true,
   ["MNT1_RETRACT_X"] = true,
   ["MNT1_RETRACT_Y"] = true,
   ["MNT1_RETRACT_Z"] = true,

   -- Pilot input shaping
   ["PHLD_BRAKE_ANGLE"] = true,
   ["PHLD_BRAKE_RATE"] = true,
   ["PILOT_ACCEL_Z"] = true,
   ["PILOT_SPEED_DN"] = true,
   ["PILOT_SPEED_UP"] = true,
   ["PILOT_TKOFF_ALT"] = true,

   -- Trajectory Shaping
   ["PSC_ANGLE_MAX"] = true,

   -- RTL behavior
   ["RTL_ALT"] = true,
   ["RTL_ALT_FINAL"] = true,
   ["RTL_ALT_TYPE"] = true,
   ["RTL_CLIMB_MIN"] = true,
   ["RTL_CONE_SLOPE"] = true,
   ["RTL_LOIT_TIME"] = true,
   ["RTL_OPTIONS"] = true,
   ["RTL_SPEED"] = true,

   -- RC input channel configuration (RC5–RC16)
   ["RC5_OPTION"] = true,
   ["RC5_REVERSED"] = true,
   ["RC5_TRIM"] = true,
   ["RC5_MIN"] = true,
   ["RC5_MAX"] = true,
   ["RC5_DZ"] = true,

   ["RC6_OPTION"] = true,
   ["RC6_REVERSED"] = true,
   ["RC6_TRIM"] = true,
   ["RC6_MIN"] = true,
   ["RC6_MAX"] = true,
   ["RC6_DZ"] = true,

   ["RC7_OPTION"] = true,
   ["RC7_REVERSED"] = true,
   ["RC7_TRIM"] = true,
   ["RC7_MIN"] = true,
   ["RC7_MAX"] = true,
   ["RC7_DZ"] = true,

   ["RC8_OPTION"] = true,
   ["RC8_REVERSED"] = true,
   ["RC8_TRIM"] = true,
   ["RC8_MIN"] = true,
   ["RC8_MAX"] = true,
   ["RC8_DZ"] = true,

   ["RC9_OPTION"] = true,
   ["RC9_REVERSED"] = true,
   ["RC9_TRIM"] = true,
   ["RC9_MIN"] = true,
   ["RC9_MAX"] = true,
   ["RC9_DZ"] = true,

   ["RC10_OPTION"] = true,
   ["RC10_REVERSED"] = true,
   ["RC10_TRIM"] = true,
   ["RC10_MIN"] = true,
   ["RC10_MAX"] = true,
   ["RC10_DZ"] = true,

   ["RC11_OPTION"] = true,
   ["RC11_REVERSED"] = true,
   ["RC11_TRIM"] = true,
   ["RC11_MIN"] = true,
   ["RC11_MAX"] = true,
   ["RC11_DZ"] = true,

   ["RC12_OPTION"] = true,
   ["RC12_REVERSED"] = true,
   ["RC12_TRIM"] = true,
   ["RC12_MIN"] = true,
   ["RC12_MAX"] = true,
   ["RC12_DZ"] = true,

   ["RC13_OPTION"] = true,
   ["RC13_REVERSED"] = true,
   ["RC13_TRIM"] = true,
   ["RC13_MIN"] = true,
   ["RC13_MAX"] = true,
   ["RC13_DZ"] = true,

   ["RC14_OPTION"] = true,
   ["RC14_REVERSED"] = true,
   ["RC14_TRIM"] = true,
   ["RC14_MIN"] = true,
   ["RC14_MAX"] = true,
   ["RC14_DZ"] = true,

   ["RC15_OPTION"] = true,
   ["RC15_REVERSED"] = true,
   ["RC15_TRIM"] = true,
   ["RC15_MIN"] = true,
   ["RC15_MAX"] = true,
   ["RC15_DZ"] = true,

   ["RC16_OPTION"] = true,
   ["RC16_REVERSED"] = true,
   ["RC16_TRIM"] = true,
   ["RC16_MIN"] = true,
   ["RC16_MAX"] = true,
   ["RC16_DZ"] = true,

   -- Payload: Servo outputs (e.g. gripper, zoom, tilt, shutter, etc.)
   ["SERVO9_FUNCTION"] = true,
   ["SERVO9_REVERSED"] = true,
   ["SERVO9_MIN"] = true,
   ["SERVO9_MAX"] = true,
   ["SERVO9_TRIM"] = true,

   ["SERVO10_FUNCTION"] = true,
   ["SERVO10_REVERSED"] = true,
   ["SERVO10_MIN"] = true,
   ["SERVO10_MAX"] = true,
   ["SERVO10_TRIM"] = true,

   ["SERVO11_FUNCTION"] = true,
   ["SERVO11_REVERSED"] = true,
   ["SERVO11_MIN"] = true,
   ["SERVO11_MAX"] = true,
   ["SERVO11_TRIM"] = true,

   ["SERVO12_FUNCTION"] = true,
   ["SERVO12_REVERSED"] = true,
   ["SERVO12_MIN"] = true,
   ["SERVO12_MAX"] = true,
   ["SERVO12_TRIM"] = true,

   ["SERVO13_FUNCTION"] = true,
   ["SERVO13_REVERSED"] = true,
   ["SERVO13_MIN"] = true,
   ["SERVO13_MAX"] = true,
   ["SERVO13_TRIM"] = true,

   -- Terrain following
   ["TERRAIN_ENABLE"] = true,
   ["TERRAIN_MARGIN"] = true,
   ["TERRAIN_OFS_MAX"] = true,
   ["TERRAIN_OPTIONS"] = true,
   ["TERRAIN_SPACING"] = true,

   -- Trajectory shaping
   ["PSC_JERK_XY"] = true,
   ["PSC_JERK_Z"] = true,

   -- Waypoint navigation
   ["WP_NAVALT_MIN"] = true,
   ["WP_YAW_BEHAVIOR"] = true,
   ["WPNAV_ACCEL"] = true,
   ["WPNAV_ACCEL_C"] = true,
   ["WPNAV_ACCEL_Z"] = true,
   ["WPNAV_JERK"] = true,
   ["WPNAV_RADIUS"] = true,
   ["WPNAV_RFND_USE"] = true,
   ["WPNAV_SPEED"] = true,
   ["WPNAV_SPEED_DN"] = true,
   ["WPNAV_SPEED_UP"] = true,
   ["WPNAV_TER_MARGIN"] = true,

   -- ShipOps
   ["SHIP_HHV_ALT"] = true,
   ["SHIP_HTL_ALT"] = true,
   ["SHIP_HTL_ANG"] = true,
   ["SHIP_HTL_RAD"] = true,
   ["SHIP_KOZ_CCW"] = true,
   ["SHIP_KOZ_CW"] = true,
   ["SHIP_KOZ_DKR"] = true,
   ["SHIP_KOZ_RAD"] = true,
   ["SHIP1_HHV_ALT"] = true,
   ["SHIP1_HTL_ALT"] = true,
   ["SHIP1_HTL_ANG"] = true,
   ["SHIP1_HTL_RAD"] = true,
   ["SHIP1_KOZ_CCW"] = true,
   ["SHIP1_KOZ_CW"] = true,
   ["SHIP1_KOZ_DKR"] = true,
   ["SHIP1_KOZ_RAD"] = true,
   ["SHIP2_HHV_ALT"] = true,
   ["SHIP2_HTL_ALT"] = true,
   ["SHIP2_HTL_ANG"] = true,
   ["SHIP2_HTL_RAD"] = true,
   ["SHIP2_KOZ_CCW"] = true,
   ["SHIP2_KOZ_CW"] = true,
   ["SHIP2_KOZ_DKR"] = true,
   ["SHIP2_KOZ_RAD"] = true,
}


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

-- initialize MAVLink rx with buffer depth and number of rx message IDs to register
mavlink:init(5, 1)

-- register message id to receive
local PARAM_SET_ID = 23
mavlink:register_rx_msgid(PARAM_SET_ID)

-- initialise our knowledge of the GCS's allow-set-parameters state.
--   We do not want to fight over setting this GCS state via other
--   mechanisms (eg. an auxiliary function), so we keep this state
--   around to track what we last set:
local gcs_allow_set = gcs:get_allow_param_set()

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
         for param_name, _ in pairs(profile.params) do
 --            default_overridden[param_name] = true
            local param_default = domain.all_param_defaults[param_name]
            if param_default == nil then
               send_text(3, string.format("%s exists in %s[%f](%s) but not in %s.all_param_defaults", param_name, domain.param_name, profile_num, profile.name, domain.param_name))
               return false
            end
--            if param_default == param_value then
--               send_text(3, string.format("%s in %s[%f](%s) has the same value as %s.all_param_defaults", param_name, domain.param_name, profile_num, profile.name, domain.param_name))
--               return false
--            end
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
   for _, domain in pairs(config_domains) do
      for param_name, value in pairs(domain.all_param_defaults) do
         if value ~= must_be_set then
            goto vmbsp_next_param
         end
         local first_value = nil
         for profile_num, profile in pairs(domain.profiles) do
            param_value = profile.params[param_name]
            if param_value == nil then
               send_text(3, string.format("%s exists in %s.all_param_defaults with must-be-set-value but profile %s(%s) params does not have a value for it", param_name, domain.param_name, profile_num, profile.name))
               return false
            end
            if first_value == nil then
               first_value = param_value
            end
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

local function validate_param_profiles()
   for _, domain in pairs(config_domains) do
      for profile_num, profile in pairs(domain.profiles) do
         if profile.name == nil then
            send_text(3, string.format("%s profile is missing a name", domain.param_name))
            return false
         end
         if profile_num == 0 then
            send_text(3, string.format("%s profile (%s) has zero index, which conflicts with the domain \"apply defaults\" option", domain.param_name, profile.name))
            return false
         end
      end
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
      validate_param_profiles,
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
local CFG_ENABLE = bind_add_param("ENABLE", param_index["enable"], 1)

--[[
  // @Param: CFG_FLTR_PM_SET
  // @DisplayName: Filter parameter setting
  // @Description: Disallows setting of parameters outside whitelist
  // @Values: 0:Do not filter,1:Filter
  // @User: Standard
--]]
local FLT_PM_SET = bind_add_param("FLT_PM_SET", param_index["pm_filter"], 0)

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
   domain.sel_param = add_param_for_domain(domain, param_index[domain.param_name], "SEL", domain_sel_default)
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
local last_active_domains_print_time = millis()
local function handle_show_domain_statuses()
   local now = millis()
   if now - last_active_domains_print_time < 30000 then
      return
   end
   last_active_domains_print_time = now
   for _, domain in pairs(config_domains) do
      local sel_value = domain.sel_param:get()
      if sel_value == SEL_DO_NOTHING then
         send_text(6, string.format("%s: doing nothing", domain.param_name))
         goto hsds_next_domain
      end
      if sel_value == SEL_APPLY_DEFAULTS then
         send_text(6, string.format("%s: using defaults", domain.param_name))
         goto hsds_next_domain
      end
      local profile = domain.profiles[sel_value]
      if profile == nil then
         send_text(6, string.format("%s: Invalid profile selected", domain.param_name))
         goto hsds_next_domain
      end
      send_text(6, string.format("%s: using profile %s", domain.param_name, profile.name))
      ::hsds_next_domain::
   end
end

local selected_profile_was_nil = false
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
         if not selected_profile_was_nil then
            send_text(6, string.format("Invalid profile selected for %s", domain.param_name))
            selected_profile_was_nil = true
         end
         set_aux_auth_failed(string.format("Invalid profile selected for %s", domain.param_name))
         success = false
         goto cd_next_domain
      end
      selected_profile_was_nil = false

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
      return
   end
   if not success then
      return
   end

   set_aux_auth_passed()

   handle_show_domain_statuses()
end

-- support for parameter-set-filtering:
local function should_set_parameter_id(param_id)
   -- first check the static list:
   if parameters_which_can_be_set[param_id] ~= nil then
      return parameters_which_can_be_set[param_id]
   end

   -- now check to see if this is one of the configuration selection
   -- parameters:
   for _, domain in pairs(config_domains) do
      domain_sel_param_name = string.format("CFG_%s_SEL", domain.param_name)
      if param_id == domain_sel_param_name then
         return true
      end
   end

   return false
end

local function handle_param_set(name, value)
    -- we will not receive packets in here for the wrong system ID /
    --   component ID; this is handled by ArduPilot's MAVLink routing
    --   code

    -- check for this specific ID:
    if not should_set_parameter_id(name) then
       send_text(3, string.format("param set denied (%s)", name))
        return
    end

    param:set_and_save(name, value)
    gcs:send_text(3, string.format("param set applied"))
end

local function handle_param_setting()
   if gcs_allow_set and FLT_PM_SET:get() == 1 then
      -- this script is filtering, disallow setting via normal means (once):
      gcs:set_allow_param_set(false)
      gcs_allow_set = false
   elseif not gcs_allow_set and FLT_PM_SET:get() == 0 then
      -- this script is not filtering, allow setting via normal means (once):
      gcs:set_allow_param_set(true)
      gcs_allow_set = true
   end

   while true do
      -- we consume all mavlink messages even when we won't set parameters
      local msg, _ = mavlink:receive_chan()
      if msg == nil then
         break
      end

      if gcs_allow_set == false then -- we are in change of param setting
         local param_value, _, _, param_id, _ = string.unpack("<fBBc16B", string.sub(msg, 13, 36))
         param_id = string.gsub(param_id, string.char(0), "")

         handle_param_set(param_id, param_value)
      end
   end
end

-- update function
local domains_valid = false
function update()
   if CFG_ENABLE:get() == 0 then
      -- permanently exit
        if not gcs_allow_set then
          gcs:set_allow_param_set(true)
          gcs_allow_set = true
        end
      send_text(3, string.format("exitting"))
      return
   end

   handle_param_setting()

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
