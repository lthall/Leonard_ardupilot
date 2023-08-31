/*
Important note:
    Most of the code in this file is copied from AP_Mission.cpp. In an ideal world,
    AP_Fallback_Mission should inherit AP_Mission. however, AP_Mission is a singleton so it
    can't really be inherited.
*/

#include "AP_Fallback_Mission.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;

// storage object
StorageAccess AP_Fallback_Mission::_storage(StorageManager::StorageFallback);

HAL_Semaphore AP_Fallback_Mission::_rsem;

/// init - initialises this library including checks the version in eeprom matches this library
void AP_Fallback_Mission::init()
{
    // check_eeprom_version - checks version of missions stored in eeprom matches this library
    // command list will be cleared if they do not match
    check_eeprom_version();

    // clear the fallback mission.
    gcs().send_text(MAV_SEVERITY_INFO, "Clearing Fallback Mission");
    clear();  
}

/// clear - clears out mission
///     returns true if mission was running so it could not be cleared
bool AP_Fallback_Mission::clear()
{
    // remove all commands
    _cmd_total.set_and_save(0);

    return true;
}

/// trucate - truncate any mission items beyond index
void AP_Fallback_Mission::truncate(uint16_t index)
{
    if ((unsigned)_cmd_total > index) {        
        _cmd_total.set_and_save(index);
    }
}

/// add_cmd - adds a command to the end of the command list and writes to storage
///     returns true if successfully added, false on failure
///     cmd.index is updated with it's new position in the mission
bool AP_Fallback_Mission::add_cmd(AP_Mission::Mission_Command& cmd)
{
    // attempt to write the command to storage
    bool ret = write_cmd_to_storage(_cmd_total, cmd);

    if (ret) {
        // update command's index
        cmd.index = _cmd_total;
        // increment total number of commands
        _cmd_total.set_and_save(_cmd_total + 1);
    }

    return ret;
}

/// replace_cmd - replaces the command at position 'index' in the command list with the provided cmd
///     replacing the current active command will have no effect until the command is restarted
///     returns true if successfully replaced, false on failure
bool AP_Fallback_Mission::replace_cmd(uint16_t index, const AP_Mission::Mission_Command& cmd)
{
    // sanity check index
    if (index >= (unsigned)_cmd_total) {
        return false;
    }

    // attempt to write the command to storage
    return write_cmd_to_storage(index, cmd);
}

struct PACKED Packed_Location_Option_Flags {
    uint8_t relative_alt : 1;           // 1 if altitude is relative to home
    uint8_t unused1      : 1;           // unused flag (defined so that loiter_ccw uses the correct bit)
    uint8_t loiter_ccw   : 1;           // 0 if clockwise, 1 if counter clockwise
    uint8_t terrain_alt  : 1;           // this altitude is above terrain
    uint8_t origin_alt   : 1;           // this altitude is above ekf origin
    uint8_t loiter_xtrack : 1;          // 0 to crosstrack from center of waypoint, 1 to crosstrack from tangent exit location
    uint8_t type_specific_bit_0 : 1;    // each mission item type can use this for storing 1 bit of extra data
};

struct PACKED PackedLocation {
    union {
        Packed_Location_Option_Flags flags;                    ///< options bitmask (1<<0 = relative altitude)
        uint8_t options;                                /// allows writing all flags to eeprom as one byte
    };
    // by making alt 24 bit we can make p1 in a command 16 bit,
    // allowing an accurate angle in centi-degrees. This keeps the
    // storage cost per mission item at 15 bytes, and allows mission
    // altitudes of up to +/- 83km
    int32_t alt:24;                                     ///< param 2 - Altitude in centimeters (meters * 100) see LOCATION_ALT_MAX_M
    int32_t lat;                                        ///< param 3 - Latitude * 10**7
    int32_t lng;                                        ///< param 4 - Longitude * 10**7
};

union PackedContent {
    // location
    PackedLocation location;      // Waypoint location

    // raw bytes, for reading/writing to eeprom. Note that only 10
    // bytes are available if a 16 bit command ID is used
    uint8_t bytes[12];

};

/// load_cmd_from_storage - load command from storage
///     true is return if successful
bool AP_Fallback_Mission::read_cmd_from_storage(uint16_t index, AP_Mission::Mission_Command& cmd) const
{
    WITH_SEMAPHORE(_rsem);

    // special handling for command #0 which is home
    if (index == 0) {
        cmd.index = 0;
        cmd.id = MAV_CMD_NAV_WAYPOINT;
        cmd.p1 = 0;
        cmd.content.location = AP::ahrs().get_home();
        return true;
    }

    if (index >= (unsigned)_cmd_total) {
        return false;
    }

    // Find out proper location in memory by using the start_byte position + the index
    // we can load a command, we don't process it yet
    // read WP position
    const uint16_t pos_in_storage = 4 + (index * AP_MISSION_EEPROM_COMMAND_SIZE);

    PackedContent packed_content {};

    const uint8_t b1 = _storage.read_byte(pos_in_storage);
    if (b1 == 0) {
        cmd.id = _storage.read_uint16(pos_in_storage+1);
        cmd.p1 = _storage.read_uint16(pos_in_storage+3);
        _storage.read_block(packed_content.bytes, pos_in_storage+5, 10);
    } else {
        cmd.id = b1;
        cmd.p1 = _storage.read_uint16(pos_in_storage+1);
        _storage.read_block(packed_content.bytes, pos_in_storage+3, 12);
    }

    if (stored_in_location(cmd.id)) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        // NOTE!  no 16-bit command may be stored_in_location as only
        // 10 bytes are available for storage and lat/lon/alt required
        // 4*sizeof(float) == 12 bytes of storage.
        if (b1 == 0) {
            AP_HAL::panic("May not store location for 16-bit commands");
        }
#endif
        // Location is not PACKED; field-wise copy it:
        cmd.content.location.relative_alt = packed_content.location.flags.relative_alt;
        cmd.content.location.loiter_ccw = packed_content.location.flags.loiter_ccw;
        cmd.content.location.terrain_alt = packed_content.location.flags.terrain_alt;
        cmd.content.location.origin_alt = packed_content.location.flags.origin_alt;
        cmd.content.location.loiter_xtrack = packed_content.location.flags.loiter_xtrack;
        cmd.content.location.alt = packed_content.location.alt;
        cmd.content.location.lat = packed_content.location.lat;
        cmd.content.location.lng = packed_content.location.lng;

        if (packed_content.location.flags.type_specific_bit_0) {
            cmd.type_specific_bits = 1U << 0;
        }
    } else {
        // all other options in Content are assumed to be packed:
        static_assert(sizeof(cmd.content) >= 12,
                      "content is big enough to take bytes");
        // (void *) cast to specify gcc that we know that we are copy byte into a non trivial type and leaving 4 bytes untouched
        memcpy((void *)&cmd.content, packed_content.bytes, 12);
    }

    // set command's index to it's position in eeprom
    cmd.index = index;

    // return success
    return true;
}

bool AP_Fallback_Mission::stored_in_location(uint16_t id)
{
    switch (id) {
    case MAV_CMD_NAV_WAYPOINT:
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TURNS:
    case MAV_CMD_NAV_LOITER_TIME:
    case MAV_CMD_NAV_LAND:
    case MAV_CMD_NAV_TAKEOFF:
    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
    case MAV_CMD_NAV_LOITER_TO_ALT:
    case MAV_CMD_NAV_SPLINE_WAYPOINT:
    case MAV_CMD_NAV_GUIDED_ENABLE:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_LAND_START:
    case MAV_CMD_DO_GO_AROUND:
    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_NAV_VTOL_TAKEOFF:
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_PAYLOAD_PLACE:
        return true;
    default:
        return false;
    }
}

/// write_cmd_to_storage - write a command to storage
///     index is used to calculate the storage location
///     true is returned if successful
bool AP_Fallback_Mission::write_cmd_to_storage(uint16_t index, const AP_Mission::Mission_Command& cmd)
{
    WITH_SEMAPHORE(_rsem);
    
    // range check cmd's index
    if (index >= num_commands_max()) {
        return false;
    }

    PackedContent packed {};
    if (stored_in_location(cmd.id)) {
        // Location is not PACKED; field-wise copy it:
        packed.location.flags.relative_alt = cmd.content.location.relative_alt;
        packed.location.flags.loiter_ccw = cmd.content.location.loiter_ccw;
        packed.location.flags.terrain_alt = cmd.content.location.terrain_alt;
        packed.location.flags.origin_alt = cmd.content.location.origin_alt;
        packed.location.flags.loiter_xtrack = cmd.content.location.loiter_xtrack;
        packed.location.flags.type_specific_bit_0 = cmd.type_specific_bits & (1U<<0);
        packed.location.alt = cmd.content.location.alt;
        packed.location.lat = cmd.content.location.lat;
        packed.location.lng = cmd.content.location.lng;
    } else {
        // all other options in Content are assumed to be packed:
        static_assert(sizeof(packed.bytes) >= 12,
                      "packed.bytes is big enough to take content");
        memcpy(packed.bytes, &cmd.content, 12);
    }

    // calculate where in storage the command should be placed
    uint16_t pos_in_storage = 4 + (index * AP_MISSION_EEPROM_COMMAND_SIZE);

    if (cmd.id < 256) {
        _storage.write_byte(pos_in_storage, cmd.id);
        _storage.write_uint16(pos_in_storage+1, cmd.p1);
        _storage.write_block(pos_in_storage+3, packed.bytes, 12);
    } else {
        // if the command ID is above 256 we store a 0 followed by the 16 bit command ID
        _storage.write_byte(pos_in_storage, 0);
        _storage.write_uint16(pos_in_storage+1, cmd.id);
        _storage.write_uint16(pos_in_storage+3, cmd.p1);
        _storage.write_block(pos_in_storage+5, packed.bytes, 10);
    }

    // return success
    return true;
}

// check_eeprom_version - checks version of missions stored in eeprom matches this library
// command list will be cleared if they do not match
void AP_Fallback_Mission::check_eeprom_version()
{
    uint32_t eeprom_version = _storage.read_uint32(0);

    // if eeprom version does not match, clear the command list and update the eeprom version
    if (eeprom_version != AP_MISSION_EEPROM_VERSION) {
        if (clear()) {
            _storage.write_uint32(0, AP_MISSION_EEPROM_VERSION);
        }
    }
}

/*
  return total number of commands that can fit in storage space
 */
uint16_t AP_Fallback_Mission::num_commands_max(void) const
{
    // -4 to remove space for eeprom version number
    return (_storage.size() - 4) / AP_MISSION_EEPROM_COMMAND_SIZE;
}

// singleton instance
AP_Fallback_Mission *AP_Fallback_Mission::_singleton;

namespace AP {

AP_Fallback_Mission *fallback_mission()
{
    return AP_Fallback_Mission::get_singleton();
}

}
