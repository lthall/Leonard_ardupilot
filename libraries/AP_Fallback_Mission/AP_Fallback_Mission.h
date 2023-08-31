/// @file    AP_Fallback_Mission.h
/// @brief   Handles the MAVLINK command mission stack.  Reads and writes fallback mission to storage.
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Mission/AP_Mission.h>

class AP_Fallback_Mission {
public:
    // constructor
    AP_Fallback_Mission()
    {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        if (_singleton != nullptr) {
            AP_HAL::panic("Fallback Mission must be singleton");
        }
#endif
        _singleton = this;
    }

    /// init - initialises this library including checks the version in eeprom matches this library
    void init();

    /// num_commands - returns total number of commands in the fallback mission
    ///                 this number includes offset 0, the home location
    uint16_t num_commands() const { return _cmd_total; }

    /// num_commands_max - returns maximum number of commands that can be stored
    uint16_t num_commands_max() const;

    /// clear - clears out mission
    bool clear();

    /// truncate - truncate any mission items beyond given index
    void truncate(uint16_t index);

    /// add_cmd - adds a command to the end of the command list and writes to storage
    ///     returns true if successfully added, false on failure
    ///     cmd.index is updated with it's new position in the mission
    bool add_cmd(AP_Mission::Mission_Command& cmd);

    /// replace_cmd - replaces the command at position 'index' in the command list with the provided cmd
    ///     replacing the current active command will have no effect until the command is restarted
    ///     returns true if successfully replaced, false on failure
    bool replace_cmd(uint16_t index, const AP_Mission::Mission_Command& cmd);

    /// load_cmd_from_storage - load command from storage
    ///     true is return if successful
    bool read_cmd_from_storage(uint16_t index, AP_Mission::Mission_Command& cmd) const;

    /// write_cmd_to_storage - write a command to storage
    ///     cmd.index is used to calculate the storage location
    ///     true is returned if successful
    bool write_cmd_to_storage(uint16_t index, const AP_Mission::Mission_Command& cmd);

    // get singleton instance
    static AP_Fallback_Mission *get_singleton() {
        return _singleton;
    }

    /* Do not allow copies */
    AP_Fallback_Mission(const AP_Fallback_Mission &other) = delete;
    AP_Fallback_Mission &operator=(const AP_Fallback_Mission&) = delete; 

    // get a reference to the AP_Mission semaphore, allowing an external caller to lock the
    // storage while working with multiple waypoints
    HAL_Semaphore &get_semaphore(void) {
        return _rsem;
    } 

private:
    static AP_Fallback_Mission *_singleton;

    static StorageAccess _storage;

    static bool stored_in_location(uint16_t id);

    /// check_eeprom_version - checks version of missions stored in eeprom matches this library
    /// command list will be cleared if they do not match
    void check_eeprom_version();

    AP_Int16                _cmd_total;  // total number of commands in the mission

    // multi-thread support. This is static so it can be used from
    // const functions
    static HAL_Semaphore _rsem;
};

namespace AP {
    AP_Fallback_Mission *fallback_mission();
};
