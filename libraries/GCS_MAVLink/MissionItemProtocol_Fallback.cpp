/*
  Implementation details for transfering waypoint information using
  the MISSION_ITEM protocol to and from a GCS.
*/

#include "MissionItemProtocol_Fallback.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_Mission/AP_Mission.h>

#include "GCS.h"

MAV_MISSION_RESULT MissionItemProtocol_Fallback::append_item(const mavlink_mission_item_int_t &mission_item_int)
{
    // sanity check for DO_JUMP command
    AP_Mission::Mission_Command cmd;

    const MAV_MISSION_RESULT res = AP_Mission::mavlink_int_to_mission_cmd(mission_item_int, cmd);
    if (res != MAV_MISSION_ACCEPTED) {
        return res;
    }

    if (cmd.id == MAV_CMD_DO_JUMP) {
        if ((cmd.content.jump.target >= item_count() && cmd.content.jump.target > request_last) || cmd.content.jump.target == 0) {
            return MAV_MISSION_ERROR;
        }
    }

    if (!fallback_mission.add_cmd(cmd)) {
        return MAV_MISSION_ERROR;
    }
    return MAV_MISSION_ACCEPTED;
}

bool MissionItemProtocol_Fallback::clear_all_items()
{
    return fallback_mission.clear();
}

MAV_MISSION_RESULT MissionItemProtocol_Fallback::complete(const GCS_MAVLINK &_link)
{
    MAV_MISSION_RESULT result = MissionItemProtocol::complete(_link);
    if (result == MAV_MISSION_ACCEPTED) {
        _link.send_text(MAV_SEVERITY_INFO, "Flight fallback plan received");
    }
    return result;
}

MAV_MISSION_RESULT MissionItemProtocol_Fallback::get_item(const GCS_MAVLINK &_link,
                                                           const mavlink_message_t &msg,
                                                           const mavlink_mission_request_int_t &packet,
                                                           mavlink_mission_item_int_t &ret_packet)
{
    if (packet.seq != 0 && // always allow HOME to be read
        packet.seq >= fallback_mission.num_commands()) {
        // try to educate the GCS on the actual size of the mission:
        mavlink_msg_mission_count_send(_link.get_chan(),
                                       msg.sysid,
                                       msg.compid,
                                       fallback_mission.num_commands(),
                                       MAV_MISSION_TYPE_FALLBACK,
                                       packet.tid);
        return MAV_MISSION_ERROR;
    }

    AP_Mission::Mission_Command cmd;

    // retrieve mission from eeprom
    if (!fallback_mission.read_cmd_from_storage(packet.seq, cmd)) {
        return MAV_MISSION_ERROR;
    }

    if (!AP_Mission::mission_cmd_to_mavlink_int(cmd, ret_packet)) {
        return MAV_MISSION_ERROR;
    }

    ret_packet.current = 0;

    // set auto continue to 1
    ret_packet.autocontinue = 1;     // 1 (true), 0 (false)

    ret_packet.command = cmd.id;

    return MAV_MISSION_ACCEPTED;
}

uint16_t MissionItemProtocol_Fallback::item_count() const {
    return fallback_mission.num_commands();
}

uint16_t MissionItemProtocol_Fallback::max_items() const {
    return fallback_mission.num_commands_max();
}

MAV_MISSION_RESULT MissionItemProtocol_Fallback::replace_item(const mavlink_mission_item_int_t &mission_item_int)
{
    AP_Mission::Mission_Command cmd;

    const MAV_MISSION_RESULT res = AP_Mission::mavlink_int_to_mission_cmd(mission_item_int, cmd);
    if (res != MAV_MISSION_ACCEPTED) {
        return res;
    }

    // sanity check for DO_JUMP command
    if (cmd.id == MAV_CMD_DO_JUMP) {
        if ((cmd.content.jump.target >= item_count() && cmd.content.jump.target > request_last) || cmd.content.jump.target == 0) {
            return MAV_MISSION_ERROR;
        }
    }
    if (!fallback_mission.replace_cmd(cmd.index, cmd)) {
        return MAV_MISSION_ERROR;
    }
    return MAV_MISSION_ACCEPTED;
}

void MissionItemProtocol_Fallback::timeout()
{
    link->send_text(MAV_SEVERITY_WARNING, "Fallback mission upload timeout");
}

void MissionItemProtocol_Fallback::truncate(const uint16_t mission_items)
{
    // new mission arriving, truncate mission to be the same length
    fallback_mission.truncate(mission_items);
}

// replace_cmd - replaces the command at position 'index' in the command list with the provided cmd
//     returns true if successfully replaced, false on failure
bool MissionItemProtocol_Fallback::replace_cmd(uint16_t index, const AP_Mission::Mission_Command& cmd) {
    return fallback_mission.replace_cmd(index, cmd);
}

// add_cmd - adds a command to the end of the command list and writes to storage
//     returns true if successfully added, false on failure
bool MissionItemProtocol_Fallback::add_cmd(AP_Mission::Mission_Command& cmd) {
    return fallback_mission.add_cmd(cmd);
}
