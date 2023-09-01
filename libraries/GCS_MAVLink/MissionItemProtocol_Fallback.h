#pragma once

#include "MissionItemProtocol.h"

#include <AP_Mission/AP_Mission.h>

// MissionItemProtocol_Fallback objects are used for transfering fallback
// missions from a GCS to ArduPilot and vice-versa.
//
// There exists one MissionItemProtocol_Fallback instance for each 
// of the types of item that might be transfered - e.g. MissionItemProtocol_Rally
// for rally point uploads.  These objects are static in GCS_MAVLINK
// and used by all of the backends.
//
// While prompting the GCS for items required to complete the mission,
// a link is stored to the link the MissionItemProtocol_Fallback should send
// requests out on, and the "receiving" boolean is true.  In this
// state downloading of items (and the item count!) is blocked.
// Starting of uploads (for the same protocol) is also blocked -
// essentially the GCS uploading a set of items (e.g. a fallback mission) has a
// mutex over the mission.
class MissionItemProtocol_Fallback : public MissionItemProtocol {
public:
    MissionItemProtocol_Fallback(class AP_Fallback_Mission &_fallback_mission) :
        fallback_mission(_fallback_mission) { }

    // mission_type returns the MAV_MISSION mavlink enumeration value
    // which this module is responsible for handling
    MAV_MISSION_TYPE mission_type() const override {
        return MAV_MISSION_TYPE_FALLBACK;
    }

    // complete() is called by the base class after all waypoints have
    // been received.  _link is the link which the last item was
    // transfered on.
    MAV_MISSION_RESULT complete(const GCS_MAVLINK &_link) override;
    // timeout() is called by the base class in the case that the GCS
    // does not transfer all waypoints to the vehicle.
    void timeout() override;
    // truncate() is called to set the absolute number of items.  It
    // must be less than or equal to the current number of items (you
    // can't truncate-to a longer list)
    void truncate(uint16_t mission_items) override;

protected:
    AP_Fallback_Mission &fallback_mission;

    // clear_all_items() is called to clear all items on the vehicle
    bool clear_all_items() override WARN_IF_UNUSED;

    // next_item_ap_message_id returns an item from the ap_message
    // enumeration which (when acted upon by the GCS class) will send
    // a mavlink message to the GCS requesting it upload the next
    // required waypoint.
    ap_message next_item_ap_message_id() const override {
        return MSG_NEXT_MISSION_REQUEST_FALLBACK;
    }

    // replace_cmd - replaces the command at position 'index' in the command list with the provided cmd
    //     returns true if successfully replaced, false on failure
    bool replace_cmd(uint16_t index, const AP_Mission::Mission_Command& cmd) override;

    // add_cmd - adds a command to the end of the command list and writes to storage
    //     returns true if successfully added, false on failure
    bool add_cmd(AP_Mission::Mission_Command& cmd) override;

private:
    // append_item() is called by the base class to add the supplied
    // item to the end of the list of stored items.
    MAV_MISSION_RESULT append_item(const mavlink_mission_item_int_t &) override WARN_IF_UNUSED;

    // get_item() fills in ret_packet based on packet; _link is the
    // link the request was received on, and msg is the undecoded
    // request.  Note that msg may not actually decode to a
    // request_int_t!
    MAV_MISSION_RESULT get_item(const GCS_MAVLINK &_link,
                                const mavlink_message_t &msg,
                                const mavlink_mission_request_int_t &packet,
                                mavlink_mission_item_int_t &ret_packet) override WARN_IF_UNUSED;

    // item_count() returns the number of stored items
    uint16_t item_count() const override;

    // item_count() returns the maximum number of items which could be
    // stored on-board
    uint16_t max_items() const override;

    // replace_item() replaces an item in the stored list
    MAV_MISSION_RESULT replace_item(const mavlink_mission_item_int_t &) override WARN_IF_UNUSED;
};
