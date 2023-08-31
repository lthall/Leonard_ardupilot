/*
  MAVLink logfile transfer functions
 */

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h> // for LOG_ENTRY

#if HAL_GCS_ENABLED

extern const AP_HAL::HAL& hal;

/**
   handle all types of log download requests from the GCS
 */
void AP_Logger::handle_log_message(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    if (!WritesEnabled()) {
        // this is currently used as a proxy for "in_mavlink_delay"
        return;
    }
    if (vehicle_is_armed() && (msg.msgid != MAVLINK_MSG_ID_SET_LOG_LABEL)) {
        if (!_warned_log_disarm) {
            link.send_text(MAV_SEVERITY_ERROR, "Disarm for log download");
            _warned_log_disarm = true;
        }
        return;
    }
    _warned_log_disarm = false;
    _last_mavlink_log_transfer_message_handled_ms = AP_HAL::millis();
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        handle_log_request_list(link, msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
        handle_log_request_data(link, msg);
        break;
    case MAVLINK_MSG_ID_LOG_ERASE:
        handle_log_request_erase(link, msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        handle_log_request_end(link, msg);
        break;
    case MAVLINK_MSG_ID_SET_LOG_LABEL:
        handle_log_set_label(link, msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA_BY_LABEL:
        handle_log_request_data_by_label(link, msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_LABEL_LIST:
        handle_log_request_label_list(link, msg);
        break;
    }
}

/**
   handle set current log file with specified label
 */
void AP_Logger::handle_log_set_label(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    mavlink_set_log_label_t packet;
    mavlink_msg_set_log_label_decode(&msg, &packet);

    if (_next_backend == 0) {
        return;
    }
    backends[0]->set_label(packet.label, sizeof(packet.label));
}

/**
   handle log request list of a labels
 */
void AP_Logger::handle_log_request_label_list(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    WITH_SEMAPHORE(_log_send_sem);

    if (_log_sending_link != nullptr) {
        link.send_text(MAV_SEVERITY_INFO, "Log download in progress");
        return;
    }
    if (_next_backend == 0) {
        return;
    }

    backends[0]->list_labels(&link);
}


/**
   handle all types of log download requests from the GCS
 */
void AP_Logger::handle_log_request_list(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    WITH_SEMAPHORE(_log_send_sem);

    if (_log_sending_link != nullptr) {
        link.send_text(MAV_SEVERITY_INFO, "Log download in progress");
        return;
    }

    mavlink_log_request_list_t packet;
    mavlink_msg_log_request_list_decode(&msg, &packet);

    _log_num_logs = get_num_logs();

    if (_log_num_logs == 0) {
        _log_next_list_entry = 0;
        _log_last_list_entry = 0;        
    } else {
        _log_next_list_entry = packet.start;
        _log_last_list_entry = packet.end;

        if (_log_last_list_entry > _log_num_logs) {
            _log_last_list_entry = _log_num_logs;
        }
        if (_log_next_list_entry < 1) {
            _log_next_list_entry = 1;
        }
    }

    transfer_activity = TransferActivity::LISTING;
    _log_sending_link = &link;

    handle_log_send_listing();
}

/**
   handle request for log data by it's label
 */
void AP_Logger::handle_log_request_data_by_label(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    WITH_SEMAPHORE(_log_send_sem);

    if (_log_sending_link != nullptr) {
        // some GCS (e.g. MAVProxy) attempt to stream request_data
        // messages when they're filling gaps in the downloaded logs.
        // This channel check avoids complaining to them, at the cost
        // of silently dropping any repeated attempts to start logging
        if (_log_sending_link->get_chan() != link.get_chan()) {
            link.send_text(MAV_SEVERITY_INFO, "Log download in progress");
        }
        return;
    }

    mavlink_log_request_data_by_label_t packet;
    mavlink_msg_log_request_data_by_label_decode(&msg, &packet);

    // consider opening or switching logs:
    if (transfer_activity != TransferActivity::SENDING || strncmp(_log_label_data, packet.label, sizeof(_log_label_data)) != 0) {
        _log_data_extended = packet.extended_data != 0;
        _log_num_data = 0;
        strncpy(_log_label_data, packet.label, sizeof(_log_label_data));
        _log_data_page = 0;
        _log_data_offset = packet.ofs;
        _log_data_remaining = packet.count;
        _log_data_remaining_initialized = false;
    }

    transfer_activity = TransferActivity::SENDING;
    _log_sending_link = &link;

    handle_log_send();
}

/**
   handle request for log data
 */
void AP_Logger::handle_log_request_data(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    WITH_SEMAPHORE(_log_send_sem);

    if (_log_sending_link != nullptr) {
        // some GCS (e.g. MAVProxy) attempt to stream request_data
        // messages when they're filling gaps in the downloaded logs.
        // This channel check avoids complaining to them, at the cost
        // of silently dropping any repeated attempts to start logging
        if (_log_sending_link->get_chan() != link.get_chan()) {
            link.send_text(MAV_SEVERITY_INFO, "Log download in progress");
        }
        return;
    }

    mavlink_log_request_data_t packet;
    mavlink_msg_log_request_data_decode(&msg, &packet);

    // consider opening or switching logs:
    if (transfer_activity != TransferActivity::SENDING || _log_num_data != packet.id) {

        uint16_t num_logs = get_num_logs();
        if (packet.id > num_logs || packet.id < 1) {
            // request for an invalid log; cancel any current download
            transfer_activity = TransferActivity::IDLE;
            return;
        }

        uint32_t time_utc, size;
        get_log_info(packet.id, size, time_utc);
        _log_num_data = packet.id;
        _log_data_size = size;

        uint32_t end;
        get_log_boundaries(packet.id, _log_data_page, end);
    }

    _log_data_offset = packet.ofs;
    if (_log_data_offset >= _log_data_size) {
        _log_data_remaining = 0;
    } else {
        _log_data_remaining = _log_data_size - _log_data_offset;
    }
    if (_log_data_remaining > packet.count) {
        _log_data_remaining = packet.count;
    }

    transfer_activity = TransferActivity::SENDING;
    _log_sending_link = &link;

    handle_log_send();
}

/**
   handle request to erase log data
 */
void AP_Logger::handle_log_request_erase(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    // mavlink_log_erase_t packet;
    // mavlink_msg_log_erase_decode(&msg, &packet);

    EraseAll();
}

/**
   handle request to stop transfer and resume normal logging
 */
void AP_Logger::handle_log_request_end(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    WITH_SEMAPHORE(_log_send_sem);
    mavlink_log_request_end_t packet;
    mavlink_msg_log_request_end_decode(&msg, &packet);

    transfer_activity = TransferActivity::IDLE;
    _log_sending_link = nullptr;
}

/**
   trigger sending of log messages if there are some pending
 */
void AP_Logger::handle_log_send()
{
    WITH_SEMAPHORE(_log_send_sem);

    if (_log_sending_link == nullptr) {
        return;
    }
    if (hal.util->get_soft_armed()) {
        // might be flying
        return;
    }
    switch (transfer_activity) {
    case TransferActivity::IDLE:
        break;
    case TransferActivity::LISTING:
        handle_log_send_listing();
        break;
    case TransferActivity::SENDING:
        handle_log_sending();
        break;
    }
}

void AP_Logger::handle_log_sending()
{
    WITH_SEMAPHORE(_log_send_sem);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // assume USB speeds in SITL for the purposes of log download
    const uint8_t num_sends = 40;
#else
    uint8_t num_sends = 1;
    if (_log_sending_link->is_high_bandwidth() && hal.gpio->usb_connected()) {
        // when on USB we can send a lot more data
        num_sends = 250;
    } else if (_log_sending_link->have_flow_control()) {
    #if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
        num_sends = 80;
    #else
        num_sends = 10;
    #endif
    }
#endif

    for (uint8_t i=0; i<num_sends; i++) {
        if (transfer_activity != TransferActivity::SENDING) {
            // may have completed sending data
            break;
        }
        if (!handle_log_send_data()) {
            break;
        }
    }
}

/**
   trigger sending of log messages if there are some pending
 */
void AP_Logger::handle_log_send_listing()
{
    WITH_SEMAPHORE(_log_send_sem);

    if (!HAVE_PAYLOAD_SPACE(_log_sending_link->get_chan(), LOG_ENTRY)) {
        // no space
        return;
    }
    if (AP_HAL::millis() - _log_sending_link->get_last_heartbeat_time() > 3000) {
        // give a heartbeat a chance
        return;
    }

    if (_log_next_list_entry > _log_num_logs) {
        _log_next_list_entry = 0;
    }

    uint32_t size, time_utc;
    if (_log_next_list_entry == 0) {
        size = 0;
        time_utc = 0;
    } else {
        get_log_info(_log_next_list_entry, size, time_utc);
        if (size == 0) {
            _log_next_list_entry = 0;
        }
    }

    if (_log_next_list_entry == 0) {
        _log_last_list_entry = 0;
    }

    mavlink_msg_log_entry_send(_log_sending_link->get_chan(),
                               _log_next_list_entry,
                               _log_num_logs,
                               _log_last_list_entry,
                               time_utc,
                               size);
    if (_log_next_list_entry == _log_last_list_entry) {
        transfer_activity = TransferActivity::IDLE;
        _log_sending_link = nullptr;
    } else {
        _log_next_list_entry++;
    }
}

/**
   trigger sending of log data if there are some pending
 */
bool AP_Logger::handle_log_send_data()
{
    WITH_SEMAPHORE(_log_send_sem);

    if ((_log_data_extended && !HAVE_PAYLOAD_SPACE(_log_sending_link->get_chan(), LOG_DATA_EXTENDED)) ||
        (!_log_data_extended && !HAVE_PAYLOAD_SPACE(_log_sending_link->get_chan(), LOG_DATA))) {
        // no space
        return false;
    }
    if (AP_HAL::millis() - _log_sending_link->get_last_heartbeat_time() > 3000) {
        // give a heartbeat a chance
        return false;
    }

    uint32_t log_data_remaining;
    if (!_log_data_remaining_initialized) {
        uint32_t time_utc, size;
        if ((_log_num_data == 0) && (_log_label_data[0] != '\0')) {
            // we have a label
            if (!get_log_info(_log_label_data, size, time_utc)) {
                return false;  // not reay yet
            }
        } else {
            get_log_info(_log_num_data, size, time_utc);
        }
        _log_data_size = size;
        if (_log_data_offset >= _log_data_size) {
            log_data_remaining = 0;
        } else {
            log_data_remaining = _log_data_size - _log_data_offset;
        }
        if (log_data_remaining < _log_data_remaining) {
            _log_data_remaining = log_data_remaining;
        }
        _log_data_remaining_initialized = true;
    }

    int16_t nbytes = 0;
    uint32_t len = _log_data_remaining;
    int16_t log_num = _log_num_data;
	mavlink_log_data_t log_data_packet;
    mavlink_log_data_extended_t log_data_extended_packet;

    const uint16_t log_data_len = (_log_data_extended ? MAVLINK_MSG_LOG_DATA_EXTENDED_FIELD_DATA_LEN : MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN);
    uint8_t* const data = (_log_data_extended ? log_data_extended_packet.data : log_data_packet.data);

    if (len > log_data_len) {
        len = log_data_len;
    }

    if ((_log_num_data == 0) && (_log_label_data[0] != '\0')) {
        // we have a label
        nbytes = get_log_data(_log_label_data, _log_data_page, _log_data_offset, len, data);
        log_num = get_log_num_from_label(_log_label_data);
    } else {
        nbytes = get_log_data(_log_num_data, _log_data_page, _log_data_offset, len, data);
    }

    if ((nbytes == 0) && (_log_data_remaining > 0)) {
        // we didn't get any data
        return false;
    }

    if (nbytes < 0) {
        // report as EOF on error
        nbytes = 0;
    }
    if (nbytes < log_data_len) {
        memset(&data[nbytes], 0, log_data_len-nbytes);
    }

    if (_log_data_extended) {
        log_data_extended_packet.ofs = _log_data_offset;
        log_data_extended_packet.id = log_num;
        log_data_extended_packet.count = nbytes;
        _mav_finalize_message_chan_send(_log_sending_link->get_chan(),
                                        MAVLINK_MSG_ID_LOG_DATA_EXTENDED,
                                        (const char *)&log_data_extended_packet,
                                        MAVLINK_MSG_ID_LOG_DATA_EXTENDED_MIN_LEN,
                                        MAVLINK_MSG_ID_LOG_DATA_EXTENDED_LEN,
                                        MAVLINK_MSG_ID_LOG_DATA_EXTENDED_CRC);
    } else {
        log_data_packet.ofs = _log_data_offset;
        log_data_packet.id = log_num;
        log_data_packet.count = nbytes;
        _mav_finalize_message_chan_send(_log_sending_link->get_chan(),
                                        MAVLINK_MSG_ID_LOG_DATA,
                                        (const char *)&log_data_packet,
                                        MAVLINK_MSG_ID_LOG_DATA_MIN_LEN,
                                        MAVLINK_MSG_ID_LOG_DATA_LEN,
                                        MAVLINK_MSG_ID_LOG_DATA_CRC);
    }

    _log_data_offset += nbytes;
    _log_data_remaining -= nbytes;
    if (nbytes < log_data_len || _log_data_remaining == 0) {
        transfer_activity = TransferActivity::IDLE;
        _log_sending_link = nullptr;
    }
    return true;
}

#endif
