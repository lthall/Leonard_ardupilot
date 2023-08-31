/* 
   AP_Logger logging - compressed file oriented variant

   This uses heatshrink library to create compressed log files
 */

#include "AP_Logger_Compressed.h"

#if HAL_LOGGING_FILESYSTEM_ENABLED

#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define HEATSHRINK_WINDOW_SIZE    12  // must be between 4 and 15. larger window size will use more memory, but compress more effectively.
#define HEATSHRINK_LOOKAHEAD_SIZE 5   // must be between 3 and (window_size - 1).
#define HEATSHRINK_MAGIC          "HEATSHRINK"
#define HEATSHRINK_MAGIC_SIZE     (sizeof(HEATSHRINK_MAGIC) - 1)
#define HEATSHRINK_HEADER_SIZE    (HEATSHRINK_MAGIC_SIZE + 2 * sizeof(uint8_t))

void AP_Logger_Compressed::Init()
{
    hse = heatshrink_encoder_alloc(HEATSHRINK_WINDOW_SIZE, HEATSHRINK_LOOKAHEAD_SIZE);
    if (!hse) {
        log_init_error("can't allocate heatshrink\n");
        return;
    }

    AP_Logger_File::Init();
}

/*
  construct a compressed log file name given a log number.
  The number in the log filename will be zero-padded.
  Note: Caller must free.
 */
char *AP_Logger_Compressed::_log_file_name_long(const uint16_t log_num) const
{
    char *buf = nullptr;
    if (asprintf(&buf, "%s/%08u.BIN.HS", _log_directory, (unsigned)log_num) == -1) {
        return nullptr;
    }
    return buf;
}

/*
  stop logging
 */
void AP_Logger_Compressed::stop_logging(void)
{
    // best-case effort to avoid annoying the IO thread
    const bool have_sem = write_fd_semaphore.take(hal.util->get_soft_armed()?1:50);

    if (_write_fd != -1) {
        size_t compressed_write_size = 0;
        while (heatshrink_encoder_finish(hse) == HSER_FINISH_MORE) {
            if (heatshrink_encoder_poll(hse, _compressed_write_buffer, _writebuf_chunk, &compressed_write_size) < 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "stop_logging: heatshrink_encoder_poll error");
            }
            if (compressed_write_size > 0) {
                ssize_t cnwritten = -1;
                cnwritten = AP::FS().write(_write_fd, _compressed_write_buffer, compressed_write_size);
                if (cnwritten < (ssize_t)compressed_write_size) {
                    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "cnwritten=%ld < compressed_write_size=%ld", (long int)cnwritten, (long int)compressed_write_size);
                    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "stop_logging: log file short write");
                }
            }
        }
        int fd = _write_fd;
        _write_fd = -1;
        AP::FS().close(fd);
    }

    if (have_sem) {
        write_fd_semaphore.give();
    }
}

/*
  write a header in a new log file
 */
bool AP_Logger_Compressed::write_new_log_header(void)
{
    heatshrink_encoder_reset(hse);

    // first, write the heatshrink header
    uint8_t header_buffer[HEATSHRINK_HEADER_SIZE] = { };
    memcpy(header_buffer, HEATSHRINK_MAGIC, HEATSHRINK_MAGIC_SIZE);
    header_buffer[HEATSHRINK_MAGIC_SIZE] = HEATSHRINK_WINDOW_SIZE;
    header_buffer[HEATSHRINK_MAGIC_SIZE + 1] = HEATSHRINK_LOOKAHEAD_SIZE;
    if (AP::FS().write(_write_fd, header_buffer, sizeof(header_buffer)) < (ssize_t)sizeof(header_buffer)) {
        _open_error_ms = AP_HAL::millis();
        log_init_error("Failed to write heatshrink header");
        AP::FS().close(_write_fd);
        _write_fd = -1;
        return false;
    }

    return true;
}

ssize_t AP_Logger_Compressed::_write_compressed_log_chunk(uint8_t *buffer, ssize_t input_len)
{
    int32_t total = 0;
    if (input_len <= 0) {
        return total;
    }

    size_t input_bytes_compressed;
    size_t compressed_write_size;;
    HSE_sink_res sink_res;
    HSE_poll_res poll_res;
    do {
        if ((sink_res = heatshrink_encoder_sink(hse, buffer, input_len, &input_bytes_compressed)) != HSER_SINK_OK) {
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "heatshrink_encoder_sink error (%d, hse->state=%d)", sink_res, hse->state);
            return total;
        }
        total += input_bytes_compressed;

        compressed_write_size = 0;
        do {
            if ((poll_res = heatshrink_encoder_poll(hse, _compressed_write_buffer, _writebuf_chunk, &compressed_write_size)) < 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "heatshrink_encoder_poll error");
                return total;
            }
            if (compressed_write_size > 0) {
                ssize_t cnwritten = -1;
                cnwritten = AP::FS().write(_write_fd, _compressed_write_buffer, compressed_write_size);
                if (cnwritten < (ssize_t)compressed_write_size) {
                    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "cnwritten=%ld < compressed_write_size=%ld", (long int)cnwritten, (long int)compressed_write_size);
                    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "stop_logging: log file short write");
                    return -1;
                }
            }
        } while (poll_res == HSER_POLL_MORE);
        
        buffer += input_bytes_compressed;
        input_len -= input_bytes_compressed;
    } while (input_len > 0);

    return total;
}

/*
  convert a dirent to a log number
 */
bool AP_Logger_Compressed::dirent_to_log_num(const dirent *de, uint16_t &log_num) const
{
    uint8_t length = strlen(de->d_name);
    if (length < 5) {
        return false;
    }
    if (strncmp(&de->d_name[length-7], ".BIN.HS", 7) != 0) {
        // doesn't end in .BIN.HS
        return false;
    }

    uint16_t thisnum = strtoul(de->d_name, nullptr, 10);
    if (thisnum > MAX_LOG_FILES) {
        return false;
    }
    log_num = thisnum;
    return true;
}

ssize_t AP_Logger_Compressed::write_to_file(const void *buf, uint32_t count)
{
    return _write_compressed_log_chunk((uint8_t *)buf, count);
}

#endif // HAL_LOGGING_FILESYSTEM_ENABLED
