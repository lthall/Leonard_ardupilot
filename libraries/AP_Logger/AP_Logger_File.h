/* 
   AP_Logger logging - file oriented variant

   This uses posix file IO to create log files called logNN.dat in the
   given directory
 */
#pragma once

#include <AP_Filesystem/AP_Filesystem.h>

#include <AP_Common/AP_FWVersion.h>

#include <AP_HAL/utility/RingBuffer.h>
#include "AP_Logger_Backend.h"

#if HAL_LOGGING_FILESYSTEM_ENABLED

#ifndef HAL_LOGGER_WRITE_CHUNK_SIZE
#define HAL_LOGGER_WRITE_CHUNK_SIZE 4096
#endif

#define CACHED_FILE_STAT_LEN 5

#define LOG_INIT_ERRORS_BUFFER_SIZE 1024

class AP_Logger_File : public AP_Logger_Backend
{
public:
    // constructor
    AP_Logger_File(AP_Logger &front,
                   LoggerMessageWriter_DFLogStart *,
                   const char *log_directory);

    static AP_Logger_Backend  *probe(AP_Logger &front,
                                     LoggerMessageWriter_DFLogStart *ls) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        char *log_dir = nullptr;
        if (asprintf(&log_dir, "%s_%s", AP::fwversion().middleware_name, HAL_BOARD_LOG_DIRECTORY) == -1) {
            AP_HAL::panic("Out of memory");
            return nullptr;
        }
#else
        const char *log_dir = HAL_BOARD_LOG_DIRECTORY;
#endif
        AP_HAL::get_HAL().console->printf("log directory: %s\n", log_dir);
        return new AP_Logger_File(front, ls, log_dir);
    }

    // initialisation
    void Init() override;
    bool CardInserted(void) const override;

    // erase handling
    void EraseAll() override;

    /* Write a block of data at current offset */
    bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) override;
    uint32_t bufferspace_available() override;

    // high level interface
    uint16_t find_last_log() override;
    void get_log_boundaries(uint16_t log_num, uint32_t & start_page, uint32_t & end_page) override;
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) override;
    bool get_log_info(const char log_label[], uint32_t &size, uint32_t &time_utc) override;
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) override;
    int16_t get_log_data(const char log_label[], uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) override;
    uint16_t get_num_logs() override;
    void start_new_log(void) override;
    virtual bool write_new_log_header(void) { return true; };
    uint16_t find_oldest_log() override;

    void set_label(const char label[], size_t label_size) override;
    void list_labels(GCS_MAVLINK* log_sending_link) override;
    // convert between log numbering in storage and label. 0 means not ready yet, -1 means not found.
    int16_t log_num_from_label(const char log_label[]) override;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    void flush(void) override;
#endif
    void periodic_1Hz() override;
    void periodic_fullrate() override;

    // this method is used when reporting system status over mavlink
    bool logging_failed() const override;

    bool logging_started(void) const override { return _write_fd != -1; }
    void io_timer(void) override;

protected:

    bool WritesOK() const override;
    bool StartNewLogOK() const override;
    void PrepForArming_start_logging() override;

    bool log_exists(const uint16_t lognum) override;

    void log_init_error(const char *fmt, ...) const;

    virtual char *_log_file_name_long(const uint16_t log_num) const;
    virtual bool dirent_to_log_num(const dirent *de, uint16_t &log_num) const;

    void stop_logging(void) override;

    virtual ssize_t write_to_file(const void *buf, uint32_t count);

    int _write_fd = -1;
    const char *_log_directory;
    volatile uint32_t _open_error_ms;

    const uint16_t _writebuf_chunk = HAL_LOGGER_WRITE_CHUNK_SIZE;

    // write_fd_semaphore mediates access to write_fd so the frontend
    // can open/close files without causing the backend to write to a
    // bad fd
    HAL_Semaphore write_fd_semaphore;

private:
    char *_write_filename;
    int16_t _write_log_num;
    uint32_t _last_write_ms;
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    bool _need_rtc_update;
#endif
    
    int _read_fd = -1;
    uint16_t _read_fd_log_num;
    uint32_t _read_offset;
    uint32_t _write_offset;
    bool _last_write_failed;

    char *_log_init_errors_buffer;

    uint32_t _io_timer_heartbeat;
    bool io_thread_alive() const;
    uint8_t io_thread_warning_decimation_counter;

    // number of log files. if negative - should be recalculte
    int32_t _cached_num_logs;

    // for each file: -1 - file doesn't exist, 1 - file exist, 0 - unknown
    int8_t *_cached_logs_exist;
    const uint16_t _max_log_files;

    // next log index to check existence
    uint16_t _next_log_index_check;

    // cached last log index
    uint16_t _cached_last_log;

    void clear_cache();

    void print_log_init_errors() const;

    struct {
        struct stat stat;
        uint32_t last_accessed_time;
        uint16_t log_num;
    } _cached_file_stat;

    // do we have a recent open error?
    bool recent_open_error(void) const;

    // possibly time-consuming preparations handling
    void Prep_MinSpace();
    int64_t disk_space_avail();
    int64_t disk_space();

    void ensure_log_directory_exists();

    bool file_exists(const char *filename) const;

    // write buffer
    ByteBuffer _writebuf{0};
    uint32_t _last_write_time;

    /* construct a file name given a log number. Caller must free. */
    char *_log_file_name(const uint16_t log_num) const;
    char *_log_file_name_short(const uint16_t log_num) const;
    char *_lastlog_file_name() const;
    uint32_t _get_log_size(const uint16_t log_num);
    uint32_t _get_log_time(const uint16_t log_num);
    
    int16_t get_log_data_by_log_num(uint16_t log_num, const uint16_t page, const uint32_t offset, const uint16_t len, uint8_t *data);

    int stat(const uint16_t log_num, const char *name, struct stat *buf);

    uint32_t last_messagewrite_message_sent;

    // free-space checks; filling up SD cards under NuttX leads to
    // corrupt filesystems which cause loss of data, failure to gather
    // data and failures-to-boot.
    uint32_t _free_space_last_check_time; // milliseconds
    const uint32_t _free_space_check_interval = 1000UL; // milliseconds
    const uint32_t _free_space_min_avail = 8388608; // bytes

    // semaphore mediates access to the ringbuffer
    HAL_Semaphore semaphore;

    // async erase state
    struct {
        bool was_logging;
        uint16_t log_num;
    } erase;
    void erase_next(void);

    typedef struct {
        int32_t log_num;  // 0 means not set, -1 means not found
        char label[MAVLINK_MSG_SET_LOG_LABEL_FIELD_LABEL_LEN];
    } log_num_label_map_t;

    struct {
        char current_log_label[MAVLINK_MSG_SET_LOG_LABEL_FIELD_LABEL_LEN] = { 0 };
        bool current_log_label_dirty = false;

        log_num_label_map_t sending;

        char *labels_file_name = nullptr;
        bool labels_file_truncated = false;

        uint16_t log_num_erased = 0;

        bool list_labels = false;
        GCS_MAVLINK* log_sending_link = nullptr;
    } log_label_data;
    void _log_label_update();
    bool _init_log_labels_file_name();
    void _log_label_handle_log_erase_all();
    void _log_label_handle_log_erase();
    bool _set_current_log_label();
    void _list_labels();
    int32_t _label_to_log_num(const char label[]);
    void _erase_label(const char label[]);
    bool _erase_labels_for_log_num(const uint16_t log_num);
    bool _write_to_labels_file(uint8_t new_content[], uint16_t new_content_len);

    size_t _get_line_length(const char *line, size_t count) const;
    char *_allocate_copy_line(const char *str, size_t count) const;

    int32_t _get_log_num_label_map(log_num_label_map_t **map);

    // convert log num for label, for sending
    void _retrieve_log_num();

    const char *last_io_operation = "";

    bool start_new_log_pending;
};

#endif // HAL_LOGGING_FILESYSTEM_ENABLED
