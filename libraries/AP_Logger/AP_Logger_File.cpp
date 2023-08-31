/* 
   AP_Logger logging - file oriented variant

   This uses posix file IO to create log files called logs/NN.bin in the
   given directory

   SD Card Rates on PixHawk:
    - deletion rate seems to be ~50 files/second.
    - stat seems to be ~150/second
    - readdir loop of 511 entry directory ~62,000 microseconds
 */

#include <ctype.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Filesystem/AP_Filesystem.h>

#include "AP_Logger_File.h"

#if HAL_LOGGING_FILESYSTEM_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_RTC/AP_RTC.h>

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>


extern const AP_HAL::HAL& hal;

#define LOGGER_PAGE_SIZE 1024UL

#define MB_to_B 1000000
#define B_to_MB 0.000001

// time between tries to open log
#define LOGGER_FILE_REOPEN_MS 5000

#define MAX_LAST_ACCESSED_TIME_MS 500

#define LOG_FILES_LABEL_MAP_FILE_NAME "labels.txt"

#define LABELS_FILE_LINE_MAX_LENGTH 128

/*
  constructor
 */
AP_Logger_File::AP_Logger_File(AP_Logger &front,
                               LoggerMessageWriter_DFLogStart *writer,
                               const char *log_directory) :
    AP_Logger_Backend(front, writer),
    _log_directory(log_directory),
    _cached_num_logs(-1),
    _next_log_index_check(0),
    _cached_last_log(0),
    _max_log_files(front._params.max_log_files)
{
    _cached_logs_exist = (int8_t*)calloc(_max_log_files, sizeof(_cached_logs_exist[0]));
    df_stats_clear();

    _log_init_errors_buffer = new char[LOG_INIT_ERRORS_BUFFER_SIZE];
    _log_init_errors_buffer[0] = '\0';
}

void AP_Logger_File::clear_cache()
{
    _cached_oldest_log = 0;
    _cached_last_log = 0;
    _cached_num_logs = -1;
    memset(_cached_logs_exist, 0, sizeof(_cached_logs_exist[0]) * _max_log_files);
}

void AP_Logger_File::ensure_log_directory_exists()
{
    int ret;
    struct stat st;

    EXPECT_DELAY_MS(3000);
    ret = AP::FS().stat(_log_directory, &st);
    if (ret == -1) {
        ret = AP::FS().mkdir(_log_directory);
        clear_cache();
    }
    if (ret == -1 && errno != EEXIST) {
        printf("Failed to create log directory %s : %s\n", _log_directory, strerror(errno));
    }
}

void AP_Logger_File::log_init_error(const char *fmt, ...) const
{

    va_list arg_list;
    va_start(arg_list, fmt);

    size_t current_buffer_length = strlen(_log_init_errors_buffer);
    int vsnprintf_ret = hal.util->vsnprintf(_log_init_errors_buffer + current_buffer_length, LOG_INIT_ERRORS_BUFFER_SIZE - current_buffer_length, fmt, arg_list);
    current_buffer_length = ((vsnprintf_ret < 0) ? strlen(_log_init_errors_buffer) : MIN(current_buffer_length + vsnprintf_ret, (unsigned long)LOG_INIT_ERRORS_BUFFER_SIZE));
    if ((current_buffer_length > 0) &&
        (current_buffer_length < (LOG_INIT_ERRORS_BUFFER_SIZE - 1)) &&
        (_log_init_errors_buffer[current_buffer_length - 1] != '\n')) {
        _log_init_errors_buffer[current_buffer_length] = '\n';
        _log_init_errors_buffer[current_buffer_length + 1] = '\0';
    }

    va_end(arg_list);
}

void AP_Logger_File::Init()
{
    clear_cache();
    // determine and limit file backend buffersize
    uint32_t bufsize = _front._params.file_bufsize;
    bufsize *= 1024;

    const uint32_t desired_bufsize = bufsize;

    // If we can't allocate the full size, try to reduce it until we can allocate it
    while (!_writebuf.set_size(bufsize) && bufsize >= _writebuf_chunk) {
        bufsize *= 0.9;
    }
    if (bufsize >= _writebuf_chunk && bufsize != desired_bufsize) {
        hal.console->printf("AP_Logger: reduced buffer %u/%u\n", (unsigned)bufsize, (unsigned)desired_bufsize);
    }

    if (!_writebuf.get_size()) {
        hal.console->printf("Out of memory for logging\n");
        log_init_error("Out of memory for logging\n");
        return;
    }

    hal.console->printf("AP_Logger_File: buffer size=%u\n", (unsigned)bufsize);

    _initialised = true;

    const char* custom_dir = hal.util->get_custom_log_directory();
    if (custom_dir != nullptr){
        _log_directory = custom_dir;
    }

    Prep_MinSpace();
}

bool AP_Logger_File::file_exists(const char *filename) const
{
    struct stat st;
    EXPECT_DELAY_MS(3000);
    if (AP::FS().stat(filename, &st) == -1) {
        // hopefully errno==ENOENT.  If some error occurs it is
        // probably better to assume this file exists.
        return false;
    }
    return true;
}

bool AP_Logger_File::log_exists(const uint16_t lognum)
{
    if ((lognum >= 1) && (lognum <= _max_log_files)) {
        switch (_cached_logs_exist[lognum - 1]) {
            case 1:
                return true;
            case -1:
                return false;
            default:
                break;  // we don't know if the log exist yet, so we continue with the function to find out
        }
    }

    char *filename = _log_file_name(lognum);
    if (filename == nullptr) {
        if ((lognum >= 1) && (lognum <= _max_log_files)) {
            _cached_logs_exist[lognum - 1] = -1;
        }
        return false; // ?!
    }
    bool ret = file_exists(filename);
    free(filename);
    if ((lognum >= 1) && (lognum <= _max_log_files)) {
        _cached_logs_exist[lognum - 1] = (ret ? 1 : -1);
    }
    return ret;
}

void AP_Logger_File::periodic_1Hz()
{
    AP_Logger_Backend::periodic_1Hz();

    if (_initialised &&
        _write_fd == -1 && _read_fd == -1 &&
        erase.log_num == 0 &&
        erase.was_logging) {
        // restart logging after an erase if needed
        erase.was_logging = false;
        // setup to open the log in the backend thread
        start_new_log_pending = true;
    }
    
    if (_initialised &&
        !start_new_log_pending &&
        _write_fd == -1 && _read_fd == -1 &&
        logging_enabled() &&
        !recent_open_error()) {
        // setup to open the log in the backend thread
        start_new_log_pending = true;
    }

    if (!io_thread_alive()) {
        if (io_thread_warning_decimation_counter == 0 && _initialised) {
            // we don't print this error unless we did initialise. When _initialised is set to true
            // we register the IO timer callback
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AP_Logger: stuck thread (%s)", last_io_operation);
        }
        if (io_thread_warning_decimation_counter++ > 30) {
            io_thread_warning_decimation_counter = 0;
        }
    }

    if (rate_limiter == nullptr && _front._params.file_ratemax > 0) {
        // setup rate limiting
        rate_limiter = new AP_Logger_RateLimiter(_front, _front._params.file_ratemax);
    }
}

void AP_Logger_File::periodic_fullrate()
{
    AP_Logger_Backend::push_log_blocks();
}

uint32_t AP_Logger_File::bufferspace_available()
{
    const uint32_t space = _writebuf.space();
    const uint32_t crit = critical_message_reserved_space(_writebuf.get_size());

    return (space > crit) ? space - crit : 0;
}

bool AP_Logger_File::recent_open_error(void) const
{
    if (_open_error_ms == 0) {
        return false;
    }
    return AP_HAL::millis() - _open_error_ms < LOGGER_FILE_REOPEN_MS;
}

// return true for CardInserted() if we successfully initialized
bool AP_Logger_File::CardInserted(void) const
{
    return _initialised && !recent_open_error();
}

// returns the amount of disk space available in _log_directory (in bytes)
// returns -1 on error
int64_t AP_Logger_File::disk_space_avail()
{
    return AP::FS().disk_free(_log_directory);
}

// returns the total amount of disk space (in use + available) in
// _log_directory (in bytes).
// returns -1 on error
int64_t AP_Logger_File::disk_space()
{
    return AP::FS().disk_space(_log_directory);
}

/*
  convert a dirent to a log number
 */
bool AP_Logger_File::dirent_to_log_num(const dirent *de, uint16_t &log_num) const
{
    uint8_t length = strlen(de->d_name);
    if (length < 5) {
        return false;
    }
    if (strncmp(&de->d_name[length-4], ".BIN", 4) != 0) {
        // doesn't end in .BIN
        return false;
    }

    uint16_t thisnum = strtoul(de->d_name, nullptr, 10);
    if (thisnum > MAX_LOG_FILES) {
        return false;
    }
    log_num = thisnum;
    return true;
}


// find_oldest_log - find oldest log in _log_directory
// returns 0 if no log was found
uint16_t AP_Logger_File::find_oldest_log()
{
    if (_cached_oldest_log != 0) {
        return _cached_oldest_log;
    }

    uint16_t last_log_num = find_last_log();
    if (last_log_num == 0) {
        return 0;
    }

    uint16_t current_oldest_log = 0; // 0 is invalid

    // We could count up to find_last_log(), but if people start
    // relying on the min_avail_space_percent feature we could end up
    // doing a *lot* of asprintf()s and stat()s
    EXPECT_DELAY_MS(3000);
    auto *d = AP::FS().opendir(_log_directory);
    if (d == nullptr) {
        // SD card may have died?  On linux someone may have rm-rf-d
        return 0;
    }

    // we only remove files which look like xxx.BIN
    EXPECT_DELAY_MS(3000);
    for (struct dirent *de=AP::FS().readdir(d); de; de=AP::FS().readdir(d)) {
        EXPECT_DELAY_MS(3000);
        uint16_t thisnum;
        if (!dirent_to_log_num(de, thisnum)) {
            // not a log filename
            continue;
        }
        if (current_oldest_log == 0) {
            current_oldest_log = thisnum;
        } else {
            if (current_oldest_log <= last_log_num) {
                if (thisnum > last_log_num) {
                    current_oldest_log = thisnum;
                } else if (thisnum < current_oldest_log) {
                    current_oldest_log = thisnum;
                }
            } else { // current_oldest_log > last_log_num
                if (thisnum > last_log_num) {
                    if (thisnum < current_oldest_log) {
                        current_oldest_log = thisnum;
                    }
                }
            }
        }
    }
    AP::FS().closedir(d);
    _cached_oldest_log = current_oldest_log;

    return current_oldest_log;
}

void AP_Logger_File::Prep_MinSpace()
{
    if (hal.util->was_watchdog_reset()) {
        // don't clear space if watchdog reset, it takes too long
        return;
    }

    if (!CardInserted()) {
        return;
    }

    const uint16_t first_log_to_remove = find_oldest_log();
    if (first_log_to_remove == 0) {
        // no files to remove
        return;
    }

    const int64_t target_free = (int64_t)_front._params.min_MB_free * MB_to_B;

    uint16_t log_to_remove = first_log_to_remove;

    uint16_t count = 0;
    do {
        int64_t avail = disk_space_avail();
        if (avail == -1) {
            break;
        }
        if (avail >= target_free) {
            break;
        }
        if (count++ > _max_log_files+10) {
            // *way* too many deletions going on here.  Possible internal error.
            INTERNAL_ERROR(AP_InternalError::error_t::logger_too_many_deletions);
            break;
        }
        char *filename_to_remove = _log_file_name(log_to_remove);
        if (filename_to_remove == nullptr) {
            INTERNAL_ERROR(AP_InternalError::error_t::logger_bad_getfilename);
            break;
        }
        if (file_exists(filename_to_remove)) {
            hal.console->printf("Removing (%s) for minimum-space requirements (%.0fMB < %.0fMB)\n",
                                filename_to_remove, (double)avail*B_to_MB, (double)target_free*B_to_MB);
            EXPECT_DELAY_MS(2000);
            if (AP::FS().unlink(filename_to_remove) == -1) {
                clear_cache();
                hal.console->printf("Failed to remove %s: %s\n", filename_to_remove, strerror(errno));
                free(filename_to_remove);
                if (errno == ENOENT) {
                    // corruption - should always have a continuous
                    // sequence of files...  however, there may be still
                    // files out there, so keep going.
                } else {
                    break;
                }
            } else {
                free(filename_to_remove);
            }
        }
        log_to_remove++;
        if (log_to_remove > _max_log_files) {
            log_to_remove = 1;
        }
    } while (log_to_remove != first_log_to_remove);
}

/*
  construct a log file name given a log number. 
  The number in the log filename will *not* be zero-padded.
  Note: Caller must free.
 */
char *AP_Logger_File::_log_file_name_short(const uint16_t log_num) const
{
    char *buf = nullptr;
    if (asprintf(&buf, "%s/%u.BIN", _log_directory, (unsigned)log_num) == -1) {
        return nullptr;
    }
    return buf;
}

/*
  construct a log file name given a log number.
  The number in the log filename will be zero-padded.
  Note: Caller must free.
 */
char *AP_Logger_File::_log_file_name_long(const uint16_t log_num) const
{
    char *buf = nullptr;
    if (asprintf(&buf, "%s/%08u.BIN", _log_directory, (unsigned)log_num) == -1) {
        return nullptr;
    }
    return buf;
}

/*
  return a log filename appropriate for the supplied log_num if a
  filename exists with the short (not-zero-padded name) then it is the
  appropirate name, otherwise the long (zero-padded) version is.
  Note: Caller must free.
 */
char *AP_Logger_File::_log_file_name(const uint16_t log_num) const
{
    return _log_file_name_long(log_num);
}

/*
  return path name of the lastlog.txt marker file
  Note: Caller must free.
 */
char *AP_Logger_File::_lastlog_file_name(void) const
{
    char *buf = nullptr;
    if (asprintf(&buf, "%s/LASTLOG.TXT", _log_directory) == -1) {
        return nullptr;
    }
    return buf;
}


// remove all log files
void AP_Logger_File::EraseAll()
{
    if (hal.util->get_soft_armed()) {
        // do not want to do any filesystem operations while we are e.g. flying
        return;
    }
    if (!_initialised) {
        return;
    }

    erase.was_logging = (_write_fd != -1);
    stop_logging();

    erase.log_num = 1;
    clear_cache();
}

bool AP_Logger_File::WritesOK() const
{
    if (_write_fd == -1) {
        return false;
    }
    if (recent_open_error()) {
        return false;
    }
    return true;
}


bool AP_Logger_File::StartNewLogOK() const
{
    if (recent_open_error()) {
        return false;
    }
#if !APM_BUILD_TYPE(APM_BUILD_Replay)
    if (hal.scheduler->in_main_thread()) {
        return false;
    }
#endif
    return AP_Logger_Backend::StartNewLogOK();
}

/* Write a block of data at current offset */
bool AP_Logger_File::_WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical)
{
    WITH_SEMAPHORE(semaphore);

    if (! WriteBlockCheckStartupMessages()) {
        _dropped++;
        return false;
    }

#if APM_BUILD_TYPE(APM_BUILD_Replay)
    if (AP::FS().write(_write_fd, pBuffer, size) != size) {
        AP_HAL::panic("Short write");
    }
    return true;
#endif


    uint32_t space = _writebuf.space();

    if (_writing_startup_messages &&
        _startup_messagewriter->fmt_done()) {
        // the state machine has called us, and it has finished
        // writing format messages out.  It can always get back to us
        // with more messages later, so let's leave room for other
        // things:
        const uint32_t now = AP_HAL::millis();
        const bool must_dribble = (now - last_messagewrite_message_sent) > 100;
        if (!must_dribble &&
            space < non_messagewriter_message_reserved_space(_writebuf.get_size())) {
            // this message isn't dropped, it will be sent again...
            return false;
        }
        last_messagewrite_message_sent = now;
    } else {
        // we reserve some amount of space for critical messages:
        if (!is_critical && space < critical_message_reserved_space(_writebuf.get_size())) {
            _dropped++;
            return false;
        }
    }

    // if no room for entire message - drop it:
    if (space < size) {
        _dropped++;
        return false;
    }

    _writebuf.write((uint8_t*)pBuffer, size);
    df_stats_gather(size, _writebuf.space());
    return true;
}

/*
  asynchroniously send list of labels on MAVLink stream
 */
void AP_Logger_File::list_labels(GCS_MAVLINK* log_sending_link)
{
    if (log_label_data.log_sending_link) {
        // we are already sending a list of labels
        return;
    }
    log_label_data.log_sending_link = log_sending_link;
    log_label_data.list_labels = true;
}

/*
  set the current log file with the specified label
 */
void AP_Logger_File::set_label(const char label[], size_t label_size)
{
    if ((label == nullptr) || (label[0] == '\0')) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "can't set empty label");
        return;
    }
    for (size_t i = 0; i < label_size; ++i) {
        if (label[i] == '\0') {
            break;
        }
        if (!isalnum(label[i])) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "can't set label with non-alphanumeric characters");
            return;
        }
    }

    strncpy(log_label_data.current_log_label, label, sizeof(log_label_data.current_log_label));
    log_label_data.current_log_label[sizeof(log_label_data.current_log_label) - 1] = '\0';
    log_label_data.current_log_label_dirty = true;
}

/*
  Request inner log num from a log label. Returns the log num if found, 0 if the log num will be set later, in which case this should be called again. -1 is returned if log not found.
 */
int16_t AP_Logger_File::log_num_from_label(const char log_label[])
{
    if (strncmp(log_label_data.sending.label, log_label, sizeof(log_label_data.sending.label)) == 0) {
        return log_label_data.sending.log_num;
    }
    strncpy(log_label_data.sending.label, log_label, sizeof(log_label_data.sending.label));
    log_label_data.sending.label[sizeof(log_label_data.sending.label) - 1] = '\0';
    log_label_data.sending.log_num = 0;
    return 0;
}

/*
  find the highest log number
 */
uint16_t AP_Logger_File::find_last_log()
{
    if (_cached_last_log != 0) {
        return _cached_last_log;
    }

    unsigned ret = 0;
    char *fname = _lastlog_file_name();
    if (fname == nullptr) {
        return ret;
    }
    EXPECT_DELAY_MS(3000);
    FileData *fd = AP::FS().load_file(fname);
    free(fname);
    if (fd != nullptr) {
        ret = strtol((const char *)fd->data, nullptr, 10);
        delete fd;
    }
    _cached_last_log = ret;
    return ret;
}

int AP_Logger_File::stat(const uint16_t log_num, const char *name, struct stat *buf)
{
    if ((_cached_file_stat.log_num == log_num) &&
        ((_cached_file_stat.last_accessed_time - AP_HAL::millis()) < MAX_LAST_ACCESSED_TIME_MS)) {
            *buf = _cached_file_stat.stat;
            return 0;
    }
    char *fname = nullptr;
    if (name == nullptr) {
        fname = _log_file_name(log_num);
        if (fname == nullptr) {
            return -1;
        }
        name = fname;
    }
    EXPECT_DELAY_MS(3000);
    int stat_ret = AP::FS().stat(name, buf);
    free(fname);
    _cached_file_stat.log_num = log_num;
    _cached_file_stat.stat = *buf;
    _cached_file_stat.last_accessed_time = AP_HAL::millis();
    return stat_ret;
}

uint32_t AP_Logger_File::_get_log_size(const uint16_t log_num)
{
    char *fname = _log_file_name(log_num);
    if (fname == nullptr) {
        return 0;
    }
    if (_write_fd != -1 && write_fd_semaphore.take_nonblocking()) {
        if (_write_filename != nullptr && strcmp(_write_filename, fname) == 0) {
            // it is the file we are currently writing
            free(fname);
            write_fd_semaphore.give();
            return _write_offset;
        }
        write_fd_semaphore.give();
    }
    struct stat st;
    EXPECT_DELAY_MS(3000);
    if (AP::FS().stat(fname, &st) != 0) {
        if (_open_error_ms == 0) {
            printf("Unable to fetch Log File Size (%s): %s\n", fname, strerror(errno));
        }
        free(fname);
        return 0;
    }
    free(fname);
    return st.st_size;
}

uint32_t AP_Logger_File::_get_log_time(const uint16_t log_num)
{
    char *fname = _log_file_name(log_num);
    if (fname == nullptr) {
        return 0;
    }
    if (_write_fd != -1 && write_fd_semaphore.take_nonblocking()) {
        if (_write_filename != nullptr && strcmp(_write_filename, fname) == 0) {
            // it is the file we are currently writing
            free(fname);
            write_fd_semaphore.give();
            uint64_t utc_usec;
            if (!AP::rtc().get_utc_usec(utc_usec)) {
                return 0;
            }
            return utc_usec / 1000000U;
        }
        write_fd_semaphore.give();
    }
    struct stat st;
    if (stat(log_num, fname, &st) != 0) {
        free(fname);
        return 0;
    }
    free(fname);
    return st.st_mtime;
}

/*
  find the number of pages in a log
 */
void AP_Logger_File::get_log_boundaries(const uint16_t list_entry, uint32_t & start_page, uint32_t & end_page)
{
    const uint16_t log_num = log_num_from_list_entry(list_entry);
    if (log_num == 0) {
        // that failed - probably no logs
        start_page = 0;
        end_page = 0;
        return;
    }

    start_page = 0;
    end_page = _get_log_size(log_num) / LOGGER_PAGE_SIZE;
}

int16_t AP_Logger_File::get_log_data_by_log_num(uint16_t log_num, const uint16_t page, const uint32_t offset, const uint16_t len, uint8_t *data)
{
    if (_read_fd != -1 && log_num != _read_fd_log_num) {
        AP::FS().close(_read_fd);
        _read_fd = -1;
    }
    if (_read_fd == -1) {
        char *fname = _log_file_name(log_num);
        if (fname == nullptr) {
            return -1;
        }
        stop_logging();
        EXPECT_DELAY_MS(3000);
        _read_fd = AP::FS().open(fname, O_RDONLY);
        if (_read_fd == -1) {
            _open_error_ms = AP_HAL::millis();
            int saved_errno = errno;
            ::printf("Log read open fail for %s - %s\n",
                     fname, strerror(saved_errno));
            hal.console->printf("Log read open fail for %s - %s\n",
                                fname, strerror(saved_errno));
            log_init_error("Log read open fail for %s - %s\n",
                     fname, strerror(saved_errno));
            free(fname);
            return -1;            
        }
        free(fname);
        _read_offset = 0;
        _read_fd_log_num = log_num;
    }
    uint32_t ofs = page * (uint32_t)LOGGER_PAGE_SIZE + offset;

    if (ofs != _read_offset) {
        if (AP::FS().lseek(_read_fd, ofs, SEEK_SET) == (off_t)-1) {
            AP::FS().close(_read_fd);
            _read_fd = -1;
            return -1;
        }
        _read_offset = ofs;
    }
    int16_t ret = (int16_t)AP::FS().read(_read_fd, data, len);
    if (ret > 0) {
        _read_offset += ret;
    }
    return ret;
}

/*
  retrieve data from a log file, by label
 */
int16_t AP_Logger_File::get_log_data(const char log_label[], const uint16_t page, const uint32_t offset, const uint16_t len, uint8_t *data)
{
    if (!_initialised || recent_open_error()) {
        return -1;
    }

    const int16_t log_num = log_num_from_label(log_label);
    if (log_num <= 0) {
        // that failed - probably no logs
        return log_num;
    }

    return get_log_data_by_log_num(log_num, page, offset, len, data);
}

/*
  retrieve data from a log file
 */
int16_t AP_Logger_File::get_log_data(const uint16_t list_entry, const uint16_t page, const uint32_t offset, const uint16_t len, uint8_t *data)
{
    if (!_initialised || recent_open_error()) {
        return -1;
    }

    const uint16_t log_num = log_num_from_list_entry(list_entry);
    if (log_num == 0) {
        // that failed - probably no logs
        return -1;
    }

    return get_log_data_by_log_num(log_num, page, offset, len, data);
}

/*
  find size and date of a log, by label. if it's not ready yet, return false (true if it's ready or not found)
 */
bool AP_Logger_File::get_log_info(const char log_label[], uint32_t &size, uint32_t &time_utc)
{
    size = 0;
    time_utc = 0;
    int16_t log_num = log_num_from_label(log_label);
    if (log_num <= 0) {
        return log_num < 0;  // negative means not found, zero means not ready
    }

    size = _get_log_size(log_num);
    time_utc = _get_log_time(log_num);
    return true;
}

/*
  find size and date of a log
 */
void AP_Logger_File::get_log_info(const uint16_t list_entry, uint32_t &size, uint32_t &time_utc)
{
    uint16_t log_num = log_num_from_list_entry(list_entry);
    if (log_num == 0) {
        // that failed - probably no logs
        size = 0;
        time_utc = 0;
        return;
    }

    size = _get_log_size(log_num);
    time_utc = _get_log_time(log_num);
}


/*
  get the number of logs - note that the log numbers must be consecutive
 */
uint16_t AP_Logger_File::get_num_logs()
{
    if (_cached_num_logs >= 0) {
        return _cached_num_logs;
    }

    auto *d = AP::FS().opendir(_log_directory);
    if (d == nullptr) {
        return 0;
    }

    uint16_t high = find_last_log();
    uint16_t ret = high;
    uint16_t smallest_above_last = 0;

    EXPECT_DELAY_MS(2000);
    for (struct dirent *de=AP::FS().readdir(d); de; de=AP::FS().readdir(d)) {
        EXPECT_DELAY_MS(100);
        uint16_t thisnum;
        if (!dirent_to_log_num(de, thisnum)) {
            // not a log filename
            continue;
        }
        if (thisnum > high && (smallest_above_last == 0 || thisnum < smallest_above_last)) {
            smallest_above_last = thisnum;
        }
    }
    AP::FS().closedir(d);
    if (smallest_above_last != 0) {
        // we have wrapped, add in the logs with high numbers
        ret += (MAX_LOG_FILES - smallest_above_last) + 1;
    }

    _cached_num_logs = ret;
    return ret;
}

/*
  stop logging
 */
void AP_Logger_File::stop_logging(void)
{
    // best-case effort to avoid annoying the IO thread
    const bool have_sem = write_fd_semaphore.take(hal.util->get_soft_armed()?1:20);
    if (_write_fd != -1) {
        int fd = _write_fd;
        _write_fd = -1;
        AP::FS().close(fd);
    }

    if (have_sem) {
        write_fd_semaphore.give();
    }
}

/*
  does start_new_log in the logger thread
 */
void AP_Logger_File::PrepForArming_start_logging()
{
    if (logging_started()) {
        return;
    }

    uint32_t start_ms = AP_HAL::millis();
    const uint32_t open_limit_ms = 1000;

    /*
      log open happens in the io_timer thread. We allow for a maximum
      of 1s to complete the open
     */
    start_new_log_pending = true;
    EXPECT_DELAY_MS(1000);
    while (AP_HAL::millis() - start_ms < open_limit_ms) {
        if (logging_started()) {
            break;
        }
#if !APM_BUILD_TYPE(APM_BUILD_Replay) && !defined(HAL_BUILD_AP_PERIPH)
        // keep the EKF ticking over
        AP::ahrs().update();
#endif
        hal.scheduler->delay(1);
    }
}

/*
  start writing to a new log file
 */
void AP_Logger_File::start_new_log(void)
{
    if (recent_open_error()) {
        // we have previously failed to open a file - don't try again
        // to prevent us trying to open files while in flight
        return;
    }

    if (erase.log_num != 0) {
        // don't start a new log while erasing, but record that we
        // want to start logging when erase finished
        erase.was_logging = true;
        return;
    }

    clear_cache();

    const bool open_error_ms_was_zero = (_open_error_ms == 0);

    // set _open_error here to avoid infinite recursion.  Simply
    // writing a prioritised block may try to open a log - which means
    // if anything in the start_new_log path does a gcs().send_text()
    // (for example), you will end up recursing if we don't take
    // precautions.  We will reset _open_error if we actually manage
    // to open the log...
    _open_error_ms = AP_HAL::millis();

    stop_logging();

    start_new_log_reset_variables();

    if (_read_fd != -1) {
        AP::FS().close(_read_fd);
        _read_fd = -1;
    }

    if (disk_space_avail() < _free_space_min_avail && disk_space() > 0) {
        hal.console->printf("Out of space for logging\n");
        log_init_error("Out of space for logging");
        return;
    }

    uint16_t log_num = find_last_log();
    // re-use empty logs if possible
    if (_get_log_size(log_num) > 0 || log_num == 0) {
        log_num++;
    }
    if (log_num > _max_log_files) {
        log_num = 1;
    }
    if (!write_fd_semaphore.take(1)) {
        log_init_error("Failed to take write_fd_semaphore");
        return;
    }
    if (_write_filename) {
        free(_write_filename);
        _write_filename = nullptr;        
    }
    _write_filename = _log_file_name(log_num);
    if (_write_filename == nullptr) {
        write_fd_semaphore.give();
        log_init_error("Failed to allocate write_filename");
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    // remember if we had utc time when we opened the file
    uint64_t utc_usec;
    _need_rtc_update = !AP::rtc().get_utc_usec(utc_usec);
#endif

    // create the log directory if need be
    ensure_log_directory_exists();

    EXPECT_DELAY_MS(3000);
    _write_fd = AP::FS().open(_write_filename, O_WRONLY|O_CREAT|O_TRUNC);
    clear_cache();

    if (_write_fd == -1) {
        write_fd_semaphore.give();
        int saved_errno = errno;
        if (open_error_ms_was_zero) {
            ::printf("Log open fail for %s - %s\n",
                     _write_filename, strerror(saved_errno));
            hal.console->printf("Log open fail for %s - %s\n",
                                _write_filename, strerror(saved_errno));
            log_init_error("Log open fail for %s - %s\n",
                                    _write_filename, strerror(saved_errno));
        }
        return;
    }
    _write_log_num = log_num;
    _last_write_ms = AP_HAL::millis();
    _open_error_ms = 0;
    _write_offset = 0;
    _writebuf.clear();
    log_label_data.log_num_erased = log_num;
    if (!write_new_log_header()) {
        write_fd_semaphore.give();
        return;
    }
    write_fd_semaphore.give();

    // now update lastlog.txt with the new log number
    char *fname = _lastlog_file_name();

    EXPECT_DELAY_MS(3000);
    int fd = AP::FS().open(fname, O_WRONLY|O_CREAT);
    free(fname);
    if (fd == -1) {
        _open_error_ms = AP_HAL::millis();
        log_init_error("Failed to open %s", fname);
        return;
    }

    char buf[30];
    snprintf(buf, sizeof(buf), "%u\r\n", (unsigned)log_num);
    const ssize_t to_write = strlen(buf);
    const ssize_t written = AP::FS().write(fd, buf, to_write);
    AP::FS().close(fd);
    clear_cache();

    if (written < to_write) {
        _open_error_ms = AP_HAL::millis();
        log_init_error("Failed to write %s", fname);
        return;
    }

    return;
}


#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
void AP_Logger_File::flush(void)
#if APM_BUILD_TYPE(APM_BUILD_Replay) || APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
{
    uint32_t tnow = AP_HAL::millis();
    while (_write_fd != -1 && _initialised && !recent_open_error() && _writebuf.available()) {
        // convince the IO timer that it really is OK to write out
        // less than _writebuf_chunk bytes:
        if (tnow > 2001) { // avoid resetting _last_write_time to 0
            _last_write_time = tnow - 2001;
        }
        io_timer();
    }
    if (write_fd_semaphore.take(1)) {
        if (_write_fd != -1) {
            ::fsync(_write_fd);
        }
        write_fd_semaphore.give();
    } else {
        INTERNAL_ERROR(AP_InternalError::error_t::logger_flushing_without_sem);
    }
}
#else
{
    // flush is for replay and examples only
}
#endif // APM_BUILD_TYPE(APM_BUILD_Replay) || APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
#endif

// return true on success, false otherwise
bool AP_Logger_File::_init_log_labels_file_name()
{
    if (log_label_data.labels_file_name != nullptr) {
        return true;
    }

    // create the log directory if need be
    ensure_log_directory_exists();

    char *labels_file_name;
    if (asprintf(&labels_file_name, "%s/%s", _log_directory, LOG_FILES_LABEL_MAP_FILE_NAME) == -1) {
        log_label_data.labels_file_name = nullptr;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Can't allocate label map file name");
        return false;
    }

    // verifying file exists, and create if not
    int fd = AP::FS().open(labels_file_name, O_WRONLY|O_APPEND|O_CREAT);
    if (fd == -1) {
        _open_error_ms = AP_HAL::millis();
        int saved_errno = errno;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Can't open labels file for initializing:");
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s", strerror(saved_errno));
        log_init_error("Can't open labels file for initializing: %s", strerror(saved_errno));
        free(labels_file_name);
        return false;
    }
    AP::FS().close(fd);
    log_label_data.labels_file_name = labels_file_name;
    return true;
}

void AP_Logger_File::_log_label_handle_log_erase_all()
{
    int fd;
    if (log_label_data.labels_file_truncated) {
        return;
    }
    EXPECT_DELAY_MS(3000);
    fd = AP::FS().open(log_label_data.labels_file_name, O_WRONLY|O_CREAT|O_TRUNC);
    if (fd == -1) {
        _open_error_ms = AP_HAL::millis();
        int saved_errno = errno;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Can't open labels file for truncate:");
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s", strerror(saved_errno));
        log_init_error("Can't open labels file for truncate: %s", strerror(saved_errno));
        return;
    }
    AP::FS().close(fd);
    log_label_data.labels_file_truncated = true;
}

bool AP_Logger_File::_write_to_labels_file(uint8_t new_content[], uint16_t new_content_len)
{
    int fd = AP::FS().open(log_label_data.labels_file_name, O_WRONLY|O_TRUNC|O_CREAT);
    if (fd == -1) {
        _open_error_ms = AP_HAL::millis();
        int saved_errno = errno;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Can't open labels file for writing changes back to file:");
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s", strerror(saved_errno));
        log_init_error("Can't open labels file for writing changes back to file: %s", strerror(saved_errno));
        return false;
    }
    const ssize_t written = AP::FS().write(fd, new_content, new_content_len);
    log_label_data.labels_file_truncated = false;
    AP::FS().close(fd);

    if (written < new_content_len) {
        _open_error_ms = AP_HAL::millis();
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Short write to log label map file");
        log_init_error("Short write to log label map file");
        return false;
    }

    return true;
}

void AP_Logger_File::_erase_label(const char label[])
{
    log_num_label_map_t *map = nullptr;
    int32_t map_size = _get_log_num_label_map(&map);
    if (map_size < 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Can't retrieve map of log labels");
        return;
    }
    if (map_size == 0) {
        return;
    }

    uint32_t new_content_size = map_size * sizeof(map[0].label) * 2;
    uint8_t *new_content = new uint8_t[new_content_size] { };
    uint32_t total_bytes_written = 0;
    bool is_dirty = false;
    for (int32_t i = 0; i < map_size; ++i) {
        if (strncmp(map[i].label, label, sizeof(map[i].label)) != 0) {
            snprintf((char*)(new_content + total_bytes_written), new_content_size - total_bytes_written, "%s,%" PRId32 "\n", map[i].label, map[i].log_num);
            total_bytes_written += strlen((char*)(new_content + total_bytes_written));
        } else {
            is_dirty = true;
        }
    }
    
    delete[] map;

    if (is_dirty) {
        _write_to_labels_file(new_content, total_bytes_written);
    }

    delete[] new_content;
}

bool AP_Logger_File::_erase_labels_for_log_num(const uint16_t log_num)
{
    log_num_label_map_t *map = nullptr;
    int32_t map_size = _get_log_num_label_map(&map);
    if (map_size < 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Can't retrieve map of log labels");
        return false;
    }
    if (map_size == 0) {
        return true;
    }

    uint32_t new_content_size = map_size * sizeof(map[0].label) * 2;
    uint8_t *new_content = new uint8_t[new_content_size] { };
    uint32_t total_bytes_written = 0;
    bool is_dirty = false;
    for (int32_t i = 0; i < map_size; ++i) {
        if (map[i].log_num != log_num) {
            snprintf((char*)(new_content + total_bytes_written), new_content_size - total_bytes_written, "%s,%" PRId32 "\n", map[i].label, map[i].log_num);
            total_bytes_written += strlen((char*)(new_content + total_bytes_written));
        } else {
            is_dirty = true;
        }
    }
    
    delete[] map;

    bool write_success = true;
    if (is_dirty) {
        write_success = _write_to_labels_file(new_content, total_bytes_written);
    }

    delete[] new_content;

    return write_success;
}

void AP_Logger_File::_log_label_handle_log_erase()
{
    if (_erase_labels_for_log_num(log_label_data.log_num_erased)) {
        log_label_data.log_num_erased = 0;
    }
}

bool AP_Logger_File::_set_current_log_label()
{
    char buf[LABELS_FILE_LINE_MAX_LENGTH] = { 0 };
    if (_write_filename == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "No open log file. can't map to a label");
        return false;
    }
    int32_t log_num = _label_to_log_num(log_label_data.current_log_label);
    if (log_num > 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Log label already exists");
        if (log_num == _write_log_num) {
            return true;
        }
        _erase_label(log_label_data.current_log_label);
    }
    EXPECT_DELAY_MS(3000);
    int fd = AP::FS().open(log_label_data.labels_file_name, O_WRONLY|O_APPEND|O_CREAT);
    if (fd == -1) {
        _open_error_ms = AP_HAL::millis();
        int saved_errno = errno;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Can't open labels file for label update:");
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s", strerror(saved_errno));
        log_init_error("Can't open labels file for label update: %s", strerror(saved_errno));
        return false;
    }
    snprintf(buf, sizeof(buf), "%s,%hu\n", log_label_data.current_log_label, _write_log_num);
    const ssize_t to_write = strlen(buf);
    const ssize_t written = AP::FS().write(fd, buf, to_write);
    log_label_data.labels_file_truncated = false;
    AP::FS().close(fd);

    if (written < to_write) {
        _open_error_ms = AP_HAL::millis();
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Short write to log label map file");
        log_init_error("Short write to log label map file");
        return false;
    }
    return true;
}

int32_t AP_Logger_File::_label_to_log_num(const char label[])
{
    int32_t log_num = -1;
    log_num_label_map_t *map = nullptr;
    int32_t map_size = _get_log_num_label_map(&map);
    if (map_size < 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Can't retrieve map of log labels");
        return -1;
    }
    for (int32_t i = 0; i < map_size; ++i) {
        if (strncmp(map[i].label, label, sizeof(map[i].label)) == 0) {
            log_num = map[i].log_num;
            break;            
        }
    }
    
    delete[] map;
    return log_num;
}

void AP_Logger_File::_retrieve_log_num()
{
    log_label_data.sending.log_num = _label_to_log_num(log_label_data.sending.label);
}

// return the length of the current line in a char array, including the newline character. examine up to count characters.
// return 0 if no newline charachter is found before count characters.
size_t AP_Logger_File::_get_line_length(const char *line, size_t count) const
{
    const char *newline_ptr = (char *)memchr(line, '\n', count);
    if (newline_ptr == nullptr) {
        return 0;
    }

    return newline_ptr - line + 1;
}

// return an allocated null-terminated string containing the next line (including newline character) in the input string, up to count characters long.
// return nullptr if no newline charachter is found before count characters, or can't allocate memory.
char *AP_Logger_File::_allocate_copy_line(const char *str, size_t count) const
{
    size_t line_length = _get_line_length(str, count);
    if (line_length == 0) {
        return nullptr;
    }
    char *data_str = (char *)malloc(line_length + 1);
    if (data_str == nullptr) {
        return nullptr;
    }

    memcpy(data_str, str, line_length);
    data_str[line_length] = '\0';
    return data_str;
}

// retrieve an array of log labels from the log label file. returns the number of labels found, or -1 on error
// Use delete[] to free the map
int32_t AP_Logger_File::_get_log_num_label_map(log_num_label_map_t **map)
{
    int32_t entries = 0;
    *map = nullptr;

    EXPECT_DELAY_MS(3000);
    FileData *file_data = AP::FS().load_file(log_label_data.labels_file_name);
    if (file_data == nullptr) {
        _open_error_ms = AP_HAL::millis();
        int saved_errno = errno;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Can't open labels file for retrieve log num:");
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s", strerror(saved_errno));
        log_init_error("Can't open labels file for retrieve log num: %s", strerror(saved_errno));
        return -1;
    }

    for (uint32_t i = 0; i < file_data->length; ++i) {
        if (file_data->data[i] == '\n') {
            entries++;
        }
    }
    *map = new log_num_label_map_t[entries];

    char buf[MAVLINK_MSG_SET_LOG_LABEL_FIELD_LABEL_LEN] = { 0 };
    uint16_t log_num_iter = 0;
    int bytes_read = 0;
    uint32_t total_bytes_read = 0;
    uint32_t idx = 0;
    bool parsing_error = false;
    char *line = nullptr;
    while (total_bytes_read < file_data->length) {
        line = _allocate_copy_line((char *)(file_data->data + total_bytes_read), file_data->length - total_bytes_read);
        if (line == nullptr) {
            parsing_error = true;
            break;
        }
        if (sscanf(line, "%[^,],%hu\n%n", buf, &log_num_iter, &bytes_read) != 2) {
            parsing_error = true;
            break;
        }
        free(line);
        line = nullptr;

        (*map)[idx].log_num = log_num_iter;
        strncpy((*map)[idx].label, buf, sizeof((*map)[idx].label));
        (*map)[idx].label[sizeof((*map)[idx].label) - 1] = '\0';
        ++idx;
        total_bytes_read += bytes_read;
    }
    if (line != nullptr) {
        free(line);
        line = nullptr;
    }
    if (parsing_error) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "failed to parse %s", log_label_data.labels_file_name);
        entries = -1;
    }

    if (total_bytes_read != file_data->length) {
        entries = -1;
    }
    if (entries < 0) {
        delete[] *map;
        *map = nullptr;
    }
    delete file_data;
    return entries;
}

// send all labels to GCS
void AP_Logger_File::_list_labels()
{
    if (!HAVE_PAYLOAD_SPACE(log_label_data.log_sending_link->get_chan(), LOG_LABEL)) {
        return;  // no space
    }
    if (AP_HAL::millis() - log_label_data.log_sending_link->get_last_heartbeat_time() > 3000) {
        return;  // give a heartbeat a chance
    }

    log_num_label_map_t *map = nullptr;
    int32_t map_size = _get_log_num_label_map(&map);
    if (map_size < 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Can't retrieve map of log labels");
        return;
    }
    for (int32_t i = 0; i < map_size; ++i) {
        uint32_t size = _get_log_size(map[i].log_num);
        uint32_t time_utc = _get_log_time(map[i].log_num);
        mavlink_msg_log_label_send(log_label_data.log_sending_link->get_chan(),
                                   i,
                                   map[i].label,
                                   map_size,
                                   map[i].log_num,
                                   time_utc,
                                   size);
    }

    delete[] map;
    log_label_data.log_sending_link = nullptr;
    log_label_data.list_labels = false;
}

// should be called from the io_timer thread
void AP_Logger_File::_log_label_update()
{
    if (recent_open_error()) {
        // we have previously failed to open a file - don't try again
        // to prevent us trying to open files while in flight
        return;
    }

    if (!_init_log_labels_file_name()) {
        return;
    }

    if (erase.log_num != 0) {  // this means that eventually it will erase all logs
        _log_label_handle_log_erase_all();
        return; // don't continue while erasing. wait for it to finish all.
    }

    if (log_label_data.log_num_erased > 0) {
        _log_label_handle_log_erase();
    }

    if (log_label_data.current_log_label_dirty) {
        if (!_set_current_log_label()) {
            return;
        }
        log_label_data.current_log_label_dirty = false;
    }

    if ((log_label_data.sending.label[0] != '\0') && (log_label_data.sending.log_num == 0)) {
        _retrieve_log_num();
    }

    if (log_label_data.list_labels) {
        _list_labels();
    }
}
ssize_t AP_Logger_File::write_to_file(const void *buf, uint32_t count)
{
    return AP::FS().write(_write_fd, buf, count);
}

void AP_Logger_File::io_timer(void)
{
    uint32_t tnow = AP_HAL::millis();
    _io_timer_heartbeat = tnow;

    if (start_new_log_pending) {
        start_new_log();
        start_new_log_pending = false;
    }

    _log_label_update();

    if (erase.log_num != 0) {
        // continue erase
        erase_next();
        return;
    }

    if (_write_fd == -1 || !_initialised || recent_open_error()) {
        return;
    }

    uint32_t nbytes = _writebuf.available();
    if (nbytes == 0) {
        return;
    }
    if (nbytes < _writebuf_chunk && 
        tnow - _last_write_time < 2000UL) {
        // write in _writebuf_chunk-sized chunks, but always write at
        // least once per 2 seconds if data is available
        return;
    }
    if (tnow - _free_space_last_check_time > _free_space_check_interval) {
        _free_space_last_check_time = tnow;
        last_io_operation = "disk_space_avail";
        if (disk_space_avail() < _free_space_min_avail && disk_space() > 0) {
            hal.console->printf("Out of space for logging\n");
            log_init_error("Out of space for logging");
            stop_logging();
            _open_error_ms = AP_HAL::millis(); // prevent logging starting again for 5s
            last_io_operation = "";
            return;
        }
        last_io_operation = "";
    }

    _last_write_time = tnow;
    if (nbytes > _writebuf_chunk) {
        // be kind to the filesystem layer
        nbytes = _writebuf_chunk;
    }

    uint32_t size;
    const uint8_t *head = _writebuf.readptr(size);
    nbytes = MIN(nbytes, size);

    // try to align writes on a 512 byte boundary to avoid filesystem reads
    if ((nbytes + _write_offset) % 512 != 0) {
        uint32_t ofs = (nbytes + _write_offset) % 512;
        if (ofs < nbytes) {
            nbytes -= ofs;
        }
    }

    last_io_operation = "write";
    if (!write_fd_semaphore.take(1)) {
        return;
    }
    if (_write_fd == -1) {
        write_fd_semaphore.give();
        return;
    }
    ssize_t nwritten = write_to_file(head, nbytes);

    last_io_operation = "";
    if (nwritten <= 0) {
        if ((tnow - _last_write_ms)/1000U > unsigned(_front._params.file_timeout)) {
            // if we can't write for LOG_FILE_TIMEOUT seconds we give up and close
            // the file. This allows us to cope with temporary write
            // failures caused by directory listing
            last_io_operation = "close";
            AP::FS().close(_write_fd);
            last_io_operation = "";
            _write_fd = -1;
            stop_logging();
            printf("Failed to write to File: %s\n", strerror(errno));
            log_init_error("Failed to write to File: %s", strerror(errno));
        }
        _last_write_failed = true;
    } else {
        _last_write_failed = false;
        _last_write_ms = tnow;
        _write_offset += nwritten;
        _writebuf.advance(nwritten);
        /*
          the best strategy for minimizing corruption on microSD cards
          seems to be to write in 4k chunks and fsync the file on each
          chunk, ensuring the directory entry is updated after each
          write.
         */
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL && CONFIG_HAL_BOARD_SUBTYPE != HAL_BOARD_SUBTYPE_LINUX_NONE
        last_io_operation = "fsync";
        AP::FS().fsync(_write_fd);
        last_io_operation = "";
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        // ChibiOS does not update mtime on writes, so if we opened
        // without knowing the time we should update it later
        if (_need_rtc_update) {
            uint64_t utc_usec;
            if (AP::rtc().get_utc_usec(utc_usec)) {
                AP::FS().set_mtime(_write_filename, utc_usec/(1000U*1000U));
                _need_rtc_update = false;
            }
        }
#endif
    }

    // calling log_exists on all log indexes to cache their existence
    // this is needed due to very slow FS operations when trying to know how many log files exist at once
    // so we check one file every second and cache it instead of all at once.
    log_exists(_next_log_index_check + 1);
    _next_log_index_check = (_next_log_index_check + 1) % _max_log_files;

    write_fd_semaphore.give();
}

bool AP_Logger_File::io_thread_alive() const
{
    if (!hal.scheduler->is_system_initialized()) {
        // the system has long pauses during initialisation, assume still OK
        return true;
    }
    // if the io thread hasn't had a heartbeat in a while then it is
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    uint32_t timeout_ms = 10000;
#else
    uint32_t timeout_ms = 5000;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)
    // the IO thread is working with hardware - writing to a physical
    // disk.  Unfortunately these hardware devices do not obey our
    // SITL speedup options, so we allow for it here.
    SITL::SIM *sitl = AP::sitl();
    if (sitl != nullptr) {
        timeout_ms *= sitl->speedup;
    }
#endif
    return (AP_HAL::millis() - _io_timer_heartbeat) < timeout_ms;
}

void AP_Logger_File::print_log_init_errors() const
{
    const char* newline_ptr = nullptr;
    const char* message_start_ptr = nullptr;
    char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1] { };
    if (_log_init_errors_buffer) {
        newline_ptr = _log_init_errors_buffer;
        message_start_ptr = _log_init_errors_buffer;
        while ((newline_ptr = strchr(message_start_ptr, '\n')) != nullptr) {
            strncpy(buffer, message_start_ptr, MIN(MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN, newline_ptr - message_start_ptr));
            buffer[MIN(MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN, newline_ptr - message_start_ptr)] = '\0';
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "%s", buffer);
            message_start_ptr = newline_ptr + 1;
        }
    }
}

bool AP_Logger_File::logging_failed() const
{
    if (!_initialised) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Logging failed - not initialised");
        print_log_init_errors();
        return true;
    }
    if (recent_open_error()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Logging failed - recent open error");
        print_log_init_errors();
        return true;
    }
    if (!io_thread_alive()) {
        // No heartbeat in a second.  IO thread is dead?! Very Not
        // Good.
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Logging failed - IO thread not responding");
        print_log_init_errors();
        return true;
    }
    if (_last_write_failed) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Logging failed - last write failed");
        print_log_init_errors();
        return true;
    }

    return false;
}

/*
  erase another file in async erase operation
 */
void AP_Logger_File::erase_next(void)
{
    char *fname = _log_file_name(erase.log_num);
    if (fname == nullptr) {
        erase.log_num = 0;
        return;
    }

    clear_cache();

    AP::FS().unlink(fname);
    free(fname);

    erase.log_num++;
    if (erase.log_num <= _max_log_files) {
        return;
    }
    
    fname = _lastlog_file_name();
    if (fname != nullptr) {
        AP::FS().unlink(fname);
        free(fname);
    }

    clear_cache();

    erase.log_num = 0;
}

#endif // HAL_LOGGING_FILESYSTEM_ENABLED

