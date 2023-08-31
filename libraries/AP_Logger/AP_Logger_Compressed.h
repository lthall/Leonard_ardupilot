/* 
   AP_Logger logging - compressed file oriented variant

   This uses heatshrink library to create compressed log files
 */
#pragma once

#include "AP_Logger_File.h"

extern "C" {
#include <heatshrink_encoder.h>
}

#if HAL_LOGGING_FILESYSTEM_ENABLED

class AP_Logger_Compressed : public AP_Logger_File
{
public:
    // constructor
    AP_Logger_Compressed(AP_Logger &front,
                         LoggerMessageWriter_DFLogStart *writer,
                         const char *log_directory) :
        AP_Logger_File(front, writer, log_directory) {}

    static AP_Logger_Backend  *probe(AP_Logger &front,
                                     LoggerMessageWriter_DFLogStart *ls) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        char *log_dir = nullptr;
        if (asprintf(&log_dir, "%s_%s", AP::fwversion().middleware_name, HAL_BOARD_COMPRESSED_LOG_DIRECTORY) == -1) {
            AP_HAL::panic("Out of memory");
            return nullptr;
        }
#else
        const char *log_dir = HAL_BOARD_COMPRESSED_LOG_DIRECTORY;
#endif
        AP_HAL::get_HAL().console->printf("log directory: %s\n", log_dir);
        return new AP_Logger_Compressed(front, ls, log_dir);
    }

    // initialisation
    void Init() override;

    bool write_new_log_header(void) override;

protected:
    char *_log_file_name_long(const uint16_t log_num) const override;
    bool dirent_to_log_num(const dirent *de, uint16_t &log_num) const override;

    void stop_logging(void) override;

    ssize_t write_to_file(const void *buf, uint32_t count) override;

private:
    uint8_t _compressed_write_buffer[HAL_LOGGER_WRITE_CHUNK_SIZE];

    ssize_t _write_compressed_log_chunk(uint8_t *buffer, ssize_t input_len);

    heatshrink_encoder *hse;
};

#endif // HAL_LOGGING_FILESYSTEM_ENABLED
