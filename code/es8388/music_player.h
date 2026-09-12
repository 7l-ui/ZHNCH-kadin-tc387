/*********************************************************************************************************************
* ???????TC387???
* ?????SD???WAV???????ES8388??
********************************************************************************************************************/

#ifndef _MUSIC_PLAYER_H_
#define _MUSIC_PLAYER_H_

#include "../../libraries/zf_common/zf_common_typedef.h"
#include "sd_simple.h"
#include "es8388_unified.h"
#include "ff.h"

typedef struct
{
    uint32_t ChunkID;
    uint32_t ChunkSize;
    uint32_t Format;
} wav_riff_chunk_struct;

typedef struct
{
    uint32_t ChunkID;
    uint32_t ChunkSize;
    uint16_t AudioFormat;
    uint16_t NumOfChannels;
    uint32_t SampleRate;
    uint32_t ByteRate;
    uint16_t BlockAlign;
    uint16_t BitsPerSample;
} wav_fmt_chunk_struct;

typedef struct
{
    uint32_t ChunkID;
    uint32_t ChunkSize;
} wav_data_chunk_struct;

typedef struct
{
    wav_riff_chunk_struct riff;
    wav_fmt_chunk_struct fmt;
    wav_data_chunk_struct data;
} wav_header_struct;

typedef enum
{
    MUSIC_PLAYER_STOPPED = 0,
    MUSIC_PLAYER_PLAYING,
    MUSIC_PLAYER_PAUSED,
    MUSIC_PLAYER_ERROR
} music_player_state_enum;

typedef enum
{
    MUSIC_PLAY_MODE_SINGLE = 0,
    MUSIC_PLAY_MODE_REPEAT_ONE,
    MUSIC_PLAY_MODE_REPEAT_ALL,
    MUSIC_PLAY_MODE_RANDOM
} music_play_mode_enum;

typedef struct
{
    char filename[256];
    uint32_t sample_rate;
    uint16_t bits_per_sample;
    uint16_t channels;
    uint32_t duration_seconds;
    uint32_t data_size;
    uint32_t bitrate;
} music_info_struct;

typedef struct
{
    music_player_state_enum state;
    music_play_mode_enum play_mode;
    music_info_struct current_music;
    uint32_t current_position;
    uint32_t total_files;
    uint32_t current_file_index;
    uint8_t volume;
    uint8_t loop_count;
    uint8_t auto_play;
} music_player_struct;

#ifndef MUSIC_PLAYER_BUFFER_SIZE
#define MUSIC_PLAYER_BUFFER_SIZE  16384
#endif

#ifndef MUSIC_PLAYER_ASYNC_ENABLE
#define MUSIC_PLAYER_ASYNC_ENABLE 1U
#endif

#ifndef MUSIC_PLAYER_ASYNC_RING_FRAMES
#define MUSIC_PLAYER_ASYNC_RING_FRAMES 20480U
#endif

#ifndef MUSIC_PLAYER_ASYNC_START_FRAMES
#define MUSIC_PLAYER_ASYNC_START_FRAMES 8192U
#endif

#ifndef MUSIC_PLAYER_ASYNC_SEND_BATCH
#define MUSIC_PLAYER_ASYNC_SEND_BATCH 1024U
#endif

#ifndef MUSIC_PLAYER_ASYNC_DIAG
#define MUSIC_PLAYER_ASYNC_DIAG 0U
#endif

#ifndef MUSIC_PLAYER_ASYNC_PACE_ENABLE
#define MUSIC_PLAYER_ASYNC_PACE_ENABLE 1U
#endif

#ifndef MUSIC_PLAYER_MAX_FILES
#define MUSIC_PLAYER_MAX_FILES    8
#endif

#ifndef MUSIC_PLAYER_PATH_LEN
#define MUSIC_PLAYER_PATH_LEN     96
#endif

#ifndef MUSIC_PLAYER_OUTPUT_SAMPLE_RATE
#define MUSIC_PLAYER_OUTPUT_SAMPLE_RATE 32000U
#endif

#ifndef MUSIC_PLAYER_OUTPUT_MATCH_WAV_RATE
#define MUSIC_PLAYER_OUTPUT_MATCH_WAV_RATE 1U
#endif

#ifndef MUSIC_PLAYER_DIRECT_MIN_SAMPLE_RATE
#define MUSIC_PLAYER_DIRECT_MIN_SAMPLE_RATE 8000U
#endif

#ifndef MUSIC_PLAYER_DIRECT_MAX_SAMPLE_RATE
#define MUSIC_PLAYER_DIRECT_MAX_SAMPLE_RATE 32000U
#endif

#ifndef MUSIC_PLAYER_DEFAULT_VOLUME
#define MUSIC_PLAYER_DEFAULT_VOLUME 75U
#endif

#ifndef MUSIC_PLAYER_DIGITAL_GAIN_PERCENT
#define MUSIC_PLAYER_DIGITAL_GAIN_PERCENT 25U
#endif

#ifndef MUSIC_PLAYER_STREAM_PROBE_TONE_MS
#define MUSIC_PLAYER_STREAM_PROBE_TONE_MS 0U
#endif

#ifndef MUSIC_PLAYER_IO_TIMING_LOG
#define MUSIC_PLAYER_IO_TIMING_LOG 0U
#endif

#ifndef MUSIC_PLAYER_STREAM_STABILIZE_MS
#define MUSIC_PLAYER_STREAM_STABILIZE_MS 80U
#endif

#ifndef MUSIC_PLAYER_PROBE_TONE_AMP
#define MUSIC_PLAYER_PROBE_TONE_AMP 12000
#endif

#ifndef MUSIC_PLAYER_I2S_FORMAT
#define MUSIC_PLAYER_I2S_FORMAT 1U
#endif

#ifndef MUSIC_PLAYER_TX_BACKEND_SOFT_IIS
#define MUSIC_PLAYER_TX_BACKEND_SOFT_IIS 0U
#endif

#ifndef MUSIC_PLAYER_TX_BACKEND_QSPI1_SLAVE
#define MUSIC_PLAYER_TX_BACKEND_QSPI1_SLAVE 1U
#endif

#ifndef MUSIC_PLAYER_TX_BACKEND
#define MUSIC_PLAYER_TX_BACKEND MUSIC_PLAYER_TX_BACKEND_SOFT_IIS
#endif

#ifndef MUSIC_PLAYER_QSPI_MCLK_RATIO
#define MUSIC_PLAYER_QSPI_MCLK_RATIO 256U
#endif

#ifndef MUSIC_PLAYER_QSPI_TX_WAIT_GUARD
#define MUSIC_PLAYER_QSPI_TX_WAIT_GUARD 1000000U
#endif

#ifndef MUSIC_PLAYER_QSPI_SHIFT_TRAILING
#define MUSIC_PLAYER_QSPI_SHIFT_TRAILING 1U
#endif

#ifndef MUSIC_PLAYER_QSPI_START_SYNC_WS
#define MUSIC_PLAYER_QSPI_START_SYNC_WS 1U
#endif

#ifndef MUSIC_PLAYER_QSPI_WS_SYNC_TIMEOUT_MS
#define MUSIC_PLAYER_QSPI_WS_SYNC_TIMEOUT_MS 20U
#endif

#ifndef MUSIC_PLAYER_QSPI_PREFILL_WORDS
#define MUSIC_PLAYER_QSPI_PREFILL_WORDS 7U
#endif

#ifdef MUSIC_PLAYER_I2S_SLOT_BITS
#undef MUSIC_PLAYER_I2S_SLOT_BITS
#endif
#define MUSIC_PLAYER_I2S_SLOT_BITS 16U

#ifdef MUSIC_PLAYER_CODEC_DATA_LEN
#undef MUSIC_PLAYER_CODEC_DATA_LEN
#endif
#define MUSIC_PLAYER_CODEC_DATA_LEN ES8388_DEFAULT_DATA_LEN

#ifndef MUSIC_PLAYER_TONE_SAMPLE_RATE
#define MUSIC_PLAYER_TONE_SAMPLE_RATE 16000U
#endif

#ifndef MUSIC_PLAYER_TONE_SEND_BATCH
#define MUSIC_PLAYER_TONE_SEND_BATCH 512U
#endif

#ifndef MUSIC_PLAYER_TONE_MAX_BURST
#define MUSIC_PLAYER_TONE_MAX_BURST 2048U
#endif

#ifndef MUSIC_PLAYER_TONE_DEFAULT_AMP
#define MUSIC_PLAYER_TONE_DEFAULT_AMP 12000
#endif

uint8_t music_player_init(void);
uint8_t music_player_scan_folder(const char *path, uint32_t *count);
uint8_t music_player_get_info(const char *filename, music_info_struct *info);
uint8_t music_player_play(const char *filename);
void music_player_stop(void);
uint8_t music_player_pause(void);
uint8_t music_player_resume(void);
music_player_state_enum music_player_get_state(void);
uint8_t music_player_get_progress(uint32_t *total_seconds, uint32_t *current_seconds);
void music_player_set_volume(uint8_t volume);
uint8_t music_player_get_volume(void);
void music_player_set_mode(music_play_mode_enum mode);
music_play_mode_enum music_player_get_mode(void);
uint8_t music_player_next(void);
uint8_t music_player_previous(void);
uint8_t music_player_play_index(uint32_t index);
uint8_t music_player_get_current_info(music_info_struct *info);
void music_player_demo(void);
uint8_t music_player_test(void);

uint8_t music_player_play_test_tone(uint32_t duration_ms, uint32_t freq_hz);
uint8_t music_player_play_output_scan_tone(void);
uint8_t music_player_tone_start(uint32_t freq_hz, int16_t amp);
uint8_t music_player_tone_play_blocking(uint32_t freq_hz, int16_t amp, uint32_t duration_ms);
void music_player_tone_stop(void);
uint8_t music_player_tone_is_active(void);
void music_player_tone_task(void);
uint8_t music_player_play_first_in_folder(const char *path);
uint8_t music_player_play_first_in_folder_async(const char *path);
void music_player_io_task(void);
void music_player_core_task(void);
uint8_t music_player_async_is_active(void);
uint8_t music_player_async_result(void);
uint32_t music_player_async_underruns(void);
uint32_t music_player_async_play_ms(void);
void music_player_async_stats(uint32 *sent, uint32 *nonzero, uint32 *peak, int16 *last_left, int16 *last_right, uint32 *fill);

#endif
