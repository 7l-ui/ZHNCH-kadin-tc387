#include "camera_debug_link.h"

#include <stdlib.h>

#include "blue_target.h"
#include "guimai/guimai_board.h"
#include "zf_common_headfile.h"

#define CAMERA_DEBUG_LINE_CAPACITY          (64U)
#define CAMERA_DEBUG_FRAME_HEADER_SIZE      (48U)
#define CAMERA_DEBUG_FRAME_FORMAT_RGB565_BE (1U)
#define CAMERA_DEBUG_FRAME_VERSION          (1U)
#define CAMERA_DEBUG_TX_CHUNK_SIZE           (320U)
#define CAMERA_DEBUG_MAX_FRAME_AGE_MS        (1000U)

static char g_camera_debug_line[CAMERA_DEBUG_LINE_CAPACITY];
static uint8 g_camera_debug_line_length = 0U;
static uint16 g_camera_debug_brightness = SCC8660_BRIGHT_DEF;
static uint16 g_camera_debug_white_balance = SCC8660_MANUAL_WB_DEF;
static uint32 g_camera_debug_sequence = 0U;

static void camera_debug_put_u16_le(uint8 *buffer, uint16 value)
{
    buffer[0] = (uint8)value;
    buffer[1] = (uint8)(value >> 8);
}

static void camera_debug_put_u32_le(uint8 *buffer, uint32 value)
{
    buffer[0] = (uint8)value;
    buffer[1] = (uint8)(value >> 8);
    buffer[2] = (uint8)(value >> 16);
    buffer[3] = (uint8)(value >> 24);
}

static uint16 camera_debug_crc16(const uint8 *data, uint32 length)
{
    uint16 crc = 0xFFFFU;

    while (length-- != 0U)
    {
        uint8 bit;

        crc ^= (uint16)(*data++) << 8;
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 0x8000U) ? (uint16)((crc << 1) ^ 0x1021U)
                                  : (uint16)(crc << 1);
        }
    }
    return crc;
}

static uint32 camera_debug_crc32_update(uint32 crc,
                                        const uint8 *data,
                                        uint32 length)
{
    while (length-- != 0U)
    {
        uint8 bit;

        crc ^= *data++;
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
        }
    }
    return crc;
}

static void camera_debug_reply(const char *text)
{
    debug_send_buffer((const uint8 *)text, (uint32)strlen(text));
}

static uint8 camera_debug_camera_ready(void)
{
    return blue_target_camera_is_ready();
}

static uint8 camera_debug_parse_u16(const char *text, uint16 *value)
{
    char *end = NULL;
    unsigned long parsed;

    if ((text == NULL) || (value == NULL) || (*text == '\0'))
    {
        return 0U;
    }

    parsed = strtoul(text, &end, 0);
    if ((*end != '\0') || (parsed > 65535UL))
    {
        return 0U;
    }

    *value = (uint16)parsed;
    return 1U;
}

static void camera_debug_send_status(void)
{
    char reply[320];
    blue_target_result_t target;
    uint32 now_ms = system_getval_ms();
    uint32 worker_heartbeat = 0U;
    uint32 worker_last_task_ms = 0U;
    uint32 last_frame_ms = 0U;
    uint32 worker_age_ms;
    uint32 frame_age_ms;
    uint8 worker_enabled = 0U;
    uint8 reset_pending = 0U;

    blue_target_get_result(&target);
    blue_target_camera_get_health(&worker_heartbeat,
                                  &worker_last_task_ms,
                                  &last_frame_ms,
                                  &worker_enabled,
                                  &reset_pending);
    worker_age_ms = (worker_last_task_ms != 0U) ?
                    (now_ms - worker_last_task_ms) : 0xFFFFFFFFUL;
    frame_age_ms = (last_frame_ms != 0U) ?
                   (now_ms - last_frame_ms) : 0xFFFFFFFFUL;
    sprintf(reply,
            "@CAM OK READY=%u AE=%u BR=%u WB=0x%02X FRAME=%lu AGE=%lu "
            "HB=%lu HAGE=%lu EN=%u RST=%u VOICE=%u INIT=%u TRY=%u "
            "VS=%lu DMA=%lu LOST=%lu "
            "FLAG=%u TARGET=%u CONF=%u\r\n",
            (unsigned)camera_debug_camera_ready(),
            (unsigned)SCC8660_AUTO_EXP_DEF,
            (unsigned)g_camera_debug_brightness,
            (unsigned)g_camera_debug_white_balance,
            (unsigned long)blue_target_camera_get_frame_count(),
            (unsigned long)frame_age_ms,
            (unsigned long)worker_heartbeat,
            (unsigned long)worker_age_ms,
            (unsigned)worker_enabled,
            (unsigned)reset_pending,
            (unsigned)guimai_voice_is_recording(),
            (unsigned)scc8660_get_init_result(),
            (unsigned)scc8660_get_init_attempts(),
            (unsigned long)scc8660_get_vsync_count(),
            (unsigned long)scc8660_get_dma_frame_count(),
            (unsigned long)scc8660_get_dma_lost_count(),
            (unsigned)scc8660_frame_is_ready(),
            (unsigned)target.valid,
            (unsigned)target.confidence_pct);
    camera_debug_reply(reply);
}

static uint8 camera_debug_snapshot_step(const char *argument, uint8 *step)
{
    if ((argument == NULL) || (*argument == '\0') || (strcmp(argument, "FULL") == 0))
    {
        *step = 1U;
        return 1U;
    }
    if (strcmp(argument, "HALF") == 0)
    {
        *step = 2U;
        return 1U;
    }
    if ((strcmp(argument, "QTR") == 0) || (strcmp(argument, "QUARTER") == 0))
    {
        *step = 4U;
        return 1U;
    }
    return 0U;
}

static void camera_debug_send_snapshot(uint8 step)
{
    const uint16 *image;
    blue_target_result_t target;
    uint32 source_frame_count;
    uint32 payload_size;
    uint32 payload_crc = 0xFFFFFFFFUL;
    uint32 now_ms = system_getval_ms();
    uint32 sequence;
    uint16 output_width = (uint16)(SCC8660_W / step);
    uint16 output_height = (uint16)(SCC8660_H / step);
    uint8 header[CAMERA_DEBUG_FRAME_HEADER_SIZE];
    uint8 trailer[4];
    uint8 tx_chunk[CAMERA_DEBUG_TX_CHUNK_SIZE];
    uint16 source_y;
    uint32 chunk_length = 0U;
    uint32 last_frame_ms = 0U;
    char reply[96];

    blue_target_camera_get_health(NULL, NULL, &last_frame_ms, NULL, NULL);
    if ((last_frame_ms == 0U) ||
        ((system_getval_ms() - last_frame_ms) > CAMERA_DEBUG_MAX_FRAME_AGE_MS))
    {
        camera_debug_reply("@CAM ERR STALE_FRAME use GET for diagnostics\r\n");
        return;
    }
    if (guimai_voice_is_recording() != 0U)
    {
        camera_debug_reply("@CAM ERR VOICE_BUSY\r\n");
        return;
    }

    if (blue_target_preview_acquire(&image, &target, &source_frame_count) == 0U)
    {
        camera_debug_reply("@CAM ERR NO_FRAME open Task3 Page4 first\r\n");
        return;
    }

    sequence = ++g_camera_debug_sequence;
    payload_size = (uint32)output_width * (uint32)output_height * 2U;
    memset(header, 0, sizeof(header));
    header[0] = 'C';
    header[1] = 'I';
    header[2] = 'M';
    header[3] = 'G';
    header[4] = CAMERA_DEBUG_FRAME_VERSION;
    header[5] = CAMERA_DEBUG_FRAME_FORMAT_RGB565_BE;
    header[6] = step;
    header[7] = CAMERA_DEBUG_FRAME_HEADER_SIZE;
    camera_debug_put_u32_le(&header[8], sequence);
    camera_debug_put_u16_le(&header[12], output_width);
    camera_debug_put_u16_le(&header[14], output_height);
    camera_debug_put_u32_le(&header[16], payload_size);
    camera_debug_put_u32_le(&header[20], source_frame_count);
    camera_debug_put_u32_le(&header[24], now_ms);
    camera_debug_put_u16_le(&header[28], g_camera_debug_brightness);
    camera_debug_put_u16_le(&header[30], g_camera_debug_white_balance);
    header[32] = SCC8660_AUTO_EXP_DEF;
    header[33] = target.valid;
    header[34] = target.confidence_pct;
    header[35] = target.fill_pct;
    camera_debug_put_u16_le(&header[36], target.center_x);
    camera_debug_put_u16_le(&header[38], target.center_y);
    camera_debug_put_u16_le(&header[40], (uint16)target.offset_x);
    camera_debug_put_u16_le(&header[42], target.bbox_width);
    camera_debug_put_u16_le(&header[44], target.bbox_height);
    camera_debug_put_u16_le(&header[46], camera_debug_crc16(header, 46U));

    sprintf(reply,
            "@CAM SNAP BEGIN SEQ=%lu %ux%u BYTES=%lu\r\n",
            (unsigned long)sequence,
            (unsigned)output_width,
            (unsigned)output_height,
            (unsigned long)payload_size);
    camera_debug_reply(reply);
    debug_uart_output_suppress(1U);
    system_delay_ms(5U);
    debug_send_buffer(header, sizeof(header));

    for (source_y = 0U; source_y < SCC8660_H; source_y = (uint16)(source_y + step))
    {
        uint16 source_x;

        for (source_x = 0U; source_x < SCC8660_W; source_x = (uint16)(source_x + step))
        {
            const uint8 *pixel = (const uint8 *)&image[(uint32)source_y * SCC8660_W + source_x];

            tx_chunk[chunk_length++] = pixel[0];
            tx_chunk[chunk_length++] = pixel[1];
            if (chunk_length == sizeof(tx_chunk))
            {
                payload_crc = camera_debug_crc32_update(payload_crc, tx_chunk, chunk_length);
                debug_send_buffer(tx_chunk, chunk_length);
                chunk_length = 0U;
            }
        }
    }

    if (chunk_length != 0U)
    {
        payload_crc = camera_debug_crc32_update(payload_crc, tx_chunk, chunk_length);
        debug_send_buffer(tx_chunk, chunk_length);
    }
    payload_crc ^= 0xFFFFFFFFUL;
    camera_debug_put_u32_le(trailer, payload_crc);
    debug_send_buffer(trailer, sizeof(trailer));
    blue_target_preview_release();

    sprintf(reply,
            "\r\n@CAM SNAP END SEQ=%lu CRC=0x%08lX\r\n",
            (unsigned long)sequence,
            (unsigned long)payload_crc);
    camera_debug_reply(reply);
    debug_uart_output_suppress(0U);
}

static void camera_debug_handle_line(char *line)
{
    char *command;
    char *argument;
    uint16 value;
    uint8 result;
    char reply[96];

    if (strncmp(line, "CAM", 3U) != 0)
    {
        return;
    }

    command = line + 3;
    while (*command == ' ')
    {
        command++;
    }
    argument = strchr(command, ' ');
    if (argument != NULL)
    {
        *argument++ = '\0';
        while (*argument == ' ')
        {
            argument++;
        }
    }

    if ((strcmp(command, "PING") == 0) || (strcmp(command, "GET") == 0))
    {
        camera_debug_send_status();
        return;
    }

    if (strcmp(command, "RETRY") == 0)
    {
        result = blue_target_camera_retry();
        if (result != 0U)
        {
            camera_debug_reply("@CAM ERR INIT_RETRY\r\n");
            return;
        }
        camera_debug_send_status();
        return;
    }

    if (strcmp(command, "BR") == 0)
    {
        if ((camera_debug_parse_u16(argument, &value) == 0U) || (value > 255U))
        {
            camera_debug_reply("@CAM ERR BR_RANGE 0..255\r\n");
            return;
        }
        if (camera_debug_camera_ready() == 0U)
        {
            camera_debug_reply("@CAM ERR NOT_READY open Task3 Page4 first\r\n");
            return;
        }
        result = scc8660_set_brightness(value);
        if (result == 0U)
        {
            g_camera_debug_brightness = value;
        }
        sprintf(reply, "@CAM %s BR=%u\r\n", (result == 0U) ? "OK" : "ERR SET", (unsigned)value);
        camera_debug_reply(reply);
        return;
    }

    if (strcmp(command, "WB") == 0)
    {
        if ((camera_debug_parse_u16(argument, &value) == 0U) ||
            ((value != 0U) && ((value < 0x65U) || (value > 0xA0U))))
        {
            camera_debug_reply("@CAM ERR WB_RANGE 0 or 0x65..0xA0\r\n");
            return;
        }
        if (camera_debug_camera_ready() == 0U)
        {
            camera_debug_reply("@CAM ERR NOT_READY open Task3 Page4 first\r\n");
            return;
        }
        result = scc8660_set_white_balance(value);
        if (result == 0U)
        {
            g_camera_debug_white_balance = value;
        }
        sprintf(reply, "@CAM %s WB=0x%02X\r\n", (result == 0U) ? "OK" : "ERR SET", (unsigned)value);
        camera_debug_reply(reply);
        return;
    }

    if (strcmp(command, "SNAP") == 0)
    {
        uint8 step;

        if (camera_debug_snapshot_step(argument, &step) == 0U)
        {
            camera_debug_reply("@CAM ERR SNAP_SIZE FULL|HALF|QTR\r\n");
            return;
        }
        camera_debug_send_snapshot(step);
        return;
    }

    camera_debug_reply("@CAM ERR COMMAND use GET RETRY BR WB SNAP\r\n");
}

void camera_debug_link_task(void)
{
    uint8 receive_buffer[16];
    uint32 receive_length = debug_read_ring_buffer(receive_buffer, sizeof(receive_buffer));
    uint32 index;

    for (index = 0U; index < receive_length; index++)
    {
        uint8 byte = receive_buffer[index];

        if ((byte == '\r') || (byte == '\n'))
        {
            if (g_camera_debug_line_length != 0U)
            {
                g_camera_debug_line[g_camera_debug_line_length] = '\0';
                camera_debug_handle_line(g_camera_debug_line);
                g_camera_debug_line_length = 0U;
            }
        }
        else if ((byte >= 0x20U) && (byte <= 0x7EU))
        {
            if (g_camera_debug_line_length < (CAMERA_DEBUG_LINE_CAPACITY - 1U))
            {
                g_camera_debug_line[g_camera_debug_line_length++] = (char)byte;
            }
            else
            {
                g_camera_debug_line_length = 0U;
                camera_debug_reply("@CAM ERR LINE_TOO_LONG\r\n");
            }
        }
    }
}
