#include "music_player.h"
#if MUSIC_PLAYER_TX_BACKEND != MUSIC_PLAYER_TX_BACKEND_QSPI1_SLAVE
#include "zf_driver_soft_iis.h"
#endif
#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"
#include "zf_driver_timer.h"
#include "es8388_config.h"
#if MUSIC_PLAYER_TX_BACKEND == MUSIC_PLAYER_TX_BACKEND_QSPI1_SLAVE
#include "IfxQspi.h"
#include "IfxScuWdt.h"
#endif
#include <string.h>
#include <stdio.h>

#pragma section all "lmubss"
static music_player_struct g_music_player;
static uint8_t g_audio_buffer[MUSIC_PLAYER_BUFFER_SIZE];
static FATFS g_fatfs;
static uint8_t g_music_player_inited = 0;
static volatile uint8_t g_play_stop_request = 0;
static volatile uint8_t g_play_paused = 0;

#if MUSIC_PLAYER_ASYNC_ENABLE
static FIL g_async_file;
static uint8_t g_async_file_open = 0U;
static music_info_struct g_async_info;
static uint32_t g_async_remain = 0U;
static uint32_t g_async_bytes_per_frame = 0U;
static volatile uint8_t g_async_running = 0U;
static volatile uint8_t g_async_started = 0U;
static volatile uint8_t g_async_eof = 0U;
static volatile uint8_t g_async_done = 0U;
static volatile uint8_t g_async_result = 0U;
static volatile uint8_t g_async_iis_ready = 0U;
static volatile uint8_t g_async_read_log_once = 0U;
static volatile uint8_t g_async_core_log_once = 0U;
static volatile uint32_t g_async_wr = 0U;
static volatile uint32_t g_async_rd = 0U;
static volatile uint32_t g_async_underruns = 0U;
static volatile uint32_t g_async_sent_frames = 0U;
static volatile uint32_t g_async_nonzero_frames = 0U;
static volatile uint32_t g_async_peak = 0U;
static volatile uint32_t g_async_stop_fill = 0U;
static volatile uint32_t g_async_pace_next_us = 0U;
static volatile uint32_t g_async_pace_frac_us = 0U;
static volatile uint32_t g_async_play_start_us = 0U;
static volatile uint32_t g_async_play_elapsed_us = 0U;
static int16_t g_async_ring_left[MUSIC_PLAYER_ASYNC_RING_FRAMES];
static int16_t g_async_ring_right[MUSIC_PLAYER_ASYNC_RING_FRAMES];
#endif

static uint8_t g_files_valid = 0;
static uint32_t g_file_count = 0;
static uint32_t g_resample_pos_q16 = 0U;
static uint32_t g_resample_step_q16 = 0U;
static uint32_t g_resample_rate = 0U;
static int16_t g_resample_last_left = 0;
static int16_t g_resample_last_right = 0;
static uint8_t g_resample_has_last = 0U;
static uint32_t g_music_output_sample_rate = MUSIC_PLAYER_OUTPUT_SAMPLE_RATE;
static char g_file_list[MUSIC_PLAYER_MAX_FILES][MUSIC_PLAYER_PATH_LEN];
#pragma section all restore

#pragma section all "cpu1_dsram"
#if MUSIC_PLAYER_TX_BACKEND != MUSIC_PLAYER_TX_BACKEND_QSPI1_SLAVE
static soft_iis_info_struct g_iis_obj;
#endif
#if MUSIC_PLAYER_ASYNC_ENABLE
static int16_t g_async_last_left = 0;
static int16_t g_async_last_right = 0;
#endif
#pragma section all restore

#define MUSIC_PLAYER_SLAVE_TX_EDGE_WAIT_LOOPS (60000U)
#define MUSIC_PLAYER_TONE_END_SILENCE_FRAMES  (64U)

static void music_player_release_iis_pins(void)
{
    gpio_init(HORN_BCK_PIN, GPI, GPIO_LOW, GPI_FLOATING_IN);
    gpio_init(HORN_WS_PIN, GPI, GPIO_LOW, GPI_FLOATING_IN);
    gpio_init(HORN_SD_PIN, GPI, GPIO_LOW, GPI_FLOATING_IN);
    if (HORN_MCLK_PIN != 0xFFFFFFFF)
    {
        gpio_init(HORN_MCLK_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    }
}

static volatile uint8_t g_tone_active = 0U;
static uint32_t g_tone_freq_hz = 0U;
static uint32_t g_tone_half_period = 1U;
static uint32_t g_tone_sample_index = 0U;
static uint32_t g_tone_start_ms = 0U;
static int16_t g_tone_amp = MUSIC_PLAYER_TONE_DEFAULT_AMP;
static int16_t g_tone_sample = MUSIC_PLAYER_TONE_DEFAULT_AMP;

#define WAV_ID_RIFF 0x46464952UL
#define WAV_ID_WAVE 0x45564157UL
#define WAV_ID_FMT  0x20746D66UL
#define WAV_ID_DATA 0x61746164UL

#define WAV_FORMAT_PCM        0x0001U
#define WAV_FORMAT_EXTENSIBLE 0xFFFEU

typedef struct
{
    uint8_t code;
    uint32_t chunk_id;
    uint32_t chunk_size;
    uint16_t audio_format;
    uint16_t channels;
    uint32_t sample_rate;
    uint16_t bits_per_sample;
    uint16_t valid_bits_per_sample;
    uint32_t channel_mask;
    uint16_t sub_format;
} wav_parse_diag_struct;

static wav_parse_diag_struct g_wav_parse_diag;

static uint16_t wav_read_le16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t wav_read_le32(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static void wav_diag_reset(void)
{
    memset(&g_wav_parse_diag, 0, sizeof(g_wav_parse_diag));
}

static uint8_t wav_parse_fail(uint8_t code, uint32_t chunk_id, uint32_t chunk_size)
{
    g_wav_parse_diag.code = code;
    g_wav_parse_diag.chunk_id = chunk_id;
    g_wav_parse_diag.chunk_size = chunk_size;
    return code;
}

static uint16_t wav_parse_extensible_sub_format(const uint8_t *p, uint32_t fmt_size)
{
    static const uint8_t pcm_guid_tail[14] = {
        0x00U, 0x00U, 0x00U, 0x00U, 0x10U, 0x00U, 0x80U,
        0x00U, 0x00U, 0xAAU, 0x00U, 0x38U, 0x9BU, 0x71U
    };

    if ((p == NULL) || (fmt_size < 40U) || (wav_read_le16(p + 16U) < 22U))
    {
        return 0U;
    }

    g_wav_parse_diag.valid_bits_per_sample = wav_read_le16(p + 18U);
    g_wav_parse_diag.channel_mask = wav_read_le32(p + 20U);
    g_wav_parse_diag.sub_format = wav_read_le16(p + 24U);

    if (memcmp(p + 26U, pcm_guid_tail, sizeof(pcm_guid_tail)) != 0)
    {
        return 0U;
    }

    return g_wav_parse_diag.sub_format;
}

static void wav_print_parse_diag(const char *filename, uint8_t code)
{
    uint32_t chunk = g_wav_parse_diag.chunk_id;

    printf("[MUSIC_WAV] parse failed file=%s code=%u chunk=%c%c%c%c size=%lu fmt=0x%04X sub=0x%04X ch=%u sr=%lu bits=%u valid=%u mask=0x%08lX\r\n",
           (filename != NULL) ? filename : "(null)",
           (unsigned)code,
           (char)(chunk & 0xFFU),
           (char)((chunk >> 8U) & 0xFFU),
           (char)((chunk >> 16U) & 0xFFU),
           (char)((chunk >> 24U) & 0xFFU),
           (unsigned long)g_wav_parse_diag.chunk_size,
           (unsigned)g_wav_parse_diag.audio_format,
           (unsigned)g_wav_parse_diag.sub_format,
           (unsigned)g_wav_parse_diag.channels,
           (unsigned long)g_wav_parse_diag.sample_rate,
           (unsigned)g_wav_parse_diag.bits_per_sample,
           (unsigned)g_wav_parse_diag.valid_bits_per_sample,
           (unsigned long)g_wav_parse_diag.channel_mask);
}

static int16_t wav_sample24_to16(const uint8_t *p)
{
    int32_t s24 = ((int32_t)((uint32_t)p[2] << 24)
                 | (int32_t)((uint32_t)p[1] << 16)
                 | (int32_t)((uint32_t)p[0] << 8)) >> 8;
    return (int16_t)(s24 >> 8);
}

static int16_t music_player_apply_gain(int32_t sample)
{
    sample = (sample * (int32_t)MUSIC_PLAYER_DIGITAL_GAIN_PERCENT) / 100;
    if (sample > 32767)
    {
        sample = 32767;
    }
    else if (sample < -32768)
    {
        sample = -32768;
    }
    return (int16_t)sample;
}

static void music_player_read_frame_raw(const uint8_t *p, const music_info_struct *info, int16_t *left, int16_t *right)
{
    int16_t l = 0;
    int16_t r = 0;

    if (info->bits_per_sample == 16U)
    {
        l = (int16_t)wav_read_le16(p);
        if (info->channels == 2U)
        {
            r = (int16_t)wav_read_le16(p + 2U);
        }
        else
        {
            r = l;
        }
    }
    else
    {
        l = wav_sample24_to16(p);
        if (info->channels == 2U)
        {
            r = wav_sample24_to16(p + 3U);
        }
        else
        {
            r = l;
        }
    }

    *left = l;
    *right = r;
}

static void music_player_read_frame_16(const uint8_t *p, const music_info_struct *info, int16_t *left, int16_t *right)
{
    int16_t l = 0;
    int16_t r = 0;

    music_player_read_frame_raw(p, info, &l, &r);
    *left = music_player_apply_gain(l);
    *right = music_player_apply_gain(r);
}

static uint8_t music_player_convert_volume(uint8_t user_volume)
{
    if (user_volume > 100U)
    {
        user_volume = 100U;
    }
    return (uint8_t)((user_volume * 33U) / 100U);
}

static void music_player_tx_init(void);
static void music_player_set_tx_rate(uint32_t sample_rate);
static void music_player_tx_start(void);
static void music_player_tx_stop(void);

static uint8_t music_player_codec_init_i2s(void)
{
    uint8_t ret;

    ret = es8388_init();
    if (ret != 0U)
    {
        printf("[MUSIC] es8388_init failed, ret=%u\r\n", ret);
        return ret;
    }
    printf("[MUSIC] es8388_init ok\r\n");

    es8388_i2s_cfg((uint8_t)MUSIC_PLAYER_I2S_FORMAT, (uint8_t)MUSIC_PLAYER_CODEC_DATA_LEN);
    printf("[MUSIC] codec i2s cfg ok fmt=%u len=%u\r\n",
           (unsigned)MUSIC_PLAYER_I2S_FORMAT,
           (unsigned)MUSIC_PLAYER_CODEC_DATA_LEN);

    music_player_tx_init();
    return 0U;
}

static void music_player_codec_play_mode(void)
{
    uint8_t codec_volume = music_player_convert_volume(g_music_player.volume);

    if (codec_volume == 0U)
    {
        codec_volume = music_player_convert_volume(MUSIC_PLAYER_DEFAULT_VOLUME);
    }

    es8388_adda_cfg(1U, 0U);
    es8388_i2s_cfg((uint8_t)MUSIC_PLAYER_I2S_FORMAT, (uint8_t)MUSIC_PLAYER_CODEC_DATA_LEN);
    es8388_output_cfg(ES8388_OUTPUT_O1_ENABLE, ES8388_OUTPUT_O2_ENABLE);
    es8388_hpvol_set(codec_volume);
    es8388_spkvol_set(codec_volume);
}

static void music_player_dump_play_regs(const char *tag)
{
    printf("[MUSIC_REG] %s 02=%02X 03=%02X 04=%02X 08=%02X 17=%02X 18=%02X 1A=%02X 1B=%02X 26=%02X 27=%02X 28=%02X 29=%02X 2A=%02X 2B=%02X 2C=%02X 2D=%02X 2E=%02X 2F=%02X 30=%02X 31=%02X\r\n",
           (tag != NULL) ? tag : "play",
           es8388_read_reg(0x02),
           es8388_read_reg(0x03),
           es8388_read_reg(0x04),
           es8388_read_reg(0x08),
           es8388_read_reg(0x17),
           es8388_read_reg(0x18),
           es8388_read_reg(0x1A),
           es8388_read_reg(0x1B),
           es8388_read_reg(0x26),
           es8388_read_reg(0x27),
           es8388_read_reg(0x28),
           es8388_read_reg(0x29),
           es8388_read_reg(0x2A),
           es8388_read_reg(0x2B),
           es8388_read_reg(0x2C),
           es8388_read_reg(0x2D),
           es8388_read_reg(0x2E),
           es8388_read_reg(0x2F),
           es8388_read_reg(0x30),
           es8388_read_reg(0x31));
}

static void music_player_resample_reset(void)
{
    g_resample_pos_q16 = 0U;
    g_resample_step_q16 = 0U;
    g_resample_rate = 0U;
    g_resample_last_left = 0;
    g_resample_last_right = 0;
    g_resample_has_last = 0U;
}

static uint32_t music_player_select_output_rate(const music_info_struct *info)
{
    uint32_t rate = MUSIC_PLAYER_OUTPUT_SAMPLE_RATE;

#if MUSIC_PLAYER_OUTPUT_MATCH_WAV_RATE
    if ((info != NULL) &&
        (info->sample_rate >= MUSIC_PLAYER_DIRECT_MIN_SAMPLE_RATE) &&
        (info->sample_rate <= MUSIC_PLAYER_DIRECT_MAX_SAMPLE_RATE))
    {
        rate = info->sample_rate;
    }
#else
    (void)info;
#endif

    if (rate == 0U)
    {
        rate = 32000U;
    }
    return rate;
}

static uint32_t music_player_output_rate(void)
{
    if (g_music_output_sample_rate == 0U)
    {
        g_music_output_sample_rate = MUSIC_PLAYER_OUTPUT_SAMPLE_RATE;
    }
    return g_music_output_sample_rate;
}

static void music_player_set_output_rate_from_info(const music_info_struct *info)
{
    g_music_output_sample_rate = music_player_select_output_rate(info);
}

#if MUSIC_PLAYER_TX_BACKEND == MUSIC_PLAYER_TX_BACKEND_QSPI1_SLAVE
static uint32_t g_music_tx_sample_rate = MUSIC_PLAYER_OUTPUT_SAMPLE_RATE;
static uint8_t g_music_qspi_ready = 0U;
static uint8_t g_music_qspi_clocks_started = 0U;
static uint32_t g_music_qspi_bacon = 0U;
static uint32_t g_music_qspi_econ = 0U;
static uint32_t g_music_qspi_errors = 0U;
static uint32_t g_music_qspi_waits = 0U;
static uint32_t g_music_qspi_rx_drains = 0U;

static uint32_t music_player_elapsed_ms(uint32_t start_ms, uint32_t now_ms)
{
    return now_ms - start_ms;
}

static void music_player_qspi1_calc_clocks(uint32_t sample_rate,
                                           uint32_t *bck_freq,
                                           uint32_t *mclk_freq,
                                           uint32_t *slot_bits)
{
    uint32_t bits = (MUSIC_PLAYER_I2S_SLOT_BITS == 32U) ? 32U : 16U;
    uint32_t bck;
    uint32_t mclk;

    if (sample_rate == 0U)
    {
        sample_rate = MUSIC_PLAYER_OUTPUT_SAMPLE_RATE;
    }

    bck = sample_rate * 2U * bits;
    if (bck == 0U)
    {
        bck = sample_rate * 32U;
    }

    mclk = sample_rate * MUSIC_PLAYER_QSPI_MCLK_RATIO;
    if (mclk == 0U)
    {
        mclk = 8192000U;
    }

    if (bck_freq != NULL)
    {
        *bck_freq = bck;
    }
    if (mclk_freq != NULL)
    {
        *mclk_freq = mclk;
    }
    if (slot_bits != NULL)
    {
        *slot_bits = bits;
    }
}

static void music_player_qspi1_stop_clocks(void)
{
    if (g_music_qspi_clocks_started)
    {
        pwm_set_duty(ATOM2_CH3_P11_6, 0U);
        pwm_set_duty(ATOM2_CH1_P11_2, 0U);
        pwm_set_duty(ATOM2_CH6_P11_11, 0U);
        g_music_qspi_clocks_started = 0U;
    }
    gpio_init(HORN_BCK_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(HORN_WS_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(HORN_MCLK_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

static void music_player_qspi1_start_mclk_ws(uint32_t sample_rate,
                                             uint32_t *bck_freq,
                                             uint32_t *mclk_freq,
                                             uint32_t *slot_bits)
{
    uint32_t calc_bck_freq;
    uint32_t calc_mclk_freq;
    uint32_t calc_slot_bits;

    if (sample_rate == 0U)
    {
        sample_rate = MUSIC_PLAYER_OUTPUT_SAMPLE_RATE;
    }

    music_player_qspi1_calc_clocks(sample_rate, &calc_bck_freq, &calc_mclk_freq, &calc_slot_bits);
    pwm_init(ATOM2_CH6_P11_11, calc_mclk_freq, PWM_DUTY_MAX / 2U);
    pwm_init(ATOM2_CH1_P11_2, sample_rate, PWM_DUTY_MAX / 2U);
    gpio_init(HORN_BCK_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    g_music_qspi_clocks_started = 1U;

    if (bck_freq != NULL)
    {
        *bck_freq = calc_bck_freq;
    }
    if (mclk_freq != NULL)
    {
        *mclk_freq = calc_mclk_freq;
    }
    if (slot_bits != NULL)
    {
        *slot_bits = calc_slot_bits;
    }
}

static void music_player_qspi1_start_bck(uint32_t bck_freq)
{
    if (bck_freq == 0U)
    {
        bck_freq = MUSIC_PLAYER_OUTPUT_SAMPLE_RATE * 32U;
    }
    pwm_init(ATOM2_CH3_P11_6, bck_freq, PWM_DUTY_MAX / 2U);
    g_music_qspi_clocks_started = 1U;
}

static void music_player_qspi1_start_clocks(uint32_t sample_rate)
{
    uint32_t bck_freq;
    uint32_t mclk_freq;
    uint32_t slot_bits;

    music_player_qspi1_calc_clocks(sample_rate, &bck_freq, &mclk_freq, &slot_bits);
    pwm_init(ATOM2_CH6_P11_11, mclk_freq, PWM_DUTY_MAX / 2U);
    pwm_init(ATOM2_CH1_P11_2, sample_rate, PWM_DUTY_MAX / 2U);
    music_player_qspi1_start_bck(bck_freq);
    printf("[MUSIC_QSPI] clocks bck=%lu ws=%lu mclk=%lu slot=%lu\r\n",
           (unsigned long)bck_freq,
           (unsigned long)sample_rate,
           (unsigned long)mclk_freq,
           (unsigned long)slot_bits);
}

static uint32_t music_player_qspi1_flush_rx(void)
{
    uint32_t guard = 0U;

    while ((IfxQspi_getReceiveFifoLevel(&MODULE_QSPI1) > 0U) && (guard < 32U))
    {
        (void)IfxQspi_readReceiveFifo(&MODULE_QSPI1);
        guard++;
    }
    g_music_qspi_rx_drains += guard;
    IfxQspi_clearAllEventFlags(&MODULE_QSPI1);
    return guard;
}

static uint8_t music_player_qspi1_wait_ws_edge(uint32_t timeout_ms, uint8_t *edge_level)
{
    uint8_t last = gpio_get_level(HORN_WS_PIN);
    uint32_t start_ms = system_getval_ms();

    while (music_player_elapsed_ms(start_ms, system_getval_ms()) < timeout_ms)
    {
        uint8_t cur = gpio_get_level(HORN_WS_PIN);
        if (cur != last)
        {
            if (edge_level != NULL)
            {
                *edge_level = cur;
            }
            return 1U;
        }
    }

    if (edge_level != NULL)
    {
        *edge_level = last;
    }
    return 0U;
}

static void music_player_qspi1_reset_stream_state(void)
{
    MODULE_QSPI1.GLOBALCON.B.RESETS = IfxQspi_Reset_stateMachineAndFifo;
    MODULE_QSPI1.ECON[0].U = g_music_qspi_econ;
    IfxQspi_writeBasicConfigurationBeginStream(&MODULE_QSPI1, g_music_qspi_bacon);
    (void)music_player_qspi1_flush_rx();
    IfxQspi_clearAllEventFlags(&MODULE_QSPI1);
    IfxQspi_run(&MODULE_QSPI1);
}

static void music_player_qspi1_init_slave_tx(void)
{
    Ifx_QSPI *qspi = &MODULE_QSPI1;
    uint16 password;
    SpiIf_ChConfig ch_config;
    uint32 bacon;
    uint32 econ;
    uint32 max_baud;
    uint32 max_rate;

    max_rate = MUSIC_PLAYER_DIRECT_MAX_SAMPLE_RATE;
    if (max_rate < MUSIC_PLAYER_OUTPUT_SAMPLE_RATE)
    {
        max_rate = MUSIC_PLAYER_OUTPUT_SAMPLE_RATE;
    }
    max_baud = max_rate * 2U * ((MUSIC_PLAYER_I2S_SLOT_BITS == 32U) ? 32U : 16U);
    if (max_baud < 2000000U)
    {
        max_baud = 2000000U;
    }

    password = IfxScuWdt_getCpuWatchdogPassword();
    IfxScuWdt_clearCpuEndinit(password);
    IfxQspi_setEnableModuleRequest(qspi);
    IfxScuWdt_setCpuEndinit(password);

    qspi->GLOBALCON.U = 0U;
    qspi->GLOBALCON.B.TQ = IfxQspi_calculateTimeQuantumLength(qspi, (float32)max_baud);
    qspi->GLOBALCON.B.EXPECT = IfxQspi_ExpectTimeout_2097152;
    qspi->GLOBALCON.B.SRF = 0U;
    qspi->GLOBALCON.B.STIP = 0U;
    qspi->GLOBALCON.B.MS = IfxQspi_Mode_master;
    qspi->GLOBALCON.B.CLKSEL = 1U;
    qspi->GLOBALCON.B.RESETS = IfxQspi_Reset_stateMachineAndFifo;

    qspi->GLOBALCON1.U = 0U;
    qspi->GLOBALCON1.B.RXFIFOINT = IfxQspi_RxFifoInt_0;
    qspi->GLOBALCON1.B.TXFIFOINT = IfxQspi_TxFifoInt_1;
    qspi->GLOBALCON1.B.RXFM = IfxQspi_FifoMode_combinedMove;
    qspi->GLOBALCON1.B.TXFM = IfxQspi_FifoMode_combinedMove;

    SpiIf_initChannelConfig(&ch_config, NULL_PTR);
    ch_config.baudrate = (float32)max_baud;
    ch_config.mode.clockPolarity = SpiIf_ClockPolarity_idleLow;
#if MUSIC_PLAYER_QSPI_SHIFT_TRAILING
    ch_config.mode.shiftClock = SpiIf_ShiftClock_shiftTransmitDataOnTrailingEdge;
#else
    ch_config.mode.shiftClock = SpiIf_ShiftClock_shiftTransmitDataOnLeadingEdge;
#endif
    ch_config.mode.dataHeading = SpiIf_DataHeading_msbFirst;
    ch_config.mode.dataWidth = (MUSIC_PLAYER_I2S_SLOT_BITS == 32U) ? 32U : 16U;
    ch_config.mode.parityCheck = 0U;
    ch_config.mode.parityMode = Ifx_ParityMode_even;

    econ = IfxQspi_calculateExtendedConfigurationValue(qspi, 0U, &ch_config);
    bacon = IfxQspi_calculateBasicConfigurationValue(qspi, IfxQspi_ChannelId_0, &ch_config.mode, (float32)max_baud);
    g_music_qspi_bacon = bacon;
    g_music_qspi_econ = econ;
    qspi->ECON[0].U = econ;
    IfxQspi_writeBasicConfigurationBeginStream(qspi, bacon);

    IfxQspi_initSclkInPinWithPadLevel(&IfxQspi1_SCLKB_P11_6_IN, IfxPort_InputMode_noPullDevice, IfxPort_PadDriver_cmosAutomotiveSpeed3);
    IfxQspi_initMrstOutPin(&IfxQspi1_MRST_P11_3_OUT, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed3);
    IfxQspi_initSlsiWithPadLevel(&IfxQspi1_SLSIA_P11_10_IN, IfxPort_InputMode_pullDown, IfxPort_PadDriver_cmosAutomotiveSpeed3);
    qspi->GLOBALCON.B.MS = IfxQspi_Mode_slave;
    IfxQspi_run(qspi);
    (void)music_player_qspi1_flush_rx();
    music_player_qspi1_reset_stream_state();

    g_music_qspi_ready = 1U;
    g_music_qspi_errors = 0U;
    g_music_qspi_waits = 0U;
    g_music_qspi_rx_drains = 0U;
    printf("[MUSIC_QSPI] init slave tx ok sclk=P11_6 mrst=P11_3 edge=%s slsis=%u PISEL=0x%08lX GLOBALCON=0x%08lX ECON0=0x%08lX BACON=0x%08lX STATUS=0x%08lX STATUS1=0x%08lX\r\n",
#if MUSIC_PLAYER_QSPI_SHIFT_TRAILING
           "trailing",
#else
           "leading",
#endif
           (unsigned)qspi->PISEL.B.SLSIS,
           (unsigned long)qspi->PISEL.U,
           (unsigned long)qspi->GLOBALCON.U,
           (unsigned long)qspi->ECON[0].U,
           (unsigned long)qspi->BACON.U,
           (unsigned long)qspi->STATUS.U,
           (unsigned long)qspi->STATUS1.U);
}

static void music_player_qspi1_wait_tx_space(void)
{
    uint32_t guard = MUSIC_PLAYER_QSPI_TX_WAIT_GUARD;

    while (IfxQspi_getTransmitFifoLevel(&MODULE_QSPI1) >= (IFXQSPI_HWFIFO_DEPTH - 1U))
    {
        g_music_qspi_waits++;
        if (guard == 0U)
        {
            g_music_qspi_errors |= 0x80000000UL;
            MODULE_QSPI1.GLOBALCON.B.RESETS = IfxQspi_Reset_stateMachineAndFifo;
            IfxQspi_writeBasicConfigurationBeginStream(&MODULE_QSPI1, g_music_qspi_bacon);
            IfxQspi_clearAllEventFlags(&MODULE_QSPI1);
            return;
        }
        guard--;
    }
}

static void music_player_qspi1_write_word_raw(uint32_t word)
{
    music_player_qspi1_wait_tx_space();
    IfxQspi_writeTransmitFifo(&MODULE_QSPI1, word);
}

static void music_player_qspi1_write_word(uint32_t word)
{
    uint16 err;

    if (!g_music_qspi_ready)
    {
        return;
    }

    err = IfxQspi_getErrorFlags(&MODULE_QSPI1);
    if (err != 0U)
    {
        g_music_qspi_errors |= err;
        (void)music_player_qspi1_flush_rx();
    }

    music_player_qspi1_write_word_raw(word);
}

static void music_player_qspi1_prefill_silence(void)
{
    uint32_t i;
    uint32_t words = MUSIC_PLAYER_QSPI_PREFILL_WORDS;

    if ((!g_music_qspi_ready) || (g_music_qspi_bacon == 0U))
    {
        return;
    }

    if (words > (IFXQSPI_HWFIFO_DEPTH - 1U))
    {
        words = IFXQSPI_HWFIFO_DEPTH - 1U;
    }
    for (i = 0U; i < words; i++)
    {
        music_player_qspi1_write_word_raw(0U);
    }
}

static void music_player_qspi1_sync_and_start(uint32_t sample_rate)
{
    uint8_t ws_edge_level = 0U;
    uint8_t ws_ok = 0U;
    uint32_t bck_freq = 0U;
    uint32_t mclk_freq = 0U;
    uint32_t slot_bits = 0U;

    music_player_qspi1_reset_stream_state();
    g_music_qspi_errors = 0U;
    IfxQspi_clearAllEventFlags(&MODULE_QSPI1);
#if MUSIC_PLAYER_QSPI_START_SYNC_WS
    music_player_qspi1_start_mclk_ws(sample_rate, &bck_freq, &mclk_freq, &slot_bits);
    printf("[MUSIC_QSPI] preclocks bck=%lu ws=%lu mclk=%lu slot=%lu\r\n",
           (unsigned long)bck_freq,
           (unsigned long)sample_rate,
           (unsigned long)mclk_freq,
           (unsigned long)slot_bits);
    ws_ok = music_player_qspi1_wait_ws_edge(MUSIC_PLAYER_QSPI_WS_SYNC_TIMEOUT_MS, &ws_edge_level);
    music_player_qspi1_reset_stream_state();
    music_player_qspi1_prefill_silence();
    IfxQspi_clearAllEventFlags(&MODULE_QSPI1);
    music_player_qspi1_start_bck(bck_freq);
#else
    music_player_qspi1_prefill_silence();
    music_player_qspi1_start_clocks(sample_rate);
    ws_edge_level = gpio_get_level(HORN_WS_PIN);
#endif
    (void)music_player_qspi1_flush_rx();
    printf("[MUSIC_QSPI] start sync=%s ws=%u tx=%u rx=%u phase=%u bitcnt=%u status=0x%08lX status1=0x%08lX\r\n",
           ws_ok ? "edge" : "none",
           (unsigned)ws_edge_level,
           (unsigned)IfxQspi_getTransmitFifoLevel(&MODULE_QSPI1),
           (unsigned)IfxQspi_getReceiveFifoLevel(&MODULE_QSPI1),
           (unsigned)MODULE_QSPI1.STATUS.B.PHASE,
           (unsigned)MODULE_QSPI1.STATUS1.B.BITCOUNT,
           (unsigned long)MODULE_QSPI1.STATUS.U,
           (unsigned long)MODULE_QSPI1.STATUS1.U);
}

static void music_player_qspi1_send_sample(int16_t left, int16_t right)
{
#if MUSIC_PLAYER_I2S_SLOT_BITS == 32U
    music_player_qspi1_write_word(((uint32_t)(uint16_t)left) << 16);
    music_player_qspi1_write_word(((uint32_t)(uint16_t)right) << 16);
#else
    music_player_qspi1_write_word((uint16_t)left);
    music_player_qspi1_write_word((uint16_t)right);
#endif
}
#endif

static void music_player_tx_init(void)
{
#if MUSIC_PLAYER_TX_BACKEND != MUSIC_PLAYER_TX_BACKEND_QSPI1_SLAVE
    soft_iis_bits_enum slot_bits = (MUSIC_PLAYER_I2S_SLOT_BITS == 32U) ? SOFT_IIS_BITS_32 : SOFT_IIS_BITS_16;
#endif

#if MUSIC_PLAYER_TX_BACKEND == MUSIC_PLAYER_TX_BACKEND_QSPI1_SLAVE
    printf("[MUSIC] qspi slave tx pins bck=%u ws=%u sd=QSPI1_MRST_P11_3 mclk=%u\r\n",
           (unsigned)HORN_BCK_PIN,
           (unsigned)HORN_WS_PIN,
           (unsigned)HORN_MCLK_PIN);
    music_player_qspi1_stop_clocks();
    music_player_qspi1_init_slave_tx();
#else
    printf("[MUSIC] soft_iis init tx pins bck=%u ws=%u sd=%u mclk=%u\r\n",
           (unsigned)HORN_BCK_PIN,
           (unsigned)HORN_WS_PIN,
           (unsigned)HORN_SD_PIN,
           (unsigned)HORN_MCLK_PIN);
    soft_iis_init_master_tx(&g_iis_obj,
                            ES8388_DEFAULT_SAMPLE_RATE,
                            slot_bits,
                            (soft_iis_format_enum)MUSIC_PLAYER_I2S_FORMAT,
                            HORN_BCK_PIN,
                            HORN_WS_PIN,
                            HORN_SD_PIN,
                            HORN_MCLK_PIN);
    printf("[MUSIC] soft_iis init tx ok\r\n");
#endif
}

static void music_player_set_tx_rate(uint32_t sample_rate)
{
#if MUSIC_PLAYER_TX_BACKEND == MUSIC_PLAYER_TX_BACKEND_QSPI1_SLAVE
    g_music_tx_sample_rate = sample_rate;
#else
    soft_iis_set_sample_rate(&g_iis_obj, sample_rate);
#endif
}

static void music_player_tx_start(void)
{
#if MUSIC_PLAYER_TX_BACKEND == MUSIC_PLAYER_TX_BACKEND_QSPI1_SLAVE
    music_player_qspi1_sync_and_start(g_music_tx_sample_rate);
#else
    soft_iis_start(&g_iis_obj);
#endif
}

static void music_player_tx_stop(void)
{
#if MUSIC_PLAYER_TX_BACKEND == MUSIC_PLAYER_TX_BACKEND_QSPI1_SLAVE
    uint32_t status = MODULE_QSPI1.STATUS.U;
    uint32_t status1 = MODULE_QSPI1.STATUS1.U;
    uint8_t tx_level = IfxQspi_getTransmitFifoLevel(&MODULE_QSPI1);
    uint8_t rx_level = IfxQspi_getReceiveFifoLevel(&MODULE_QSPI1);
    MODULE_QSPI1.GLOBALCON.B.EN = 0U;
    MODULE_QSPI1.GLOBALCON.B.RESETS = IfxQspi_Reset_stateMachineAndFifo;
    music_player_qspi1_stop_clocks();
    gpio_init(HORN_SD_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    printf("[MUSIC_QSPI] stop tx=%u rx=%u err=0x%lX waits=%lu drains=%lu phase=%u bitcnt=%u status=0x%08lX status1=0x%08lX\r\n",
           (unsigned)tx_level,
           (unsigned)rx_level,
           (unsigned long)g_music_qspi_errors,
           (unsigned long)g_music_qspi_waits,
           (unsigned long)g_music_qspi_rx_drains,
           (unsigned)((status >> 28) & 0x0FU),
           (unsigned)(status1 & 0xFFU),
           (unsigned long)status,
           (unsigned long)status1);
#else
    soft_iis_stop(&g_iis_obj);
#endif
}

#if MUSIC_PLAYER_TX_BACKEND != MUSIC_PLAYER_TX_BACKEND_QSPI1_SLAVE
static uint8_t music_player_i2s_level(gpio_pin_enum pin)
{
    return (uint8_t)((get_port(pin)->IN.U >> ((uint32_t)pin & 0x1FU)) & 0x01U);
}

static void music_player_i2s_set_sd(uint8_t level)
{
    if (level)
    {
        gpio_high(HORN_SD_PIN);
    }
    else
    {
        gpio_low(HORN_SD_PIN);
    }
}

static uint8_t music_player_wait_i2s_level(gpio_pin_enum pin, uint8_t level, uint32_t wait_loops)
{
    while (music_player_i2s_level(pin) != level)
    {
        if (wait_loops == 0U)
        {
            return 0U;
        }
        wait_loops--;
    }
    return 1U;
}

static uint8_t music_player_wait_i2s_edge(gpio_pin_enum pin, uint8_t from_level, uint8_t to_level, uint32_t wait_loops)
{
    return music_player_wait_i2s_level(pin, from_level, wait_loops) &&
           music_player_wait_i2s_level(pin, to_level, wait_loops);
}

static uint8_t music_player_send_slave_bit(uint8_t bit)
{
    music_player_i2s_set_sd(bit);
    return music_player_wait_i2s_edge(HORN_BCK_PIN, 0U, 1U, MUSIC_PLAYER_SLAVE_TX_EDGE_WAIT_LOOPS);
}

static uint8_t music_player_send_slave_word_i2s(int16_t sample)
{
    uint16_t data = (uint16_t)sample;
    uint16_t mask = 0x8000U;

    if (!music_player_wait_i2s_edge(HORN_BCK_PIN, 1U, 0U, MUSIC_PLAYER_SLAVE_TX_EDGE_WAIT_LOOPS))
    {
        return 0U;
    }

    while (mask != 0U)
    {
        if (!music_player_send_slave_bit((data & mask) ? 1U : 0U))
        {
            return 0U;
        }
        mask >>= 1U;
    }
    return 1U;
}

static uint8_t music_player_send_slave_frame_i2s(int16_t left, int16_t right)
{
    if (!music_player_wait_i2s_edge(HORN_WS_PIN, 1U, 0U, MUSIC_PLAYER_SLAVE_TX_EDGE_WAIT_LOOPS))
    {
        return 0U;
    }
    if (!music_player_send_slave_word_i2s(left))
    {
        return 0U;
    }

    if (!music_player_wait_i2s_edge(HORN_WS_PIN, 0U, 1U, MUSIC_PLAYER_SLAVE_TX_EDGE_WAIT_LOOPS))
    {
        return 0U;
    }
    return music_player_send_slave_word_i2s(right);
}

static void music_player_codec_master_clock(void)
{
    uint8_t clock_reg = es8388_read_reg(0x08);

    if (clock_reg == 0xFFU)
    {
        clock_reg = 0x00U;
    }
    clock_reg &= (uint8_t)(~0x7FU);
    clock_reg |= 0x86U;
    es8388_write_reg(0x08, clock_reg);
}

static void music_player_start_slave_tx_clock(soft_iis_bits_enum slot_bits)
{
    soft_iis_init_slave_tx(&g_iis_obj,
                           slot_bits,
                           SOFT_IIS_FORMAT_I2S,
                           HORN_BCK_PIN,
                           HORN_WS_PIN,
                           HORN_SD_PIN);
    g_iis_obj.sample_rate = MUSIC_PLAYER_TONE_SAMPLE_RATE;
    g_iis_obj.mclk_pin = HORN_MCLK_PIN;
    if (HORN_MCLK_PIN != 0xFFFFFFFF)
    {
        g_iis_obj.port_mclk = (void *)get_port(HORN_MCLK_PIN);
        gpio_init(HORN_MCLK_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    }
    soft_iis_start(&g_iis_obj);
}
#endif

static void music_player_send_sample(int16_t left, int16_t right)
{
#if MUSIC_PLAYER_TX_BACKEND == MUSIC_PLAYER_TX_BACKEND_QSPI1_SLAVE
    music_player_qspi1_send_sample(left, right);
#else
#if MUSIC_PLAYER_I2S_SLOT_BITS == 32U
    soft_iis_send_32bit(&g_iis_obj, ((int32_t)left) << 16, ((int32_t)right) << 16);
#else
    soft_iis_send_16bit(&g_iis_obj, left, right);
#endif
#endif
}

static int16_t music_player_triangle_sample(uint32_t index, uint32_t half_period, int16_t amp)
{
    uint32_t period = half_period * 2U;
    uint32_t pos = 0U;
    int32_t value = 0;

    if ((period == 0U) || (half_period == 0U))
    {
        return 0;
    }

    pos = index % period;
    if (pos < half_period)
    {
        value = -(int32_t)amp + (((int32_t)amp * 2 * (int32_t)pos) / (int32_t)half_period);
    }
    else
    {
        value = (int32_t)amp - (((int32_t)amp * 2 * (int32_t)(pos - half_period)) / (int32_t)half_period);
    }

    return (int16_t)value;
}

static void music_player_wait_until_us(uint32_t target_us)
{
    while (((uint32_t)(system_getval_us() - target_us)) > 0x80000000UL)
    {
    }
}

static void music_player_send_probe_tone(uint32_t duration_ms, uint32_t freq_hz)
{
    uint32_t sample_rate = music_player_output_rate();
    uint32_t total_samples;
    uint32_t half_period;
    uint32_t i;
    int16_t sample = MUSIC_PLAYER_PROBE_TONE_AMP;

    if (duration_ms == 0U)
    {
        return;
    }
    if (freq_hz == 0U)
    {
        freq_hz = 1000U;
    }

    total_samples = (sample_rate * duration_ms) / 1000U;
    half_period = sample_rate / (freq_hz * 2U);
    if (half_period == 0U)
    {
        half_period = 1U;
    }

    printf("[MUSIC_PROBE] tone in stream ms=%lu freq=%lu amp=%d\r\n",
           (unsigned long)duration_ms,
           (unsigned long)freq_hz,
           (int)sample);

    for (i = 0; (i < total_samples) && !g_play_stop_request; i++)
    {
        if ((i % half_period) == 0U)
        {
            sample = (sample > 0) ? -MUSIC_PLAYER_PROBE_TONE_AMP : MUSIC_PLAYER_PROBE_TONE_AMP;
        }
        music_player_send_sample(sample, sample);
    }
}

static void music_player_stream_stabilize(uint32_t sample_rate)
{
    uint32_t i;
    uint32_t samples;

    if (MUSIC_PLAYER_STREAM_STABILIZE_MS == 0U)
    {
        return;
    }
    if (sample_rate == 0U)
    {
        sample_rate = MUSIC_PLAYER_OUTPUT_SAMPLE_RATE;
    }

    samples = (sample_rate * MUSIC_PLAYER_STREAM_STABILIZE_MS) / 1000U;
    for (i = 0U; (i < samples) && !g_play_stop_request; i++)
    {
        music_player_send_sample(0, 0);
    }
}

static void music_player_log_chunk_stats(uint8_t *data, uint32_t size, const music_info_struct *info, uint32_t chunk_index)
{
    uint32_t frame_bytes;
    uint32_t frames;
    uint32_t i;
    int16_t min_l = 32767;
    int16_t max_l = -32768;
    int16_t min_r = 32767;
    int16_t max_r = -32768;
    uint32_t nonzero = 0U;

    if ((chunk_index >= 4U) || (data == NULL) || (info == NULL))
    {
        return;
    }

    frame_bytes = (uint32_t)info->channels * ((uint32_t)info->bits_per_sample / 8U);
    if (frame_bytes == 0U)
    {
        return;
    }
    frames = size / frame_bytes;

    for (i = 0; i < frames; i++)
    {
        const uint8_t *p = data + i * frame_bytes;
        int16_t left = 0;
        int16_t right = 0;
        music_player_read_frame_raw(p, info, &left, &right);
        if ((left != 0) || (right != 0))
        {
            nonzero++;
        }
        if (left < min_l) { min_l = left; }
        if (left > max_l) { max_l = left; }
        if (right < min_r) { min_r = right; }
        if (right > max_r) { max_r = right; }
    }

    printf("[MUSIC_DATA] chunk=%lu frames=%lu nonzero=%lu L=%d..%d R=%d..%d first=%02X %02X %02X %02X\r\n",
           (unsigned long)chunk_index,
           (unsigned long)frames,
           (unsigned long)nonzero,
           (int)min_l,
           (int)max_l,
           (int)min_r,
           (int)max_r,
           (size > 0U) ? data[0] : 0U,
           (size > 1U) ? data[1] : 0U,
           (size > 2U) ? data[2] : 0U,
           (size > 3U) ? data[3] : 0U);
}

#if MUSIC_PLAYER_ASYNC_ENABLE
static uint32_t music_player_async_ring_used(void)
{
    uint32_t wr = g_async_wr;
    uint32_t rd = g_async_rd;

    if (wr >= rd)
    {
        return wr - rd;
    }
    return MUSIC_PLAYER_ASYNC_RING_FRAMES - rd + wr;
}

static uint32_t music_player_async_ring_free(void)
{
    return (MUSIC_PLAYER_ASYNC_RING_FRAMES - 1U) - music_player_async_ring_used();
}

static void music_player_async_ring_clear(void)
{
    g_async_wr = 0U;
    g_async_rd = 0U;
    g_async_underruns = 0U;
    g_async_sent_frames = 0U;
    g_async_nonzero_frames = 0U;
    g_async_peak = 0U;
    g_async_stop_fill = 0U;
    g_async_iis_ready = 0U;
    g_async_read_log_once = 0U;
    g_async_core_log_once = 0U;
    g_async_pace_next_us = 0U;
    g_async_pace_frac_us = 0U;
    g_async_play_start_us = 0U;
    g_async_play_elapsed_us = 0U;
    g_async_last_left = 0;
    g_async_last_right = 0;
}

static void music_player_async_pace_reset(void)
{
    g_async_pace_next_us = 0U;
    g_async_pace_frac_us = 0U;
}

static void music_player_async_pace_one_frame(uint32_t sample_rate)
{
#if MUSIC_PLAYER_ASYNC_PACE_ENABLE
    uint64_t frame_us;
    uint32_t step_us;

    if (sample_rate == 0U)
    {
        return;
    }
    if (g_async_pace_next_us == 0U)
    {
        g_async_pace_next_us = system_getval_us();
    }

    frame_us = 1000000ULL + (uint64_t)g_async_pace_frac_us;
    step_us = (uint32_t)(frame_us / (uint64_t)sample_rate);
    g_async_pace_frac_us = (uint32_t)(frame_us - ((uint64_t)step_us * (uint64_t)sample_rate));
    g_async_pace_next_us += step_us;

    while (((uint32_t)(system_getval_us() - g_async_pace_next_us)) > 0x80000000UL)
    {
    }
#else
    (void)sample_rate;
#endif
}

static void music_player_async_ring_push(int16_t left, int16_t right)
{
    uint32_t wr = g_async_wr;
    uint32_t next = wr + 1U;

    if (next >= MUSIC_PLAYER_ASYNC_RING_FRAMES)
    {
        next = 0U;
    }
    if (next == g_async_rd)
    {
        return;
    }

    g_async_ring_left[wr] = left;
    g_async_ring_right[wr] = right;
    g_async_wr = next;
}

static uint8_t music_player_async_ring_pop(int16_t *left, int16_t *right)
{
    uint32_t rd = g_async_rd;

    if (rd == g_async_wr)
    {
        return 0U;
    }

    *left = g_async_ring_left[rd];
    *right = g_async_ring_right[rd];
    rd++;
    if (rd >= MUSIC_PLAYER_ASYNC_RING_FRAMES)
    {
        rd = 0U;
    }
    g_async_rd = rd;
    return 1U;
}

static uint8_t music_player_async_push_audio_data(uint8_t *data, uint32_t size, const music_info_struct *info)
{
    uint32_t frame_bytes;
    uint32_t frames;
    uint32_t step_q16;
    uint32_t output_rate;
    uint32_t i;

    if ((data == NULL) || (size == 0U) || (info == NULL))
    {
        return 1U;
    }
    if ((info->bits_per_sample != 16U) && (info->bits_per_sample != 24U))
    {
        return 2U;
    }
    if ((info->channels != 1U) && (info->channels != 2U))
    {
        return 2U;
    }

    frame_bytes = (uint32_t)info->channels * ((uint32_t)info->bits_per_sample / 8U);
    if (frame_bytes == 0U)
    {
        return 2U;
    }

    frames = size / frame_bytes;
    if (frames == 0U)
    {
        return 0U;
    }

    output_rate = music_player_output_rate();

    if (info->sample_rate != output_rate)
    {
        if (info->sample_rate == 0U)
        {
            step_q16 = 65536U;
        }
        else
        {
            step_q16 = (uint32_t)((((uint64_t)info->sample_rate) << 16) / output_rate);
            if (step_q16 == 0U)
            {
                step_q16 = 65536U;
            }
        }

        if ((g_resample_rate != info->sample_rate) || (g_resample_step_q16 != step_q16))
        {
            g_resample_pos_q16 = 0U;
            g_resample_rate = info->sample_rate;
            g_resample_step_q16 = step_q16;
            g_resample_has_last = 0U;
        }

        if (info->sample_rate < output_rate)
        {
            while ((g_resample_pos_q16 >> 16) < frames)
            {
                uint32_t src_index = g_resample_pos_q16 >> 16;
                uint32_t frac = g_resample_pos_q16 & 0xFFFFU;
                const uint8_t *p0 = data + src_index * frame_bytes;
                int16_t left0 = 0;
                int16_t right0 = 0;
                int16_t left1 = 0;
                int16_t right1 = 0;
                int32_t left;
                int32_t right;

                if ((src_index == 0U) && g_resample_has_last)
                {
                    left0 = g_resample_last_left;
                    right0 = g_resample_last_right;
                    music_player_read_frame_raw(p0, info, &left1, &right1);
                }
                else
                {
                    if (g_resample_has_last)
                    {
                        p0 = data + (src_index - 1U) * frame_bytes;
                    }
                    music_player_read_frame_raw(p0, info, &left0, &right0);
                    if (g_resample_has_last ? (src_index < frames) : ((src_index + 1U) < frames))
                    {
                        const uint8_t *p1 = g_resample_has_last ?
                                            (data + src_index * frame_bytes) :
                                            (data + (src_index + 1U) * frame_bytes);
                        music_player_read_frame_raw(p1, info, &left1, &right1);
                    }
                    else
                    {
                        left1 = left0;
                        right1 = right0;
                    }
                }

                left = (int32_t)left0 + ((((int32_t)left1 - (int32_t)left0) * (int32_t)frac) >> 16);
                right = (int32_t)right0 + ((((int32_t)right1 - (int32_t)right0) * (int32_t)frac) >> 16);
                music_player_async_ring_push(music_player_apply_gain(left),
                                             music_player_apply_gain(right));
                g_resample_pos_q16 += step_q16;
            }

            g_resample_pos_q16 -= (frames << 16);
            if (frames > 0U)
            {
                const uint8_t *plast = data + (frames - 1U) * frame_bytes;
                music_player_read_frame_raw(plast, info, &g_resample_last_left, &g_resample_last_right);
                g_resample_has_last = 1U;
            }
            return 0U;
        }

        while ((g_resample_pos_q16 >> 16) < frames)
        {
            uint32_t start_index = g_resample_pos_q16 >> 16;
            uint32_t next_pos_q16 = g_resample_pos_q16 + step_q16;
            uint32_t end_index = next_pos_q16 >> 16;
            int32_t left_sum = 0;
            int32_t right_sum = 0;
            uint32_t count = 0U;
            uint32_t frame_index;

            if (end_index <= start_index)
            {
                end_index = start_index + 1U;
            }
            if (end_index > frames)
            {
                end_index = frames;
            }

            for (frame_index = start_index; frame_index < end_index; frame_index++)
            {
                const uint8_t *p = data + frame_index * frame_bytes;
                int16_t left = 0;
                int16_t right = 0;

                music_player_read_frame_raw(p, info, &left, &right);
                left_sum += left;
                right_sum += right;
                count++;
            }

            if (count == 0U)
            {
                break;
            }
            music_player_async_ring_push(music_player_apply_gain(left_sum / (int32_t)count),
                                         music_player_apply_gain(right_sum / (int32_t)count));
            g_resample_pos_q16 = next_pos_q16;
        }

        g_resample_pos_q16 -= (frames << 16);
        return 0U;
    }

    music_player_resample_reset();
    for (i = 0U; i < frames; i++)
    {
        const uint8_t *p = data + i * frame_bytes;
        int16_t left = 0;
        int16_t right = 0;
        music_player_read_frame_raw(p, info, &left, &right);
        music_player_async_ring_push(music_player_apply_gain(left),
                                     music_player_apply_gain(right));
    }
    return 0U;
}

static void music_player_async_finish(uint8_t result)
{
    if ((result == 0U) && (g_async_result != 0U))
    {
        result = g_async_result;
    }
    if (g_async_file_open)
    {
        (void)f_close(&g_async_file);
        g_async_file_open = 0U;
    }
    g_async_result = result;
    g_async_done = 1U;
    g_async_running = 0U;
    g_music_player.state = (result == 0U) ? MUSIC_PLAYER_STOPPED : MUSIC_PLAYER_ERROR;
}

static void music_player_async_start_iis_on_core(void)
{
    uint32_t output_rate = music_player_output_rate();

    music_player_tx_init();
    music_player_set_tx_rate(output_rate);
    music_player_tx_start();
    music_player_stream_stabilize(output_rate);
    music_player_async_pace_reset();
    g_async_play_start_us = system_getval_us();
    g_async_iis_ready = 1U;
}
#endif

static void music_player_send_square_tone(uint32_t duration_ms, uint32_t freq_hz, uint32_t sample_rate, int16_t amp)
{
    uint32_t total_samples;
    uint32_t half_period;
    uint32_t i;
    int16_t sample = amp;

    if (duration_ms == 0U)
    {
        return;
    }
    if (freq_hz == 0U)
    {
        freq_hz = 1000U;
    }
    if (sample_rate == 0U)
    {
        sample_rate = 16000U;
    }

    total_samples = (sample_rate * duration_ms) / 1000U;
    half_period = sample_rate / (freq_hz * 2U);
    if (half_period == 0U)
    {
        half_period = 1U;
    }

    for (i = 0; i < total_samples; i++)
    {
        if ((i % half_period) == 0U)
        {
            sample = (sample > 0) ? -amp : amp;
        }
        music_player_send_sample(sample, sample);
    }
}

uint8_t music_player_play_test_tone(uint32_t duration_ms, uint32_t freq_hz)
{
    uint32_t sample_rate = 16000U;
    uint8_t ret;

    if (duration_ms == 0U)
    {
        duration_ms = 800U;
    }
    if (freq_hz == 0U)
    {
        freq_hz = 1000U;
    }

    ret = music_player_codec_init_i2s();
    if (ret != 0U)
    {
        return ret;
    }

    if (g_music_player.volume == 0U)
    {
        g_music_player.volume = MUSIC_PLAYER_DEFAULT_VOLUME;
    }

    music_player_codec_play_mode();
    music_player_dump_play_regs("tone");
    music_player_set_tx_rate(sample_rate);
    printf("[MUSIC_TEST] tone start ms=%lu freq=%lu fs=%lu\r\n",
           (unsigned long)duration_ms,
           (unsigned long)freq_hz,
           (unsigned long)sample_rate);
    music_player_tx_start();

    music_player_send_square_tone(duration_ms, freq_hz, sample_rate, 12000);

    printf("[MUSIC_TEST] tone stop begin\r\n");
    music_player_tx_stop();
    printf("[MUSIC_TEST] tone stop ok\r\n");
    return 0U;
}

uint8_t music_player_play_output_scan_tone(void)
{
    uint8_t ret;
    uint8_t i;
    static const uint8_t o1_enable[3] = {1U, 0U, 1U};
    static const uint8_t o2_enable[3] = {0U, 1U, 1U};
    static const char *tag[3] = {"O1", "O2", "O1O2"};
    const uint8_t scan_volume = 23U;
    const uint32_t sample_rate = 16000U;

    ret = music_player_codec_init_i2s();
    if (ret != 0U)
    {
        return ret;
    }

    es8388_adda_cfg(1U, 0U);
    es8388_i2s_cfg((uint8_t)MUSIC_PLAYER_I2S_FORMAT, (uint8_t)MUSIC_PLAYER_CODEC_DATA_LEN);
    es8388_hpvol_set(scan_volume);
    es8388_spkvol_set(scan_volume);
    music_player_set_tx_rate(sample_rate);
    music_player_tx_start();

    for (i = 0U; i < 3U; i++)
    {
        es8388_output_cfg(o1_enable[i], o2_enable[i]);
        printf("[MUSIC_SCAN] output=%s o1=%u o2=%u vol=%u reg04=0x%02X\r\n",
               tag[i],
               (unsigned)o1_enable[i],
               (unsigned)o2_enable[i],
               (unsigned)scan_volume,
               es8388_read_reg(0x04));
        music_player_dump_play_regs(tag[i]);
        music_player_send_square_tone(650U, 1000U, sample_rate, 12000);
        music_player_send_square_tone(120U, 0U, sample_rate, 0);
    }

    music_player_tx_stop();
    printf("[MUSIC_SCAN] done\r\n");
    return 0U;
}

static void music_player_tone_claim_output(void)
{
#if MUSIC_PLAYER_ASYNC_ENABLE
    if (g_async_running)
    {
        g_play_stop_request = 0U;
        g_play_paused = 0U;
        g_async_iis_ready = 0U;
        music_player_async_ring_clear();
        music_player_async_finish(0U);
    }
#endif
}

uint8_t music_player_tone_start(uint32_t freq_hz, int16_t amp)
{
    uint8_t ret;

    if (freq_hz == 0U)
    {
        freq_hz = 1000U;
    }
    if (amp == 0)
    {
        amp = MUSIC_PLAYER_TONE_DEFAULT_AMP;
    }
    if (amp < 0)
    {
        amp = (int16_t)(-amp);
    }

    g_tone_freq_hz = freq_hz;
    g_tone_half_period = MUSIC_PLAYER_TONE_SAMPLE_RATE / (freq_hz * 2U);
    if (g_tone_half_period == 0U)
    {
        g_tone_half_period = 1U;
    }
    g_tone_sample_index = 0U;
    g_tone_amp = amp;
    g_tone_sample = amp;

    if (g_tone_active)
    {
        printf("[MUSIC_TONE] update freq=%lu amp=%d\r\n",
               (unsigned long)freq_hz,
               (int)amp);
        return 0U;
    }
    if (music_player_async_is_active())
    {
        music_player_tone_claim_output();
    }

    ret = es8388_init();
    if (ret != 0U)
    {
        printf("[MUSIC_TONE] init failed ret=%u\r\n", ret);
        return ret;
    }

    if (g_music_player.volume == 0U)
    {
        g_music_player.volume = MUSIC_PLAYER_DEFAULT_VOLUME;
    }

    es8388_adda_cfg(1U, 1U);
    es8388_i2s_cfg(ES8388_DEFAULT_I2S_FORMAT, ES8388_DEFAULT_DATA_LEN);
    es8388_write_reg(0x0CU, 0x4CU);
    es8388_write_reg(0x17U, 0x18U);
    es8388_write_reg(0x18U, 0x12U);
    es8388_write_reg(0x2BU, 0x80U);
    es8388_output_cfg(ES8388_OUTPUT_O1_ENABLE, ES8388_OUTPUT_O2_ENABLE);
    es8388_hpvol_set(music_player_convert_volume(g_music_player.volume));
    es8388_spkvol_set(music_player_convert_volume(g_music_player.volume));
    music_player_tx_init();
    music_player_set_tx_rate(MUSIC_PLAYER_TONE_SAMPLE_RATE);
    music_player_tx_start();

    g_tone_active = 1U;
    g_tone_start_ms = system_getval_ms();
    g_music_player.state = MUSIC_PLAYER_PLAYING;

    printf("[MUSIC_TONE] start freq=%lu amp=%d fs=%u batch=%u max=%u fmt=I2S\r\n",
           (unsigned long)freq_hz,
           (int)amp,
           (unsigned)MUSIC_PLAYER_TONE_SAMPLE_RATE,
           (unsigned)MUSIC_PLAYER_TONE_SEND_BATCH,
           (unsigned)MUSIC_PLAYER_TONE_MAX_BURST);
    return 0U;
}

uint8_t music_player_tone_play_blocking(uint32_t freq_hz, int16_t amp, uint32_t duration_ms)
{
    uint8_t ret;
    uint32_t total_samples;
    uint32_t i;
    uint32_t half_period;
    uint32_t play_start_us;
    uint32_t frame_target_us;
    uint32_t tone_elapsed_us;
    int16_t sample;

    if (duration_ms == 0U)
    {
        return 0U;
    }
    if (freq_hz == 0U)
    {
        freq_hz = 1000U;
    }
    if (amp == 0)
    {
        amp = MUSIC_PLAYER_TONE_DEFAULT_AMP;
    }
    if (amp < 0)
    {
        amp = (int16_t)(-amp);
    }
    if (music_player_async_is_active())
    {
        music_player_tone_claim_output();
    }
    if (g_tone_active)
    {
        music_player_tone_stop();
    }

    ret = es8388_init();
    if (ret != 0U)
    {
        printf("[MUSIC_TONE_BLOCK] init failed ret=%u\r\n", ret);
        return ret;
    }

    if (g_music_player.volume == 0U)
    {
        g_music_player.volume = MUSIC_PLAYER_DEFAULT_VOLUME;
    }

    es8388_adda_cfg(1U, 0U);
    es8388_i2s_cfg(ES8388_DEFAULT_I2S_FORMAT, ES8388_DEFAULT_DATA_LEN);
    es8388_write_reg(0x08U, 0x00U);
    es8388_write_reg(0x0CU, 0x4CU);
    es8388_write_reg(0x17U, 0x18U);
    es8388_write_reg(0x18U, 0x12U);
    es8388_write_reg(0x2BU, 0x80U);
    es8388_output_cfg(ES8388_OUTPUT_O1_ENABLE, ES8388_OUTPUT_O2_ENABLE);
    es8388_hpvol_set(music_player_convert_volume(g_music_player.volume));
    es8388_spkvol_set(music_player_convert_volume(g_music_player.volume));
    music_player_tx_init();
    music_player_set_tx_rate(MUSIC_PLAYER_TONE_SAMPLE_RATE);
    music_player_tx_start();
    music_player_stream_stabilize(MUSIC_PLAYER_TONE_SAMPLE_RATE);
    g_music_player.state = MUSIC_PLAYER_PLAYING;

    total_samples = (MUSIC_PLAYER_TONE_SAMPLE_RATE * duration_ms) / 1000U;
    half_period = MUSIC_PLAYER_TONE_SAMPLE_RATE / (freq_hz * 2U);
    if (half_period == 0U)
    {
        half_period = 1U;
    }
    sample = 0;

    printf("[MUSIC_TONE_BLOCK] start freq=%lu amp=%d ms=%lu samples=%lu fs=%u fmt=I2S mcu_master_tx=1\r\n",
           (unsigned long)freq_hz,
           (int)amp,
           (unsigned long)duration_ms,
           (unsigned long)total_samples,
           (unsigned)MUSIC_PLAYER_TONE_SAMPLE_RATE);

    play_start_us = system_getval_us();
    for (i = 0U; i < total_samples; i++)
    {
        sample = music_player_triangle_sample(i, half_period, amp);
        music_player_send_sample(sample, sample);
        frame_target_us = play_start_us + (uint32_t)((((uint64_t)(i + 1U)) * 1000000ULL) / MUSIC_PLAYER_TONE_SAMPLE_RATE);
        music_player_wait_until_us(frame_target_us);
    }
    tone_elapsed_us = system_getval_us() - play_start_us;

    if (ret == 0U)
    {
        uint32_t tail = 0U;
        for (tail = 0U; tail < MUSIC_PLAYER_TONE_END_SILENCE_FRAMES; tail++)
        {
            music_player_send_sample(0, 0);
            frame_target_us = play_start_us + (uint32_t)((((uint64_t)(i + tail + 1U)) * 1000000ULL) / MUSIC_PLAYER_TONE_SAMPLE_RATE);
            music_player_wait_until_us(frame_target_us);
        }
    }

    music_player_tx_stop();
    music_player_release_iis_pins();
    g_music_player.state = MUSIC_PLAYER_STOPPED;
    printf("[MUSIC_TONE_BLOCK] stop freq=%lu samples=%lu elapsed_ms=%lu ret=%u record_recover=defer reg08=%02X\r\n",
           (unsigned long)freq_hz,
           (unsigned long)i,
           (unsigned long)((tone_elapsed_us + 500U) / 1000U),
           (unsigned)ret,
           es8388_read_reg(0x08));
    return ret;
}

void music_player_tone_stop(void)
{
    if (!g_tone_active)
    {
        return;
    }

    g_tone_active = 0U;
    music_player_tx_stop();
    music_player_release_iis_pins();
    g_music_player.state = MUSIC_PLAYER_STOPPED;
    printf("[MUSIC_TONE] stop freq=%lu samples=%lu\r\n",
           (unsigned long)g_tone_freq_hz,
           (unsigned long)g_tone_sample_index);
}

uint8_t music_player_tone_is_active(void)
{
    return g_tone_active ? 1U : 0U;
}

void music_player_tone_task(void)
{
    uint32_t i;
    uint32_t target_samples;
    uint32_t now_ms;
    uint32_t to_send;

    if (!g_tone_active)
    {
        return;
    }

    now_ms = system_getval_ms();
    target_samples = ((now_ms - g_tone_start_ms) * MUSIC_PLAYER_TONE_SAMPLE_RATE) / 1000U;
    if (target_samples <= g_tone_sample_index)
    {
        return;
    }
    else
    {
        to_send = target_samples - g_tone_sample_index;
        if (to_send > MUSIC_PLAYER_TONE_MAX_BURST)
        {
            to_send = MUSIC_PLAYER_TONE_MAX_BURST;
        }
    }

    for (i = 0U; i < to_send; i++)
    {
        if ((g_tone_sample_index % g_tone_half_period) == 0U)
        {
            g_tone_sample = (g_tone_sample > 0) ? (int16_t)(-g_tone_amp) : g_tone_amp;
        }
        music_player_send_sample(g_tone_sample, g_tone_sample);
        g_tone_sample_index++;
    }
}

static uint8_t has_wav_ext(const char *name)
{
    uint32_t len;

    if (name == NULL)
    {
        return 0;
    }

    len = (uint32_t)strlen(name);
    if (len < 4U)
    {
        return 0;
    }

    return ((name[len - 4U] == '.') &&
            ((name[len - 3U] == 'w') || (name[len - 3U] == 'W')) &&
            ((name[len - 2U] == 'a') || (name[len - 2U] == 'A')) &&
            ((name[len - 1U] == 'v') || (name[len - 1U] == 'V')));
}

static uint8_t wav_header_parse(FIL *fp, music_info_struct *info, uint32_t *data_start)
{
    uint8_t header[12];
    uint8_t fmt_buf[40];
    UINT br = 0;
    uint8_t got_fmt = 0;
    uint16_t fmt_audio = 0;
    uint16_t fmt_channels = 0;
    uint16_t fmt_bits = 0;
    uint32_t fmt_sample_rate = 0;
    uint32_t fmt_byte_rate = 0;

    if ((fp == NULL) || (info == NULL) || (data_start == NULL))
    {
        return wav_parse_fail(1U, 0U, 0U);
    }

    wav_diag_reset();

    if ((f_lseek(fp, 0) != FR_OK) ||
        (f_read(fp, header, sizeof(header), &br) != FR_OK) ||
        (br != sizeof(header)))
    {
        return wav_parse_fail(2U, 0U, 0U);
    }

    if ((wav_read_le32(header + 0) != WAV_ID_RIFF) ||
        (wav_read_le32(header + 8) != WAV_ID_WAVE))
    {
        return wav_parse_fail(3U, wav_read_le32(header + 0), wav_read_le32(header + 8));
    }

    while (1)
    {
        uint8_t chunk_header[8];
        uint32_t chunk_id;
        uint32_t chunk_size;
        FSIZE_t chunk_data_pos;
        FSIZE_t next_chunk_pos;

        if ((f_read(fp, chunk_header, sizeof(chunk_header), &br) != FR_OK) ||
            (br != sizeof(chunk_header)))
        {
            return wav_parse_fail(4U, 0U, 0U);
        }

        chunk_id = wav_read_le32(chunk_header + 0);
        chunk_size = wav_read_le32(chunk_header + 4);
        chunk_data_pos = (FSIZE_t)f_tell(fp);

        if (chunk_id == WAV_ID_FMT)
        {
            uint32_t read_size;

            if (chunk_size < 16U)
            {
                return wav_parse_fail(5U, chunk_id, chunk_size);
            }

            read_size = (chunk_size > sizeof(fmt_buf)) ? sizeof(fmt_buf) : chunk_size;
            memset(fmt_buf, 0, sizeof(fmt_buf));
            if ((f_read(fp, fmt_buf, read_size, &br) != FR_OK) ||
                (br != read_size))
            {
                return wav_parse_fail(6U, chunk_id, chunk_size);
            }

            fmt_audio = wav_read_le16(fmt_buf + 0);
            fmt_channels = wav_read_le16(fmt_buf + 2);
            fmt_sample_rate = wav_read_le32(fmt_buf + 4);
            fmt_byte_rate = wav_read_le32(fmt_buf + 8);
            fmt_bits = wav_read_le16(fmt_buf + 14);

            g_wav_parse_diag.audio_format = fmt_audio;
            g_wav_parse_diag.channels = fmt_channels;
            g_wav_parse_diag.sample_rate = fmt_sample_rate;
            g_wav_parse_diag.bits_per_sample = fmt_bits;

            if (fmt_audio == WAV_FORMAT_EXTENSIBLE)
            {
                fmt_audio = wav_parse_extensible_sub_format(fmt_buf, read_size);
            }
            got_fmt = 1;
        }
        else if (chunk_id == WAV_ID_DATA)
        {
            if (!got_fmt)
            {
                return wav_parse_fail(7U, chunk_id, chunk_size);
            }

            if ((fmt_audio != WAV_FORMAT_PCM) ||
                (fmt_sample_rate == 0U) ||
                ((fmt_channels != 1U) && (fmt_channels != 2U)) ||
                ((fmt_bits != 16U) && (fmt_bits != 24U)))
            {
                return wav_parse_fail(8U, chunk_id, chunk_size);
            }

            info->sample_rate = fmt_sample_rate;
            info->bits_per_sample = fmt_bits;
            info->channels = fmt_channels;
            info->data_size = chunk_size;
            info->bitrate = fmt_byte_rate * 8U;

            if (fmt_byte_rate != 0U)
            {
                info->duration_seconds = info->data_size / fmt_byte_rate;
            }
            else
            {
                info->duration_seconds = 0U;
            }

            *data_start = (uint32_t)chunk_data_pos;
            return 0;
        }

        next_chunk_pos = chunk_data_pos + (FSIZE_t)chunk_size;
        if ((chunk_size & 1U) != 0U)
        {
            next_chunk_pos += 1U;
        }
        if (f_lseek(fp, next_chunk_pos) != FR_OK)
        {
            return wav_parse_fail(9U, chunk_id, chunk_size);
        }
    }
}

static uint8_t music_player_send_audio_data(uint8_t *data, uint32_t size, const music_info_struct *info)
{
    uint32_t output_rate = music_player_output_rate();

    if ((data == NULL) || (size == 0U) || (info == NULL))
    {
        return 1;
    }

    if (info->sample_rate != output_rate)
    {
        uint32_t frame_bytes;
        uint32_t frames;
        uint32_t step_q16;

        if ((info->bits_per_sample != 16U) && (info->bits_per_sample != 24U))
        {
            return 2;
        }
        if ((info->channels != 1U) && (info->channels != 2U))
        {
            return 2;
        }

        frame_bytes = (uint32_t)info->channels * ((uint32_t)info->bits_per_sample / 8U);
        if (frame_bytes == 0U)
        {
            return 2;
        }

        frames = size / frame_bytes;
        if (frames == 0U)
        {
            return 0;
        }

        if (info->sample_rate == 0U)
        {
            step_q16 = 65536U;
        }
        else
        {
            step_q16 = (uint32_t)((((uint64_t)info->sample_rate) << 16) / output_rate);
            if (step_q16 == 0U)
            {
                step_q16 = 65536U;
            }
        }

        if ((g_resample_rate != info->sample_rate) || (g_resample_step_q16 != step_q16))
        {
            g_resample_pos_q16 = 0U;
            g_resample_rate = info->sample_rate;
            g_resample_step_q16 = step_q16;
            g_resample_has_last = 0U;
        }

        if (info->sample_rate < output_rate)
        {
            while ((g_resample_pos_q16 >> 16) < frames)
            {
                uint32_t src_index = g_resample_pos_q16 >> 16;
                uint32_t frac = g_resample_pos_q16 & 0xFFFFU;
                const uint8_t *p0 = data + src_index * frame_bytes;
                int16_t left0 = 0;
                int16_t right0 = 0;
                int16_t left1 = 0;
                int16_t right1 = 0;
                int32_t left;
                int32_t right;

                if ((src_index == 0U) && g_resample_has_last)
                {
                    left0 = g_resample_last_left;
                    right0 = g_resample_last_right;
                    music_player_read_frame_raw(p0, info, &left1, &right1);
                }
                else
                {
                    if (g_resample_has_last)
                    {
                        p0 = data + (src_index - 1U) * frame_bytes;
                    }
                    music_player_read_frame_raw(p0, info, &left0, &right0);
                    if (g_resample_has_last ? (src_index < frames) : ((src_index + 1U) < frames))
                    {
                        const uint8_t *p1 = g_resample_has_last ?
                                            (data + src_index * frame_bytes) :
                                            (data + (src_index + 1U) * frame_bytes);
                        music_player_read_frame_raw(p1, info, &left1, &right1);
                    }
                    else
                    {
                        left1 = left0;
                        right1 = right0;
                    }
                }

                left = (int32_t)left0 + ((((int32_t)left1 - (int32_t)left0) * (int32_t)frac) >> 16);
                right = (int32_t)right0 + ((((int32_t)right1 - (int32_t)right0) * (int32_t)frac) >> 16);
                music_player_send_sample(music_player_apply_gain(left),
                                         music_player_apply_gain(right));
                g_resample_pos_q16 += step_q16;
            }

            g_resample_pos_q16 -= (frames << 16);
            if (frames > 0U)
            {
                const uint8_t *plast = data + (frames - 1U) * frame_bytes;
                music_player_read_frame_raw(plast, info, &g_resample_last_left, &g_resample_last_right);
                g_resample_has_last = 1U;
            }
            return 0;
        }

        while ((g_resample_pos_q16 >> 16) < frames)
        {
            uint32_t start_index = g_resample_pos_q16 >> 16;
            uint32_t next_pos_q16 = g_resample_pos_q16 + step_q16;
            uint32_t end_index = next_pos_q16 >> 16;
            int32_t left_sum = 0;
            int32_t right_sum = 0;
            uint32_t count = 0U;
            uint32_t frame_index;

            if (end_index <= start_index)
            {
                end_index = start_index + 1U;
            }
            if (end_index > frames)
            {
                end_index = frames;
            }

            for (frame_index = start_index; frame_index < end_index; frame_index++)
            {
                const uint8_t *p = data + frame_index * frame_bytes;
                int16_t left = 0;
                int16_t right = 0;

                music_player_read_frame_raw(p, info, &left, &right);
                left_sum += left;
                right_sum += right;
                count++;
            }

            if (count == 0U)
            {
                break;
            }

            music_player_send_sample(music_player_apply_gain(left_sum / (int32_t)count),
                                     music_player_apply_gain(right_sum / (int32_t)count));
            g_resample_pos_q16 = next_pos_q16;
        }

        g_resample_pos_q16 -= (frames << 16);

        return 0;
    }

    music_player_resample_reset();

    if (info->bits_per_sample == 16U)
    {
        uint32_t frame_bytes;
        uint32_t frames;
        uint32_t i;

        if ((info->channels != 1U) && (info->channels != 2U))
        {
            return 2;
        }

        frame_bytes = (info->channels == 2U) ? 4U : 2U;
        frames = size / frame_bytes;

        for (i = 0; i < frames; i++)
        {
            uint8_t *p = data + i * frame_bytes;
            int16_t left = (int16_t)wav_read_le16(p);
            int16_t right = left;

            if (info->channels == 2U)
            {
                right = (int16_t)wav_read_le16(p + 2U);
            }

            music_player_send_sample(left, right);
        }

        return 0;
    }

    if (info->bits_per_sample == 24U)
    {
        uint32_t i;
        uint32_t frame_bytes;
        uint32_t frames;
        uint8_t *p = data;

        if ((info->channels != 1U) && (info->channels != 2U))
        {
            return 2;
        }

        frame_bytes = (info->channels == 2U) ? 6U : 3U;
        frames = size / frame_bytes;

        for (i = 0; i < frames; i++)
        {
            int16_t left = wav_sample24_to16(p);
            int16_t right = left;

            if (info->channels == 2U)
            {
                right = wav_sample24_to16(p + 3U);
                p += 6U;
            }
            else
            {
                p += 3U;
            }

            music_player_send_sample(left, right);
        }

        return 0;
    }

    return 2;
}

uint8_t music_player_init(void)
{
    FRESULT fr;
    uint8_t ret;

    if (g_music_player_inited)
    {
        printf("[MUSIC] init already ok\r\n");
        return 0;
    }

    printf("[MUSIC] init begin\r\n");
    ret = sd_simple_init();
    if (ret != 0U)
    {
        printf("[MUSIC] sd_simple_init failed, ret=%u\r\n", ret);
        return 1;
    }

    fr = f_mount(&g_fatfs, "0:", 1);
    if (fr != FR_OK)
    {
        printf("[MUSIC] f_mount failed, fr=%d\r\n", (int)fr);
        return 2;
    }

    ret = music_player_codec_init_i2s();
    if (ret != 0U)
    {
        return 3;
    }

    memset(&g_music_player, 0, sizeof(g_music_player));
    g_music_player.state = MUSIC_PLAYER_STOPPED;
    g_music_player.play_mode = MUSIC_PLAY_MODE_SINGLE;
    g_music_player.volume = MUSIC_PLAYER_DEFAULT_VOLUME;
    g_music_player.auto_play = 1U;

    es8388_hpvol_set(music_player_convert_volume(g_music_player.volume));
    es8388_spkvol_set(music_player_convert_volume(g_music_player.volume));
    printf("[MUSIC] volume set hp/spk=%u\r\n", (unsigned)music_player_convert_volume(g_music_player.volume));
    g_music_player_inited = 1U;
    printf("[MUSIC] init ok\r\n");
    return 0;
}

uint8_t music_player_scan_folder(const char *path, uint32_t *count)
{
    DIR dir;
    FILINFO finfo;
    FRESULT fr;

    if ((path == NULL) || (count == NULL))
    {
        return 1;
    }

    g_file_count = 0;
    memset(g_file_list, 0, sizeof(g_file_list));

    fr = f_opendir(&dir, path);
    if (fr != FR_OK)
    {
        printf("[MUSIC] open folder failed path=%s fr=%d\r\n", path, (int)fr);
        return 2;
    }

    while (1)
    {
        const char *name;

        fr = f_readdir(&dir, &finfo);
        if ((fr != FR_OK) || (finfo.fname[0] == '\0'))
        {
            break;
        }

        name = finfo.fname;

        if ((finfo.fattrib & AM_DIR) != 0U)
        {
            continue;
        }

#if FF_USE_LFN
        if (name[0] == '\0')
        {
            name = finfo.altname;
        }
#endif

        if ((name[0] == '\0') || (!has_wav_ext(name)))
        {
            continue;
        }

        if (g_file_count < (sizeof(g_file_list) / sizeof(g_file_list[0])))
        {
            snprintf(g_file_list[g_file_count], sizeof(g_file_list[g_file_count]), "%s/%s", path, name);
            g_file_count++;
        }
    }

    (void)f_closedir(&dir);
    *count = g_file_count;
    g_files_valid = 1;
    printf("[MUSIC] scan folder path=%s count=%lu\r\n",
           path,
           (unsigned long)g_file_count);
    return 0;
}

uint8_t music_player_get_info(const char *filename, music_info_struct *info)
{
    FIL f;
    FRESULT fr;
    uint32_t data_start = 0;

    if ((filename == NULL) || (info == NULL))
    {
        return 1;
    }

    fr = f_open(&f, filename, FA_READ);
    if (fr != FR_OK)
    {
        return 2;
    }

    memset(info, 0, sizeof(*info));
    snprintf(info->filename, sizeof(info->filename), "%s", filename);

    {
        uint8_t parse_ret = wav_header_parse(&f, info, &data_start);
        if (parse_ret != 0U)
        {
            wav_print_parse_diag(filename, parse_ret);
            (void)f_close(&f);
            return 3;
        }
    }

    (void)f_close(&f);
    return 0;
}

uint8_t music_player_play(const char *filename)
{
    FIL f;
    FRESULT fr;
    music_info_struct info;
    uint32_t data_start = 0;
    uint32_t remain;
    uint32_t bytes_per_frame;
    uint32_t chunk_index = 0U;

    if (filename == NULL)
    {
        return 1;
    }

    fr = f_open(&f, filename, FA_READ);
    if (fr != FR_OK)
    {
        printf("[MUSIC] open failed file=%s fr=%d\r\n", filename, (int)fr);
        return 2;
    }

    memset(&info, 0, sizeof(info));
    snprintf(info.filename, sizeof(info.filename), "%s", filename);

    {
        uint8_t parse_ret = wav_header_parse(&f, &info, &data_start);
        if (parse_ret != 0U)
        {
            wav_print_parse_diag(filename, parse_ret);
            (void)f_close(&f);
            return 3;
        }
    }

    printf("[MUSIC] wav ok file=%s sr=%lu ch=%u bits=%u data=%lu\r\n",
           filename,
           (unsigned long)info.sample_rate,
           (unsigned)info.channels,
           (unsigned)info.bits_per_sample,
           (unsigned long)info.data_size);

    if (f_lseek(&f, data_start) != FR_OK)
    {
        (void)f_close(&f);
        return 4;
    }

    bytes_per_frame = (uint32_t)info.channels * ((uint32_t)info.bits_per_sample / 8U);
    if (bytes_per_frame == 0U)
    {
        (void)f_close(&f);
        return 5;
    }

    music_player_codec_play_mode();
    music_player_dump_play_regs("wav");
    music_player_resample_reset();
    music_player_set_output_rate_from_info(&info);
    g_play_stop_request = 0;
    g_play_paused = 0;
    music_player_tx_init();
    music_player_set_tx_rate(music_player_output_rate());
    music_player_tx_start();
    music_player_stream_stabilize(music_player_output_rate());

    g_music_player.current_music = info;
    g_music_player.current_position = 0;
    g_music_player.state = MUSIC_PLAYER_PLAYING;
    printf("[MUSIC] stream start out_fs=%lu mode=%s gain=%u\r\n",
           (unsigned long)music_player_output_rate(),
           (info.sample_rate == music_player_output_rate()) ? "direct" : "resample",
           (unsigned)MUSIC_PLAYER_DIGITAL_GAIN_PERCENT);

#if MUSIC_PLAYER_STREAM_PROBE_TONE_MS > 0U
    music_player_send_probe_tone(MUSIC_PLAYER_STREAM_PROBE_TONE_MS, 1000U);
#endif

    remain = info.data_size;
    while ((remain >= bytes_per_frame) && !g_play_stop_request)
    {
        UINT br = 0;
        FRESULT fr_read;
        uint32_t chunk = (remain > MUSIC_PLAYER_BUFFER_SIZE) ? MUSIC_PLAYER_BUFFER_SIZE : remain;
        uint32_t valid_bytes;
#if MUSIC_PLAYER_IO_TIMING_LOG
        uint32_t read_ms;
        uint32_t play_ms;
        uint32_t t0;
#endif

        while (g_play_paused && !g_play_stop_request)
        {
            system_delay_ms(5);
        }

        if (chunk > bytes_per_frame)
        {
            chunk -= (chunk % bytes_per_frame);
        }
        if (chunk == 0U)
        {
            chunk = bytes_per_frame;
        }

#if MUSIC_PLAYER_IO_TIMING_LOG
        t0 = system_getval_ms();
#endif
        fr_read = f_read(&f, g_audio_buffer, chunk, &br);
        if (fr_read != FR_OK)
        {
            printf("[MUSIC] read failed fr=%d chunk=%lu req=%lu br=%u remain=%lu pos=%lu\r\n",
                   (int)fr_read,
                   (unsigned long)chunk_index,
                   (unsigned long)chunk,
                   (unsigned)br,
                   (unsigned long)remain,
                   (unsigned long)g_music_player.current_position);
            g_music_player.state = MUSIC_PLAYER_ERROR;
            music_player_tx_stop();
            (void)f_close(&f);
            return 6;
        }
#if MUSIC_PLAYER_IO_TIMING_LOG
        read_ms = system_getval_ms() - t0;
#endif

        if (br == 0U)
        {
            break;
        }

        valid_bytes = (uint32_t)br - ((uint32_t)br % bytes_per_frame);
        if (valid_bytes == 0U)
        {
            break;
        }

#if MUSIC_PLAYER_IO_TIMING_LOG
        music_player_log_chunk_stats(g_audio_buffer, valid_bytes, &info, chunk_index);
#endif
        chunk_index++;

#if MUSIC_PLAYER_IO_TIMING_LOG
        t0 = system_getval_ms();
#endif
        if (music_player_send_audio_data(g_audio_buffer, valid_bytes, &info) != 0U)
        {
            g_music_player.state = MUSIC_PLAYER_ERROR;
            music_player_tx_stop();
            (void)f_close(&f);
            return 7;
        }
#if MUSIC_PLAYER_IO_TIMING_LOG
        play_ms = system_getval_ms() - t0;
        if ((chunk_index <= 4U) || (read_ms > 5U))
        {
            printf("[MUSIC_IO] chunk=%lu read_ms=%lu play_ms=%lu bytes=%lu remain=%lu\r\n",
                   (unsigned long)(chunk_index - 1U),
                   (unsigned long)read_ms,
                   (unsigned long)play_ms,
                   (unsigned long)valid_bytes,
                   (unsigned long)remain);
        }
#endif

        if (remain >= (uint32_t)br)
        {
            remain -= (uint32_t)br;
        }
        else
        {
            remain = 0;
        }
        g_music_player.current_position += valid_bytes;
    }

    (void)f_close(&f);
    music_player_tx_stop();
    g_music_player.state = MUSIC_PLAYER_STOPPED;
    printf("[MUSIC] stream end\r\n");
    return 0;
}

#if MUSIC_PLAYER_ASYNC_ENABLE
static uint8_t music_player_start_async_file(const char *filename)
{
    FRESULT fr;
    uint32_t data_start = 0U;

    if (filename == NULL)
    {
        return 1U;
    }
    if (g_async_running)
    {
        return 8U;
    }

    fr = f_open(&g_async_file, filename, FA_READ);
    if (fr != FR_OK)
    {
        printf("[MUSIC_ASYNC] open failed file=%s fr=%d\r\n", filename, (int)fr);
        return 2U;
    }
    g_async_file_open = 1U;

    memset(&g_async_info, 0, sizeof(g_async_info));
    snprintf(g_async_info.filename, sizeof(g_async_info.filename), "%s", filename);
    {
        uint8_t parse_ret = wav_header_parse(&g_async_file, &g_async_info, &data_start);
        if (parse_ret != 0U)
        {
            wav_print_parse_diag(filename, parse_ret);
            music_player_async_finish(3U);
            return 3U;
        }
    }

    if (f_lseek(&g_async_file, data_start) != FR_OK)
    {
        music_player_async_finish(4U);
        return 4U;
    }

    g_async_bytes_per_frame = (uint32_t)g_async_info.channels * ((uint32_t)g_async_info.bits_per_sample / 8U);
    if (g_async_bytes_per_frame == 0U)
    {
        music_player_async_finish(5U);
        return 5U;
    }

    music_player_codec_play_mode();
    music_player_dump_play_regs("async");
    music_player_resample_reset();
    music_player_set_output_rate_from_info(&g_async_info);
    music_player_async_ring_clear();

    g_async_remain = g_async_info.data_size;
    g_async_started = 0U;
    g_async_eof = 0U;
    g_async_done = 0U;
    g_async_result = 0U;
    g_play_stop_request = 0U;
    g_play_paused = 0U;
    g_music_player.current_music = g_async_info;
    g_music_player.current_position = 0U;
    g_music_player.state = MUSIC_PLAYER_PLAYING;
    g_async_running = 1U;
    printf("[MUSIC_ASYNC] stream start file=%s sr=%lu ch=%u bits=%u data=%lu out_fs=%lu mode=%s ring=%lu gain=%u\r\n",
           filename,
           (unsigned long)g_async_info.sample_rate,
           (unsigned)g_async_info.channels,
           (unsigned)g_async_info.bits_per_sample,
           (unsigned long)g_async_info.data_size,
           (unsigned long)music_player_output_rate(),
           (g_async_info.sample_rate == music_player_output_rate()) ? "direct" : "resample",
           (unsigned long)MUSIC_PLAYER_ASYNC_RING_FRAMES,
           (unsigned)MUSIC_PLAYER_DIGITAL_GAIN_PERCENT);
    printf("[MUSIC_ASYNC] pace=%u batch=%u start_frames=%u\r\n",
           (unsigned)MUSIC_PLAYER_ASYNC_PACE_ENABLE,
           (unsigned)MUSIC_PLAYER_ASYNC_SEND_BATCH,
           (unsigned)MUSIC_PLAYER_ASYNC_START_FRAMES);
    return 0U;
}

uint8_t music_player_play_first_in_folder_async(const char *path)
{
    uint32_t cnt = 0U;

    if (music_player_scan_folder(path, &cnt) != 0U)
    {
        return 1U;
    }
    if (cnt == 0U)
    {
        return 2U;
    }

    g_music_player.current_file_index = 0U;
    return music_player_start_async_file(g_file_list[0]);
}

void music_player_io_task(void)
{
    UINT br = 0U;
    FRESULT fr;
    uint32_t free_frames;
    uint32_t max_src_frames;
    uint32_t free_bytes;
    uint32_t chunk;
    uint32_t valid_bytes;
    uint32_t output_rate;

    if (!g_async_running)
    {
        return;
    }
    if (g_play_stop_request)
    {
        g_async_eof = 1U;
        return;
    }
    if (g_async_eof || g_play_paused)
    {
        return;
    }

    free_frames = music_player_async_ring_free();
    if (free_frames < 256U)
    {
        return;
    }

    output_rate = music_player_output_rate();
    max_src_frames = free_frames;
    if ((g_async_info.sample_rate != 0U) && (g_async_info.sample_rate < output_rate))
    {
        max_src_frames = (uint32_t)((((uint64_t)free_frames) * g_async_info.sample_rate) / output_rate);
        if (max_src_frames > 8U)
        {
            max_src_frames -= 8U;
        }
    }
    free_bytes = max_src_frames * g_async_bytes_per_frame;
    chunk = g_async_remain;
    if (chunk > MUSIC_PLAYER_BUFFER_SIZE)
    {
        chunk = MUSIC_PLAYER_BUFFER_SIZE;
    }
    if (chunk > free_bytes)
    {
        chunk = free_bytes;
    }
    if (chunk > g_async_bytes_per_frame)
    {
        chunk -= (chunk % g_async_bytes_per_frame);
    }
    if ((chunk > 4096U) && ((chunk % 4096U) != 0U))
    {
        chunk -= (chunk % 4096U);
    }
    if (chunk == 0U)
    {
        return;
    }

    fr = f_read(&g_async_file, g_audio_buffer, chunk, &br);
    if (fr != FR_OK)
    {
        printf("[MUSIC_ASYNC] read failed fr=%d req=%lu br=%u remain=%lu fill=%lu\r\n",
               (int)fr,
               (unsigned long)chunk,
               (unsigned)br,
               (unsigned long)g_async_remain,
               (unsigned long)music_player_async_ring_used());
        music_player_async_finish(6U);
        return;
    }
    if (br == 0U)
    {
        g_async_eof = 1U;
        return;
    }

    valid_bytes = (uint32_t)br - ((uint32_t)br % g_async_bytes_per_frame);
    if (valid_bytes > 0U)
    {
        if (music_player_async_push_audio_data(g_audio_buffer, valid_bytes, &g_async_info) != 0U)
        {
            music_player_async_finish(7U);
            return;
        }
        g_music_player.current_position += valid_bytes;
#if MUSIC_PLAYER_ASYNC_DIAG
        if (!g_async_read_log_once)
        {
            g_async_read_log_once = 1U;
            printf("[MUSIC_ASYNC] first read br=%u valid=%lu fill=%lu free=%lu\r\n",
                   (unsigned)br,
                   (unsigned long)valid_bytes,
                   (unsigned long)music_player_async_ring_used(),
                   (unsigned long)music_player_async_ring_free());
        }
#endif
    }

    if (g_async_remain >= (uint32_t)br)
    {
        g_async_remain -= (uint32_t)br;
    }
    else
    {
        g_async_remain = 0U;
    }
    if (g_async_remain == 0U)
    {
        g_async_eof = 1U;
    }
    if (!g_async_started)
    {
        uint32_t used_frames = music_player_async_ring_used();
        if (g_async_eof && (used_frames == 0U))
        {
            music_player_async_finish(0U);
            return;
        }
        if ((used_frames < MUSIC_PLAYER_ASYNC_START_FRAMES) &&
            ((!g_async_eof) || (used_frames == 0U)))
        {
            return;
        }
#if MUSIC_PLAYER_ASYNC_DIAG
        printf("[MUSIC_ASYNC] start fill=%lu threshold=%lu\r\n",
               (unsigned long)used_frames,
               (unsigned long)MUSIC_PLAYER_ASYNC_START_FRAMES);
#endif
        g_async_started = 1U;
    }
}

void music_player_core_task(void)
{
    int16_t left = 0;
    int16_t right = 0;
    uint32_t sent = 0U;

    if (!g_async_running)
    {
        return;
    }
    if (g_play_stop_request)
    {
        g_async_stop_fill = music_player_async_ring_used();
        if (g_async_play_start_us != 0U)
        {
            g_async_play_elapsed_us = system_getval_us() - g_async_play_start_us;
        }
        while (music_player_async_ring_pop(&left, &right))
        {
        }
        music_player_tx_stop();
        music_player_async_finish(0U);
        return;
    }
    if (g_play_paused || (!g_async_started))
    {
        return;
    }

    if (!g_async_iis_ready)
    {
        music_player_async_start_iis_on_core();
    }

    while (sent < MUSIC_PLAYER_ASYNC_SEND_BATCH)
    {
        if (!music_player_async_ring_pop(&left, &right))
        {
            break;
        }

        g_async_last_left = left;
        g_async_last_right = right;
        {
            int32_t abs_l = (left < 0) ? -(int32_t)left : (int32_t)left;
            int32_t abs_r = (right < 0) ? -(int32_t)right : (int32_t)right;
            uint32_t peak = (abs_l > abs_r) ? (uint32_t)abs_l : (uint32_t)abs_r;
            g_async_sent_frames++;
            if ((left != 0) || (right != 0))
            {
                g_async_nonzero_frames++;
            }
            if (peak > g_async_peak)
            {
                g_async_peak = peak;
            }
        }
#if MUSIC_PLAYER_ASYNC_DIAG
        if (!g_async_core_log_once)
        {
            g_async_core_log_once = 1U;
            printf("[MUSIC_ASYNC] first sample L=%d R=%d fill=%lu\r\n",
                   (int)left,
                   (int)right,
                   (unsigned long)music_player_async_ring_used());
        }
#endif
        music_player_send_sample(left, right);
        music_player_async_pace_one_frame(music_player_output_rate());
        sent++;
    }

    if (sent > 0U)
    {
        return;
    }

    if (g_async_eof)
    {
        g_async_stop_fill = music_player_async_ring_used();
        if (g_async_play_start_us != 0U)
        {
            g_async_play_elapsed_us = system_getval_us() - g_async_play_start_us;
        }
        music_player_tx_stop();
        music_player_async_finish(0U);
        return;
    }

    g_async_underruns++;
    music_player_send_sample(g_async_last_left, g_async_last_right);
    music_player_async_pace_one_frame(music_player_output_rate());
}

uint8_t music_player_async_is_active(void)
{
    return g_async_running ? 1U : 0U;
}

uint8_t music_player_async_result(void)
{
    return g_async_result;
}

uint32_t music_player_async_underruns(void)
{
    return g_async_underruns;
}

uint32_t music_player_async_play_ms(void)
{
    if ((g_async_running != 0U) && (g_async_play_start_us != 0U))
    {
        return (system_getval_us() - g_async_play_start_us + 500U) / 1000U;
    }
    return (g_async_play_elapsed_us + 500U) / 1000U;
}

void music_player_async_stats(uint32 *sent, uint32 *nonzero, uint32 *peak, int16 *last_left, int16 *last_right, uint32 *fill)
{
    if (sent != NULL) { *sent = g_async_sent_frames; }
    if (nonzero != NULL) { *nonzero = g_async_nonzero_frames; }
    if (peak != NULL) { *peak = g_async_peak; }
    if (last_left != NULL) { *last_left = g_async_last_left; }
    if (last_right != NULL) { *last_right = g_async_last_right; }
    if (fill != NULL)
    {
        *fill = g_async_running ? music_player_async_ring_used() : g_async_stop_fill;
    }
}
#else
uint8_t music_player_play_first_in_folder_async(const char *path)
{
    return music_player_play_first_in_folder(path);
}

void music_player_io_task(void)
{
}

void music_player_core_task(void)
{
}

uint8_t music_player_async_is_active(void)
{
    return 0U;
}

uint8_t music_player_async_result(void)
{
    return 0U;
}

uint32_t music_player_async_underruns(void)
{
    return 0U;
}

uint32_t music_player_async_play_ms(void)
{
    return 0U;
}

void music_player_async_stats(uint32 *sent, uint32 *nonzero, uint32 *peak, int16 *last_left, int16 *last_right, uint32 *fill)
{
    if (sent != NULL) { *sent = 0U; }
    if (nonzero != NULL) { *nonzero = 0U; }
    if (peak != NULL) { *peak = 0U; }
    if (last_left != NULL) { *last_left = 0; }
    if (last_right != NULL) { *last_right = 0; }
    if (fill != NULL) { *fill = 0U; }
}
#endif

void music_player_stop(void)
{
    if (g_tone_active)
    {
        music_player_tone_stop();
    }
    g_play_stop_request = 1;
    g_play_paused = 0;
    g_music_player.state = MUSIC_PLAYER_STOPPED;
}

uint8_t music_player_pause(void)
{
    if (g_music_player.state != MUSIC_PLAYER_PLAYING)
    {
        return 1;
    }

    g_play_paused = 1;
    g_music_player.state = MUSIC_PLAYER_PAUSED;
    return 0;
}

uint8_t music_player_resume(void)
{
    if (g_music_player.state != MUSIC_PLAYER_PAUSED)
    {
        return 1;
    }

    g_play_paused = 0;
    g_music_player.state = MUSIC_PLAYER_PLAYING;
    return 0;
}

music_player_state_enum music_player_get_state(void)
{
    return g_music_player.state;
}

uint8_t music_player_get_progress(uint32_t *total_seconds, uint32_t *current_seconds)
{
    uint32_t bytes_per_second;

    if ((total_seconds == NULL) || (current_seconds == NULL))
    {
        return 1;
    }

    if ((g_music_player.state == MUSIC_PLAYER_STOPPED) ||
        (g_music_player.state == MUSIC_PLAYER_ERROR))
    {
        return 2;
    }

    *total_seconds = g_music_player.current_music.duration_seconds;

    bytes_per_second = g_music_player.current_music.sample_rate
                     * g_music_player.current_music.channels
                     * (g_music_player.current_music.bits_per_sample / 8U);

    if (bytes_per_second == 0U)
    {
        *current_seconds = 0;
    }
    else
    {
        *current_seconds = g_music_player.current_position / bytes_per_second;
    }

    return 0;
}

void music_player_set_volume(uint8_t volume)
{
    if (volume > 100U)
    {
        volume = 100U;
    }

    g_music_player.volume = volume;
    es8388_hpvol_set(music_player_convert_volume(volume));
    es8388_spkvol_set(music_player_convert_volume(volume));
}

uint8_t music_player_get_volume(void)
{
    return g_music_player.volume;
}

void music_player_set_mode(music_play_mode_enum mode)
{
    g_music_player.play_mode = mode;
}

music_play_mode_enum music_player_get_mode(void)
{
    return g_music_player.play_mode;
}

uint8_t music_player_play_index(uint32_t index)
{
    if ((!g_files_valid) || (index >= g_file_count))
    {
        return 1;
    }

    g_music_player.current_file_index = index;
    return music_player_play(g_file_list[index]);
}

uint8_t music_player_next(void)
{
    if ((!g_files_valid) || (g_file_count == 0U))
    {
        return 1;
    }

    g_music_player.current_file_index = (g_music_player.current_file_index + 1U) % g_file_count;
    return music_player_play(g_file_list[g_music_player.current_file_index]);
}

uint8_t music_player_previous(void)
{
    if ((!g_files_valid) || (g_file_count == 0U))
    {
        return 1;
    }

    if (g_music_player.current_file_index == 0U)
    {
        g_music_player.current_file_index = g_file_count - 1U;
    }
    else
    {
        g_music_player.current_file_index--;
    }

    return music_player_play(g_file_list[g_music_player.current_file_index]);
}

uint8_t music_player_get_current_info(music_info_struct *info)
{
    if (info == NULL)
    {
        return 1;
    }

    if ((g_music_player.state == MUSIC_PLAYER_STOPPED) ||
        (g_music_player.state == MUSIC_PLAYER_ERROR))
    {
        return 2;
    }

    *info = g_music_player.current_music;
    return 0;
}

uint8_t music_player_play_first_in_folder(const char *path)
{
    uint32_t cnt = 0;

    if (music_player_scan_folder(path, &cnt) != 0U)
    {
        return 1;
    }

    if (cnt == 0U)
    {
        return 2;
    }

    g_music_player.current_file_index = 0;
    return music_player_play(g_file_list[0]);
}

void music_player_demo(void)
{
    if (music_player_init() == 0U)
    {
        (void)music_player_play_first_in_folder("0:/MUSIC");
    }
}

uint8_t music_player_test(void)
{
    uint8_t ret = music_player_init();
    if (ret != 0U)
    {
        return ret;
    }

    return music_player_play_first_in_folder("0:/MUSIC");
}
