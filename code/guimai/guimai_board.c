#include "guimai_board.h"
#if GUIMAI_AUDIO_USE_ADC
#include "guimai_adc_audio.h"
#endif
#if GUIMAI_TRIGGER_USE_XL9555_KEY3
#include "../anjian/xl9555_app.h"
#endif
#include "../es8388/es8388_unified.h"
#if GUIMAI_MUSIC_ENABLE || GUIMAI_HORN_ENABLE
#include "../es8388/music_player.h"
#define GUIMAI_BUILD_TONE_SAMPLE_RATE MUSIC_PLAYER_TONE_SAMPLE_RATE
#else
#define GUIMAI_BUILD_TONE_SAMPLE_RATE 0U
#endif
#if GUIMAI_HORN_USE_BUZZER
#define GUIMAI_BUILD_HORN_TX "buzzer"
#else
#define GUIMAI_BUILD_HORN_TX "mcu_master"
#endif
#include "zf_driver_soft_iis.h"
#include "zf_driver_pwm.h"
#include "../ui/UI.h"
#include "IfxCpu.h"
#include "IfxDma_Dma.h"
#include "IfxGtm.h"
#include "IfxGtm_Cmu.h"
#include "IfxGtm_Psm.h"
#include "IfxGtm_Tim.h"
#include "IfxGtm_PinMap.h"
#include "IfxQspi.h"
#include "IfxScuWdt.h"

static uint32 guimai_elapsed_ms(uint32 start_ms, uint32 now_ms)
{
    return now_ms - start_ms;
}

static uint32 guimai_time_add_ms(uint32 start_ms, uint32 delta_ms)
{
    return start_ms + delta_ms;
}

static void guimai_horn_pwm_test_init(void)
{
#if GUIMAI_HORN_PWM_TEST_ALWAYS_ON
    pwm_init(ATOM3_CH0_P33_10,
             GUIMAI_HORN_PWM_TEST_FREQ_HZ,
             GUIMAI_HORN_PWM_TEST_DUTY);
    printf("[GUIMAI_PWM_TEST] P33_10 ATOM3_CH0 freq=%lu duty=%lu/10000 continuous\r\n",
           (unsigned long)GUIMAI_HORN_PWM_TEST_FREQ_HZ,
           (unsigned long)GUIMAI_HORN_PWM_TEST_DUTY);
#endif
}

static uint8 guimai_time_before_ms(uint32 now_ms, uint32 target_ms)
{
    return (((int32)(target_ms - now_ms)) > 0) ? 1U : 0U;
}

typedef enum
{
    GUIMAI_BUF_0 = 0,
    GUIMAI_BUF_1 = 1,
} guimai_buf_index_enum;

#if GUIMAI_HORN_ENABLE
#define GUIMAI_HORN_MAX_STEPS 10U

typedef struct
{
    uint32 freq_hz;
    uint32 on_ms;
    uint32 off_ms;
} guimai_horn_step_struct;
#endif

#if GUIMAI_GTM_TBCM_DECODE_DIAG
typedef struct
{
    uint8 ws_bit;
    uint8 sd_bit;
    uint8 invert_ws;
    uint8 left_ws_level;
    const char *name;
} guimai_tbcm_decode_candidate_config;

typedef struct
{
    uint8 last_ws;
    uint32 slot_pos;
    uint8 bits;
    uint32 shift;
    uint32 words;
    uint32 ws_edges;
    uint32 samples;
    uint32 left_samples;
    uint32 right_samples;
    uint32 bad_slots;
    uint32 max_slot;
    uint32 ones;
    uint32 peak;
    uint32 abs_sum;
    int16 last_sample;
} guimai_tbcm_decode_candidate_state;

typedef struct
{
    uint8 half;
    uint8 shift;
    uint8 invert;
    const char *name;
} guimai_tssm_slice_config;

typedef struct
{
    uint32 words;
    uint32 changed;
    uint32 zero;
    uint32 neg1;
    uint32 sat;
    uint32 peak;
    uint32 abs_sum;
    uint16 last_raw;
    int16 last_sample;
    int16 min_sample;
    int16 max_sample;
    uint8 has_last;
} guimai_tssm_slice_state;
#endif

static int16 s_mic_voice[2][GUIMAI_MIC_BUF_LEN];
static volatile uint16 s_voice_pos = 0;
static volatile guimai_buf_index_enum s_fill_buf = GUIMAI_BUF_0;
static volatile uint8 s_voice_frame_ready = 0;

static uint8 s_switch_origin_level = GPIO_LOW;
static uint8 s_switch_state = GPIO_LOW;
static uint8 s_switch_event = 0;

static volatile uint8 s_streaming = 0;
static volatile uint8 s_wifi_ready = 0;
static uint8 s_lora_ready = 0;
static const char *s_record_status_text = "REC OFF";
static const char *s_wifi_status_text = "WIFI OFF";
#if GUIMAI_MUSIC_ENABLE
static volatile uint8 s_record_pending_after_music = 0;
static volatile uint8 s_record_pending_external = 0;
#endif

static uint8 s_frame[GUIMAI_FRAME_SIZE] = {GUIMAI_FRAME_HEADER_0, GUIMAI_FRAME_HEADER_1, GUIMAI_FRAME_HEADER_2, GUIMAI_FRAME_HEADER_3};
#if GUIMAI_RECORD_STREAM_TX
#pragma section all "lmubss"
static uint8 s_stream_tx_queue[GUIMAI_STREAM_TX_QUEUE_DEPTH][GUIMAI_FRAME_SIZE];
static volatile uint8 s_stream_tx_head = 0U;
static volatile uint8 s_stream_tx_tail = 0U;
static volatile uint8 s_stream_tx_active = 0U;
static volatile uint8 s_stream_tx_overflow = 0U;
#pragma section all restore
#endif
#if GUIMAI_RECORD_DEFER_TX
#pragma section all "cpu1_dsram"
static int16 s_record_store[GUIMAI_RECORD_STORE_SAMPLES];
#pragma section all restore
static uint32 s_record_samples = 0;
static uint32 s_record_start_ms = 0;
static uint32 s_record_elapsed_ms = 0;
static uint8 s_record_overflow = 0;
static volatile uint8 s_record_core_req = 0;
static volatile uint8 s_record_core_running = 0;
static volatile uint8 s_record_core_stop_req = 0;
static volatile uint8 s_record_core_done = 0;
static uint8 s_record_tx_active = 0U;
static uint8 s_record_tx_sent_first = 0U;
static uint32 s_record_tx_frame_count = 0U;
static uint32 s_record_tx_frame_index = 0U;
#endif

static uint8 s_rx_byte = 0;
static uint8 s_normal_command = 0;
static uint8 s_command_buffer[GUIMAI_COMMAND_QUEUE_SIZE] = {0};
static uint8 s_command_in_progress = 0;
static uint8 s_command_count = 0;
static uint8 s_voice_cmd_queue[GUIMAI_COMMAND_QUEUE_SIZE] = {0};
static volatile uint8 s_voice_cmd_head = 0;
static volatile uint8 s_voice_cmd_tail = 0;
static uint8 s_text_rx_active = 0;
static uint8 s_text_start_match = 0;
static uint8 s_text_end_match = 0;
static char s_text_buffer[192] = {0};
static uint8 s_text_len = 0;
static uint32 s_text_rx_start_ms = 0U;
#if GUIMAI_ASR_COMMAND_WAIT_ENABLE
static volatile uint8 s_asr_command_waiting = 0U;
static volatile uint32 s_asr_command_wait_start_ms = 0U;
#endif

static uint8 s_linein_ready = 0;
static uint8 s_linein_running = 0;
static uint8 s_es8388_ready = 0;
#if GUIMAI_HORN_ENABLE
static guimai_horn_step_struct s_horn_steps[GUIMAI_HORN_MAX_STEPS];
static uint8 s_horn_step_count = 0U;
static uint8 s_horn_step_index = 0U;
static uint8 s_horn_active = 0U;
static uint8 s_horn_tone_on = 0U;
static uint8 s_horn_last_cmd = 0U;
static uint32 s_horn_phase_until_ms = 0U;
#endif
#if GUIMAI_MUSIC_ENABLE
static uint8 s_music_ready = 0;
static volatile uint8 s_music_play_req = 0;
static volatile uint8 s_music_playing = 0;
static uint8 s_music_used_codec = 0;
static uint8 s_music_pin_diag_done = 0U;
#endif
static uint8 s_linein_input_source = GUIMAI_LINEIN_SRC_D;
static uint8 s_linein_iis_format = GUIMAI_LINEIN_IIS_FORMAT;
static gpio_pin_enum s_linein_ws_pin = HORN_WS_PIN;
#if GUIMAI_GTM_DMA_CAPTURE && GUIMAI_RECORD_CLOCK_PUMP_ENABLE && ES8388_RECORD_MCU_MASTER
static uint32 s_record_clock_pump_last_us = 0U;
static uint32 s_record_clock_pump_frac_us = 0U;
#endif
#pragma section all "cpu1_dsram"
static int16 s_linein_buf[GUIMAI_LINEIN_READ_BATCH];
#pragma section all restore
#if GUIMAI_RECORD_PRE_MCLK_ENABLE
static soft_iis_info_struct s_record_pre_mclk_iis;
static uint8 s_record_pre_mclk_active = 0U;
#endif
#if GUIMAI_AUDIO_HOTPATH_LOG
static uint32 s_audio_level_last_ms = 0;
#endif
static uint32 s_audio_fs_stat_ms = 0;
static uint32 s_audio_fs_stat_samples = 0;
static uint32 s_audio_fs_est = 0;
#if GUIMAI_QSPI1_SLAVE_RX
static uint8 s_qspi1_ready = 0;
static uint32 s_qspi1_rx_words = 0;
static uint32 s_qspi1_last_error = 0;
static uint8 s_qspi1_sync_ok = 0;
static uint32 s_qspi1_bacon = 0;
#endif
#if GUIMAI_GTM_DMA_CAPTURE
static IfxDma_Dma s_gtm_dma_module;
static IfxDma_Dma_Channel s_gtm_dma_channel;
static Ifx_GTM_TIM_CH *s_gtm_dma_tim_ch = NULL_PTR;
#pragma section all "cpu1_dsram"
IFX_ALIGN(16384) static volatile uint32 s_gtm_dma_port_words[GUIMAI_GTM_DMA_RING_WORDS];
#pragma section all restore
static uint32 s_gtm_dma_dest_addr = 0U;
static uint8 s_gtm_dma_read_half = 1U;
static uint8 s_gtm_dma_ready = 0;
static uint8 s_gtm_dma_active = 0;
static uint32 s_gtm_dma_chunks = 0;
static uint32 s_gtm_dma_words = 0;
static uint32 s_gtm_dma_samples = 0;
static uint32 s_gtm_dma_lost = 0;
static uint8 s_gtm_dma_left_ws_level = GUIMAI_GTM_DMA_LEFT_WS_LEVEL;
static uint8 s_gtm_dma_i2s_delay_bits = GUIMAI_GTM_DMA_I2S_DELAY_BITS;
static uint8 s_gtm_i2s_ws = 0xFFU;
static uint8 s_gtm_i2s_slot_pos = 0U;
static uint8 s_gtm_i2s_bits = 0U;
static uint32 s_gtm_i2s_shift = 0U;
#endif
#if GUIMAI_GTM_TBCM_DIAG
static Ifx_GTM_TIM_CH *s_gtm_tbcm_ch = NULL_PTR;
static uint8 s_gtm_tbcm_ready = 0U;
static uint8 s_gtm_tbcm_active = 0U;
static uint32 s_gtm_tbcm_polls = 0U;
static uint32 s_gtm_tbcm_events = 0U;
static uint32 s_gtm_tbcm_newval = 0U;
static uint32 s_gtm_tbcm_gprofl = 0U;
static uint32 s_gtm_tbcm_ws_one = 0U;
static uint32 s_gtm_tbcm_sd_one = 0U;
static uint32 s_gtm_tbcm_last_gpr0 = 0U;
static uint32 s_gtm_tbcm_last_gpr1 = 0U;
static uint32 s_gtm_tbcm_last_cnts = 0U;
static uint32 s_gtm_tbcm_last_irq = 0U;
static uint8 s_gtm_tbcm_last_ecnt = 0U;
#if GUIMAI_GTM_ARU_FIFO_DIAG
static uint8 s_gtm_aru_fifo_ready = 0U;
static uint8 s_gtm_aru_fifo_active = 0U;
static uint32 s_gtm_aru_fifo_polls = 0U;
static const IfxGtm_Psm_F2aStream s_gtm_aru_fifo_stream[GUIMAI_GTM_ARU_FIFO_COUNT] = {
    IfxGtm_Psm_F2aStream_0,
    IfxGtm_Psm_F2aStream_1,
    IfxGtm_Psm_F2aStream_2,
};
static const IfxGtm_Psm_FifoChannel s_gtm_aru_fifo_channel[GUIMAI_GTM_ARU_FIFO_COUNT] = {
    IfxGtm_Psm_FifoChannel_0,
    IfxGtm_Psm_FifoChannel_1,
    IfxGtm_Psm_FifoChannel_2,
};
static const uint32 s_gtm_aru_fifo_src_addr[GUIMAI_GTM_ARU_FIFO_COUNT] = {
    0x015U, /* TIM2_CH4: SD TBCM diagnostic channel */
    0x014U, /* TIM2_CH3: BCK trigger channel */
    0x012U, /* TIM2_CH1: WS input channel */
};
static uint32 s_gtm_aru_fifo_fill[GUIMAI_GTM_ARU_FIFO_COUNT] = {0U};
static uint32 s_gtm_aru_fifo_max_fill[GUIMAI_GTM_ARU_FIFO_COUNT] = {0U};
static uint32 s_gtm_aru_fifo_wr[GUIMAI_GTM_ARU_FIFO_COUNT] = {0U};
static uint32 s_gtm_aru_fifo_rd[GUIMAI_GTM_ARU_FIFO_COUNT] = {0U};
static uint32 s_gtm_aru_fifo_status[GUIMAI_GTM_ARU_FIFO_COUNT] = {0U};
static uint32 s_gtm_aru_fifo_irq[GUIMAI_GTM_ARU_FIFO_COUNT] = {0U};
static uint32 s_gtm_aru_fifo_stream_state[GUIMAI_GTM_ARU_FIFO_COUNT] = {0U};
static uint32 s_gtm_aru_fifo_enable = 0U;
static uint32 s_gtm_aru_fifo_str_cfg[GUIMAI_GTM_ARU_FIFO_COUNT] = {0U};
static uint32 s_gtm_aru_fifo_aru_addr[GUIMAI_GTM_ARU_FIFO_COUNT] = {0U};
static uint32 s_gtm_afd_dump_words[GUIMAI_GTM_AFD_DUMP_WORDS] = {0U};
static uint32 s_gtm_afd_dump_count = 0U;
static uint32 s_gtm_afd_dump_fill_before = 0U;
static uint32 s_gtm_afd_dump_fill_after = 0U;
static uint32 s_gtm_afd_dump_rd_before = 0U;
static uint32 s_gtm_afd_dump_rd_after = 0U;
static uint32 s_gtm_afd_dump_wr_before = 0U;
static uint32 s_gtm_afd_dump_wr_after = 0U;
static uint32 s_gtm_afd_live_words[GUIMAI_GTM_AFD_DUMP_WORDS] = {0U};
static uint32 s_gtm_afd_live_write = 0U;
static uint32 s_gtm_afd_live_drained = 0U;
static uint32 s_gtm_afd_live_nonzero = 0U;
static uint32 s_gtm_afd_live_changed = 0U;
static uint32 s_gtm_afd_live_last_word = 0U;
static uint8 s_gtm_afd_live_has_last = 0U;
#if GUIMAI_GTM_AFD_DMA_DIAG
static IfxDma_Dma s_gtm_afd_dma_module;
static IfxDma_Dma_Channel s_gtm_afd_dma_channel;
static volatile uint32 s_gtm_afd_dma_words[GUIMAI_GTM_AFD_DMA_WORDS];
static uint32 s_gtm_afd_dma_dest_addr = 0U;
static uint8 s_gtm_afd_dma_ready = 0U;
static uint8 s_gtm_afd_dma_active = 0U;
static uint8 s_gtm_afd_dma_done = 0U;
static uint32 s_gtm_afd_dma_lost = 0U;
static uint32 s_gtm_afd_dma_irq_notify = 0U;
static uint32 s_gtm_afd_dma_src = 0U;
static uint32 s_gtm_afd_dma_chcsr = 0U;
static uint32 s_gtm_afd_dma_tcnt = 0U;
static uint32 s_gtm_afd_dma_chunks = 0U;
static uint32 s_gtm_afd_dma_words_total = 0U;
static uint32 s_gtm_afd_dma_nonzero_total = 0U;
static uint32 s_gtm_afd_dma_changed_total = 0U;
static uint32 s_gtm_afd_dma_last_word = 0U;
static uint8 s_gtm_afd_dma_has_last = 0U;
static uint32 s_gtm_afd_dma_history[GUIMAI_GTM_AFD_DMA_HISTORY_WORDS];
static uint32 s_gtm_afd_dma_history_write = 0U;
#endif
#if GUIMAI_GTM_TBCM_DECODE_DIAG
static const guimai_tbcm_decode_candidate_config s_gtm_tbcm_decode_cfg[GUIMAI_GTM_TBCM_DECODE_CANDIDATES] = {
    {1U, 4U, 0U, 0U, "ws1_sd4_l0"},
    {1U, 4U, 0U, 1U, "ws1_sd4_l1"},
    {1U, 3U, 0U, 0U, "ws1_sd3_l0"},
    {1U, 4U, 1U, 0U, "ws1i_sd4_l0"},
};
static const guimai_tssm_slice_config s_gtm_tssm_slice_cfg[GUIMAI_GTM_TSSM_SLICE_CANDIDATES] = {
    {1U, 0U, 0U, "h1_b0_15"},
    {1U, 4U, 0U, "h1_b4_19"},
    {1U, 8U, 0U, "h1_b8_23"},
    {1U, 0U, 1U, "h1_b0_15_inv"},
    {1U, 4U, 1U, "h1_b4_19_inv"},
    {1U, 8U, 1U, "h1_b8_23_inv"},
};
static guimai_tbcm_decode_candidate_state s_gtm_tbcm_decode_state[GUIMAI_GTM_TBCM_DECODE_CANDIDATES];
static guimai_tbcm_decode_candidate_state s_gtm_tbcm_decode_rle_state[GUIMAI_GTM_TBCM_DECODE_RLE_MODES][GUIMAI_GTM_TBCM_DECODE_CANDIDATES];
static guimai_tssm_slice_state s_gtm_tssm_slice_state[GUIMAI_GTM_TSSM_SLICE_CANDIDATES];
static uint32 s_gtm_tbcm_bit_ones[32];
static uint32 s_gtm_tbcm_bit_toggles[32];
static uint32 s_gtm_tbcm_bit_last = 0U;
static uint8 s_gtm_tbcm_bit_has_last = 0U;
static uint32 s_gtm_tbcm_decode_word_index = 0U;
static uint32 s_gtm_tbcm_pair_bit_ones[2][32];
static uint32 s_gtm_tbcm_pair_bit_toggles[2][32];
static uint32 s_gtm_tbcm_pair_words[2] = {0U, 0U};
static uint32 s_gtm_tbcm_pair_last[2] = {0U, 0U};
static uint8 s_gtm_tbcm_pair_has_last[2] = {0U, 0U};
static guimai_tbcm_decode_candidate_state s_gtm_tbcm_pair_state[2][GUIMAI_GTM_TBCM_DECODE_CANDIDATES];
static uint32 s_gtm_tbcm_rle_total[GUIMAI_GTM_TBCM_DECODE_RLE_MODES] = {0U};
static uint32 s_gtm_tbcm_rle_zero[GUIMAI_GTM_TBCM_DECODE_RLE_MODES] = {0U};
static uint32 s_gtm_tbcm_rle_max[GUIMAI_GTM_TBCM_DECODE_RLE_MODES] = {0U};
#endif
#endif
#endif

static void guimai_send_frame(uint8 flag);
#if GUIMAI_RECORD_STREAM_TX
static void guimai_stream_tx_reset(void);
static uint8 guimai_stream_tx_queue_frame(uint8 flag, const int16 *samples, uint32 sample_count);
static void guimai_stream_tx_send_task(void);
#endif
static void guimai_linein_stop(void);
static uint8 guimai_switch_read_level(void);
static void guimai_record_ui_status(const char *status);
static uint8 guimai_voice_begin_record(void);
static void guimai_voice_end_record(void);
static void guimai_voice_queue_command(uint8 cmd);
#if !GUIMAI_AUDIO_USE_ADC
static uint32 guimai_pin_toggle_count(gpio_pin_enum pin, uint32 loops);
#endif
static void guimai_audio_release_iis_pins(void);
#if GUIMAI_ASR_COMMAND_WAIT_ENABLE
static void guimai_asr_command_wait_begin(void);
static void guimai_asr_command_wait_clear(const char *reason);
static uint8 guimai_asr_command_wait_active(void);
#endif
#if GUIMAI_HORN_ENABLE
static uint8 guimai_horn_handle_command(uint8 cmd);
static void guimai_horn_task(uint32 now_ms);
static void guimai_horn_stop(void);
static void guimai_audio_release_record_path(const char *reason);
#endif
#if GUIMAI_QSPI1_SLAVE_RX
static void guimai_qspi1_slave_init(void);
static void guimai_qspi1_sync_to_ws_edge(void);
static uint32 guimai_qspi1_read_samples(int16 *dst, uint32 max_samples);
#endif
#if GUIMAI_GTM_DMA_CAPTURE
static void guimai_gtm_dma_init(void);
static void guimai_gtm_dma_i2s_reset(void);
static void guimai_gtm_dma_start_chunk(void);
static void guimai_gtm_dma_stop(void);
static uint32 guimai_gtm_dma_read_samples(int16 *dst, uint32 max_samples);
#endif
#if GUIMAI_GTM_TBCM_DIAG
static void guimai_gtm_tbcm_init(void);
static void guimai_gtm_tbcm_start(void);
static void guimai_gtm_tbcm_stop(void);
static uint32 guimai_gtm_tbcm_read_samples(int16 *dst, uint32 max_samples);
static void guimai_gtm_tbcm_no_event_diag(void);
#if GUIMAI_GTM_ARU_FIFO_DIAG
static void guimai_gtm_aru_fifo_init(void);
static void guimai_gtm_aru_fifo_start(void);
static void guimai_gtm_aru_fifo_stop(void);
static void guimai_gtm_aru_fifo_poll_diag(void);
#if GUIMAI_GTM_AFD_DMA_DIAG
static void guimai_gtm_afd_dma_init(void);
static void guimai_gtm_afd_dma_arm(void);
static void guimai_gtm_afd_dma_start(void);
static void guimai_gtm_afd_dma_stop(void);
static void guimai_gtm_afd_dma_poll(void);
static void guimai_gtm_afd_dma_print(void);
#endif
#if GUIMAI_GTM_TBCM_DECODE_DIAG
static void guimai_gtm_tbcm_decode_reset(void);
static void guimai_gtm_tbcm_decode_process_word(uint32 word);
static void guimai_gtm_tbcm_decode_print(void);
#endif
#endif
#endif
#if GUIMAI_RECORD_DEFER_TX
static void guimai_send_stored_audio(void);
static void guimai_send_stored_audio_task(void);
#endif

static void guimai_text_rx_reset(const char *reason)
{
    if (s_text_rx_active)
    {
        uint32 now_ms = system_getval_ms();
        uint32 elapsed = guimai_elapsed_ms(s_text_rx_start_ms, now_ms);
        printf("[XFYUN_RX] text reset reason=%s len=%u elapsed=%lu\r\n",
               (reason != NULL_PTR) ? reason : "?",
               (unsigned)s_text_len,
               (unsigned long)elapsed);
    }

    s_text_rx_active = 0;
    s_text_start_match = 0;
    s_text_end_match = 0;
    s_text_len = 0;
    s_text_buffer[0] = '\0';
}

static void guimai_text_rx_timeout_task(void)
{
    uint32 elapsed;

    if (!s_text_rx_active)
    {
        return;
    }

    elapsed = guimai_elapsed_ms(s_text_rx_start_ms, system_getval_ms());
    if (elapsed > (uint32)GUIMAI_TEXT_RX_TIMEOUT_MS)
    {
        guimai_text_rx_reset("timeout");
#if GUIMAI_ASR_COMMAND_WAIT_ENABLE
        guimai_asr_command_wait_clear("text_timeout");
#endif
    }
}

static void guimai_record_ui_status(const char *status)
{
    s_record_status_text = status;
    Gui_Refersh_Bool = ZF_TRUE;
}

static void guimai_wifi_ui_status(const char *status)
{
    s_wifi_status_text = status;
    Gui_Refersh_Bool = ZF_TRUE;
}

#if GUIMAI_ASR_COMMAND_WAIT_ENABLE
static uint32 guimai_asr_command_wait_elapsed(uint32 now_ms)
{
    return guimai_elapsed_ms(s_asr_command_wait_start_ms, now_ms);
}

static void guimai_asr_command_wait_begin(void)
{
    s_asr_command_waiting = 1U;
    s_asr_command_wait_start_ms = system_getval_ms();
    printf("[GUIMAI_ASR] wait command ms=%u\r\n",
           (unsigned)GUIMAI_ASR_COMMAND_WAIT_MS);
}

static void guimai_asr_command_wait_clear(const char *reason)
{
    if (s_asr_command_waiting)
    {
        uint32 elapsed = guimai_asr_command_wait_elapsed(system_getval_ms());
        printf("[GUIMAI_ASR] wait clear reason=%s elapsed=%lu\r\n",
               (reason != NULL_PTR) ? reason : "?",
               (unsigned long)elapsed);
    }
    s_asr_command_waiting = 0U;
}

static uint8 guimai_asr_command_wait_active(void)
{
    uint32 elapsed = 0U;

    if (!s_asr_command_waiting)
    {
        return 0U;
    }

    elapsed = guimai_asr_command_wait_elapsed(system_getval_ms());
    if (elapsed < (uint32)GUIMAI_ASR_COMMAND_WAIT_MS)
    {
        return 1U;
    }

    printf("[GUIMAI_ASR] wait timeout elapsed=%lu\r\n", (unsigned long)elapsed);
    s_asr_command_waiting = 0U;
    return 0U;
}
#endif

static void guimai_audio_release_iis_pins(void)
{
#if !GUIMAI_AUDIO_USE_ADC
#if GUIMAI_RECORD_PRE_MCLK_ENABLE
    if (s_record_pre_mclk_active)
    {
        soft_iis_stop(&s_record_pre_mclk_iis);
        s_record_pre_mclk_active = 0U;
    }
#endif
    gpio_init(HORN_BCK_PIN, GPI, GPIO_LOW, GPI_FLOATING_IN);
    gpio_init(HORN_WS_PIN, GPI, GPIO_LOW, GPI_FLOATING_IN);
    gpio_init(HORN_SD_PIN, GPI, GPIO_LOW, GPI_FLOATING_IN);
    gpio_init(ES8388_ADC_SDOUT_PIN, GPI, GPIO_LOW, GPI_FLOATING_IN);
    if (HORN_MCLK_PIN != 0xFFFFFFFF)
    {
        gpio_init(HORN_MCLK_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    }
#endif
}

#if GUIMAI_RECORD_PRE_MCLK_ENABLE
static void guimai_record_pre_mclk_start(const char *tag)
{
    uint32 mclk_toggles = 0U;

    if (HORN_MCLK_PIN == 0xFFFFFFFF)
    {
        return;
    }

    if (s_record_pre_mclk_active)
    {
        soft_iis_stop(&s_record_pre_mclk_iis);
        s_record_pre_mclk_active = 0U;
    }

    soft_iis_init_master_tx(&s_record_pre_mclk_iis,
                            (uint32)GUIMAI_AUDIO_SAMPLE_RATE,
                            SOFT_IIS_BITS_16,
                            SOFT_IIS_FORMAT_I2S,
                            HORN_BCK_PIN,
                            HORN_WS_PIN,
                            HORN_SD_PIN,
                            HORN_MCLK_PIN);
    soft_iis_start(&s_record_pre_mclk_iis);
    s_record_pre_mclk_active = 1U;

    gpio_init(HORN_BCK_PIN, GPI, GPIO_LOW, GPI_FLOATING_IN);
    gpio_init(HORN_WS_PIN, GPI, GPIO_LOW, GPI_FLOATING_IN);
    gpio_init(HORN_SD_PIN, GPI, GPIO_LOW, GPI_FLOATING_IN);
    gpio_init(ES8388_ADC_SDOUT_PIN, GPI, GPIO_LOW, GPI_FLOATING_IN);

    system_delay_ms((uint32)GUIMAI_RECORD_PRE_MCLK_MS);
    mclk_toggles = guimai_pin_toggle_count(HORN_MCLK_PIN, GUIMAI_IIS_SCAN_LOOPS);
    printf("[GUIMAI_CLOCK] pre_mclk %s ms=%u mclk=%lu lvl=%u\r\n",
           (tag != NULL_PTR) ? tag : "?",
           (unsigned)GUIMAI_RECORD_PRE_MCLK_MS,
           (unsigned long)mclk_toggles,
           (unsigned)gpio_get_level(HORN_MCLK_PIN));
}
#endif

#if GUIMAI_HORN_ENABLE
static void guimai_horn_reset_sequence(void)
{
    uint8 i;

    for (i = 0U; i < GUIMAI_HORN_MAX_STEPS; i++)
    {
        s_horn_steps[i].freq_hz = 0U;
        s_horn_steps[i].on_ms = 0U;
        s_horn_steps[i].off_ms = 0U;
    }
    s_horn_step_count = 0U;
    s_horn_step_index = 0U;
    s_horn_tone_on = 0U;
    s_horn_phase_until_ms = 0U;
}

static uint8 guimai_horn_add_step(uint32 freq_hz, uint32 on_ms, uint32 off_ms)
{
    if ((s_horn_step_count >= GUIMAI_HORN_MAX_STEPS) || (0U == on_ms))
    {
        return 0U;
    }

    s_horn_steps[s_horn_step_count].freq_hz = freq_hz;
    s_horn_steps[s_horn_step_count].on_ms = on_ms;
    s_horn_steps[s_horn_step_count].off_ms = off_ms;
    s_horn_step_count++;
    return 1U;
}

static void guimai_horn_add_repeat(uint8 count, uint32 on_ms, uint32 off_ms)
{
    uint8 i;

    for (i = 0U; i < count; i++)
    {
        (void)guimai_horn_add_step((uint32)HORN_BASE_FREQ,
                                   on_ms,
                                   ((i + 1U) < count) ? off_ms : 0U);
    }
}

static uint8 guimai_horn_prepare_sequence(uint8 cmd)
{
    uint8 i;

    guimai_horn_reset_sequence();

    switch (cmd)
    {
        case GUIMAI_CMD_HORN_1S:
            return guimai_horn_add_step((uint32)HORN_BASE_FREQ, 1000U, 0U);

        case GUIMAI_CMD_HORN_2S:
            return guimai_horn_add_step((uint32)HORN_BASE_FREQ, 2000U, 0U);

        case GUIMAI_CMD_HORN_3S:
            return guimai_horn_add_step((uint32)HORN_BASE_FREQ, 3000U, 0U);

        case GUIMAI_CMD_HORN_2_BEEP:
            guimai_horn_add_repeat(2U, (uint32)HORN_BEEP_TIME_MS, (uint32)HORN_INTERVAL_MS);
            return (s_horn_step_count != 0U) ? 1U : 0U;

        case GUIMAI_CMD_HORN_3_BEEP:
            guimai_horn_add_repeat(3U, (uint32)HORN_BEEP_TIME_MS, (uint32)HORN_INTERVAL_MS);
            return (s_horn_step_count != 0U) ? 1U : 0U;

        case GUIMAI_CMD_HORN_4_BEEP:
            guimai_horn_add_repeat(4U, (uint32)HORN_BEEP_TIME_MS, (uint32)HORN_INTERVAL_MS);
            return (s_horn_step_count != 0U) ? 1U : 0U;

        case GUIMAI_CMD_HORN_SHORT_LONG:
            (void)guimai_horn_add_step((uint32)HORN_BASE_FREQ, 1000U, (uint32)HORN_INTERVAL_MS);
            return guimai_horn_add_step((uint32)HORN_BASE_FREQ, 3000U, 0U);

        case GUIMAI_CMD_HORN_RAPID:
            guimai_horn_add_repeat((uint8)HORN_RAPID_COUNT,
                                   (uint32)HORN_RAPID_BEEP_MS,
                                   (uint32)HORN_RAPID_INTERVAL_MS);
            return (s_horn_step_count != 0U) ? 1U : 0U;

        case GUIMAI_CMD_HORN_ALARM:
            for (i = 0U; i < (uint8)(HORN_ALARM_COUNT * 2U); i++)
            {
                (void)guimai_horn_add_step((0U == (i & 1U)) ? (uint32)HORN_ALARM_FREQ1 : (uint32)HORN_ALARM_FREQ2,
                                           (uint32)HORN_ALARM_SWITCH_MS,
                                           0U);
            }
            return (s_horn_step_count != 0U) ? 1U : 0U;

        default:
            break;
    }

    return 0U;
}

#if GUIMAI_HORN_USE_BUZZER
static void guimai_horn_buzzer_init(void)
{
    pwm_init(ATOM3_CH0_P33_10, 1000U, 0U);
}

static void guimai_horn_buzzer_stop(void)
{
    pwm_set_duty(ATOM3_CH0_P33_10, 0U);
}

static void guimai_horn_buzzer_start(uint32 freq_hz)
{
    if (freq_hz == 0U)
    {
        freq_hz = (uint32)HORN_BASE_FREQ;
    }

    pwm_init(ATOM3_CH0_P33_10, freq_hz, 0U);
    pwm_set_duty(ATOM3_CH0_P33_10, GUIMAI_HORN_PWM_DUTY);
    printf("[GUIMAI_HORN_PWM] start pin=P33_10 freq=%lu duty=%lu/10000\r\n",
           (unsigned long)freq_hz,
           (unsigned long)GUIMAI_HORN_PWM_DUTY);
}

static uint8 guimai_horn_buzzer_play_blocking(uint32 freq_hz, uint32 duration_ms)
{
    if (duration_ms == 0U)
    {
        return 0U;
    }

    guimai_horn_buzzer_start(freq_hz);
    system_delay_ms(duration_ms);
    guimai_horn_buzzer_stop();
    printf("[GUIMAI_HORN_PWM] stop freq=%lu ms=%lu\r\n",
           (unsigned long)freq_hz,
           (unsigned long)duration_ms);
    return 0U;
}
#endif

static void guimai_audio_release_record_path(const char *reason)
{
#if GUIMAI_AUDIO_USE_ADC
    (void)reason;
    if (s_linein_running)
    {
        guimai_linein_stop();
    }
#else
#if GUIMAI_GTM_DMA_CAPTURE
    uint8 dma_was_ready = s_gtm_dma_ready;
    uint8 dma_was_active = s_gtm_dma_active;
#endif

    if (music_player_tone_is_active())
    {
        music_player_tone_stop();
    }

    if (s_linein_running)
    {
        guimai_linein_stop();
    }

#if GUIMAI_GTM_DMA_CAPTURE
    if ((0U != dma_was_ready) || (0U != dma_was_active) || (s_gtm_dma_tim_ch != NULL_PTR))
    {
        guimai_gtm_dma_stop();
    }
    s_gtm_dma_ready = 0U;
    s_gtm_dma_active = 0U;
    s_gtm_dma_tim_ch = NULL_PTR;
    s_gtm_dma_dest_addr = 0U;
    s_gtm_dma_read_half = 1U;
    guimai_gtm_dma_i2s_reset();
#endif

    guimai_audio_release_iis_pins();
    system_delay_ms((uint32)GUIMAI_AUDIO_RELEASE_SETTLE_MS);

    s_linein_running = 0U;
    s_linein_ready = 0U;
    s_es8388_ready = 0U;
#if GUIMAI_MUSIC_ENABLE
    s_music_ready = 0U;
    s_music_used_codec = 0U;
#endif

    printf("[GUIMAI_AUDIO] force record reinit reason=%s"
#if GUIMAI_GTM_DMA_CAPTURE
           " dma_ready=%u dma_active=%u"
#endif
           "\r\n",
           (reason != NULL_PTR) ? reason : "?"
#if GUIMAI_GTM_DMA_CAPTURE
           ,
           (unsigned)dma_was_ready,
           (unsigned)dma_was_active
#endif
           );
#endif
}

static void guimai_horn_stop(void)
{
    if (s_horn_active)
    {
        printf("[GUIMAI_HORN] stop cmd=%u step=%u/%u\r\n",
               (unsigned)s_horn_last_cmd,
               (unsigned)s_horn_step_index,
               (unsigned)s_horn_step_count);
    }

#if !GUIMAI_HORN_USE_BUZZER
    if (music_player_tone_is_active())
    {
        music_player_tone_stop();
    }
#endif
#if GUIMAI_HORN_USE_BUZZER
    guimai_horn_buzzer_stop();
#endif
    s_horn_active = 0U;
    s_horn_tone_on = 0U;
    s_horn_step_index = 0U;
    s_horn_step_count = 0U;
#if !GUIMAI_HORN_USE_BUZZER
    s_linein_ready = 0U;
    s_es8388_ready = 0U;
#endif
    guimai_record_ui_status("REC OFF");
}

static uint8 guimai_horn_start_step(uint32 now_ms)
{
    guimai_horn_step_struct *step;
    uint8 ret;

    if (s_horn_step_index >= s_horn_step_count)
    {
        guimai_horn_stop();
        return 0U;
    }

    step = &s_horn_steps[s_horn_step_index];
#if GUIMAI_HORN_USE_BUZZER
    guimai_horn_buzzer_start(step->freq_hz);
    ret = 0U;
#else
    ret = music_player_tone_start(step->freq_hz, (int16_t)HORN_AMPLITUDE);
#endif
    if (ret != 0U)
    {
        printf("[GUIMAI_HORN] tone start failed ret=%u cmd=%u step=%u\r\n",
               (unsigned)ret,
               (unsigned)s_horn_last_cmd,
               (unsigned)s_horn_step_index);
        guimai_horn_stop();
        return 0U;
    }

    s_horn_tone_on = 1U;
    s_horn_phase_until_ms = guimai_time_add_ms(now_ms, step->on_ms);
    guimai_record_ui_status("HORN   ");
    printf("[GUIMAI_HORN] step %u/%u freq=%lu on=%lu off=%lu\r\n",
           (unsigned)(s_horn_step_index + 1U),
           (unsigned)s_horn_step_count,
           (unsigned long)step->freq_hz,
           (unsigned long)step->on_ms,
           (unsigned long)step->off_ms);
#if !GUIMAI_HORN_USE_BUZZER
    music_player_tone_task();
#endif
    return 1U;
}

static uint8 guimai_horn_handle_command(uint8 cmd)
{
#if GUIMAI_HORN_BLOCKING_PLAY
    uint8 i;
    uint8 ret = 0U;
#else
    uint32 now_ms;
#endif

    if ((cmd < GUIMAI_CMD_HORN_1S) || (cmd > GUIMAI_CMD_HORN_ALARM))
    {
        return 0U;
    }

    if (guimai_voice_is_recording())
    {
        printf("[GUIMAI_HORN] reject while recording cmd=%u\r\n", (unsigned)cmd);
        return 1U;
    }

#if GUIMAI_MUSIC_ENABLE
    if (guimai_music_is_playing())
    {
        guimai_music_stop();
    }
#endif

    guimai_horn_stop();
#if !GUIMAI_HORN_USE_BUZZER
    guimai_audio_release_record_path("horn_start");
#endif
    if (!guimai_horn_prepare_sequence(cmd))
    {
        printf("[GUIMAI_HORN] unsupported cmd=%u\r\n", (unsigned)cmd);
        return 1U;
    }

#if GUIMAI_HORN_BLOCKING_PLAY
#if !GUIMAI_HORN_USE_BUZZER
    s_linein_ready = 0U;
    s_es8388_ready = 0U;
#endif
    s_horn_active = 1U;
    s_horn_tone_on = 0U;
    s_horn_last_cmd = cmd;
    guimai_record_ui_status("HORN   ");
    printf("[GUIMAI_HORN] blocking start cmd=%u steps=%u\r\n",
           (unsigned)cmd,
           (unsigned)s_horn_step_count);

    for (i = 0U; i < s_horn_step_count; i++)
    {
        guimai_horn_step_struct *step = &s_horn_steps[i];
        s_horn_step_index = i;
        printf("[GUIMAI_HORN] blocking step %u/%u freq=%lu on=%lu off=%lu\r\n",
               (unsigned)(i + 1U),
               (unsigned)s_horn_step_count,
               (unsigned long)step->freq_hz,
               (unsigned long)step->on_ms,
               (unsigned long)step->off_ms);
#if GUIMAI_HORN_USE_BUZZER
        ret = guimai_horn_buzzer_play_blocking(step->freq_hz,
                                               step->on_ms);
#else
        ret = music_player_tone_play_blocking(step->freq_hz,
                                              (int16_t)HORN_AMPLITUDE,
                                              step->on_ms);
#endif
        if (ret != 0U)
        {
            printf("[GUIMAI_HORN] blocking tone failed ret=%u cmd=%u step=%u\r\n",
                   (unsigned)ret,
                   (unsigned)cmd,
                   (unsigned)i);
            break;
        }
        if ((step->off_ms != 0U) && ((i + 1U) < s_horn_step_count))
        {
            system_delay_ms(step->off_ms);
        }
    }

    printf("[GUIMAI_HORN] blocking done cmd=%u ret=%u\r\n",
           (unsigned)cmd,
           (unsigned)ret);
    s_horn_step_index = s_horn_step_count;
    guimai_horn_stop();
    return 1U;
#else
    s_linein_ready = 0U;
    s_es8388_ready = 0U;
    s_horn_active = 1U;
    s_horn_last_cmd = cmd;
    now_ms = system_getval_ms();
    printf("[GUIMAI_HORN] start cmd=%u steps=%u\r\n",
           (unsigned)cmd,
           (unsigned)s_horn_step_count);
    (void)guimai_horn_start_step(now_ms);
    return 1U;
#endif
}

static void guimai_horn_task(uint32 now_ms)
{
    guimai_horn_step_struct *step;

    if (!s_horn_active)
    {
        return;
    }

    if (s_horn_tone_on)
    {
        if (guimai_time_before_ms(now_ms, s_horn_phase_until_ms))
        {
#if !GUIMAI_HORN_USE_BUZZER
            music_player_tone_task();
#endif
            return;
        }

        step = &s_horn_steps[s_horn_step_index];
        if ((step->off_ms == 0U) && ((s_horn_step_index + 1U) < s_horn_step_count))
        {
            s_horn_step_index++;
            (void)guimai_horn_start_step(now_ms);
            return;
        }

#if GUIMAI_HORN_USE_BUZZER
        guimai_horn_buzzer_stop();
#else
        music_player_tone_stop();
#endif
        s_horn_tone_on = 0U;
        if (step->off_ms != 0U)
        {
            s_horn_phase_until_ms = guimai_time_add_ms(now_ms, step->off_ms);
            return;
        }
    }
    else if (guimai_time_before_ms(now_ms, s_horn_phase_until_ms))
    {
        return;
    }

    s_horn_step_index++;
    if (s_horn_step_index >= s_horn_step_count)
    {
        printf("[GUIMAI_HORN] done cmd=%u\r\n", (unsigned)s_horn_last_cmd);
        guimai_horn_stop();
        return;
    }

    (void)guimai_horn_start_step(now_ms);
}

#endif

uint8 guimai_voice_handle_horn_command(uint8 cmd)
{
#if GUIMAI_HORN_ENABLE
    return guimai_horn_handle_command(cmd);
#else
    (void)cmd;
    return 0U;
#endif
}

#if GUIMAI_MUSIC_ENABLE
static uint8 guimai_music_init_once(void)
{
    uint8 ret = 0U;

    if (s_music_ready)
    {
        return 1U;
    }

    ret = music_player_init();
    if (0U != ret)
    {
        printf("[GUIMAI_MUSIC] init failed ret=%u\r\n", ret);
        return 0U;
    }

    music_player_set_volume((uint8)GUIMAI_MUSIC_VOLUME);
    s_music_ready = 1U;
    s_es8388_ready = 1U;
    printf("[GUIMAI_MUSIC] init ok folder=%s volume=%u\r\n",
           GUIMAI_MUSIC_FOLDER,
           (unsigned)GUIMAI_MUSIC_VOLUME);
    return 1U;
}

static uint8 guimai_music_play_first_now(void)
{
    uint8 ret = 0U;
#if GUIMAI_MUSIC_TEST_TONE_ON_PLAY
    uint8 tone_ret = 0U;
#endif

    if (guimai_voice_is_recording())
    {
        printf("[GUIMAI_MUSIC] blocked: recording\r\n");
        return 0U;
    }

#if GUIMAI_MUSIC_TEST_TONE_ON_PLAY
    s_music_playing = 1U;
    printf("[GUIMAI_MUSIC] output scan tone first\r\n");
    tone_ret = music_player_play_output_scan_tone();
    s_music_playing = 0U;
    if (0U != tone_ret)
    {
        printf("[GUIMAI_MUSIC] test tone failed ret=%u\r\n", tone_ret);
        return 0U;
    }
#endif

    if (!guimai_music_init_once())
    {
        return 0U;
    }

    s_music_playing = 1U;
    s_music_used_codec = 1U;
    s_music_pin_diag_done = 0U;
    printf("[GUIMAI_MUSIC] play first in %s\r\n", GUIMAI_MUSIC_FOLDER);
#if MUSIC_PLAYER_ASYNC_ENABLE
    ret = music_player_play_first_in_folder_async(GUIMAI_MUSIC_FOLDER);
    if (0U != ret)
    {
        s_music_playing = 0U;
        s_linein_ready = 0U;
        s_es8388_ready = 0U;
        printf("[GUIMAI_MUSIC] play failed ret=%u\r\n", ret);
        return 0U;
    }
    return 1U;
#else
    ret = music_player_play_first_in_folder(GUIMAI_MUSIC_FOLDER);
    s_music_playing = 0U;
    s_linein_ready = 0U;
    s_es8388_ready = 0U;

    if (0U != ret)
    {
        printf("[GUIMAI_MUSIC] play failed ret=%u\r\n", ret);
        return 0U;
    }

    printf("[GUIMAI_MUSIC] play done\r\n");
    return 1U;
#endif
}

static void guimai_music_handle_task(void)
{
#if MUSIC_PLAYER_ASYNC_ENABLE
    if (s_music_playing)
    {
        music_player_io_task();
#if MUSIC_PLAYER_ASYNC_ENABLE
        if (!s_music_pin_diag_done)
        {
            uint32 sent = 0U;
            uint32 nonzero = 0U;
            uint32 peak = 0U;
            uint32 fill = 0U;
            int16 last_l = 0;
            int16 last_r = 0;

            music_player_async_stats(&sent, &nonzero, &peak, &last_l, &last_r, &fill);
            if (((sent >= 8192U) && (nonzero >= 4096U) && (peak >= 1000U)) ||
                (sent >= (MUSIC_PLAYER_OUTPUT_SAMPLE_RATE * 2U)))
            {
                uint32 bck_tog = guimai_pin_toggle_count(HORN_BCK_PIN, GUIMAI_IIS_SCAN_LOOPS);
                uint32 ws_tog = guimai_pin_toggle_count(HORN_WS_PIN, GUIMAI_IIS_SCAN_LOOPS);
                uint32 sd_tog = guimai_pin_toggle_count(HORN_SD_PIN, GUIMAI_IIS_SCAN_LOOPS);
                uint32 mclk_tog = guimai_pin_toggle_count(HORN_MCLK_PIN, GUIMAI_IIS_SCAN_LOOPS);
                s_music_pin_diag_done = 1U;
                printf("[GUIMAI_MUSIC_PIN] bck=%lu ws=%lu sd=%lu mclk=%lu lvl=%u/%u/%u/%u sent=%lu nonzero=%lu peak=%lu fill=%lu\r\n",
                       (unsigned long)bck_tog,
                       (unsigned long)ws_tog,
                       (unsigned long)sd_tog,
                       (unsigned long)mclk_tog,
                       (unsigned)gpio_get_level(HORN_BCK_PIN),
                       (unsigned)gpio_get_level(HORN_WS_PIN),
                       (unsigned)gpio_get_level(HORN_SD_PIN),
                       (unsigned)gpio_get_level(HORN_MCLK_PIN),
                       (unsigned long)sent,
                       (unsigned long)nonzero,
                       (unsigned long)peak,
                       (unsigned long)fill);
            }
        }
#endif
        if (!music_player_async_is_active())
        {
            uint8 ret = music_player_async_result();
            uint32 sent = 0U;
            uint32 nonzero = 0U;
            uint32 peak = 0U;
            uint32 fill = 0U;
            int16 last_l = 0;
            int16 last_r = 0;

            music_player_async_stats(&sent, &nonzero, &peak, &last_l, &last_r, &fill);
            s_music_playing = 0U;
            s_linein_ready = 0U;
            s_es8388_ready = 0U;
            if (0U != ret)
            {
                printf("[GUIMAI_MUSIC] play failed ret=%u underruns=%lu sent=%lu nonzero=%lu peak=%lu last=%d/%d fill=%lu\r\n",
                       ret,
                       (unsigned long)music_player_async_underruns(),
                       (unsigned long)sent,
                       (unsigned long)nonzero,
                       (unsigned long)peak,
                       (int)last_l,
                       (int)last_r,
                       (unsigned long)fill);
            }
            else
            {
                printf("[GUIMAI_MUSIC] play done underruns=%lu sent=%lu nonzero=%lu peak=%lu last=%d/%d fill=%lu\r\n",
                       (unsigned long)music_player_async_underruns(),
                       (unsigned long)sent,
                       (unsigned long)nonzero,
                       (unsigned long)peak,
                       (int)last_l,
                       (int)last_r,
                       (unsigned long)fill);
            }
        }
    }
#endif

    if (!s_music_play_req)
    {
        return;
    }

    s_music_play_req = 0U;
    (void)guimai_music_play_first_now();
}
#endif

static uint8 guimai_try_consume_text_byte(uint8 value)
{
    static const uint8 start_marker[5] = {'[', 'T', 'X', 'T', ']'};
    static const uint8 end_marker[5] = {'[', 'E', 'N', 'D', ']'};
    uint8 i = 0;

    if (s_text_rx_active)
    {
        guimai_text_rx_timeout_task();
    }

    if (!s_text_rx_active)
    {
        if (value == start_marker[s_text_start_match])
        {
            s_text_start_match++;
            if (s_text_start_match >= (uint8)sizeof(start_marker))
            {
                s_text_rx_active = 1;
                s_text_start_match = 0;
                s_text_end_match = 0;
                s_text_len = 0;
                s_text_rx_start_ms = system_getval_ms();
                s_text_buffer[0] = '\0';
                printf("[XFYUN_RX] start marker detected.\r\n");
            }
            return 1;
        }

        s_text_start_match = (value == start_marker[0]) ? 1 : 0;
        return 0;
    }

    if (value == end_marker[s_text_end_match])
    {
        s_text_end_match++;
        if (s_text_end_match >= (uint8)sizeof(end_marker))
        {
            s_text_buffer[s_text_len] = '\0';
            printf("[XFYUN_RX] end marker detected, len=%u.\r\n", s_text_len);
            if (s_text_len > 0)
            {
                printf("[XFYUN] %s\r\n", s_text_buffer);
            }
            else
            {
                printf("[XFYUN] <empty>\r\n");
            }
            guimai_text_rx_reset("end");
#if GUIMAI_ASR_COMMAND_WAIT_ENABLE
            guimai_asr_command_wait_clear("text");
#endif
        }
        return 1;
    }

    if (s_text_end_match > 0)
    {
        for (i = 0; i < s_text_end_match; i++)
        {
            if (s_text_len < (uint8)(sizeof(s_text_buffer) - 1))
            {
                s_text_buffer[s_text_len++] = (char)end_marker[i];
            }
        }
        s_text_end_match = 0;
    }

    if (value == end_marker[0])
    {
        s_text_end_match = 1;
        return 1;
    }

    if (s_text_len < (uint8)(sizeof(s_text_buffer) - 1))
    {
        s_text_buffer[s_text_len++] = (char)value;
    }
    return 1;
}

static void guimai_voice_queue_command(uint8 cmd)
{
    uint8 next_head = (uint8)((s_voice_cmd_head + 1U) % GUIMAI_COMMAND_QUEUE_SIZE);

    if ((0U == cmd) || (next_head == s_voice_cmd_tail))
    {
        return;
    }

    s_voice_cmd_queue[s_voice_cmd_head] = cmd;
    s_voice_cmd_head = next_head;
}

static void guimai_audio_buffer_reset(void)
{
    memset((void *)s_mic_voice, 0, sizeof(s_mic_voice));
    s_voice_pos = 0;
    s_fill_buf = GUIMAI_BUF_0;
    s_voice_frame_ready = 0;
#if GUIMAI_RECORD_STREAM_TX
    guimai_stream_tx_reset();
#endif
#if GUIMAI_AUDIO_HOTPATH_LOG
    s_audio_level_last_ms = 0;
#endif
    s_audio_fs_stat_ms = 0;
    s_audio_fs_stat_samples = 0;
    s_audio_fs_est = 0;
#if GUIMAI_RECORD_DEFER_TX
    s_record_samples = 0;
    s_record_start_ms = 0;
    s_record_elapsed_ms = 0;
    s_record_overflow = 0;
    s_record_core_req = 0;
    s_record_core_running = 0;
    s_record_core_stop_req = 0;
    s_record_core_done = 0;
#endif
#if GUIMAI_GTM_DMA_CAPTURE
    s_gtm_dma_chunks = 0;
    s_gtm_dma_words = 0;
    s_gtm_dma_samples = 0;
    s_gtm_dma_lost = 0;
    s_gtm_dma_read_half = 1U;
    guimai_gtm_dma_i2s_reset();
#endif
#if GUIMAI_GTM_TBCM_DIAG
    s_gtm_tbcm_polls = 0U;
    s_gtm_tbcm_events = 0U;
    s_gtm_tbcm_newval = 0U;
    s_gtm_tbcm_gprofl = 0U;
    s_gtm_tbcm_ws_one = 0U;
    s_gtm_tbcm_sd_one = 0U;
    s_gtm_tbcm_last_gpr0 = 0U;
    s_gtm_tbcm_last_gpr1 = 0U;
    s_gtm_tbcm_last_cnts = 0U;
    s_gtm_tbcm_last_irq = 0U;
    s_gtm_tbcm_last_ecnt = 0U;
#endif
}

#if !GUIMAI_AUDIO_USE_ADC
static const char *guimai_linein_src_name(uint8 src)
{
    switch (src)
    {
        case GUIMAI_LINEIN_SRC_A: return "SRC_A(0x0A=0x00)";
        case GUIMAI_LINEIN_SRC_B: return "SRC_B(0x0A=0x50)";
        case GUIMAI_LINEIN_SRC_C: return "SRC_C(0x0A=0xA0)";
        case GUIMAI_LINEIN_SRC_D: return "SRC_D(0x0A=0xF0)";
        default: return "SRC_UNKNOWN";
    }
}

static const char *guimai_linein_fmt_name(uint8 fmt)
{
    switch (fmt)
    {
        case GUIMAI_IIS_FMT_I2S: return "I2S";
        case GUIMAI_IIS_FMT_LEFT_J: return "LEFT_J";
        case GUIMAI_IIS_FMT_RIGHT_J: return "RIGHT_J";
        default: return "FMT_UNKNOWN";
    }
}

typedef struct
{
    uint32 samples;
    uint32 nonzero;
    uint32 clipped;
    uint32 zc;
    uint32 abs_sum;
    int16 min_sample;
    int16 max_sample;
    int16 last_sample;
    uint8 has_last;
} guimai_audio_probe_stats;

static void guimai_audio_probe_stats_reset(guimai_audio_probe_stats *stats)
{
    stats->samples = 0U;
    stats->nonzero = 0U;
    stats->clipped = 0U;
    stats->zc = 0U;
    stats->abs_sum = 0U;
    stats->min_sample = 32767;
    stats->max_sample = -32768;
    stats->last_sample = 0;
    stats->has_last = 0U;
}

static void guimai_audio_probe_stats_add(guimai_audio_probe_stats *stats, const int16 *buf, uint32 samples)
{
    uint32 i = 0U;

    for (i = 0U; i < samples; i++)
    {
        int16 v = buf[i];
        int32 av = (v >= 0) ? (int32)v : -(int32)v;

        if (v != 0)
        {
            stats->nonzero++;
        }
        if (av >= GUIMAI_LINEIN_SRC_SCAN_CLIP_LEVEL)
        {
            stats->clipped++;
        }
        if (v < stats->min_sample)
        {
            stats->min_sample = v;
        }
        if (v > stats->max_sample)
        {
            stats->max_sample = v;
        }
        if (stats->has_last)
        {
            if (((stats->last_sample < 0) && (v >= 0)) || ((stats->last_sample >= 0) && (v < 0)))
            {
                stats->zc++;
            }
        }
        stats->last_sample = v;
        stats->has_last = 1U;
        stats->abs_sum += (uint32)av;
        stats->samples++;
    }
}

static uint16 guimai_bitrev16(uint16 value)
{
    uint8 i = 0U;
    uint16 out = 0U;

    for (i = 0U; i < 16U; i++)
    {
        out = (uint16)((out << 1U) | ((value >> i) & 1U));
    }
    return out;
}

static int16 guimai_transform_sample(int16 sample, uint8 mode)
{
    uint16 u = (uint16)sample;

    switch (mode)
    {
        case 0U:
            return sample;
        case 1U:
            return (int16)((uint16)((u << 8U) | (u >> 8U)));
        case 2U:
            return (int16)((uint16)(~u));
        case 3U:
            return (int16)guimai_bitrev16(u);
        case 4U:
            return (int16)(sample / 2);
        case 5U:
            return (int16)(sample / 4);
        default:
            return sample;
    }
}

static const char *guimai_transform_name(uint8 mode)
{
    switch (mode)
    {
        case 0U: return "orig";
        case 1U: return "byteswap";
        case 2U: return "invert";
        case 3U: return "bitrev";
        case 4U: return "shr1";
        case 5U: return "shr2";
        default: return "unknown";
    }
}

#if GUIMAI_RECORD_DEFER_TX && GUIMAI_RECORD_TRANSFORM_DIAG
static void guimai_record_transform_diag(void)
{
    uint8 mode = 0U;

    if (s_record_samples == 0U)
    {
        return;
    }

    for (mode = 0U; mode < 6U; mode++)
    {
        uint32 i = 0U;
        uint32 clipped = 0U;
        uint32 zc = 0U;
        uint32 nonzero = 0U;
        uint32 avg_abs = 0U;
        uint32 score = 0U;
        unsigned long long sum_sq = 0ULL;
        unsigned long long abs_sum = 0ULL;
        int16 min_sample = 32767;
        int16 max_sample = -32768;
        int16 last_sample = 0;
        uint8 has_last = 0U;

        for (i = 0U; i < s_record_samples; i++)
        {
            int16 v = guimai_transform_sample(s_record_store[i], mode);
            int32 av = (v >= 0) ? (int32)v : -(int32)v;

            if (v != 0)
            {
                nonzero++;
            }
            if (av >= GUIMAI_LINEIN_SRC_SCAN_CLIP_LEVEL)
            {
                clipped++;
            }
            if (v < min_sample)
            {
                min_sample = v;
            }
            if (v > max_sample)
            {
                max_sample = v;
            }
            if (has_last)
            {
                if (((last_sample < 0) && (v >= 0)) || ((last_sample >= 0) && (v < 0)))
                {
                    zc++;
                }
            }
            last_sample = v;
            has_last = 1U;
            abs_sum += (unsigned long long)av;
            sum_sq += (unsigned long long)((int32)v * (int32)v);
        }

        avg_abs = (uint32)(abs_sum / s_record_samples);
        score = (clipped * 32U) + (avg_abs / 128U) + (zc / 8U);
        printf("[GUIMAI_XFORM] mode=%s n=%lu avg_abs=%lu rms2=%lu zc=%lu clip=%lu nonzero=%lu min=%d max=%d score=%lu\r\n",
               guimai_transform_name(mode),
               (unsigned long)s_record_samples,
               (unsigned long)avg_abs,
               (unsigned long)(sum_sq / s_record_samples),
               (unsigned long)zc,
               (unsigned long)clipped,
               (unsigned long)nonzero,
               min_sample,
               max_sample,
               (unsigned long)score);
    }
}
#endif

static uint32 guimai_sdout_toggle_count(uint32 loops)
{
    uint32 i = 0;
    uint32 toggles = 0;
    uint8 last = gpio_get_level(ES8388_ADC_SDOUT_PIN);

    for (i = 0; i < loops; i++)
    {
        uint8 cur = gpio_get_level(ES8388_ADC_SDOUT_PIN);
        if (cur != last)
        {
            toggles++;
            last = cur;
        }
    }
    return toggles;
}

#if GUIMAI_QSPI1_SLAVE_RX
static int16 guimai_qspi1_word_to_sample(uint32 word)
{
    uint16 sample = (uint16)((word >> GUIMAI_QSPI1_SAMPLE_SHIFT) & 0xFFFFU);
#if GUIMAI_QSPI1_SWAP16
    sample = (uint16)((sample << 8) | (sample >> 8));
#endif
    return (int16)sample;
}

static void guimai_qspi1_flush_rx(void)
{
    uint32 guard = 0;

    while ((IfxQspi_getReceiveFifoLevel(&MODULE_QSPI1) > 0U) && (guard < 32U))
    {
        (void)IfxQspi_readReceiveFifo(&MODULE_QSPI1);
        guard++;
    }
    IfxQspi_clearAllEventFlags(&MODULE_QSPI1);
}

static uint8 guimai_qspi1_wait_ws_edge(uint32 timeout_ms, uint8 *edge_level)
{
    uint8 last = gpio_get_level(HORN_WS_PIN);
    uint32 start_ms = system_getval_ms();

    while (guimai_elapsed_ms(start_ms, system_getval_ms()) < timeout_ms)
    {
        uint8 cur = gpio_get_level(HORN_WS_PIN);
        if (cur != last)
        {
            if (edge_level != NULL_PTR)
            {
                *edge_level = cur;
            }
            return 1U;
        }
    }

    if (edge_level != NULL_PTR)
    {
        *edge_level = last;
    }
    return 0U;
}

static void guimai_qspi1_sync_to_ws_edge(void)
{
#if GUIMAI_QSPI1_SYNC_ON_WS
    Ifx_QSPI *qspi = &MODULE_QSPI1;
    uint8 edge_level = 0U;
    uint8 ok = 0U;
    uint32 fifo_before = IfxQspi_getReceiveFifoLevel(qspi);

    ok = guimai_qspi1_wait_ws_edge(GUIMAI_QSPI1_WS_SYNC_TIMEOUT_MS, &edge_level);
    qspi->GLOBALCON.B.RESETS = IfxQspi_Reset_stateMachineAndFifo;
    IfxQspi_writeBasicConfigurationBeginStream(qspi, s_qspi1_bacon);
    IfxQspi_run(qspi);
    guimai_qspi1_flush_rx();

    s_qspi1_sync_ok = ok;
    printf("[GUIMAI_QSPI1] ws sync %s level=%u fifo_before=%lu status=0x%08lX\r\n",
           ok ? "edge" : "timeout",
           (unsigned)edge_level,
           (unsigned long)fifo_before,
           (unsigned long)qspi->STATUS.U);
#else
    s_qspi1_sync_ok = 0U;
#endif
}

static void guimai_qspi1_slave_init(void)
{
    Ifx_QSPI *qspi = &MODULE_QSPI1;
    uint16 password = 0;
    SpiIf_ChConfig ch_config;
    uint32 bacon = 0;
    uint32 econ = 0;

    password = IfxScuWdt_getCpuWatchdogPassword();
    IfxScuWdt_clearCpuEndinit(password);
    IfxQspi_setEnableModuleRequest(qspi);
    IfxScuWdt_setCpuEndinit(password);

    qspi->GLOBALCON.U = 0U;
    qspi->GLOBALCON.B.TQ = IfxQspi_calculateTimeQuantumLength(qspi, GUIMAI_QSPI1_MAX_BAUD);
    qspi->GLOBALCON.B.EXPECT = IfxQspi_ExpectTimeout_2097152;
    qspi->GLOBALCON.B.SRF = 0U;
    qspi->GLOBALCON.B.MS = IfxQspi_Mode_master;
    qspi->GLOBALCON.B.CLKSEL = 1U;
    qspi->GLOBALCON.B.RESETS = IfxQspi_Reset_stateMachineAndFifo;

    qspi->GLOBALCON1.U = 0U;
    qspi->GLOBALCON1.B.RXFIFOINT = IfxQspi_RxFifoInt_1;
    qspi->GLOBALCON1.B.TXFIFOINT = IfxQspi_TxFifoInt_1;
    qspi->GLOBALCON1.B.RXFM = IfxQspi_FifoMode_combinedMove;
    qspi->GLOBALCON1.B.TXFM = IfxQspi_FifoMode_combinedMove;

    SpiIf_initChannelConfig(&ch_config, NULL_PTR);
    ch_config.baudrate = GUIMAI_QSPI1_MAX_BAUD;
    ch_config.mode.clockPolarity = SpiIf_ClockPolarity_idleLow;
#if GUIMAI_QSPI1_SHIFT_TRAILING
    ch_config.mode.shiftClock = SpiIf_ShiftClock_shiftTransmitDataOnTrailingEdge;
#else
    ch_config.mode.shiftClock = SpiIf_ShiftClock_shiftTransmitDataOnLeadingEdge;
#endif
    ch_config.mode.dataHeading = SpiIf_DataHeading_msbFirst;
    ch_config.mode.dataWidth = GUIMAI_QSPI1_WORD_BITS;
    ch_config.mode.parityCheck = 0U;
    ch_config.mode.parityMode = Ifx_ParityMode_even;

    econ = IfxQspi_calculateExtendedConfigurationValue(qspi, 0U, &ch_config);
    bacon = IfxQspi_calculateBasicConfigurationValue(qspi, IfxQspi_ChannelId_0, &ch_config.mode, GUIMAI_QSPI1_MAX_BAUD);
    s_qspi1_bacon = bacon;
    qspi->ECON[0].U = econ;
    IfxQspi_writeBasicConfigurationBeginStream(qspi, bacon);

    IfxQspi_initSclkInPinWithPadLevel(&IfxQspi1_SCLKB_P11_6_IN, IfxPort_InputMode_noPullDevice, IfxPort_PadDriver_cmosAutomotiveSpeed3);
    IfxQspi_initMtsrInPinWithPadLevel(&IfxQspi1_MTSRB_P11_9_IN, IfxPort_InputMode_noPullDevice, IfxPort_PadDriver_cmosAutomotiveSpeed3);
#if GUIMAI_QSPI1_USE_SLSI_P11_10
    IfxQspi_initSlsiWithPadLevel(&IfxQspi1_SLSIA_P11_10_IN, IfxPort_InputMode_pullUp, IfxPort_PadDriver_cmosAutomotiveSpeed3);
#endif

    qspi->GLOBALCON.B.MS = IfxQspi_Mode_slave;
    IfxQspi_run(qspi);
    guimai_qspi1_flush_rx();

    s_qspi1_ready = 1U;
    s_qspi1_rx_words = 0U;
    s_qspi1_last_error = 0U;
    s_qspi1_sync_ok = 0U;
    printf("[GUIMAI_QSPI1] slave rx init word=%u shift=%u edge=%s slsis=%u PISEL=0x%08lX GLOBALCON=0x%08lX ECON0=0x%08lX BACON=0x%08lX STATUS=0x%08lX\r\n",
           (unsigned)GUIMAI_QSPI1_WORD_BITS,
           (unsigned)GUIMAI_QSPI1_SAMPLE_SHIFT,
#if GUIMAI_QSPI1_SHIFT_TRAILING
           "trailing",
#else
           "leading",
#endif
           (unsigned)qspi->PISEL.B.SLSIS,
           (unsigned long)qspi->PISEL.U,
           (unsigned long)qspi->GLOBALCON.U,
           (unsigned long)qspi->ECON[0].U,
           (unsigned long)qspi->BACON.U,
           (unsigned long)qspi->STATUS.U);
}

static uint32 guimai_qspi1_read_samples(int16 *dst, uint32 max_samples)
{
    uint32 count = 0;
    uint16 err = 0;

    if (!s_qspi1_ready)
    {
        return 0U;
    }

    err = IfxQspi_getErrorFlags(&MODULE_QSPI1);
    if (err != 0U)
    {
        s_qspi1_last_error = err;
        IfxQspi_clearAllEventFlags(&MODULE_QSPI1);
    }

    while ((count < max_samples) && (IfxQspi_getReceiveFifoLevel(&MODULE_QSPI1) > 0U))
    {
        uint32 word = IfxQspi_readReceiveFifo(&MODULE_QSPI1);
        dst[count++] = guimai_qspi1_word_to_sample(word);
    }

    s_qspi1_rx_words += count;
    return count;
}
#endif

#if GUIMAI_GTM_DMA_CAPTURE
static void guimai_gtm_dma_i2s_reset(void)
{
    s_gtm_i2s_ws = 0xFFU;
    s_gtm_i2s_slot_pos = 0U;
    s_gtm_i2s_bits = 0U;
    s_gtm_i2s_shift = 0U;
}

static void guimai_gtm_dma_init(void)
{
    IfxDma_Dma_Config dma_config;
    IfxDma_Dma_ChannelConfig ch_config;
    Ifx_GTM_TIM_CH *tim_ch = NULL_PTR;
    volatile Ifx_SRC_SRCR *tim_src = NULL_PTR;
    uint16 password = 0;

    if (s_gtm_dma_ready)
    {
        return;
    }

    password = IfxScuWdt_getCpuWatchdogPassword();
    IfxScuWdt_clearCpuEndinit(password);
    IfxGtm_enable(&MODULE_GTM);
    IfxScuWdt_setCpuEndinit(password);
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0 | IFXGTM_CMU_CLKEN_FXCLK);

    IfxGtm_PinMap_setTimTin(&IfxGtm_TIM2_3_P11_6_IN,
#if ES8388_RECORD_MCU_MASTER
                            IfxPort_InputMode_undefined);
#else
                            IfxPort_InputMode_noPullDevice);
#endif
    tim_ch = IfxGtm_Tim_getChannel(&MODULE_GTM.TIM[IfxGtm_Tim_2], IfxGtm_Tim_Ch_3);
    s_gtm_dma_tim_ch = tim_ch;

    tim_ch->CTRL.U = 0U;
    tim_ch->IRQ.EN.U = 0U;
    tim_ch->IRQ.NOTIFY.U = 0x3FU;
    tim_ch->CTRL.B.TIM_MODE = IfxGtm_Tim_Mode_inputEvent;
    tim_ch->CTRL.B.CLK_SEL = IfxGtm_Cmu_Clk_0;
    tim_ch->CTRL.B.CNTS_SEL = GUIMAI_GTM_TIM_TSSM_USE_TIM_IN ? 1U : 0U;
    tim_ch->CTRL.B.GPR0_SEL = IfxGtm_Tim_GprSel_cnts;
    tim_ch->CTRL.B.GPR1_SEL = IfxGtm_Tim_GprSel_cnts;
    tim_ch->CTRL.B.DSL = GUIMAI_GTM_DMA_SAMPLE_LEADING_EDGE ? 1U : 0U;
    tim_ch->CTRL.B.ISL = 0U;
    __ldmst_c((void *)&MODULE_GTM.TIM[IfxGtm_Tim_2].IN_SRC.U,
              (3U << (IFX_GTM_TIM_IN_SRC_MODE_0_OFF + (3U * (IFX_GTM_TIM_IN_SRC_MODE_1_OFF - IFX_GTM_TIM_IN_SRC_MODE_0_OFF)))) |
              (3U << (IFX_GTM_TIM_IN_SRC_VAL_0_OFF + (3U * (IFX_GTM_TIM_IN_SRC_VAL_1_OFF - IFX_GTM_TIM_IN_SRC_VAL_0_OFF)))),
              (1U << (IFX_GTM_TIM_IN_SRC_MODE_0_OFF + (3U * (IFX_GTM_TIM_IN_SRC_MODE_1_OFF - IFX_GTM_TIM_IN_SRC_MODE_0_OFF)))) |
              (1U << (IFX_GTM_TIM_IN_SRC_VAL_0_OFF + (3U * (IFX_GTM_TIM_IN_SRC_VAL_1_OFF - IFX_GTM_TIM_IN_SRC_VAL_0_OFF)))));
    IfxGtm_Tim_Ch_setNotificationMode(tim_ch, IfxGtm_IrqMode_pulse);
    IfxGtm_Tim_Ch_setNotification(tim_ch, IfxGtm_Tim_IrqType_newVal);

    IfxDma_Dma_initModuleConfig(&dma_config, &MODULE_DMA);
    IfxDma_Dma_initModule(&s_gtm_dma_module, &dma_config);
    IfxDma_Dma_initChannelConfig(&ch_config, &s_gtm_dma_module);
    s_gtm_dma_dest_addr = IFXCPU_GLB_ADDR_DSPR(IfxCpu_getCoreId(), (uint32)&s_gtm_dma_port_words[0]);
    ch_config.channelId = GUIMAI_GTM_DMA_CHANNEL;
    ch_config.sourceAddress = (uint32)&MODULE_P11.IN.U;
    ch_config.sourceCircularBufferEnabled = TRUE;
    ch_config.sourceAddressCircularRange = IfxDma_ChannelIncrementCircular_4;
    ch_config.destinationAddress = s_gtm_dma_dest_addr;
    ch_config.destinationAddressIncrementStep = IfxDma_ChannelIncrementStep_1;
    ch_config.destinationCircularBufferEnabled = TRUE;
    ch_config.destinationAddressCircularRange = IfxDma_ChannelIncrementCircular_16384;
    ch_config.moveSize = IfxDma_ChannelMoveSize_32bit;
    ch_config.transferCount = (uint16)GUIMAI_GTM_DMA_RING_WORDS;
    ch_config.requestMode = IfxDma_ChannelRequestMode_oneTransferPerRequest;
    ch_config.operationMode = IfxDma_ChannelOperationMode_continuous;
    ch_config.blockMode = IfxDma_ChannelMove_1;
    ch_config.busPriority = IfxDma_ChannelBusPriority_high;
    ch_config.hardwareRequestEnabled = TRUE;
    ch_config.channelInterruptEnabled = TRUE;
    ch_config.channelInterruptPriority = 0;
    ch_config.transactionRequestLostInterruptEnabled = TRUE;
    IfxDma_Dma_initChannel(&s_gtm_dma_channel, &ch_config);

    tim_src = IfxGtm_Tim_Ch_getSrcPointer(&MODULE_GTM, IfxGtm_Tim_2, IfxGtm_Tim_Ch_3);
    IfxSrc_init(tim_src, IfxSrc_Tos_dma, (Ifx_Priority)GUIMAI_GTM_DMA_CHANNEL);
    IfxSrc_enable(tim_src);

    s_gtm_dma_ready = 1U;
    printf("[GUIMAI_GTM_DMA] init ch=%u tim=TIM2_3 edge=%s irq=pulse chunk=%u ring=%u ring_bytes=%u delay=%u left_ws=%u mode=ring dst=0x%08lX SRC=0x%08lX CHCFGR=0x%08lX ADICR=0x%08lX\r\n",
           (unsigned)GUIMAI_GTM_DMA_CHANNEL,
           GUIMAI_GTM_DMA_SAMPLE_LEADING_EDGE ? "leading" : "trailing",
           (unsigned)GUIMAI_GTM_DMA_CHUNK_WORDS,
           (unsigned)GUIMAI_GTM_DMA_RING_WORDS,
           (unsigned)(GUIMAI_GTM_DMA_RING_WORDS * sizeof(uint32)),
           (unsigned)GUIMAI_GTM_DMA_I2S_DELAY_BITS,
           (unsigned)GUIMAI_GTM_DMA_LEFT_WS_LEVEL,
           (unsigned long)s_gtm_dma_dest_addr,
           (unsigned long)tim_src->U,
           (unsigned long)MODULE_DMA.CH[GUIMAI_GTM_DMA_CHANNEL].CHCFGR.U,
           (unsigned long)MODULE_DMA.CH[GUIMAI_GTM_DMA_CHANNEL].ADICR.U);
}

static void guimai_gtm_dma_force_reinit(void)
{
    if (s_gtm_dma_tim_ch != NULL_PTR)
    {
        s_gtm_dma_tim_ch->CTRL.B.TIM_EN = 0U;
        s_gtm_dma_tim_ch->IRQ.EN.U = 0U;
    }
    IfxDma_disableChannelTransaction(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL);
    s_gtm_dma_active = 0U;
    s_gtm_dma_ready = 0U;
    s_gtm_dma_tim_ch = NULL_PTR;
    guimai_gtm_dma_init();
}

static void guimai_gtm_dma_start_chunk(void)
{
    if (!s_gtm_dma_ready)
    {
        return;
    }

    IfxDma_Dma_clearChannelInterrupt(&s_gtm_dma_channel);
    IfxDma_clearChannelTransactionRequestLost(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL);
    IfxDma_Dma_setChannelSourceAddress(&s_gtm_dma_channel, (uint32)&MODULE_P11.IN.U);
    IfxDma_Dma_setChannelDestinationAddress(&s_gtm_dma_channel, s_gtm_dma_dest_addr);
    IfxDma_Dma_setChannelTransferCount(&s_gtm_dma_channel, GUIMAI_GTM_DMA_RING_WORDS);
    s_gtm_dma_read_half = 1U;
    IfxDma_enableChannelTransaction(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL);
    if (s_gtm_dma_tim_ch != NULL_PTR)
    {
        s_gtm_dma_tim_ch->IRQ.NOTIFY.U = 0x3FU;
        IfxGtm_Tim_Ch_setNotificationMode(s_gtm_dma_tim_ch, IfxGtm_IrqMode_pulse);
        IfxGtm_Tim_Ch_setNotification(s_gtm_dma_tim_ch, IfxGtm_Tim_IrqType_newVal);
        s_gtm_dma_tim_ch->CTRL.B.TIM_EN = 1U;
    }
    s_gtm_dma_active = 1U;
    printf("[GUIMAI_GTM_DMA] start tcnt=%u chcsr=0x%08lX tim_ctrl=0x%08lX irq_en=0x%08lX irq_notify=0x%08lX\r\n",
           (unsigned)IfxDma_getChannelTransferCount(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL),
           (unsigned long)MODULE_DMA.CH[GUIMAI_GTM_DMA_CHANNEL].CHCSR.U,
           (unsigned long)((s_gtm_dma_tim_ch != NULL_PTR) ? s_gtm_dma_tim_ch->CTRL.U : 0U),
           (unsigned long)((s_gtm_dma_tim_ch != NULL_PTR) ? s_gtm_dma_tim_ch->IRQ.EN.U : 0U),
           (unsigned long)((s_gtm_dma_tim_ch != NULL_PTR) ? s_gtm_dma_tim_ch->IRQ.NOTIFY.U : 0U));

    if ((IfxDma_getChannelTransferCount(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL) == 0U) ||
        (MODULE_DMA.CH[GUIMAI_GTM_DMA_CHANNEL].CHCSR.U == 0U))
    {
        printf("[GUIMAI_GTM_DMA] start invalid, force reinit.\r\n");
        guimai_gtm_dma_force_reinit();
        if (!s_gtm_dma_ready)
        {
            return;
        }

        IfxDma_Dma_clearChannelInterrupt(&s_gtm_dma_channel);
        IfxDma_clearChannelTransactionRequestLost(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL);
        IfxDma_Dma_setChannelSourceAddress(&s_gtm_dma_channel, (uint32)&MODULE_P11.IN.U);
        IfxDma_Dma_setChannelDestinationAddress(&s_gtm_dma_channel, s_gtm_dma_dest_addr);
        IfxDma_Dma_setChannelTransferCount(&s_gtm_dma_channel, GUIMAI_GTM_DMA_RING_WORDS);
        s_gtm_dma_read_half = 1U;
        IfxDma_enableChannelTransaction(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL);
        if (s_gtm_dma_tim_ch != NULL_PTR)
        {
            s_gtm_dma_tim_ch->IRQ.NOTIFY.U = 0x3FU;
            IfxGtm_Tim_Ch_setNotificationMode(s_gtm_dma_tim_ch, IfxGtm_IrqMode_pulse);
            IfxGtm_Tim_Ch_setNotification(s_gtm_dma_tim_ch, IfxGtm_Tim_IrqType_newVal);
            s_gtm_dma_tim_ch->CTRL.B.TIM_EN = 1U;
        }
        s_gtm_dma_active = 1U;
        printf("[GUIMAI_GTM_DMA] restart tcnt=%u chcsr=0x%08lX tim_ctrl=0x%08lX irq_en=0x%08lX irq_notify=0x%08lX\r\n",
               (unsigned)IfxDma_getChannelTransferCount(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL),
               (unsigned long)MODULE_DMA.CH[GUIMAI_GTM_DMA_CHANNEL].CHCSR.U,
               (unsigned long)((s_gtm_dma_tim_ch != NULL_PTR) ? s_gtm_dma_tim_ch->CTRL.U : 0U),
               (unsigned long)((s_gtm_dma_tim_ch != NULL_PTR) ? s_gtm_dma_tim_ch->IRQ.EN.U : 0U),
               (unsigned long)((s_gtm_dma_tim_ch != NULL_PTR) ? s_gtm_dma_tim_ch->IRQ.NOTIFY.U : 0U));
    }
}

static void guimai_gtm_dma_stop(void)
{
    if (s_gtm_dma_tim_ch != NULL_PTR)
    {
        s_gtm_dma_tim_ch->CTRL.B.TIM_EN = 0U;
        s_gtm_dma_tim_ch->IRQ.EN.U = 0U;
    }
    IfxDma_disableChannelTransaction(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL);
    s_gtm_dma_active = 0U;
}

static uint32 guimai_gtm_dma_process_chunk(volatile uint32 *src, int16 *dst, uint32 max_samples)
{
    uint32 i = 0;
    uint32 count = 0;
    uint8 last_ws = s_gtm_i2s_ws;

    for (i = 0; i < GUIMAI_GTM_DMA_CHUNK_WORDS; i++)
    {
        uint32 port = src[i];
        uint8 ws = (uint8)((port >> 2U) & 1U);
        uint8 sd = (uint8)((port >> 9U) & 1U);

        if (last_ws == 0xFFU)
        {
            last_ws = ws;
            s_gtm_i2s_slot_pos = 0U;
            s_gtm_i2s_bits = 0U;
            s_gtm_i2s_shift = 0U;
            continue;
        }

        if (ws != last_ws)
        {
            if ((s_gtm_dma_i2s_delay_bits != 0U) &&
                (last_ws == s_gtm_dma_left_ws_level) &&
                (s_gtm_i2s_bits > 0U) &&
                (s_gtm_i2s_bits < 16U))
            {
                s_gtm_i2s_shift = (s_gtm_i2s_shift << 1U) | (uint32)sd;
                s_gtm_i2s_bits++;
                if (s_gtm_i2s_bits == 16U)
                {
                    if (count < max_samples)
                    {
                        dst[count++] = (int16)((uint16)s_gtm_i2s_shift);
                    }
                }
            }

            last_ws = ws;
            s_gtm_i2s_slot_pos = s_gtm_dma_i2s_delay_bits ? 1U : 0U;
            s_gtm_i2s_bits = 0U;
            s_gtm_i2s_shift = 0U;
            if (s_gtm_dma_i2s_delay_bits != 0U)
            {
                continue;
            }
        }

        s_gtm_i2s_slot_pos++;

        if (ws != s_gtm_dma_left_ws_level)
        {
            continue;
        }

        if ((s_gtm_i2s_slot_pos > s_gtm_dma_i2s_delay_bits) &&
            (s_gtm_i2s_slot_pos <= (s_gtm_dma_i2s_delay_bits + 16U)))
        {
            s_gtm_i2s_shift = (s_gtm_i2s_shift << 1U) | (uint32)sd;
            s_gtm_i2s_bits++;
            if (s_gtm_i2s_bits == 16U)
            {
                if (count < max_samples)
                {
                    dst[count++] = (int16)((uint16)s_gtm_i2s_shift);
                }
                s_gtm_i2s_shift = 0U;
                s_gtm_i2s_bits = 0U;
            }
        }
    }

    s_gtm_i2s_ws = last_ws;
    return count;
}

static uint32 guimai_gtm_dma_read_samples(int16 *dst, uint32 max_samples)
{
    uint32 count = 0;
    uint32 dadr = 0U;
    uint32 offset = 0U;
    uint8 write_half = 0U;
    uint8 done_half = 0U;

    if (!s_gtm_dma_ready)
    {
        return 0U;
    }

    if (!s_gtm_dma_active)
    {
        guimai_gtm_dma_start_chunk();
        return 0U;
    }

    if (IfxDma_getChannelTransactionRequestLost(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL))
    {
        s_gtm_dma_lost++;
        IfxDma_clearChannelTransactionRequestLost(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL);
    }

    dadr = IfxDma_getChannelDestinationAddress(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL);
    offset = (dadr - s_gtm_dma_dest_addr) & ((uint32)(GUIMAI_GTM_DMA_RING_WORDS * sizeof(uint32)) - 1U);
    write_half = (offset >= (GUIMAI_GTM_DMA_CHUNK_WORDS * sizeof(uint32))) ? 1U : 0U;
    done_half = write_half ^ 1U;

    if (done_half == s_gtm_dma_read_half)
    {
        return 0U;
    }

    s_gtm_dma_chunks++;
    s_gtm_dma_words += GUIMAI_GTM_DMA_CHUNK_WORDS;
    count = guimai_gtm_dma_process_chunk(&s_gtm_dma_port_words[(uint32)done_half * GUIMAI_GTM_DMA_CHUNK_WORDS], dst, max_samples);
    s_gtm_dma_samples += count;
    s_gtm_dma_read_half = done_half;

    return count;
}

static void guimai_gtm_dma_set_decode(uint8 left_ws_level, uint8 delay_bits)
{
    s_gtm_dma_left_ws_level = left_ws_level ? 1U : 0U;
    s_gtm_dma_i2s_delay_bits = delay_bits ? 1U : 0U;
    guimai_gtm_dma_i2s_reset();
}

static void guimai_gtm_dma_reset_runtime_stats(void)
{
    uint32 dadr = IfxDma_getChannelDestinationAddress(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL);
    uint32 offset = (dadr - s_gtm_dma_dest_addr) & ((uint32)(GUIMAI_GTM_DMA_RING_WORDS * sizeof(uint32)) - 1U);
    uint8 write_half = (offset >= (GUIMAI_GTM_DMA_CHUNK_WORDS * sizeof(uint32))) ? 1U : 0U;

    s_gtm_dma_chunks = 0U;
    s_gtm_dma_words = 0U;
    s_gtm_dma_samples = 0U;
    s_gtm_dma_lost = 0U;
    s_gtm_dma_read_half = write_half ^ 1U;
    guimai_gtm_dma_i2s_reset();
}
#endif

#if GUIMAI_GTM_TBCM_DIAG
#if GUIMAI_GTM_ARU_FIFO_DIAG
static void guimai_gtm_aru_fifo_capture_state(void)
{
    Ifx_GTM_PSM_F2A *f2a = IfxGtm_Psm_F2a_getPointer(GUIMAI_GTM_ARU_FIFO_F2A);
    uint32 i = 0U;

    s_gtm_aru_fifo_enable = f2a->ENABLE.U;
    for (i = 0U; i < GUIMAI_GTM_ARU_FIFO_COUNT; i++)
    {
        Ifx_GTM_PSM_FIFO_CH *fifo_ch = IfxGtm_Psm_Fifo_getChannelPointer(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i]);
        s_gtm_aru_fifo_fill[i] = IfxGtm_Psm_Fifo_getChannelFillLevel(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i]);
        s_gtm_aru_fifo_wr[i] = IfxGtm_Psm_Fifo_getChannelWritePtr(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i]);
        s_gtm_aru_fifo_rd[i] = IfxGtm_Psm_Fifo_getChannelReadPtr(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i]);
        s_gtm_aru_fifo_status[i] = (uint32)IfxGtm_Psm_Fifo_getChannelStatus(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i]);
        s_gtm_aru_fifo_irq[i] = fifo_ch->IRQ.NOTIFY.U;
        s_gtm_aru_fifo_stream_state[i] = (uint32)IfxGtm_Psm_F2a_getStreamState(GUIMAI_GTM_ARU_FIFO_F2A, s_gtm_aru_fifo_stream[i]);
        s_gtm_aru_fifo_str_cfg[i] = f2a->STR_CH[s_gtm_aru_fifo_stream[i]].STR_CFG.U;
        s_gtm_aru_fifo_aru_addr[i] = f2a->RD_CH[s_gtm_aru_fifo_stream[i]].ARU_RD_FIFO.B.ADDR;

        if (s_gtm_aru_fifo_fill[i] > s_gtm_aru_fifo_max_fill[i])
        {
            s_gtm_aru_fifo_max_fill[i] = s_gtm_aru_fifo_fill[i];
        }
    }
}

static void guimai_gtm_afd_dump_fifo0(void)
{
    Ifx_GTM_PSM_AFD_CH *afd_ch = IfxGtm_Psm_Afd_getChannelPointer(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0);
    uint32 i = 0U;
    uint32 fill = 0U;

    s_gtm_afd_dump_count = 0U;
    s_gtm_afd_dump_fill_before = IfxGtm_Psm_Fifo_getChannelFillLevel(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0);
    s_gtm_afd_dump_rd_before = IfxGtm_Psm_Fifo_getChannelReadPtr(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0);
    s_gtm_afd_dump_wr_before = IfxGtm_Psm_Fifo_getChannelWritePtr(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0);

    fill = s_gtm_afd_dump_fill_before;
    if (fill > GUIMAI_GTM_AFD_DUMP_WORDS)
    {
        fill = GUIMAI_GTM_AFD_DUMP_WORDS;
    }

    for (i = 0U; i < fill; i++)
    {
        s_gtm_afd_dump_words[i] = afd_ch->BUF_ACC.B.DATA;
        s_gtm_afd_dump_count++;
    }

    s_gtm_afd_dump_fill_after = IfxGtm_Psm_Fifo_getChannelFillLevel(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0);
    s_gtm_afd_dump_rd_after = IfxGtm_Psm_Fifo_getChannelReadPtr(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0);
    s_gtm_afd_dump_wr_after = IfxGtm_Psm_Fifo_getChannelWritePtr(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0);

    printf("[GUIMAI_AFD] fifo0 dump n=%lu fill=%lu->%lu rd=%lu->%lu wr=%lu->%lu\r\n",
           (unsigned long)s_gtm_afd_dump_count,
           (unsigned long)s_gtm_afd_dump_fill_before,
           (unsigned long)s_gtm_afd_dump_fill_after,
           (unsigned long)s_gtm_afd_dump_rd_before,
           (unsigned long)s_gtm_afd_dump_rd_after,
           (unsigned long)s_gtm_afd_dump_wr_before,
           (unsigned long)s_gtm_afd_dump_wr_after);

    if (s_gtm_afd_dump_count > 0U)
    {
        printf("[GUIMAI_AFD] w0-7=%08lX %08lX %08lX %08lX %08lX %08lX %08lX %08lX\r\n",
               (unsigned long)s_gtm_afd_dump_words[0],
               (unsigned long)s_gtm_afd_dump_words[1],
               (unsigned long)s_gtm_afd_dump_words[2],
               (unsigned long)s_gtm_afd_dump_words[3],
               (unsigned long)s_gtm_afd_dump_words[4],
               (unsigned long)s_gtm_afd_dump_words[5],
               (unsigned long)s_gtm_afd_dump_words[6],
               (unsigned long)s_gtm_afd_dump_words[7]);
        printf("[GUIMAI_AFD] w8-15=%08lX %08lX %08lX %08lX %08lX %08lX %08lX %08lX\r\n",
               (unsigned long)s_gtm_afd_dump_words[8],
               (unsigned long)s_gtm_afd_dump_words[9],
               (unsigned long)s_gtm_afd_dump_words[10],
               (unsigned long)s_gtm_afd_dump_words[11],
               (unsigned long)s_gtm_afd_dump_words[12],
               (unsigned long)s_gtm_afd_dump_words[13],
               (unsigned long)s_gtm_afd_dump_words[14],
               (unsigned long)s_gtm_afd_dump_words[15]);
    }
}

static void guimai_gtm_afd_live_reset(void)
{
    uint32 i = 0U;

    for (i = 0U; i < GUIMAI_GTM_AFD_DUMP_WORDS; i++)
    {
        s_gtm_afd_live_words[i] = 0U;
    }
    s_gtm_afd_live_write = 0U;
    s_gtm_afd_live_drained = 0U;
    s_gtm_afd_live_nonzero = 0U;
    s_gtm_afd_live_changed = 0U;
    s_gtm_afd_live_last_word = 0U;
    s_gtm_afd_live_has_last = 0U;
}

static void guimai_gtm_afd_live_drain_fifo0(uint32 max_words)
{
    Ifx_GTM_PSM_AFD_CH *afd_ch = IfxGtm_Psm_Afd_getChannelPointer(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0);
    uint32 i = 0U;
    uint32 fill = IfxGtm_Psm_Fifo_getChannelFillLevel(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0);

    if (fill > max_words)
    {
        fill = max_words;
    }

    for (i = 0U; i < fill; i++)
    {
        uint32 word = afd_ch->BUF_ACC.B.DATA;
        s_gtm_afd_live_words[s_gtm_afd_live_write & (GUIMAI_GTM_AFD_DUMP_WORDS - 1U)] = word;
        s_gtm_afd_live_write++;
        s_gtm_afd_live_drained++;
        if (word != 0U)
        {
            s_gtm_afd_live_nonzero++;
        }
        if (s_gtm_afd_live_has_last && (word != s_gtm_afd_live_last_word))
        {
            s_gtm_afd_live_changed++;
        }
        s_gtm_afd_live_last_word = word;
        s_gtm_afd_live_has_last = 1U;
    }
}

static void guimai_gtm_afd_live_print(void)
{
    uint32 start = 0U;
    uint32 out[GUIMAI_GTM_AFD_DUMP_WORDS];
    uint32 i = 0U;

    if (s_gtm_afd_live_write >= GUIMAI_GTM_AFD_DUMP_WORDS)
    {
        start = s_gtm_afd_live_write - GUIMAI_GTM_AFD_DUMP_WORDS;
    }

    for (i = 0U; i < GUIMAI_GTM_AFD_DUMP_WORDS; i++)
    {
        out[i] = s_gtm_afd_live_words[(start + i) & (GUIMAI_GTM_AFD_DUMP_WORDS - 1U)];
    }

    printf("[GUIMAI_AFD_LIVE] drained=%lu nonzero=%lu changed=%lu last=0x%08lX wr=%lu\r\n",
           (unsigned long)s_gtm_afd_live_drained,
           (unsigned long)s_gtm_afd_live_nonzero,
           (unsigned long)s_gtm_afd_live_changed,
           (unsigned long)s_gtm_afd_live_last_word,
           (unsigned long)s_gtm_afd_live_write);
    printf("[GUIMAI_AFD_LIVE] last0-7=%08lX %08lX %08lX %08lX %08lX %08lX %08lX %08lX\r\n",
           (unsigned long)out[0],
           (unsigned long)out[1],
           (unsigned long)out[2],
           (unsigned long)out[3],
           (unsigned long)out[4],
           (unsigned long)out[5],
           (unsigned long)out[6],
           (unsigned long)out[7]);
    printf("[GUIMAI_AFD_LIVE] last8-15=%08lX %08lX %08lX %08lX %08lX %08lX %08lX %08lX\r\n",
           (unsigned long)out[8],
           (unsigned long)out[9],
           (unsigned long)out[10],
           (unsigned long)out[11],
           (unsigned long)out[12],
           (unsigned long)out[13],
           (unsigned long)out[14],
           (unsigned long)out[15]);
}

#if GUIMAI_GTM_TBCM_DECODE_DIAG
static void guimai_gtm_tbcm_decode_reset_state(guimai_tbcm_decode_candidate_state *state)
{
    state->last_ws = 0xFFU;
    state->slot_pos = 0U;
    state->bits = 0U;
    state->shift = 0U;
    state->words = 0U;
    state->ws_edges = 0U;
    state->samples = 0U;
    state->left_samples = 0U;
    state->right_samples = 0U;
    state->bad_slots = 0U;
    state->max_slot = 0U;
    state->ones = 0U;
    state->peak = 0U;
    state->abs_sum = 0U;
    state->last_sample = 0;
}

static void guimai_gtm_tssm_slice_reset_state(guimai_tssm_slice_state *state)
{
    state->words = 0U;
    state->changed = 0U;
    state->zero = 0U;
    state->neg1 = 0U;
    state->sat = 0U;
    state->peak = 0U;
    state->abs_sum = 0U;
    state->last_raw = 0U;
    state->last_sample = 0;
    state->min_sample = 0;
    state->max_sample = 0;
    state->has_last = 0U;
}

static void guimai_gtm_tbcm_decode_reset(void)
{
    uint32 i = 0U;
    uint32 mode = 0U;
    uint32 half = 0U;

    for (i = 0U; i < GUIMAI_GTM_TBCM_DECODE_CANDIDATES; i++)
    {
        guimai_gtm_tbcm_decode_reset_state(&s_gtm_tbcm_decode_state[i]);
    }
    for (i = 0U; i < GUIMAI_GTM_TSSM_SLICE_CANDIDATES; i++)
    {
        guimai_gtm_tssm_slice_reset_state(&s_gtm_tssm_slice_state[i]);
    }

    for (mode = 0U; mode < GUIMAI_GTM_TBCM_DECODE_RLE_MODES; mode++)
    {
        s_gtm_tbcm_rle_total[mode] = 0U;
        s_gtm_tbcm_rle_zero[mode] = 0U;
        s_gtm_tbcm_rle_max[mode] = 0U;
        for (i = 0U; i < GUIMAI_GTM_TBCM_DECODE_CANDIDATES; i++)
        {
            guimai_gtm_tbcm_decode_reset_state(&s_gtm_tbcm_decode_rle_state[mode][i]);
        }
    }

    for (i = 0U; i < 32U; i++)
    {
        s_gtm_tbcm_bit_ones[i] = 0U;
        s_gtm_tbcm_bit_toggles[i] = 0U;
    }
    s_gtm_tbcm_bit_last = 0U;
    s_gtm_tbcm_bit_has_last = 0U;
    s_gtm_tbcm_decode_word_index = 0U;

    for (half = 0U; half < 2U; half++)
    {
        s_gtm_tbcm_pair_words[half] = 0U;
        s_gtm_tbcm_pair_last[half] = 0U;
        s_gtm_tbcm_pair_has_last[half] = 0U;
        for (i = 0U; i < 32U; i++)
        {
            s_gtm_tbcm_pair_bit_ones[half][i] = 0U;
            s_gtm_tbcm_pair_bit_toggles[half][i] = 0U;
        }
        for (i = 0U; i < GUIMAI_GTM_TBCM_DECODE_CANDIDATES; i++)
        {
            guimai_gtm_tbcm_decode_reset_state(&s_gtm_tbcm_pair_state[half][i]);
        }
    }
}

static void guimai_gtm_tbcm_decode_emit_sample(guimai_tbcm_decode_candidate_state *state, uint8 is_left)
{
    int16 sample = (int16)(uint16)(state->shift & 0xFFFFU);
    int32 sample32 = (int32)sample;
    uint32 abs_sample = (sample32 < 0) ? (uint32)(-sample32) : (uint32)sample32;

    state->samples++;
    if (is_left)
    {
        state->left_samples++;
    }
    else
    {
        state->right_samples++;
    }
    if (abs_sample > state->peak)
    {
        state->peak = abs_sample;
    }
    if ((0xFFFFFFFFUL - state->abs_sum) < abs_sample)
    {
        state->abs_sum = 0xFFFFFFFFUL;
    }
    else
    {
        state->abs_sum += abs_sample;
    }
    state->last_sample = sample;
}

static void guimai_gtm_tbcm_decode_candidate_run(guimai_tbcm_decode_candidate_state *state,
                                                 const guimai_tbcm_decode_candidate_config *cfg,
                                                 uint32 word,
                                                 uint32 run_len)
{
    uint8 ws = (uint8)((word >> cfg->ws_bit) & 1U);
    uint8 sd = (uint8)((word >> cfg->sd_bit) & 1U);
    uint8 is_left = 0U;
    uint32 take = 0U;
    uint32 i = 0U;

    if (cfg->invert_ws)
    {
        ws ^= 1U;
    }

    state->words++;
    if (sd)
    {
        state->ones += run_len;
    }

    if (state->last_ws == 0xFFU)
    {
        state->last_ws = ws;
        state->slot_pos = 0U;
        state->bits = 0U;
        state->shift = 0U;
        return;
    }

    if (ws != state->last_ws)
    {
        state->ws_edges++;
        if (state->slot_pos > state->max_slot)
        {
            state->max_slot = state->slot_pos;
        }
        if ((state->slot_pos != 16U) && (state->slot_pos != 32U))
        {
            state->bad_slots++;
        }
        state->last_ws = ws;
        state->slot_pos = 0U;
        state->bits = 0U;
        state->shift = 0U;
    }

    is_left = (ws == cfg->left_ws_level) ? 1U : 0U;

    if (run_len == 0U)
    {
        return;
    }

    if (state->bits < 16U)
    {
        take = 16U - (uint32)state->bits;
        if (take > run_len)
        {
            take = run_len;
        }
        for (i = 0U; i < take; i++)
        {
            state->shift = (state->shift << 1U) | (uint32)sd;
        }
        state->bits = (uint8)((uint32)state->bits + take);
        if (state->bits == 16U)
        {
            guimai_gtm_tbcm_decode_emit_sample(state, is_left);
        }
    }
    state->slot_pos += run_len;
}

static void guimai_gtm_tbcm_decode_candidate_word(uint32 index, uint32 word)
{
    guimai_gtm_tbcm_decode_candidate_run(&s_gtm_tbcm_decode_state[index],
                                         &s_gtm_tbcm_decode_cfg[index],
                                         word,
                                         1U);
}

static void guimai_gtm_tbcm_decode_rle_word(uint32 word)
{
    uint32 mode = 0U;
    uint32 i = 0U;
    uint32 ecnt = (word >> 24U) & 0xFFU;
    uint32 run_len = 0U;

    for (mode = 0U; mode < GUIMAI_GTM_TBCM_DECODE_RLE_MODES; mode++)
    {
        run_len = (mode == 0U) ? ecnt : (ecnt + 1U);
        s_gtm_tbcm_rle_total[mode] += run_len;
        if (run_len == 0U)
        {
            s_gtm_tbcm_rle_zero[mode]++;
        }
        if (run_len > s_gtm_tbcm_rle_max[mode])
        {
            s_gtm_tbcm_rle_max[mode] = run_len;
        }
        for (i = 0U; i < GUIMAI_GTM_TBCM_DECODE_CANDIDATES; i++)
        {
            guimai_gtm_tbcm_decode_candidate_run(&s_gtm_tbcm_decode_rle_state[mode][i],
                                                 &s_gtm_tbcm_decode_cfg[i],
                                                 word,
                                                 run_len);
        }
    }
}

static uint16 guimai_reverse16(uint16 value)
{
    uint16 out = 0U;
    uint32 i = 0U;
    for (i = 0U; i < 16U; i++)
    {
        out = (uint16)((out << 1U) | ((value >> i) & 1U));
    }
    return out;
}

static void guimai_gtm_tssm_slice_process_word(uint32 half, uint32 word)
{
    uint32 i = 0U;

    for (i = 0U; i < GUIMAI_GTM_TSSM_SLICE_CANDIDATES; i++)
    {
        const guimai_tssm_slice_config *cfg = &s_gtm_tssm_slice_cfg[i];
        guimai_tssm_slice_state *state = &s_gtm_tssm_slice_state[i];
        uint16 raw = 0U;
        int16 sample = 0;
        int32 sample32 = 0;
        uint32 abs_sample = 0U;

        if (cfg->half != half)
        {
            continue;
        }

        raw = (uint16)((word >> cfg->shift) & 0xFFFFU);
        if (cfg->invert)
        {
            raw = guimai_reverse16(raw);
        }
        sample = (int16)raw;
        sample32 = (int32)sample;
        abs_sample = (sample32 < 0) ? (uint32)(-sample32) : (uint32)sample32;

        if (state->has_last && (raw != state->last_raw))
        {
            state->changed++;
        }
        if (raw == 0U)
        {
            state->zero++;
        }
        if (raw == 0xFFFFU)
        {
            state->neg1++;
        }
        if ((raw == 0x7FFFU) || (raw == 0x8000U))
        {
            state->sat++;
        }
        if ((state->words == 0U) || (sample < state->min_sample))
        {
            state->min_sample = sample;
        }
        if ((state->words == 0U) || (sample > state->max_sample))
        {
            state->max_sample = sample;
        }
        if (abs_sample > state->peak)
        {
            state->peak = abs_sample;
        }
        if ((0xFFFFFFFFUL - state->abs_sum) < abs_sample)
        {
            state->abs_sum = 0xFFFFFFFFUL;
        }
        else
        {
            state->abs_sum += abs_sample;
        }

        state->last_raw = raw;
        state->last_sample = sample;
        state->has_last = 1U;
        state->words++;
    }
}

static void guimai_gtm_tbcm_decode_process_word(uint32 word)
{
    uint32 i = 0U;
    uint32 half = s_gtm_tbcm_decode_word_index & 1U;
    s_gtm_tbcm_decode_word_index++;

    for (i = 0U; i < 32U; i++)
    {
        uint32 mask = 1UL << i;
        if ((word & mask) != 0U)
        {
            s_gtm_tbcm_bit_ones[i]++;
        }
        if (s_gtm_tbcm_bit_has_last && ((word ^ s_gtm_tbcm_bit_last) & mask))
        {
            s_gtm_tbcm_bit_toggles[i]++;
        }

        if ((word & mask) != 0U)
        {
            s_gtm_tbcm_pair_bit_ones[half][i]++;
        }
        if (s_gtm_tbcm_pair_has_last[half] && ((word ^ s_gtm_tbcm_pair_last[half]) & mask))
        {
            s_gtm_tbcm_pair_bit_toggles[half][i]++;
        }
    }
    s_gtm_tbcm_bit_last = word;
    s_gtm_tbcm_bit_has_last = 1U;
    s_gtm_tbcm_pair_last[half] = word;
    s_gtm_tbcm_pair_has_last[half] = 1U;
    s_gtm_tbcm_pair_words[half]++;
    guimai_gtm_tssm_slice_process_word(half, word);

    for (i = 0U; i < GUIMAI_GTM_TBCM_DECODE_CANDIDATES; i++)
    {
        guimai_gtm_tbcm_decode_candidate_word(i, word);
        guimai_gtm_tbcm_decode_candidate_run(&s_gtm_tbcm_pair_state[half][i],
                                             &s_gtm_tbcm_decode_cfg[i],
                                             word,
                                             1U);
    }
    guimai_gtm_tbcm_decode_rle_word(word);
}

static void guimai_gtm_tbcm_decode_print(void)
{
    uint32 i = 0U;
    uint32 mode = 0U;

    printf("[GUIMAI_DEC_BITS] b0-7 ones=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu tog=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu\r\n",
           (unsigned long)s_gtm_tbcm_bit_ones[0], (unsigned long)s_gtm_tbcm_bit_ones[1],
           (unsigned long)s_gtm_tbcm_bit_ones[2], (unsigned long)s_gtm_tbcm_bit_ones[3],
           (unsigned long)s_gtm_tbcm_bit_ones[4], (unsigned long)s_gtm_tbcm_bit_ones[5],
           (unsigned long)s_gtm_tbcm_bit_ones[6], (unsigned long)s_gtm_tbcm_bit_ones[7],
           (unsigned long)s_gtm_tbcm_bit_toggles[0], (unsigned long)s_gtm_tbcm_bit_toggles[1],
           (unsigned long)s_gtm_tbcm_bit_toggles[2], (unsigned long)s_gtm_tbcm_bit_toggles[3],
           (unsigned long)s_gtm_tbcm_bit_toggles[4], (unsigned long)s_gtm_tbcm_bit_toggles[5],
           (unsigned long)s_gtm_tbcm_bit_toggles[6], (unsigned long)s_gtm_tbcm_bit_toggles[7]);
    printf("[GUIMAI_DEC_BITS] b8-15 ones=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu tog=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu\r\n",
           (unsigned long)s_gtm_tbcm_bit_ones[8], (unsigned long)s_gtm_tbcm_bit_ones[9],
           (unsigned long)s_gtm_tbcm_bit_ones[10], (unsigned long)s_gtm_tbcm_bit_ones[11],
           (unsigned long)s_gtm_tbcm_bit_ones[12], (unsigned long)s_gtm_tbcm_bit_ones[13],
           (unsigned long)s_gtm_tbcm_bit_ones[14], (unsigned long)s_gtm_tbcm_bit_ones[15],
           (unsigned long)s_gtm_tbcm_bit_toggles[8], (unsigned long)s_gtm_tbcm_bit_toggles[9],
           (unsigned long)s_gtm_tbcm_bit_toggles[10], (unsigned long)s_gtm_tbcm_bit_toggles[11],
           (unsigned long)s_gtm_tbcm_bit_toggles[12], (unsigned long)s_gtm_tbcm_bit_toggles[13],
           (unsigned long)s_gtm_tbcm_bit_toggles[14], (unsigned long)s_gtm_tbcm_bit_toggles[15]);
    printf("[GUIMAI_DEC_BITS] b16-23 ones=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu tog=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu\r\n",
           (unsigned long)s_gtm_tbcm_bit_ones[16], (unsigned long)s_gtm_tbcm_bit_ones[17],
           (unsigned long)s_gtm_tbcm_bit_ones[18], (unsigned long)s_gtm_tbcm_bit_ones[19],
           (unsigned long)s_gtm_tbcm_bit_ones[20], (unsigned long)s_gtm_tbcm_bit_ones[21],
           (unsigned long)s_gtm_tbcm_bit_ones[22], (unsigned long)s_gtm_tbcm_bit_ones[23],
           (unsigned long)s_gtm_tbcm_bit_toggles[16], (unsigned long)s_gtm_tbcm_bit_toggles[17],
           (unsigned long)s_gtm_tbcm_bit_toggles[18], (unsigned long)s_gtm_tbcm_bit_toggles[19],
           (unsigned long)s_gtm_tbcm_bit_toggles[20], (unsigned long)s_gtm_tbcm_bit_toggles[21],
           (unsigned long)s_gtm_tbcm_bit_toggles[22], (unsigned long)s_gtm_tbcm_bit_toggles[23]);
    printf("[GUIMAI_DEC_BITS] b24-31 ones=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu tog=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu\r\n",
           (unsigned long)s_gtm_tbcm_bit_ones[24], (unsigned long)s_gtm_tbcm_bit_ones[25],
           (unsigned long)s_gtm_tbcm_bit_ones[26], (unsigned long)s_gtm_tbcm_bit_ones[27],
           (unsigned long)s_gtm_tbcm_bit_ones[28], (unsigned long)s_gtm_tbcm_bit_ones[29],
           (unsigned long)s_gtm_tbcm_bit_ones[30], (unsigned long)s_gtm_tbcm_bit_ones[31],
           (unsigned long)s_gtm_tbcm_bit_toggles[24], (unsigned long)s_gtm_tbcm_bit_toggles[25],
           (unsigned long)s_gtm_tbcm_bit_toggles[26], (unsigned long)s_gtm_tbcm_bit_toggles[27],
           (unsigned long)s_gtm_tbcm_bit_toggles[28], (unsigned long)s_gtm_tbcm_bit_toggles[29],
           (unsigned long)s_gtm_tbcm_bit_toggles[30], (unsigned long)s_gtm_tbcm_bit_toggles[31]);

    for (i = 0U; i < GUIMAI_GTM_TBCM_DECODE_CANDIDATES; i++)
    {
        const guimai_tbcm_decode_candidate_state *state = &s_gtm_tbcm_decode_state[i];
        uint32 avg_abs = (state->samples > 0U) ? (state->abs_sum / state->samples) : 0U;
        printf("[GUIMAI_DEC] %s words=%lu ws_edges=%lu samples=%lu L/R=%lu/%lu bad_slots=%lu max_slot=%lu sd_ones=%lu peak=%lu avg_abs=%lu last=%d\r\n",
               s_gtm_tbcm_decode_cfg[i].name,
               (unsigned long)state->words,
               (unsigned long)state->ws_edges,
               (unsigned long)state->samples,
               (unsigned long)state->left_samples,
               (unsigned long)state->right_samples,
               (unsigned long)state->bad_slots,
               (unsigned long)state->max_slot,
               (unsigned long)state->ones,
               (unsigned long)state->peak,
               (unsigned long)avg_abs,
               (int)state->last_sample);
    }

    for (i = 0U; i < GUIMAI_GTM_TSSM_SLICE_CANDIDATES; i++)
    {
        const guimai_tssm_slice_state *state = &s_gtm_tssm_slice_state[i];
        uint32 avg_abs = (state->words > 0U) ? (state->abs_sum / state->words) : 0U;
        printf("[GUIMAI_TSSM_SLICE] %s half=%u shift=%u rev=%u words=%lu changed=%lu zero=%lu neg1=%lu sat=%lu peak=%lu avg_abs=%lu min=%d max=%d last=%d raw=0x%04X\r\n",
               s_gtm_tssm_slice_cfg[i].name,
               (unsigned)s_gtm_tssm_slice_cfg[i].half,
               (unsigned)s_gtm_tssm_slice_cfg[i].shift,
               (unsigned)s_gtm_tssm_slice_cfg[i].invert,
               (unsigned long)state->words,
               (unsigned long)state->changed,
               (unsigned long)state->zero,
               (unsigned long)state->neg1,
               (unsigned long)state->sat,
               (unsigned long)state->peak,
               (unsigned long)avg_abs,
               (int)state->min_sample,
               (int)state->max_sample,
               (int)state->last_sample,
               (unsigned)state->last_raw);
    }

    for (mode = 0U; mode < 2U; mode++)
    {
        printf("[GUIMAI_DEC_PAIR_BITS] h%lu words=%lu b0-7 ones=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu tog=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu\r\n",
               (unsigned long)mode,
               (unsigned long)s_gtm_tbcm_pair_words[mode],
               (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][0], (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][1],
               (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][2], (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][3],
               (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][4], (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][5],
               (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][6], (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][7],
               (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][0], (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][1],
               (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][2], (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][3],
               (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][4], (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][5],
               (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][6], (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][7]);
        printf("[GUIMAI_DEC_PAIR_BITS] h%lu b8-15 ones=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu tog=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu\r\n",
               (unsigned long)mode,
               (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][8], (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][9],
               (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][10], (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][11],
               (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][12], (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][13],
               (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][14], (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][15],
               (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][8], (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][9],
               (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][10], (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][11],
               (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][12], (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][13],
               (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][14], (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][15]);
        printf("[GUIMAI_DEC_PAIR_BITS] h%lu b16-23 ones=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu tog=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu\r\n",
               (unsigned long)mode,
               (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][16], (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][17],
               (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][18], (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][19],
               (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][20], (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][21],
               (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][22], (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][23],
               (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][16], (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][17],
               (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][18], (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][19],
               (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][20], (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][21],
               (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][22], (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][23]);
        printf("[GUIMAI_DEC_PAIR_BITS] h%lu b24-31 ones=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu tog=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu\r\n",
               (unsigned long)mode,
               (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][24], (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][25],
               (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][26], (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][27],
               (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][28], (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][29],
               (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][30], (unsigned long)s_gtm_tbcm_pair_bit_ones[mode][31],
               (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][24], (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][25],
               (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][26], (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][27],
               (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][28], (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][29],
               (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][30], (unsigned long)s_gtm_tbcm_pair_bit_toggles[mode][31]);
        for (i = 0U; i < GUIMAI_GTM_TBCM_DECODE_CANDIDATES; i++)
        {
            const guimai_tbcm_decode_candidate_state *state = &s_gtm_tbcm_pair_state[mode][i];
            uint32 avg_abs = (state->samples > 0U) ? (state->abs_sum / state->samples) : 0U;
            printf("[GUIMAI_DEC_PAIR] h%lu %s words=%lu ws_edges=%lu samples=%lu L/R=%lu/%lu bad_slots=%lu max_slot=%lu sd_ones=%lu peak=%lu avg_abs=%lu last=%d\r\n",
                   (unsigned long)mode,
                   s_gtm_tbcm_decode_cfg[i].name,
                   (unsigned long)state->words,
                   (unsigned long)state->ws_edges,
                   (unsigned long)state->samples,
                   (unsigned long)state->left_samples,
                   (unsigned long)state->right_samples,
                   (unsigned long)state->bad_slots,
                   (unsigned long)state->max_slot,
                   (unsigned long)state->ones,
                   (unsigned long)state->peak,
                   (unsigned long)avg_abs,
                   (int)state->last_sample);
        }
    }

    for (mode = 0U; mode < GUIMAI_GTM_TBCM_DECODE_RLE_MODES; mode++)
    {
        const char *mode_name = (mode == 0U) ? "ecnt" : "ecnt1";
        printf("[GUIMAI_DEC_RLE] mode=%s total=%lu zero=%lu max=%lu\r\n",
               mode_name,
               (unsigned long)s_gtm_tbcm_rle_total[mode],
               (unsigned long)s_gtm_tbcm_rle_zero[mode],
               (unsigned long)s_gtm_tbcm_rle_max[mode]);
        for (i = 0U; i < GUIMAI_GTM_TBCM_DECODE_CANDIDATES; i++)
        {
            const guimai_tbcm_decode_candidate_state *state = &s_gtm_tbcm_decode_rle_state[mode][i];
            uint32 avg_abs = (state->samples > 0U) ? (state->abs_sum / state->samples) : 0U;
            printf("[GUIMAI_DEC_RLE] %s %s words=%lu ws_edges=%lu samples=%lu L/R=%lu/%lu bad_slots=%lu max_slot=%lu sd_ones=%lu peak=%lu avg_abs=%lu last=%d\r\n",
                   mode_name,
                   s_gtm_tbcm_decode_cfg[i].name,
                   (unsigned long)state->words,
                   (unsigned long)state->ws_edges,
                   (unsigned long)state->samples,
                   (unsigned long)state->left_samples,
                   (unsigned long)state->right_samples,
                   (unsigned long)state->bad_slots,
                   (unsigned long)state->max_slot,
                   (unsigned long)state->ones,
                   (unsigned long)state->peak,
                   (unsigned long)avg_abs,
                   (int)state->last_sample);
        }
    }
}
#endif

#if GUIMAI_GTM_AFD_DMA_DIAG
static void guimai_gtm_afd_dma_init(void)
{
    IfxDma_Dma_Config dma_config;
    IfxDma_Dma_ChannelConfig ch_config;
    Ifx_GTM_PSM_AFD_CH *afd_ch = IfxGtm_Psm_Afd_getChannelPointer(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0);
    volatile Ifx_SRC_SRCR *fifo_src = IfxGtm_Psm_Fifo_getChannelSrcPointer(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0);
    uint32 i = 0U;

    if (s_gtm_afd_dma_ready)
    {
        return;
    }

    IfxDma_Dma_initModuleConfig(&dma_config, &MODULE_DMA);
    IfxDma_Dma_initModule(&s_gtm_afd_dma_module, &dma_config);
    IfxDma_Dma_initChannelConfig(&ch_config, &s_gtm_afd_dma_module);

    for (i = 0U; i < GUIMAI_GTM_AFD_DMA_WORDS; i++)
    {
        s_gtm_afd_dma_words[i] = 0U;
    }
    for (i = 0U; i < GUIMAI_GTM_AFD_DMA_HISTORY_WORDS; i++)
    {
        s_gtm_afd_dma_history[i] = 0U;
    }

    s_gtm_afd_dma_dest_addr = IFXCPU_GLB_ADDR_DSPR(IfxCpu_getCoreId(), (uint32)&s_gtm_afd_dma_words[0]);
    ch_config.channelId = GUIMAI_GTM_DMA_CHANNEL;
    ch_config.sourceAddress = (uint32)&afd_ch->BUF_ACC.U;
    ch_config.sourceCircularBufferEnabled = TRUE;
    ch_config.sourceAddressCircularRange = IfxDma_ChannelIncrementCircular_4;
    ch_config.destinationAddress = s_gtm_afd_dma_dest_addr;
    ch_config.destinationAddressIncrementStep = IfxDma_ChannelIncrementStep_1;
    ch_config.destinationAddressCircularRange = IfxDma_ChannelIncrementCircular_none;
    ch_config.moveSize = IfxDma_ChannelMoveSize_32bit;
    ch_config.transferCount = (uint16)GUIMAI_GTM_AFD_DMA_WORDS;
    ch_config.requestMode = IfxDma_ChannelRequestMode_oneTransferPerRequest;
    ch_config.operationMode = IfxDma_ChannelOperationMode_single;
    ch_config.blockMode = IfxDma_ChannelMove_1;
    ch_config.busPriority = IfxDma_ChannelBusPriority_high;
    ch_config.hardwareRequestEnabled = TRUE;
    ch_config.channelInterruptEnabled = TRUE;
    ch_config.channelInterruptPriority = 0;
    ch_config.transactionRequestLostInterruptEnabled = TRUE;
    IfxDma_Dma_initChannel(&s_gtm_afd_dma_channel, &ch_config);

    IfxSrc_init(fifo_src, IfxSrc_Tos_dma, (Ifx_Priority)GUIMAI_GTM_DMA_CHANNEL);
    IfxSrc_enable(fifo_src);

    s_gtm_afd_dma_ready = 1U;
    s_gtm_afd_dma_src = fifo_src->U;
    printf("[GUIMAI_AFD_DMA] init ch=%u words=%u src=AFD0_CH0_BUF_ACC fifo_src=0x%08lX CHCFGR=0x%08lX ADICR=0x%08lX SADR=0x%08lX DADR=0x%08lX\r\n",
           (unsigned)GUIMAI_GTM_DMA_CHANNEL,
           (unsigned)GUIMAI_GTM_AFD_DMA_WORDS,
           (unsigned long)fifo_src->U,
           (unsigned long)MODULE_DMA.CH[GUIMAI_GTM_DMA_CHANNEL].CHCFGR.U,
           (unsigned long)MODULE_DMA.CH[GUIMAI_GTM_DMA_CHANNEL].ADICR.U,
           (unsigned long)MODULE_DMA.CH[GUIMAI_GTM_DMA_CHANNEL].SADR.U,
           (unsigned long)MODULE_DMA.CH[GUIMAI_GTM_DMA_CHANNEL].DADR.U);
}

static void guimai_gtm_afd_dma_arm(void)
{
    Ifx_GTM_PSM_AFD_CH *afd_ch = IfxGtm_Psm_Afd_getChannelPointer(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0);

    if (!s_gtm_afd_dma_ready)
    {
        return;
    }

    IfxDma_disableChannelTransaction(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL);
    IfxDma_Dma_clearChannelInterrupt(&s_gtm_afd_dma_channel);
    IfxDma_clearChannelTransactionRequestLost(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL);
    IfxDma_Dma_setChannelSourceAddress(&s_gtm_afd_dma_channel, (uint32)&afd_ch->BUF_ACC.U);
    IfxDma_Dma_setChannelDestinationAddress(&s_gtm_afd_dma_channel, s_gtm_afd_dma_dest_addr);
    IfxDma_Dma_setChannelTransferCount(&s_gtm_afd_dma_channel, GUIMAI_GTM_AFD_DMA_WORDS);
    IfxDma_enableChannelTransaction(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL);
}

static void guimai_gtm_afd_dma_start(void)
{
    Ifx_GTM_PSM_FIFO_CH *fifo_ch = IfxGtm_Psm_Fifo_getChannelPointer(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0);
    uint32 i = 0U;

    if (!s_gtm_afd_dma_ready)
    {
        return;
    }

    for (i = 0U; i < GUIMAI_GTM_AFD_DMA_WORDS; i++)
    {
        s_gtm_afd_dma_words[i] = 0U;
    }
    s_gtm_afd_dma_active = 1U;
    s_gtm_afd_dma_done = 0U;
    s_gtm_afd_dma_lost = 0U;
    s_gtm_afd_dma_irq_notify = 0U;
    s_gtm_afd_dma_chcsr = 0U;
    s_gtm_afd_dma_tcnt = 0U;
    s_gtm_afd_dma_chunks = 0U;
    s_gtm_afd_dma_words_total = 0U;
    s_gtm_afd_dma_nonzero_total = 0U;
    s_gtm_afd_dma_changed_total = 0U;
    s_gtm_afd_dma_last_word = 0U;
    s_gtm_afd_dma_has_last = 0U;
    s_gtm_afd_dma_history_write = 0U;
#if GUIMAI_GTM_TBCM_DECODE_DIAG
    guimai_gtm_tbcm_decode_reset();
#endif

    fifo_ch->IRQ.NOTIFY.U = 0xFU;
    IfxGtm_Psm_Fifo_setChannelInterruptMode(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0, IfxGtm_Psm_InterruptMode_pulse);
    IfxGtm_Psm_Fifo_setChannelDmaHystMode(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0, FALSE, IfxGtm_Psm_FifoChannelDmaHystDir_read);
    IfxGtm_Psm_Fifo_enableChannelInterrupt(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0, IfxGtm_Psm_FifoChannelInterrupt_upperWm);
    guimai_gtm_afd_dma_arm();

    printf("[GUIMAI_AFD_DMA] start uwm=%lu lwm=%lu irq_en=0x%08lX irq_mode=0x%08lX fill=%lu rd=%lu wr=%lu\r\n",
           (unsigned long)IfxGtm_Psm_Fifo_getChannelUpperWatermark(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0),
           (unsigned long)IfxGtm_Psm_Fifo_getChannelLowerWatermark(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0),
           (unsigned long)fifo_ch->IRQ.EN.U,
           (unsigned long)fifo_ch->IRQ.MODE.U,
           (unsigned long)IfxGtm_Psm_Fifo_getChannelFillLevel(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0),
           (unsigned long)IfxGtm_Psm_Fifo_getChannelReadPtr(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0),
           (unsigned long)IfxGtm_Psm_Fifo_getChannelWritePtr(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0));
}

static void guimai_gtm_afd_dma_poll(void)
{
    Ifx_GTM_PSM_FIFO_CH *fifo_ch = IfxGtm_Psm_Fifo_getChannelPointer(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0);
    uint8 done = 0U;
    uint32 i = 0U;

    if (!s_gtm_afd_dma_active)
    {
        return;
    }

    s_gtm_afd_dma_irq_notify = fifo_ch->IRQ.NOTIFY.U;
    s_gtm_afd_dma_chcsr = MODULE_DMA.CH[GUIMAI_GTM_DMA_CHANNEL].CHCSR.U;
    s_gtm_afd_dma_tcnt = MODULE_DMA.CH[GUIMAI_GTM_DMA_CHANNEL].CHCSR.B.TCOUNT;
    if (IfxDma_getChannelTransactionRequestLost(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL))
    {
        s_gtm_afd_dma_lost++;
        IfxDma_clearChannelTransactionRequestLost(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL);
    }
    if (IfxDma_Dma_getChannelInterrupt(&s_gtm_afd_dma_channel))
    {
        s_gtm_afd_dma_done = 1U;
        done = 1U;
        IfxDma_Dma_clearChannelInterrupt(&s_gtm_afd_dma_channel);
    }

    if (done)
    {
        for (i = 0U; i < GUIMAI_GTM_AFD_DMA_WORDS; i++)
        {
            uint32 word = s_gtm_afd_dma_words[i];
            s_gtm_afd_dma_history[s_gtm_afd_dma_history_write & (GUIMAI_GTM_AFD_DMA_HISTORY_WORDS - 1U)] = word;
            s_gtm_afd_dma_history_write++;
#if GUIMAI_GTM_TBCM_DECODE_DIAG
            guimai_gtm_tbcm_decode_process_word(word);
#endif
            if (word != 0U)
            {
                s_gtm_afd_dma_nonzero_total++;
            }
            if (s_gtm_afd_dma_has_last && (word != s_gtm_afd_dma_last_word))
            {
                s_gtm_afd_dma_changed_total++;
            }
            s_gtm_afd_dma_last_word = word;
            s_gtm_afd_dma_has_last = 1U;
        }
        s_gtm_afd_dma_chunks++;
        s_gtm_afd_dma_words_total += GUIMAI_GTM_AFD_DMA_WORDS;

        for (i = 0U; i < GUIMAI_GTM_AFD_DMA_WORDS; i++)
        {
            s_gtm_afd_dma_words[i] = 0U;
        }
        fifo_ch->IRQ.NOTIFY.U = 0xFU;
        guimai_gtm_afd_dma_arm();
    }
}

static void guimai_gtm_afd_dma_stop(void)
{
    Ifx_GTM_PSM_FIFO_CH *fifo_ch = IfxGtm_Psm_Fifo_getChannelPointer(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0);

    if (!s_gtm_afd_dma_ready)
    {
        return;
    }

    guimai_gtm_afd_dma_poll();
    IfxDma_disableChannelTransaction(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL);
    IfxGtm_Psm_Fifo_disableChannelInterrupt(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0, IfxGtm_Psm_FifoChannelInterrupt_upperWm);
    fifo_ch->IRQ.NOTIFY.U = 0xFU;
    s_gtm_afd_dma_active = 0U;
}

static void guimai_gtm_afd_dma_print(void)
{
    uint32 i = 0U;
    uint32 nonzero = 0U;
    uint32 changed = 0U;
    uint32 last = s_gtm_afd_dma_words[0];
    uint32 hist_start = 0U;
    uint32 hist[GUIMAI_GTM_AFD_DMA_HISTORY_WORDS];
    Ifx_GTM_PSM_FIFO_CH *fifo_ch = IfxGtm_Psm_Fifo_getChannelPointer(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0);

    for (i = 0U; i < GUIMAI_GTM_AFD_DMA_WORDS; i++)
    {
        uint32 word = s_gtm_afd_dma_words[i];
        if (word != 0U)
        {
            nonzero++;
        }
        if ((i > 0U) && (word != last))
        {
            changed++;
        }
        last = word;
    }
    if (s_gtm_afd_dma_history_write >= GUIMAI_GTM_AFD_DMA_HISTORY_WORDS)
    {
        hist_start = s_gtm_afd_dma_history_write - GUIMAI_GTM_AFD_DMA_HISTORY_WORDS;
    }
    for (i = 0U; i < GUIMAI_GTM_AFD_DMA_HISTORY_WORDS; i++)
    {
        hist[i] = s_gtm_afd_dma_history[(hist_start + i) & (GUIMAI_GTM_AFD_DMA_HISTORY_WORDS - 1U)];
    }

    printf("[GUIMAI_AFD_DMA] done=%u chunks=%lu words=%lu lost=%lu nonzero=%lu/%lu changed=%lu/%lu last=0x%08lX tcnt=%lu chcsr=0x%08lX src=0x%08lX irq=0x%08lX fill=%lu rd=%lu wr=%lu\r\n",
           (unsigned)s_gtm_afd_dma_done,
           (unsigned long)s_gtm_afd_dma_chunks,
           (unsigned long)s_gtm_afd_dma_words_total,
           (unsigned long)s_gtm_afd_dma_lost,
           (unsigned long)s_gtm_afd_dma_nonzero_total,
           (unsigned long)nonzero,
           (unsigned long)s_gtm_afd_dma_changed_total,
           (unsigned long)changed,
           (unsigned long)s_gtm_afd_dma_last_word,
           (unsigned long)s_gtm_afd_dma_tcnt,
           (unsigned long)s_gtm_afd_dma_chcsr,
           (unsigned long)s_gtm_afd_dma_src,
           (unsigned long)s_gtm_afd_dma_irq_notify,
           (unsigned long)IfxGtm_Psm_Fifo_getChannelFillLevel(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0),
           (unsigned long)IfxGtm_Psm_Fifo_getChannelReadPtr(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0),
           (unsigned long)IfxGtm_Psm_Fifo_getChannelWritePtr(GUIMAI_GTM_ARU_FIFO_PSM, IfxGtm_Psm_FifoChannel_0));
    printf("[GUIMAI_AFD_DMA_HIST] n=%lu w0-7=%08lX %08lX %08lX %08lX %08lX %08lX %08lX %08lX\r\n",
           (unsigned long)s_gtm_afd_dma_history_write,
           (unsigned long)hist[0], (unsigned long)hist[1],
           (unsigned long)hist[2], (unsigned long)hist[3],
           (unsigned long)hist[4], (unsigned long)hist[5],
           (unsigned long)hist[6], (unsigned long)hist[7]);
    printf("[GUIMAI_AFD_DMA_HIST] w8-15=%08lX %08lX %08lX %08lX %08lX %08lX %08lX %08lX\r\n",
           (unsigned long)hist[8], (unsigned long)hist[9],
           (unsigned long)hist[10], (unsigned long)hist[11],
           (unsigned long)hist[12], (unsigned long)hist[13],
           (unsigned long)hist[14], (unsigned long)hist[15]);
    printf("[GUIMAI_AFD_DMA_HIST] w16-23=%08lX %08lX %08lX %08lX %08lX %08lX %08lX %08lX\r\n",
           (unsigned long)hist[16], (unsigned long)hist[17],
           (unsigned long)hist[18], (unsigned long)hist[19],
           (unsigned long)hist[20], (unsigned long)hist[21],
           (unsigned long)hist[22], (unsigned long)hist[23]);
    printf("[GUIMAI_AFD_DMA_HIST] w24-31=%08lX %08lX %08lX %08lX %08lX %08lX %08lX %08lX\r\n",
           (unsigned long)hist[24], (unsigned long)hist[25],
           (unsigned long)hist[26], (unsigned long)hist[27],
           (unsigned long)hist[28], (unsigned long)hist[29],
           (unsigned long)hist[30], (unsigned long)hist[31]);
    printf("[GUIMAI_AFD_DMA] w0-7=%08lX %08lX %08lX %08lX %08lX %08lX %08lX %08lX\r\n",
           (unsigned long)s_gtm_afd_dma_words[0],
           (unsigned long)s_gtm_afd_dma_words[1],
           (unsigned long)s_gtm_afd_dma_words[2],
           (unsigned long)s_gtm_afd_dma_words[3],
           (unsigned long)s_gtm_afd_dma_words[4],
           (unsigned long)s_gtm_afd_dma_words[5],
           (unsigned long)s_gtm_afd_dma_words[6],
           (unsigned long)s_gtm_afd_dma_words[7]);
    printf("[GUIMAI_AFD_DMA] w%u-%u=%08lX %08lX %08lX %08lX %08lX %08lX %08lX %08lX\r\n",
           (unsigned)(GUIMAI_GTM_AFD_DMA_WORDS - 8U),
           (unsigned)(GUIMAI_GTM_AFD_DMA_WORDS - 1U),
           (unsigned long)s_gtm_afd_dma_words[GUIMAI_GTM_AFD_DMA_WORDS - 8U],
           (unsigned long)s_gtm_afd_dma_words[GUIMAI_GTM_AFD_DMA_WORDS - 7U],
           (unsigned long)s_gtm_afd_dma_words[GUIMAI_GTM_AFD_DMA_WORDS - 6U],
           (unsigned long)s_gtm_afd_dma_words[GUIMAI_GTM_AFD_DMA_WORDS - 5U],
           (unsigned long)s_gtm_afd_dma_words[GUIMAI_GTM_AFD_DMA_WORDS - 4U],
           (unsigned long)s_gtm_afd_dma_words[GUIMAI_GTM_AFD_DMA_WORDS - 3U],
           (unsigned long)s_gtm_afd_dma_words[GUIMAI_GTM_AFD_DMA_WORDS - 2U],
           (unsigned long)s_gtm_afd_dma_words[GUIMAI_GTM_AFD_DMA_WORDS - 1U]);
#if GUIMAI_GTM_TBCM_DECODE_DIAG
    guimai_gtm_tbcm_decode_print();
#endif
}
#endif

static void guimai_gtm_aru_fifo_init(void)
{
    Ifx_GTM_PSM_F2A *f2a = IfxGtm_Psm_F2a_getPointer(GUIMAI_GTM_ARU_FIFO_F2A);
    uint32 i = 0U;

    if (s_gtm_aru_fifo_ready)
    {
        return;
    }

    for (i = 0U; i < GUIMAI_GTM_ARU_FIFO_COUNT; i++)
    {
        uint32 start = i * GUIMAI_GTM_ARU_FIFO_SIZE;
        Ifx_GTM_PSM_FIFO_CH *fifo_ch = IfxGtm_Psm_Fifo_getChannelPointer(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i]);

        IfxGtm_Psm_F2a_disableStream(GUIMAI_GTM_ARU_FIFO_F2A, s_gtm_aru_fifo_stream[i]);
        IfxGtm_Psm_Fifo_setChannelMode(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i], IfxGtm_Psm_FifoChannelMode_normal);
        IfxGtm_Psm_Fifo_setChannelStartAddress(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i], start);
        IfxGtm_Psm_Fifo_setChannelSize(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i], GUIMAI_GTM_ARU_FIFO_SIZE);
        IfxGtm_Psm_Fifo_setChannelLowerWatermark(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i], 1U);
        IfxGtm_Psm_Fifo_setChannelUpperWatermark(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i], GUIMAI_GTM_ARU_FIFO_SIZE - 2U);
        IfxGtm_Psm_Fifo_clearAllChannelInterrupts(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i]);
        IfxGtm_Psm_Fifo_flushChannelFifo(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i]);

        IfxGtm_Psm_F2a_setAruReadAddress(GUIMAI_GTM_ARU_FIFO_F2A, s_gtm_aru_fifo_stream[i], s_gtm_aru_fifo_src_addr[i]);
        IfxGtm_Psm_F2a_setTransferDirection(GUIMAI_GTM_ARU_FIFO_F2A, s_gtm_aru_fifo_stream[i], IfxGtm_Psm_F2aTransferDirection_aruToFifo);
        IfxGtm_Psm_F2a_setTransferMode(GUIMAI_GTM_ARU_FIFO_F2A, s_gtm_aru_fifo_stream[i], GUIMAI_GTM_ARU_FIFO_TRANSFER_MODE);

        printf("[GUIMAI_ARU_FIFO] init idx=%lu psm=0 f2a=0 stream=%lu fifo=%lu src=0x%03lX tmode=%lu start=%lu size=%lu end=%lu lwm=%lu uwm=%lu str=0x%08lX fifo_ctrl=0x%08lX\r\n",
               (unsigned long)i,
               (unsigned long)s_gtm_aru_fifo_stream[i],
               (unsigned long)s_gtm_aru_fifo_channel[i],
               (unsigned long)s_gtm_aru_fifo_src_addr[i],
               (unsigned long)IfxGtm_Psm_F2a_getTransferMode(GUIMAI_GTM_ARU_FIFO_F2A, s_gtm_aru_fifo_stream[i]),
               (unsigned long)IfxGtm_Psm_Fifo_getChannelStartAddress(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i]),
               (unsigned long)IfxGtm_Psm_Fifo_getChannelSize(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i]),
               (unsigned long)IfxGtm_Psm_Fifo_getChannelEndAddress(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i]),
               (unsigned long)IfxGtm_Psm_Fifo_getChannelLowerWatermark(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i]),
               (unsigned long)IfxGtm_Psm_Fifo_getChannelUpperWatermark(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i]),
               (unsigned long)f2a->STR_CH[s_gtm_aru_fifo_stream[i]].STR_CFG.U,
               (unsigned long)fifo_ch->CTRL.U);
    }

    s_gtm_aru_fifo_ready = 1U;
#if GUIMAI_GTM_AFD_DMA_DIAG
    guimai_gtm_afd_dma_init();
#endif
    guimai_gtm_aru_fifo_capture_state();
    printf("[GUIMAI_ARU_FIFO] init done count=%lu enable=0x%08lX\r\n",
           (unsigned long)GUIMAI_GTM_ARU_FIFO_COUNT,
           (unsigned long)f2a->ENABLE.U);
}

static void guimai_gtm_aru_fifo_start(void)
{
    uint32 i = 0U;

    if (!s_gtm_aru_fifo_ready)
    {
        return;
    }

    s_gtm_aru_fifo_polls = 0U;
#if !GUIMAI_GTM_AFD_DMA_DIAG
    guimai_gtm_afd_live_reset();
#endif
    for (i = 0U; i < GUIMAI_GTM_ARU_FIFO_COUNT; i++)
    {
        s_gtm_aru_fifo_max_fill[i] = 0U;
        IfxGtm_Psm_Fifo_clearAllChannelInterrupts(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i]);
        IfxGtm_Psm_Fifo_flushChannelFifo(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i]);
    }
#if GUIMAI_GTM_AFD_DMA_DIAG
    guimai_gtm_afd_dma_start();
#endif
    for (i = 0U; i < GUIMAI_GTM_ARU_FIFO_COUNT; i++)
    {
        IfxGtm_Psm_F2a_enableStream(GUIMAI_GTM_ARU_FIFO_F2A, s_gtm_aru_fifo_stream[i]);
    }
    s_gtm_aru_fifo_active = 1U;
    guimai_gtm_aru_fifo_capture_state();
    printf("[GUIMAI_ARU_FIFO] start enable=0x%08lX s0=%lu f0=%lu s1=%lu f1=%lu s2=%lu f2=%lu\r\n",
           (unsigned long)s_gtm_aru_fifo_enable,
           (unsigned long)s_gtm_aru_fifo_stream_state[0],
           (unsigned long)s_gtm_aru_fifo_fill[0],
           (unsigned long)s_gtm_aru_fifo_stream_state[1],
           (unsigned long)s_gtm_aru_fifo_fill[1],
           (unsigned long)s_gtm_aru_fifo_stream_state[2],
           (unsigned long)s_gtm_aru_fifo_fill[2]);
}

static void guimai_gtm_aru_fifo_stop(void)
{
    uint32 i = 0U;

    if (s_gtm_aru_fifo_ready)
    {
        guimai_gtm_aru_fifo_capture_state();
#if GUIMAI_GTM_AFD_DMA_DIAG
        guimai_gtm_afd_dma_stop();
        guimai_gtm_afd_dma_print();
#else
        guimai_gtm_afd_live_drain_fifo0(GUIMAI_GTM_AFD_LIVE_DRAIN_WORDS);
        guimai_gtm_afd_live_print();
        guimai_gtm_afd_dump_fifo0();
#endif
        for (i = 0U; i < GUIMAI_GTM_ARU_FIFO_COUNT; i++)
        {
            IfxGtm_Psm_F2a_disableStream(GUIMAI_GTM_ARU_FIFO_F2A, s_gtm_aru_fifo_stream[i]);
            printf("[GUIMAI_ARU_FIFO] stop idx=%lu fill=%lu max=%lu wr=%lu rd=%lu status=%lu irq=0x%lX state=%lu str=0x%08lX aru=0x%03lX polls=%lu\r\n",
                   (unsigned long)i,
                   (unsigned long)s_gtm_aru_fifo_fill[i],
                   (unsigned long)s_gtm_aru_fifo_max_fill[i],
                   (unsigned long)s_gtm_aru_fifo_wr[i],
                   (unsigned long)s_gtm_aru_fifo_rd[i],
                   (unsigned long)s_gtm_aru_fifo_status[i],
                   (unsigned long)s_gtm_aru_fifo_irq[i],
                   (unsigned long)s_gtm_aru_fifo_stream_state[i],
                   (unsigned long)s_gtm_aru_fifo_str_cfg[i],
                   (unsigned long)s_gtm_aru_fifo_aru_addr[i],
                   (unsigned long)s_gtm_aru_fifo_polls);
        }
    }
    s_gtm_aru_fifo_active = 0U;
}

static void guimai_gtm_aru_fifo_poll_diag(void)
{
    uint32 i = 0U;

    if (!s_gtm_aru_fifo_active)
    {
        return;
    }

    s_gtm_aru_fifo_polls++;
    guimai_gtm_aru_fifo_capture_state();
#if GUIMAI_GTM_AFD_DMA_DIAG
    guimai_gtm_afd_dma_poll();
#else
    guimai_gtm_afd_live_drain_fifo0(GUIMAI_GTM_AFD_LIVE_DRAIN_WORDS);
#endif
    for (i = 0U; i < GUIMAI_GTM_ARU_FIFO_COUNT; i++)
    {
#if GUIMAI_GTM_AFD_DMA_DIAG
        if (s_gtm_aru_fifo_channel[i] == IfxGtm_Psm_FifoChannel_0)
        {
            continue;
        }
#endif
        if (s_gtm_aru_fifo_irq[i] != 0U)
        {
            Ifx_GTM_PSM_FIFO_CH *fifo_ch = IfxGtm_Psm_Fifo_getChannelPointer(GUIMAI_GTM_ARU_FIFO_PSM, s_gtm_aru_fifo_channel[i]);
            fifo_ch->IRQ.NOTIFY.U = s_gtm_aru_fifo_irq[i] & 0xFU;
        }
    }
}
#endif

static void guimai_gtm_tbcm_init(void)
{
    Ifx_GTM_TIM_CH *tim_ch = NULL_PTR;
#if GUIMAI_GTM_TBCM_ENABLE_BCK_PREV_CH
    Ifx_GTM_TIM_CH *bck_ch = NULL_PTR;
#endif
    uint16 password = 0;
    uint32 cnts = 0U;

    if (s_gtm_tbcm_ready)
    {
        return;
    }

    printf("[GUIMAI_TBCM] init begin\r\n");
    password = IfxScuWdt_getCpuWatchdogPassword();
    IfxScuWdt_clearCpuEndinit(password);
    IfxGtm_enable(&MODULE_GTM);
    IfxScuWdt_setCpuEndinit(password);
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0 | IFXGTM_CMU_CLKEN_FXCLK);
    printf("[GUIMAI_TBCM] gtm clock ok\r\n");

    IfxGtm_PinMap_setTimTin(&IfxGtm_TIM2_1_P11_2_IN, IfxPort_InputMode_noPullDevice);
    printf("[GUIMAI_TBCM] pin WS TIM2_1 ok PISEL=0x%08lX\r\n", (unsigned long)MODULE_GTM.TIMINSEL[IfxGtm_Tim_2].U);
    IfxGtm_PinMap_setTimTin(&IfxGtm_TIM2_3_P11_6_IN, IfxPort_InputMode_noPullDevice);
    printf("[GUIMAI_TBCM] pin BCK TIM2_3 ok PISEL=0x%08lX\r\n", (unsigned long)MODULE_GTM.TIMINSEL[IfxGtm_Tim_2].U);
    IfxGtm_PinMap_setTimTin(&IfxGtm_TIM2_4_P11_9_IN, IfxPort_InputMode_noPullDevice);
    printf("[GUIMAI_TBCM] pin SD TIM2_4 ok PISEL=0x%08lX\r\n", (unsigned long)MODULE_GTM.TIMINSEL[IfxGtm_Tim_2].U);

#if GUIMAI_GTM_TBCM_ENABLE_BCK_PREV_CH
    bck_ch = IfxGtm_Tim_getChannel(&MODULE_GTM.TIM[IfxGtm_Tim_2], IfxGtm_Tim_Ch_3);
    bck_ch->CTRL.B.TIM_EN = 0U;
    bck_ch->CTRL.U = 0U;
    bck_ch->ECTRL.U = 0U;
    bck_ch->IRQ.EN.U = 0U;
    bck_ch->IRQ.NOTIFY.U = 0x3FU;
    bck_ch->CTRL.B.TIM_MODE = IfxGtm_Tim_Mode_inputEvent;
    bck_ch->CTRL.B.CLK_SEL = IfxGtm_Cmu_Clk_0;
    bck_ch->CTRL.B.CNTS_SEL = 1U;
    bck_ch->CTRL.B.GPR0_SEL = IfxGtm_Tim_GprSel_cnts;
    bck_ch->CTRL.B.GPR1_SEL = IfxGtm_Tim_GprSel_cnts;
    bck_ch->CTRL.B.DSL = GUIMAI_GTM_TBCM_SAMPLE_LEADING_EDGE ? 1U : 0U;
    bck_ch->CTRL.B.ISL = 0U;
    bck_ch->CTRL.B.FLT_EN = 0U;
    bck_ch->CTRL.B.TIM_EN = 1U;
    printf("[GUIMAI_TBCM] bck prev ch TIM2_3 enabled CTRL=0x%08lX\r\n", (unsigned long)bck_ch->CTRL.U);
#endif

    tim_ch = IfxGtm_Tim_getChannel(&MODULE_GTM.TIM[IfxGtm_Tim_2], IfxGtm_Tim_Ch_4);
    s_gtm_tbcm_ch = tim_ch;

    tim_ch->CTRL.B.TIM_EN = 0U;
    tim_ch->CTRL.U = 0U;
    tim_ch->ECTRL.U = 0U;
    tim_ch->IRQ.EN.U = 0U;
    tim_ch->IRQ.NOTIFY.U = 0x3FU;
    printf("[GUIMAI_TBCM] tim regs cleared\r\n");

#if GUIMAI_GTM_TIM_TSSM_DIAG
    cnts = GUIMAI_GTM_TIM_TSSM_BITS |
           ((uint32)GUIMAI_GTM_TIM_TSSM_SHIFT_CLK_SEL << 16U) |
           ((uint32)GUIMAI_GTM_TIM_TSSM_EXT_CAP_SEL << 18U);
#elif GUIMAI_GTM_TBCM_SAMPLE_LEADING_EDGE
    cnts = (1U << 3U);
#else
    cnts = (1U << (8U + 3U));
#endif

#if GUIMAI_GTM_TIM_TSSM_DIAG
    tim_ch->CTRL.B.TIM_MODE = GUIMAI_GTM_TIM_TSSM_MODE;
#else
    tim_ch->CTRL.B.TIM_MODE = IfxGtm_Tim_Mode_bitCompression;
#endif
#if GUIMAI_GTM_ARU_FIFO_DIAG
    tim_ch->CTRL.B.ARU_EN = 1U;
#else
    tim_ch->CTRL.B.ARU_EN = 0U;
#endif
    tim_ch->CTRL.B.CICTRL = 0U;
    tim_ch->CTRL.B.OSM = 0U;
    tim_ch->CTRL.B.FLT_EN = 0U;
    tim_ch->CTRL.B.EXT_CAP_EN = GUIMAI_GTM_TIM_TSSM_EXT_CAP_EN ? 1U : 0U;
    tim_ch->CTRL.B.CNTS_SEL = GUIMAI_GTM_TIM_TSSM_USE_TIM_IN ? 1U : 0U;
    tim_ch->CTRL.B.GPR0_SEL = IfxGtm_Tim_GprSel_cnts;
    tim_ch->CTRL.B.GPR1_SEL = IfxGtm_Tim_GprSel_cnts;
    tim_ch->CTRL.B.EGPR0_SEL = 0U;
    tim_ch->CTRL.B.EGPR1_SEL = 0U;
    tim_ch->CTRL.B.DSL = GUIMAI_GTM_TIM_TSSM_SHIFT_RIGHT ? 1U : 0U;
    tim_ch->CTRL.B.ISL = GUIMAI_GTM_TIM_TSSM_RESET_ON_CAPTURE ? 1U : 0U;
    tim_ch->CTRL.B.ECNT_RESET = GUIMAI_GTM_TIM_TSSM_INIT_POL ? 1U : 0U;
    tim_ch->CTRL.B.CLK_SEL = IfxGtm_Cmu_Clk_0;
#if GUIMAI_GTM_TIM_TSSM_DIAG
    tim_ch->TDUV.U = 0U;
    tim_ch->TDUC.U = 0U;
#if GUIMAI_GTM_TIM_TSSM_TDU_DIAG
    tim_ch->CTRL.B.TOCTRL = IfxGtm_Tim_Timeout_disabled;
    tim_ch->ECTRL.B.USE_PREV_TDU_IN = 1U;
    tim_ch->ECTRL.B.TDU_START = 0U;
    tim_ch->ECTRL.B.TDU_STOP = 0U;
    tim_ch->ECTRL.B.TDU_RESYNC = GUIMAI_GTM_TIM_TSSM_TDU_RESYNC;
    tim_ch->ECTRL.B.ECLK_SEL = GUIMAI_GTM_TIM_TSSM_ECLK_SEL ? 1U : 0U;
    tim_ch->TDUV.B.TOV = GUIMAI_GTM_TIM_TSSM_TOV;
    tim_ch->TDUV.B.TOV1 = GUIMAI_GTM_TIM_TSSM_TOV1;
    tim_ch->TDUV.B.TOV2 = GUIMAI_GTM_TIM_TSSM_TOV2;
    tim_ch->TDUV.B.SLICING = GUIMAI_GTM_TIM_TSSM_TDU_SLICING;
    tim_ch->TDUV.B.TCS_USE_SAMPLE_EVT = GUIMAI_GTM_TIM_TSSM_TCS_USE_SAMPLE_EVT;
    tim_ch->TDUV.B.TDU_SAME_CNT_CLK = GUIMAI_GTM_TIM_TSSM_TDU_SAME_CNT_CLK;
    tim_ch->TDUV.B.TCS = IfxGtm_Cmu_Clk_0;
#endif
#endif
    printf("[GUIMAI_TBCM] ctrl configured\r\n");

    IfxGtm_Tim_Ch_setShadowCounter(tim_ch, cnts);
    printf("[GUIMAI_TBCM] cnts configured CNTS=0x%08lX\r\n", (unsigned long)tim_ch->CNTS.U);

    s_gtm_tbcm_ready = 1U;
    printf("[GUIMAI_TBCM] init tim=TIM2_4 sample=BCK_%s mode=%u tssm=%u tdu=%u shift=%s init_pol=%u isl=%u ext_cap_en=%u aru=%u gpr=%u/%u cnts_sel=%u tim_in=%u cnts=0x%06lX bits=%u shift_clk=%u ext_cap=%u toctrl=%u prev_tdu=%u tdu_start=%u tdu_stop=%u tdu_resync=%u eclk_sel=%u slicing=%u tcs_sample=%u sameclk=%u tov=%u/%u/%u PISEL=0x%08lX CTRL=0x%08lX ECTRL=0x%08lX TDUV=0x%08lX TDUC=0x%08lX\r\n",
           GUIMAI_GTM_TBCM_SAMPLE_LEADING_EDGE ? "rising" : "falling",
           (unsigned)tim_ch->CTRL.B.TIM_MODE,
           (unsigned)GUIMAI_GTM_TIM_TSSM_DIAG,
           (unsigned)GUIMAI_GTM_TIM_TSSM_TDU_DIAG,
           GUIMAI_GTM_TIM_TSSM_SHIFT_RIGHT ? "right" : "left",
           (unsigned)tim_ch->CTRL.B.ECNT_RESET,
           (unsigned)tim_ch->CTRL.B.ISL,
           (unsigned)tim_ch->CTRL.B.EXT_CAP_EN,
           (unsigned)tim_ch->CTRL.B.ARU_EN,
           (unsigned)tim_ch->CTRL.B.GPR0_SEL,
           (unsigned)tim_ch->CTRL.B.GPR1_SEL,
           (unsigned)tim_ch->CTRL.B.CNTS_SEL,
           (unsigned)GUIMAI_GTM_TIM_TSSM_USE_TIM_IN,
           (unsigned long)cnts,
           (unsigned)GUIMAI_GTM_TIM_TSSM_BITS,
           (unsigned)GUIMAI_GTM_TIM_TSSM_SHIFT_CLK_SEL,
           (unsigned)GUIMAI_GTM_TIM_TSSM_EXT_CAP_SEL,
           (unsigned)tim_ch->CTRL.B.TOCTRL,
           (unsigned)tim_ch->ECTRL.B.USE_PREV_TDU_IN,
           (unsigned)tim_ch->ECTRL.B.TDU_START,
           (unsigned)tim_ch->ECTRL.B.TDU_STOP,
           (unsigned)tim_ch->ECTRL.B.TDU_RESYNC,
           (unsigned)tim_ch->ECTRL.B.ECLK_SEL,
           (unsigned)tim_ch->TDUV.B.SLICING,
           (unsigned)tim_ch->TDUV.B.TCS_USE_SAMPLE_EVT,
           (unsigned)tim_ch->TDUV.B.TDU_SAME_CNT_CLK,
           (unsigned)tim_ch->TDUV.B.TOV,
           (unsigned)tim_ch->TDUV.B.TOV1,
           (unsigned)tim_ch->TDUV.B.TOV2,
           (unsigned long)MODULE_GTM.TIMINSEL[IfxGtm_Tim_2].U,
           (unsigned long)tim_ch->CTRL.U,
           (unsigned long)tim_ch->ECTRL.U,
           (unsigned long)tim_ch->TDUV.U,
           (unsigned long)tim_ch->TDUC.U);
#if GUIMAI_GTM_ARU_FIFO_DIAG
    guimai_gtm_aru_fifo_init();
#endif
}

static void guimai_gtm_tbcm_start(void)
{
    if (!s_gtm_tbcm_ready || (s_gtm_tbcm_ch == NULL_PTR))
    {
        return;
    }

    s_gtm_tbcm_polls = 0U;
    s_gtm_tbcm_events = 0U;
    s_gtm_tbcm_newval = 0U;
    s_gtm_tbcm_gprofl = 0U;
    s_gtm_tbcm_ws_one = 0U;
    s_gtm_tbcm_sd_one = 0U;
    s_gtm_tbcm_last_gpr0 = 0U;
    s_gtm_tbcm_last_gpr1 = 0U;
    s_gtm_tbcm_last_cnts = s_gtm_tbcm_ch->CNTS.U;
    s_gtm_tbcm_last_irq = 0U;
    s_gtm_tbcm_last_ecnt = 0U;

    s_gtm_tbcm_ch->IRQ.NOTIFY.U = 0x3FU;
#if GUIMAI_GTM_ARU_FIFO_DIAG
    guimai_gtm_aru_fifo_start();
#endif
#if GUIMAI_GTM_TIM_TSSM_DIAG && GUIMAI_GTM_TIM_TSSM_TDU_DIAG
    s_gtm_tbcm_ch->CTRL.B.TOCTRL = IfxGtm_Tim_Timeout_disabled;
#endif
    s_gtm_tbcm_ch->CTRL.B.TIM_EN = 1U;
#if GUIMAI_GTM_TIM_TSSM_DIAG && GUIMAI_GTM_TIM_TSSM_TDU_DIAG
    s_gtm_tbcm_ch->CTRL.B.TOCTRL = IfxGtm_Tim_Timeout_risingEdge;
#endif
    s_gtm_tbcm_active = 1U;
}

static void guimai_gtm_tbcm_stop(void)
{
    if (s_gtm_tbcm_ch != NULL_PTR)
    {
#if GUIMAI_GTM_TIM_TSSM_DIAG && GUIMAI_GTM_TIM_TSSM_TDU_DIAG
        s_gtm_tbcm_ch->CTRL.B.TOCTRL = IfxGtm_Tim_Timeout_disabled;
#endif
        s_gtm_tbcm_ch->CTRL.B.TIM_EN = 0U;
        s_gtm_tbcm_ch->IRQ.EN.U = 0U;
    }
    s_gtm_tbcm_active = 0U;
#if GUIMAI_GTM_ARU_FIFO_DIAG
    guimai_gtm_aru_fifo_stop();
#endif
}

static uint32 guimai_gtm_tbcm_read_samples(int16 *dst, uint32 max_samples)
{
    uint32 irq = 0U;
    uint32 gpr1 = 0U;

    (void)dst;
    (void)max_samples;

    if (!s_gtm_tbcm_ready || !s_gtm_tbcm_active || (s_gtm_tbcm_ch == NULL_PTR))
    {
        return 0U;
    }

#if GUIMAI_GTM_ARU_FIFO_DIAG
    guimai_gtm_aru_fifo_poll_diag();
#endif

    s_gtm_tbcm_polls++;
    irq = s_gtm_tbcm_ch->IRQ.NOTIFY.U;
    s_gtm_tbcm_last_irq = irq;
    s_gtm_tbcm_last_cnts = s_gtm_tbcm_ch->CNTS.U;
    s_gtm_tbcm_last_ecnt = (uint8)s_gtm_tbcm_ch->CNTS.B.ECNT;

    if ((irq & (1U << IfxGtm_Tim_IrqType_newVal)) != 0U)
    {
        s_gtm_tbcm_events++;
        s_gtm_tbcm_newval++;
        s_gtm_tbcm_last_gpr0 = s_gtm_tbcm_ch->GPR0.U;
        s_gtm_tbcm_last_gpr1 = s_gtm_tbcm_ch->GPR1.U;
        gpr1 = s_gtm_tbcm_last_gpr1;
        s_gtm_tbcm_ws_one += (gpr1 >> 1U) & 1U;
        s_gtm_tbcm_sd_one += (gpr1 >> 4U) & 1U;
    }

    if ((irq & (1U << IfxGtm_Tim_IrqType_gprOverflow)) != 0U)
    {
        s_gtm_tbcm_gprofl++;
    }

    if (irq != 0U)
    {
        s_gtm_tbcm_ch->IRQ.NOTIFY.U = irq & 0x3FU;
    }

    return 0U;
}

static void guimai_gtm_tbcm_no_event_diag(void)
{
    uint32 bck_toggles = guimai_pin_toggle_count(HORN_BCK_PIN, GUIMAI_IIS_SCAN_LOOPS);
    uint32 ws_toggles = guimai_pin_toggle_count(HORN_WS_PIN, GUIMAI_IIS_SCAN_LOOPS);
    uint32 sd_toggles = guimai_pin_toggle_count(ES8388_ADC_SDOUT_PIN, GUIMAI_IIS_SCAN_LOOPS);
    uint32 mclk_toggles = guimai_pin_toggle_count(HORN_MCLK_PIN, GUIMAI_IIS_SCAN_LOOPS);
    uint32 timinsel = MODULE_GTM.TIMINSEL[IfxGtm_Tim_2].U;
    uint32 ctrl = (s_gtm_tbcm_ch != NULL_PTR) ? s_gtm_tbcm_ch->CTRL.U : 0U;
    uint32 ectrl = (s_gtm_tbcm_ch != NULL_PTR) ? s_gtm_tbcm_ch->ECTRL.U : 0U;
    uint32 cnts = (s_gtm_tbcm_ch != NULL_PTR) ? s_gtm_tbcm_ch->CNTS.U : 0U;
    uint32 irq = (s_gtm_tbcm_ch != NULL_PTR) ? s_gtm_tbcm_ch->IRQ.NOTIFY.U : 0U;
    uint32 gpr0 = (s_gtm_tbcm_ch != NULL_PTR) ? s_gtm_tbcm_ch->GPR0.U : 0U;
    uint32 gpr1 = (s_gtm_tbcm_ch != NULL_PTR) ? s_gtm_tbcm_ch->GPR1.U : 0U;

    printf("[GUIMAI_TBCM_NOEVT] raw bck=%lu ws=%lu sd=%lu mclk=%lu lvl=%u/%u/%u/%u timinsel=0x%08lX ctrl=0x%08lX ectrl=0x%08lX cnts=0x%08lX irq=0x%02lX gpr0=0x%08lX gpr1=0x%08lX\r\n",
           (unsigned long)bck_toggles,
           (unsigned long)ws_toggles,
           (unsigned long)sd_toggles,
           (unsigned long)mclk_toggles,
           (unsigned)gpio_get_level(HORN_BCK_PIN),
           (unsigned)gpio_get_level(HORN_WS_PIN),
           (unsigned)gpio_get_level(ES8388_ADC_SDOUT_PIN),
           (unsigned)gpio_get_level(HORN_MCLK_PIN),
           (unsigned long)timinsel,
           (unsigned long)ctrl,
           (unsigned long)ectrl,
           (unsigned long)cnts,
           (unsigned long)irq,
           (unsigned long)gpr0,
           (unsigned long)gpr1);
}
#endif

static uint32 guimai_pin_toggle_count(gpio_pin_enum pin, uint32 loops)
{
    uint32 i = 0;
    uint32 toggles = 0;
    uint8 last = gpio_get_level(pin);

    for (i = 0; i < loops; i++)
    {
        uint8 cur = gpio_get_level(pin);
        if (cur != last)
        {
            toggles++;
            last = cur;
        }
    }
    return toggles;
}

#if GUIMAI_BCK_PORT_DIAG || GUIMAI_BCK_CLOCK_MISS_DIAG
static uint32 guimai_pin_iocr_value(Ifx_P *port, uint8 pin_index)
{
    switch (pin_index / 4U)
    {
        case 0U: return port->IOCR0.U;
        case 1U: return port->IOCR4.U;
        case 2U: return port->IOCR8.U;
        default: return port->IOCR12.U;
    }
}

static uint32 guimai_pin_pc_value(Ifx_P *port, uint8 pin_index)
{
    uint32 iocr = guimai_pin_iocr_value(port, pin_index);
    uint8 shift = (uint8)(3U + ((pin_index & 0x03U) * 8U));
    return (iocr >> shift) & 0x1FU;
}

static uint32 guimai_pin_pdr_value(Ifx_P *port, uint8 pin_index)
{
    uint32 pdr = (pin_index < 8U) ? port->PDR0.U : port->PDR1.U;
    uint8 shift = (uint8)((pin_index & 0x07U) * 4U);
    return (pdr >> shift) & 0x0FU;
}

static const char *guimai_gpio_mode_name(gpio_mode_enum mode)
{
    switch (mode)
    {
        case GPI_FLOATING_IN: return "FLOAT";
        case GPI_PULL_DOWN: return "PULL_DOWN";
        case GPI_PULL_UP: return "PULL_UP";
        default: return "UNKNOWN";
    }
}

static void guimai_log_pin_regs(const char *tag, gpio_pin_enum pin)
{
    Ifx_P *port = get_port(pin);
    uint8 pin_index = (uint8)(pin & 0x1FU);
    printf("[PIN_DIAG] %s P%d_%d pc=0x%02lX iocr=0x%08lX pdisc=%lu pdr=0x%lX in=0x%08lX out=0x%08lX lvl=%u\r\n",
           tag,
           (unsigned)(pin / 32),
           (unsigned)(pin % 32),
           (unsigned long)guimai_pin_pc_value(port, pin_index),
           (unsigned long)guimai_pin_iocr_value(port, pin_index),
           (unsigned long)((port->PDISC.U >> pin_index) & 0x01U),
           (unsigned long)guimai_pin_pdr_value(port, pin_index),
           (unsigned long)port->IN.U,
           (unsigned long)port->OUT.U,
           (unsigned)gpio_get_level(pin));
}

static void guimai_bck_input_mode_probe(const char *tag)
{
    const gpio_mode_enum modes[] = {
        GPI_FLOATING_IN,
        GPI_PULL_DOWN,
        GPI_PULL_UP
    };
    uint32 i = 0;

    printf("[BCK_DIAG] %s begin\r\n", tag);
    guimai_log_pin_regs("BCK", HORN_BCK_PIN);
    guimai_log_pin_regs("LRCK", HORN_WS_PIN);
    guimai_log_pin_regs("SDOUT", ES8388_ADC_SDOUT_PIN);
    guimai_log_pin_regs("MCLK", HORN_MCLK_PIN);

    printf("[BCK_DIAG] %s raw bck=%lu/%lu ws=%lu sd=%lu mclk=%lu lvl=%u/%u/%u/%u\r\n",
           tag,
           (unsigned long)guimai_pin_toggle_count(HORN_BCK_PIN, GUIMAI_IIS_SCAN_LOOPS),
           (unsigned long)guimai_pin_toggle_count(HORN_BCK_PIN, GUIMAI_IIS_SCAN_LOOPS * 10U),
           (unsigned long)guimai_pin_toggle_count(HORN_WS_PIN, GUIMAI_IIS_SCAN_LOOPS),
           (unsigned long)guimai_pin_toggle_count(ES8388_ADC_SDOUT_PIN, GUIMAI_IIS_SCAN_LOOPS),
           (unsigned long)guimai_pin_toggle_count(HORN_MCLK_PIN, GUIMAI_IIS_SCAN_LOOPS),
           (unsigned)gpio_get_level(HORN_BCK_PIN),
           (unsigned)gpio_get_level(HORN_WS_PIN),
           (unsigned)gpio_get_level(ES8388_ADC_SDOUT_PIN),
           (unsigned)gpio_get_level(HORN_MCLK_PIN));

    for (i = 0; i < (uint32)(sizeof(modes) / sizeof(modes[0])); i++)
    {
        gpio_init(HORN_BCK_PIN, GPI, GPIO_LOW, modes[i]);
        system_delay_ms(1);
        printf("[BCK_DIAG] %s mode=%s pc=0x%02lX bck=%lu/%lu lvl=%u\r\n",
               tag,
               guimai_gpio_mode_name(modes[i]),
               (unsigned long)guimai_pin_pc_value(get_port(HORN_BCK_PIN), (uint8)(HORN_BCK_PIN & 0x1FU)),
               (unsigned long)guimai_pin_toggle_count(HORN_BCK_PIN, GUIMAI_IIS_SCAN_LOOPS),
               (unsigned long)guimai_pin_toggle_count(HORN_BCK_PIN, GUIMAI_IIS_SCAN_LOOPS * 10U),
               (unsigned)gpio_get_level(HORN_BCK_PIN));
    }

    gpio_init(HORN_BCK_PIN, GPI, GPIO_LOW, GPI_FLOATING_IN);
    printf("[BCK_DIAG] %s end restore FLOAT\r\n", tag);
}
#endif

static uint8 guimai_is_iis_data_pin(gpio_pin_enum pin)
{
    return ((pin == HORN_BCK_PIN) ||
            (pin == HORN_SD_PIN) ||
            (pin == ES8388_ADC_SDOUT_PIN) ||
            (pin == HORN_MCLK_PIN)) ? 1U : 0U;
}

static gpio_pin_enum guimai_log_iis_pin_scan(void)
{
    static uint8 s_logged = 0;
    const gpio_pin_enum scan_pins[] = {
        P11_0, P11_1, P11_2, P11_3, P11_4, P11_5, P11_6,
        P11_7, P11_8, P11_9, P11_10, P11_11, P11_12
    };
    uint32 i = 0;
    uint32 bck_toggles = guimai_pin_toggle_count(HORN_BCK_PIN, GUIMAI_IIS_SCAN_LOOPS);
    uint32 target_lrck = (bck_toggles >= 64U) ? (bck_toggles / 64U) : 1U;
    uint32 best_error = 0xFFFFFFFFU;
    gpio_pin_enum best_pin = HORN_WS_PIN;
    uint8 best_valid = 0;

    if (s_logged)
    {
        return s_linein_ws_pin;
    }
    s_logged = 1;

    printf("[IIS_SCAN] begin bck_toggles=%lu lrck_target=%lu\r\n",
           (unsigned long)bck_toggles,
           (unsigned long)target_lrck);
    printf("[IIS_SCAN] configured bck=P%d_%d ws=P%d_%d sdout=P%d_%d mclk=P%d_%d\r\n",
           (unsigned)(HORN_BCK_PIN / 32),
           (unsigned)(HORN_BCK_PIN % 32),
           (unsigned)(HORN_WS_PIN / 32),
           (unsigned)(HORN_WS_PIN % 32),
           (unsigned)(ES8388_ADC_SDOUT_PIN / 32),
           (unsigned)(ES8388_ADC_SDOUT_PIN % 32),
           (unsigned)(HORN_MCLK_PIN / 32),
           (unsigned)(HORN_MCLK_PIN % 32));
    for (i = 0; i < (uint32)(sizeof(scan_pins) / sizeof(scan_pins[0])); i++)
    {
        gpio_pin_enum pin = scan_pins[i];
        uint32 toggles = (pin == HORN_BCK_PIN) ? bck_toggles : guimai_pin_toggle_count(pin, GUIMAI_IIS_SCAN_LOOPS);
        printf("[IIS_SCAN] P%d_%d toggles=%lu level=%u\r\n",
               (unsigned)(pin / 32),
               (unsigned)(pin % 32),
               (unsigned long)toggles,
               (unsigned)gpio_get_level(pin));

        if (!guimai_is_iis_data_pin(pin) && (toggles > 0U) && (bck_toggles > 100U) && (toggles <= (bck_toggles / 4U)))
        {
            uint32 error = (toggles > target_lrck) ? (toggles - target_lrck) : (target_lrck - toggles);
            if ((!best_valid) || (error < best_error))
            {
                best_valid = 1;
                best_error = error;
                best_pin = pin;
            }
        }
    }

    if (best_valid)
    {
        printf("[IIS_SCAN] lrck candidate=P%d_%d error=%lu\r\n",
               (unsigned)(best_pin / 32),
               (unsigned)(best_pin % 32),
               (unsigned long)best_error);
        return best_pin;
    }

    printf("[IIS_SCAN] lrck candidate=<none>\r\n");
    return HORN_WS_PIN;
}

static uint8 guimai_record_regs_bad(void)
{
    uint8 r08 = es8388_read_reg(0x08);
    uint8 r0c = es8388_read_reg(0x0C);
    uint8 r0d = es8388_read_reg(0x0D);
    uint8 r17 = es8388_read_reg(0x17);
    uint8 r18 = es8388_read_reg(0x18);
    uint8 r2b = es8388_read_reg(0x2B);

    if ((r08 == 0xFFU) || (r0c == 0xFFU) || (r0d == 0xFFU) ||
        (r17 == 0xFFU) || (r18 == 0xFFU) || (r2b == 0xFFU))
    {
        printf("[GUIMAI_AUDIO] bad es8388 regs ff 08=%02X 0C=%02X 0D=%02X 17=%02X 18=%02X 2B=%02X\r\n",
               r08, r0c, r0d, r17, r18, r2b);
        return 1U;
    }

    if ((r08 == 0x80U) || (r0c == 0x00U) || (r17 == 0x00U) || (r2b == 0x00U))
    {
        printf("[GUIMAI_AUDIO] bad es8388 regs value 08=%02X 0C=%02X 0D=%02X 17=%02X 18=%02X 2B=%02X\r\n",
               r08, r0c, r0d, r17, r18, r2b);
        return 1U;
    }

    return 0U;
}

#if GUIMAI_RECORD_START_HEALTH_ENABLE
static uint8 guimai_record_start_health_check(void)
{
    uint32 got_total = 0U;
    uint32 abs_sum = 0U;
    uint32 peak = 0U;
    uint32 loops = 0U;

    if (guimai_record_regs_bad())
    {
        return 0U;
    }

    while ((got_total < (uint32)GUIMAI_RECORD_START_HEALTH_SAMPLES) && (loops < 80U))
    {
        uint32 got = 0U;
        uint32 i = 0U;

#if GUIMAI_GTM_DMA_CAPTURE
        got = guimai_gtm_dma_read_samples(s_linein_buf, GUIMAI_LINEIN_READ_BATCH);
#elif GUIMAI_GTM_TBCM_DIAG
        got = guimai_gtm_tbcm_read_samples(s_linein_buf, GUIMAI_LINEIN_READ_BATCH);
#elif GUIMAI_QSPI1_SLAVE_RX
        got = guimai_qspi1_read_samples(s_linein_buf, GUIMAI_LINEIN_READ_BATCH);
#else
        got = es8388_record_try_read(s_linein_buf, GUIMAI_LINEIN_READ_BATCH);
#endif
        if (got == 0U)
        {
            system_delay_ms(1U);
            loops++;
            continue;
        }

        for (i = 0U; i < got; i++)
        {
            int16 pcm_sample = guimai_transform_sample(s_linein_buf[i], GUIMAI_RECORD_PCM_TRANSFORM_MODE);
            uint32 mag = (pcm_sample >= 0) ? (uint32)pcm_sample : (uint32)(-pcm_sample);
            abs_sum += mag;
            if (mag > peak)
            {
                peak = mag;
            }
        }
        got_total += got;
        loops++;
    }

    if (got_total == 0U)
    {
        printf("[GUIMAI_AUDIO] start health fail no_samples loops=%lu\r\n",
               (unsigned long)loops);
        return 0U;
    }

    {
        uint32 avg_abs = abs_sum / got_total;
        uint8 ok = ((avg_abs >= (uint32)GUIMAI_RECORD_START_HEALTH_MIN_AVG) ||
                    (peak >= (uint32)GUIMAI_RECORD_START_HEALTH_MIN_PEAK)) ? 1U : 0U;
        printf("[GUIMAI_AUDIO] start health n=%lu avg_abs=%lu peak=%lu ok=%u\r\n",
               (unsigned long)got_total,
               (unsigned long)avg_abs,
               (unsigned long)peak,
               (unsigned)ok);
        return ok;
    }
}
#endif

static uint8 guimai_linein_probe_source(void)
{
#if GUIMAI_LINEIN_AUTO_PROBE
    const uint8 src_candidates[4] = {
        GUIMAI_LINEIN_SRC_D,  // keep the known working route first
        GUIMAI_LINEIN_SRC_B,
        GUIMAI_LINEIN_SRC_A,
        GUIMAI_LINEIN_SRC_C
    };
    const uint8 fmt_candidates[3] = {
        GUIMAI_IIS_FMT_I2S,
        GUIMAI_IIS_FMT_LEFT_J,
        GUIMAI_IIS_FMT_RIGHT_J
    };
    int16 probe_buf[64];
    int32 best_avg = -1;
    uint32 best_toggles = 0;
    uint8 best_src = src_candidates[0];
    uint8 best_fmt = fmt_candidates[0];
    uint8 fmt_idx = 0;
    uint8 src_idx = 0;

    // Let IIS clocks and ADC pipeline settle for a short moment.
    system_delay_ms(30);

    for (fmt_idx = 0; fmt_idx < (uint8)(sizeof(fmt_candidates) / sizeof(fmt_candidates[0])); fmt_idx++)
    {
        uint8 fmt = fmt_candidates[fmt_idx];
        es8388_record_set_iis_format(fmt);
        system_delay_ms(3);

        for (src_idx = 0; src_idx < (uint8)(sizeof(src_candidates) / sizeof(src_candidates[0])); src_idx++)
        {
            uint8 src = src_candidates[src_idx];
            uint32 total_samples = 0;
            int32 abs_sum = 0;
            uint8 batch = 0;
            int32 avg = 0;
            uint32 toggles = 0;

            es8388_record_set_input_source(src);
            system_delay_ms(5);

            toggles = guimai_sdout_toggle_count(20000U);

            // Flush first short chunk after source switch.
            (void)es8388_record_try_read(probe_buf, (uint32)(sizeof(probe_buf) / sizeof(probe_buf[0])));

            for (batch = 0; batch < 6U; batch++)
            {
                uint32 n = es8388_record_try_read(probe_buf, (uint32)(sizeof(probe_buf) / sizeof(probe_buf[0])));
                uint32 i = 0;
                for (i = 0; i < n; i++)
                {
                    abs_sum += (probe_buf[i] >= 0) ? probe_buf[i] : -probe_buf[i];
                }
                total_samples += n;
            }

            if (total_samples > 0U)
            {
                avg = abs_sum / (int32)total_samples;
            }

            printf("[GUIMAI] probe: fmt=%u(%s) in=%u(%s) avg_abs=%ld n=%lu sd_toggles=%lu\r\n",
                   (unsigned)fmt,
                   guimai_linein_fmt_name(fmt),
                   (unsigned)src,
                   guimai_linein_src_name(src),
                   (long)avg,
                   (unsigned long)total_samples,
                   (unsigned long)toggles);

            if ((avg > best_avg) || ((avg == best_avg) && (toggles > best_toggles)))
            {
                best_avg = avg;
                best_toggles = toggles;
                best_src = src;
                best_fmt = fmt;
            }
        }
    }

    s_linein_iis_format = best_fmt;
    es8388_record_set_iis_format(best_fmt);
    es8388_record_set_input_source(best_src);
    printf("[GUIMAI] selected: fmt=%u(%s) in=%u(%s) avg_abs=%ld sd_toggles=%lu\r\n",
           (unsigned)best_fmt,
           guimai_linein_fmt_name(best_fmt),
           (unsigned)best_src,
           guimai_linein_src_name(best_src),
           (long)best_avg,
           (unsigned long)best_toggles);
    return best_src;
#else
    return s_linein_input_source;
#endif
}

#if GUIMAI_GTM_DMA_CAPTURE && GUIMAI_LINEIN_SRC_SCAN_ON_START
static void guimai_linein_scan_sources(void)
{
    const uint8 src_candidates[4] = {
        GUIMAI_LINEIN_SRC_A,
        GUIMAI_LINEIN_SRC_B,
        GUIMAI_LINEIN_SRC_C,
        GUIMAI_LINEIN_SRC_D
    };
    int16 probe_buf[GUIMAI_LINEIN_READ_BATCH];
    uint8 saved_src = s_linein_input_source;
    uint8 best_src = saved_src;
    uint8 best_ws = s_gtm_dma_left_ws_level;
    uint8 best_delay = s_gtm_dma_i2s_delay_bits;
    uint32 best_score = 0xFFFFFFFFU;
    uint8 src_idx = 0U;

    printf("[GUIMAI_SRC_SCAN] begin keep=%s target=%u ms=%u\r\n",
           guimai_linein_src_name(saved_src),
           (unsigned)GUIMAI_LINEIN_SRC_SCAN_TARGET_SAMPLES,
           (unsigned)GUIMAI_LINEIN_SRC_SCAN_MS);

    for (src_idx = 0U; src_idx < (uint8)(sizeof(src_candidates) / sizeof(src_candidates[0])); src_idx++)
    {
        uint8 src = src_candidates[src_idx];
        uint8 ws_level = 0U;

        es8388_record_set_input_source(src);
        system_delay_ms(40);

        for (ws_level = 0U; ws_level < 2U; ws_level++)
        {
            uint8 delay = 0U;
            for (delay = 0U; delay < 2U; delay++)
            {
                uint32 start_ms = 0U;
                uint32 sd_toggles = 0U;
                uint32 avg_abs = 0U;
                uint32 score = 0U;
                guimai_audio_probe_stats stats;

                guimai_gtm_dma_set_decode(ws_level, delay);
                (void)guimai_gtm_dma_read_samples(probe_buf, GUIMAI_LINEIN_READ_BATCH);

                guimai_audio_probe_stats_reset(&stats);
                start_ms = system_getval_ms();
                while ((guimai_elapsed_ms(start_ms, system_getval_ms()) < GUIMAI_LINEIN_SRC_SCAN_MS) &&
                       (stats.samples < GUIMAI_LINEIN_SRC_SCAN_TARGET_SAMPLES))
                {
                    uint32 n = guimai_gtm_dma_read_samples(probe_buf, GUIMAI_LINEIN_READ_BATCH);
                    if (n > 0U)
                    {
                        guimai_audio_probe_stats_add(&stats, probe_buf, n);
                    }
                }

                sd_toggles = guimai_sdout_toggle_count(12000U);
                avg_abs = (stats.samples > 0U) ? (stats.abs_sum / stats.samples) : 0U;
                if ((stats.samples == 0U) || (stats.nonzero == 0U))
                {
                    score = 0xFFFFFFFFU;
                }
                else
                {
                    uint32 capped_avg = (avg_abs > 32768U) ? 32768U : avg_abs;
                    score = (stats.clipped * 32U) + ((32768U - capped_avg) / 128U) + (stats.zc / 8U);
                }
                printf("[GUIMAI_SRC_SCAN] src=%s ws=%u delay=%u n=%lu avg_abs=%lu zc=%lu clip=%lu nonzero=%lu min=%d max=%d last=%d sd_tog=%lu score=%lu reg0A=0x%02X\r\n",
                       guimai_linein_src_name(src),
                       (unsigned)ws_level,
                       (unsigned)delay,
                       (unsigned long)stats.samples,
                       (unsigned long)avg_abs,
                       (unsigned long)stats.zc,
                       (unsigned long)stats.clipped,
                       (unsigned long)stats.nonzero,
                       stats.min_sample,
                       stats.max_sample,
                       stats.last_sample,
                       (unsigned long)sd_toggles,
                       (unsigned long)score,
                       es8388_read_reg(0x0A));

                if ((score != 0xFFFFFFFFU) && (score < best_score))
                {
                    best_score = score;
                    best_src = src;
                    best_ws = ws_level;
                    best_delay = delay;
                }
            }
        }
    }

    s_linein_input_source = best_src;
    es8388_record_set_input_source(best_src);
    guimai_gtm_dma_set_decode(best_ws, best_delay);
    system_delay_ms(20);
    guimai_gtm_dma_reset_runtime_stats();
    printf("[GUIMAI_SRC_SCAN] selected src=%s ws=%u delay=%u score=%lu reg0A=0x%02X\r\n",
           guimai_linein_src_name(best_src),
           (unsigned)best_ws,
           (unsigned)best_delay,
           (unsigned long)best_score,
           es8388_read_reg(0x0A));
}
#endif

static void guimai_linein_init(void)
{
    es8388_record_config_struct cfg;
    uint8 ret = 0;

#if GUIMAI_MUSIC_ENABLE
    if (s_music_used_codec)
    {
        s_es8388_ready = 0U;
        s_linein_ready = 0U;
        s_music_ready = 0U;
        s_music_used_codec = 0U;
        printf("[GUIMAI] reinit ES8388 for record after music.\r\n");
    }
#endif

#if GUIMAI_RECORD_PRE_MCLK_ENABLE
    guimai_record_pre_mclk_start("record_init");
#endif

    if (!s_es8388_ready)
    {
        ret = es8388_init();
        if (0 != ret)
        {
            s_linein_ready = 0;
            printf("[GUIMAI] es8388 init failed, ret=%u\r\n", ret);
            return;
        }
        s_es8388_ready = 1;
        printf("[GUIMAI] es8388 init ok.\r\n");
    }

    cfg.sample_rate = GUIMAI_AUDIO_SAMPLE_RATE;
    cfg.bits_per_sample = 16;
    cfg.channels = 1;
    cfg.input_source = s_linein_input_source;
    es8388_record_set_iis_format(s_linein_iis_format);

    ret = es8388_record_init(&cfg);
    if (0 != ret)
    {
        s_linein_ready = 0;
        printf("[GUIMAI] linein init failed, ret=%u\r\n", ret);
        return;
    }

    s_linein_ready = 1;
    printf("[GUIMAI] linein record ready (src=%s, fmt=%s, %lu/16bit/mono)\r\n",
           guimai_linein_src_name(s_linein_input_source),
           guimai_linein_fmt_name(s_linein_iis_format),
           (unsigned long)GUIMAI_AUDIO_SAMPLE_RATE);
    printf("[GUIMAI] es8388 regs: 0x00=0x%02X 0x01=0x%02X 0x02=0x%02X 0x04=0x%02X 0x08=0x%02X 0x09=0x%02X 0x0A=0x%02X 0x0C=0x%02X 0x0D=0x%02X 0x10=0x%02X 0x11=0x%02X 0x12=0x%02X 0x17=0x%02X 0x18=0x%02X 0x1A=0x%02X 0x23=0x%02X 0x2B=0x%02X\r\n",
           es8388_read_reg(0x00),
           es8388_read_reg(0x01),
           es8388_read_reg(0x02),
           es8388_read_reg(0x04),
           es8388_read_reg(0x08),
           es8388_read_reg(0x09),
           es8388_read_reg(0x0A),
           es8388_read_reg(0x0C),
           es8388_read_reg(0x0D),
           es8388_read_reg(0x10),
           es8388_read_reg(0x11),
           es8388_read_reg(0x12),
           es8388_read_reg(0x17),
           es8388_read_reg(0x18),
           es8388_read_reg(0x1A),
           es8388_read_reg(0x23),
           es8388_read_reg(0x2B));
#if GUIMAI_BCK_PORT_DIAG
    guimai_bck_input_mode_probe("after_record_init");
#endif
}

#if GUIMAI_RECORD_CLOCK_RETRY_ENABLE
static uint8 guimai_record_clock_present(const char *tag)
{
    uint32 bck_toggles = guimai_pin_toggle_count(HORN_BCK_PIN, GUIMAI_IIS_SCAN_LOOPS);
    uint32 ws_toggles = guimai_pin_toggle_count(HORN_WS_PIN, GUIMAI_IIS_SCAN_LOOPS);
    uint32 sd_toggles = guimai_pin_toggle_count(ES8388_ADC_SDOUT_PIN, GUIMAI_IIS_SCAN_LOOPS);
    uint32 mclk_toggles = 0U;

    if (HORN_MCLK_PIN != 0xFFFFFFFF)
    {
        mclk_toggles = guimai_pin_toggle_count(HORN_MCLK_PIN, GUIMAI_IIS_SCAN_LOOPS);
    }

    printf("[GUIMAI_CLOCK] %s bck=%lu ws=%lu sd=%lu mclk=%lu lvl=%u/%u/%u/%u regs 08=%02X 0C=%02X 17=%02X 2B=%02X\r\n",
           (tag != NULL_PTR) ? tag : "?",
           (unsigned long)bck_toggles,
           (unsigned long)ws_toggles,
           (unsigned long)sd_toggles,
           (unsigned long)mclk_toggles,
           (unsigned)gpio_get_level(HORN_BCK_PIN),
           (unsigned)gpio_get_level(HORN_WS_PIN),
           (unsigned)gpio_get_level(ES8388_ADC_SDOUT_PIN),
           (HORN_MCLK_PIN != 0xFFFFFFFF) ? (unsigned)gpio_get_level(HORN_MCLK_PIN) : 0U,
           es8388_read_reg(0x08),
           es8388_read_reg(0x0C),
           es8388_read_reg(0x17),
           es8388_read_reg(0x2B));

#if GUIMAI_BCK_CLOCK_MISS_DIAG
    if ((bck_toggles == 0U) && (ws_toggles > 0U))
    {
        guimai_bck_input_mode_probe("bck_missing_ws_alive");
        return 2U;
    }
#endif

    return ((bck_toggles > 0U) && (ws_toggles > 0U)) ? 1U : 0U;
}

static uint8 guimai_record_start_with_clock_retry(void)
{
    uint8 ret = 0U;
    uint8 attempt = 0U;
    uint8 clock_state = 0U;

    ret = es8388_record_start();
    if (ret != 0U)
    {
        return ret;
    }

    system_delay_ms((uint32)GUIMAI_RECORD_CLOCK_CHECK_DELAY_MS);
    clock_state = guimai_record_clock_present("record_start");
    if (clock_state == 1U)
    {
        return 0U;
    }

    for (attempt = 0U; attempt < (uint8)GUIMAI_RECORD_CLOCK_RETRY_COUNT; attempt++)
    {
        printf("[GUIMAI_CLOCK] missing BCK/WS, relock ES8388 record clock keep_mclk attempt=%u/%u state=%u.\r\n",
               (unsigned)(attempt + 1U),
               (unsigned)GUIMAI_RECORD_CLOCK_RETRY_COUNT,
               (unsigned)clock_state);

        ret = es8388_record_relock_clock();
        if (ret != 0U)
        {
            printf("[GUIMAI_CLOCK] ES8388 clock relock failed ret=%u, full reinit.\r\n",
                   (unsigned)ret);
            break;
        }

        system_delay_ms((uint32)GUIMAI_RECORD_CLOCK_RETRY_SETTLE_MS);
        clock_state = guimai_record_clock_present("record_relock");
        if (clock_state == 1U)
        {
            return 0U;
        }
    }

    printf("[GUIMAI_CLOCK] relock did not restore BCK/WS, rebuild ES8388 record path.\r\n");

    (void)es8388_record_stop();
    s_linein_ready = 0U;
    s_es8388_ready = 0U;
    guimai_audio_release_iis_pins();
    system_delay_ms((uint32)GUIMAI_RECORD_CLOCK_RETRY_SETTLE_MS);
    guimai_linein_init();
    if (!s_linein_ready)
    {
        return 3U;
    }

    ret = es8388_record_start();
    if (ret != 0U)
    {
        return ret;
    }

    system_delay_ms((uint32)GUIMAI_RECORD_CLOCK_CHECK_DELAY_MS);
    clock_state = guimai_record_clock_present("record_reinit");
    if (clock_state == 1U)
    {
        return 0U;
    }

    ret = es8388_record_relock_clock();
    if (ret == 0U)
    {
        system_delay_ms((uint32)GUIMAI_RECORD_CLOCK_RETRY_SETTLE_MS);
        clock_state = guimai_record_clock_present("record_reinit_relock");
        if (clock_state == 1U)
        {
            return 0U;
        }
    }

    (void)es8388_record_stop();
    printf("[GUIMAI_CLOCK] record clock missing after relock/reinit, abort capture.\r\n");
    return 2U;
}
#endif

#if GUIMAI_GTM_DMA_CAPTURE && GUIMAI_RECORD_CLOCK_PUMP_ENABLE && ES8388_RECORD_MCU_MASTER
static void guimai_record_clock_pump_reset(void)
{
    s_record_clock_pump_last_us = 0U;
    s_record_clock_pump_frac_us = 0U;
}

static void guimai_record_clock_pump_task(void)
{
    static int16 pump_buf[GUIMAI_RECORD_CLOCK_PUMP_MAX_FRAMES];
    uint32 now_us = 0U;
    uint32 elapsed_us = 0U;
    uint32 frames = 0U;
    unsigned long long budget = 0ULL;

    if (!s_linein_running)
    {
        guimai_record_clock_pump_reset();
        return;
    }

    now_us = system_getval_us();
    if (0U == s_record_clock_pump_last_us)
    {
        s_record_clock_pump_last_us = now_us;
        return;
    }

    elapsed_us = now_us - s_record_clock_pump_last_us;
    s_record_clock_pump_last_us = now_us;
    if (elapsed_us > 20000U)
    {
        elapsed_us = 20000U;
    }

    budget = ((unsigned long long)elapsed_us * (unsigned long long)GUIMAI_AUDIO_SAMPLE_RATE)
             + (unsigned long long)s_record_clock_pump_frac_us;
    frames = (uint32)(budget / 1000000ULL);

    if (frames > (uint32)GUIMAI_RECORD_CLOCK_PUMP_MAX_FRAMES)
    {
        frames = (uint32)GUIMAI_RECORD_CLOCK_PUMP_MAX_FRAMES;
    }
    s_record_clock_pump_frac_us = (uint32)(budget - ((unsigned long long)frames * 1000000ULL));
    if (frames > 0U)
    {
        (void)es8388_record_try_read(pump_buf, frames);
    }
}
#endif

static void guimai_linein_start(void)
{
    uint8 ret = 0;
    uint8 attempt = 0U;

    if (!s_linein_ready)
    {
        guimai_linein_init();
    }

    if (!s_linein_ready || s_linein_running)
    {
        return;
    }

    for (attempt = 0U; attempt < 2U; attempt++)
    {
        if ((attempt > 0U) || guimai_record_regs_bad())
        {
            printf("[GUIMAI_AUDIO] rebuild record path before start attempt=%u\r\n",
                   (unsigned)(attempt + 1U));
            (void)es8388_record_stop();
#if GUIMAI_GTM_DMA_CAPTURE
            guimai_gtm_dma_stop();
            s_gtm_dma_ready = 0U;
            s_gtm_dma_active = 0U;
            s_gtm_dma_tim_ch = NULL_PTR;
#endif
            s_linein_ready = 0U;
            s_es8388_ready = 0U;
            guimai_audio_release_iis_pins();
            system_delay_ms((uint32)GUIMAI_AUDIO_RELEASE_SETTLE_MS);
            guimai_linein_init();
            if (!s_linein_ready)
            {
                ret = 3U;
                continue;
            }
        }

#if GUIMAI_RECORD_CLOCK_RETRY_ENABLE
        ret = guimai_record_start_with_clock_retry();
#else
        ret = es8388_record_start();
#endif
        if (ret != 0U)
        {
            printf("[GUIMAI] linein start attempt failed attempt=%u ret=%u\r\n",
                   (unsigned)(attempt + 1U),
                   (unsigned)ret);
            continue;
        }

#if GUIMAI_GTM_TBCM_DIAG
        printf("[GUIMAI] record_start ok, TBCM diag keeps fixed src/fmt.\r\n");
#else
        s_linein_input_source = guimai_linein_probe_source();
#endif
        s_linein_ws_pin = HORN_WS_PIN;
        guimai_audio_buffer_reset();
        printf("[GUIMAI] audio buffer reset ok.\r\n");
#if GUIMAI_GTM_DMA_CAPTURE
        guimai_gtm_dma_init();
        guimai_gtm_dma_i2s_reset();
        guimai_gtm_dma_start_chunk();
#if GUIMAI_LINEIN_SRC_SCAN_ON_START
        guimai_linein_scan_sources();
#endif
#endif
#if GUIMAI_GTM_TBCM_DIAG
        guimai_gtm_tbcm_init();
        printf("[GUIMAI] TBCM init returned.\r\n");
        guimai_gtm_tbcm_start();
        printf("[GUIMAI] TBCM start returned.\r\n");
#endif
#if GUIMAI_QSPI1_SLAVE_RX
        guimai_qspi1_slave_init();
        guimai_qspi1_sync_to_ws_edge();
#endif
#if GUIMAI_RECORD_START_HEALTH_ENABLE
        if (!guimai_record_start_health_check())
        {
            printf("[GUIMAI_AUDIO] start health failed, rebuild once attempt=%u\r\n",
                   (unsigned)(attempt + 1U));
            (void)es8388_record_stop();
#if GUIMAI_GTM_DMA_CAPTURE
            guimai_gtm_dma_stop();
#endif
            guimai_audio_buffer_reset();
            ret = 4U;
            continue;
        }

        guimai_audio_buffer_reset();
#if GUIMAI_GTM_DMA_CAPTURE
        guimai_gtm_dma_stop();
        guimai_gtm_dma_i2s_reset();
        guimai_gtm_dma_start_chunk();
#endif
#endif
#if GUIMAI_RECORD_DEFER_TX
        s_record_start_ms = system_getval_ms();
#endif
        s_linein_running = 1;
#if GUIMAI_GTM_DMA_CAPTURE && GUIMAI_RECORD_CLOCK_PUMP_ENABLE && ES8388_RECORD_MCU_MASTER
        guimai_record_clock_pump_reset();
#endif
        guimai_record_ui_status("REC ON ");
#if GUIMAI_GTM_DMA_CAPTURE
        printf("[GUIMAI] linein capture start. (src=%s fmt=%s ws=%u delay=%u)\r\n",
               guimai_linein_src_name(s_linein_input_source),
               guimai_linein_fmt_name(s_linein_iis_format),
               (unsigned)s_gtm_dma_left_ws_level,
               (unsigned)s_gtm_dma_i2s_delay_bits);
#else
        printf("[GUIMAI] linein capture start. (src=%s fmt=%s)\r\n",
               guimai_linein_src_name(s_linein_input_source),
               guimai_linein_fmt_name(s_linein_iis_format));
#endif
#if GUIMAI_IIS_SCAN_ON_START
        system_delay_ms(5);
        {
#if GUIMAI_BCK_PORT_DIAG
            guimai_bck_input_mode_probe("after_record_start");
#endif
            gpio_pin_enum detected_ws_pin = guimai_log_iis_pin_scan();
            uint32 configured_ws_toggles = guimai_pin_toggle_count(HORN_WS_PIN, GUIMAI_IIS_SCAN_LOOPS);
            (void)detected_ws_pin;
            if (0U == configured_ws_toggles)
            {
                uint32 bck_toggles = guimai_pin_toggle_count(HORN_BCK_PIN, GUIMAI_IIS_SCAN_LOOPS);
                uint32 sd_toggles = guimai_pin_toggle_count(ES8388_ADC_SDOUT_PIN, GUIMAI_IIS_SCAN_LOOPS);
                printf("[IIS_DIAG] configured LRCK P%d_%d missing, bck=%lu sdout=%lu. keep fixed WS pin.\r\n",
                       (unsigned)(HORN_WS_PIN / 32),
                       (unsigned)(HORN_WS_PIN % 32),
                       (unsigned long)bck_toggles,
                       (unsigned long)sd_toggles);
            }
        }
#endif
        return;
    }

    printf("[GUIMAI] linein start failed, ret=%u\r\n", ret);
}

static void guimai_linein_stop(void)
{
    if (!s_linein_running)
    {
        return;
    }

    (void)es8388_record_stop_keep_clock();
#if GUIMAI_GTM_DMA_CAPTURE
    guimai_gtm_dma_stop();
#endif
#if GUIMAI_GTM_TBCM_DIAG
    guimai_gtm_tbcm_stop();
#endif
#if GUIMAI_RECORD_DEFER_TX
    if (s_record_start_ms != 0U)
    {
        uint32 now_ms = system_getval_ms();
        s_record_elapsed_ms = guimai_elapsed_ms(s_record_start_ms, now_ms);
        if (s_record_elapsed_ms > 60000U)
        {
            printf("[GUIMAI] record elapsed abnormal=%lu, reset to 0\r\n",
                   (unsigned long)s_record_elapsed_ms);
            s_record_elapsed_ms = 0U;
        }
    }
#endif
    s_linein_running = 0;
#if GUIMAI_GTM_DMA_CAPTURE && GUIMAI_RECORD_CLOCK_PUMP_ENABLE && ES8388_RECORD_MCU_MASTER
    guimai_record_clock_pump_reset();
#endif
    guimai_record_ui_status("REC OFF");
    printf("[GUIMAI] linein capture stop.\r\n");
}

static void guimai_linein_task(void)
{
    uint32 samples = 0;
    uint32 i = 0;
#if GUIMAI_AUDIO_HOTPATH_LOG
    int32 abs_sum = 0;
#endif
    uint32 now_ms = 0;
#if GUIMAI_AUDIO_HOTPATH_LOG
    static uint32 s_audio_zero_log_ms = 0;
    static uint32 s_audio_enter_log_ms = 0;
#if GUIMAI_BCK_PORT_DIAG
    static uint8 s_audio_zero_diag_count = 0;
#endif
#endif

    if (!s_linein_running)
    {
        return;
    }

#if GUIMAI_AUDIO_HOTPATH_LOG
    now_ms = system_getval_ms();
    if ((0 == s_audio_enter_log_ms) || ((now_ms - s_audio_enter_log_ms) >= 1000U))
    {
        s_audio_enter_log_ms = now_ms;
        printf("[AUDIO_TASK] enter state=%u running=%u stream=%u rec=%lu pos=%u batch=%u\r\n",
               (unsigned)es8388_record_get_state(),
               (unsigned)s_linein_running,
               (unsigned)s_streaming,
               (unsigned long)s_record_samples,
               (unsigned)s_voice_pos,
               (unsigned)GUIMAI_LINEIN_READ_BATCH);
    }
#endif

#if GUIMAI_GTM_DMA_CAPTURE
#if GUIMAI_RECORD_CLOCK_PUMP_ENABLE && ES8388_RECORD_MCU_MASTER
    guimai_record_clock_pump_task();
#endif
    samples = guimai_gtm_dma_read_samples(s_linein_buf, GUIMAI_LINEIN_READ_BATCH);
#elif GUIMAI_GTM_TBCM_DIAG
    samples = guimai_gtm_tbcm_read_samples(s_linein_buf, GUIMAI_LINEIN_READ_BATCH);
#elif GUIMAI_QSPI1_SLAVE_RX
    samples = guimai_qspi1_read_samples(s_linein_buf, GUIMAI_LINEIN_READ_BATCH);
#else
    samples = es8388_record_try_read(s_linein_buf, GUIMAI_LINEIN_READ_BATCH);
#endif
    if (0 == samples)
    {
#if GUIMAI_AUDIO_HOTPATH_LOG
        now_ms = system_getval_ms();
        if ((0 == s_audio_zero_log_ms) || ((now_ms - s_audio_zero_log_ms) >= 200U))
        {
            uint32 bck_toggles = 0;
            uint32 ws_toggles = 0;
            uint32 sd_toggles = 0;
#if GUIMAI_AUDIO_RUNTIME_DIAG
            bck_toggles = guimai_pin_toggle_count(HORN_BCK_PIN, GUIMAI_IIS_SCAN_LOOPS);
            ws_toggles = guimai_pin_toggle_count(s_linein_ws_pin, GUIMAI_IIS_SCAN_LOOPS);
            sd_toggles = guimai_pin_toggle_count(ES8388_ADC_SDOUT_PIN, GUIMAI_IIS_SCAN_LOOPS);
            if ((0U == ws_toggles) && (bck_toggles > 100U))
            {
                (void)guimai_log_iis_pin_scan();
            }
#endif
            s_audio_zero_log_ms = now_ms;
            printf("[AUDIO0] n=0 state=%u running=%u stream=%u rec=%lu pos=%u bck=%lu ws=%lu sd=%lu lvl=%u/%u/%u fmt=%s"
#if GUIMAI_GTM_DMA_CAPTURE
                   " gtm_chunks=%lu gtm_words=%lu gtm_samples=%lu gtm_lost=%lu dma_tcnt=%u dma_chcsr=0x%08lX"
#endif
#if GUIMAI_GTM_TBCM_DIAG
                   " tbcm_events=%lu new=%lu gprofl=%lu gpr1=0x%06lX ecnt=%u ws1=%lu sd1=%lu irq=0x%02lX"
#endif
#if GUIMAI_QSPI1_SLAVE_RX
                   " qspi_fifo=%u qspi_err=0x%lX qspi_status=0x%08lX qspi_sync=%u"
#endif
                   "\r\n",
                   (unsigned)es8388_record_get_state(),
                   (unsigned)s_linein_running,
                   (unsigned)s_streaming,
                   (unsigned long)s_record_samples,
                   (unsigned)s_voice_pos,
                   (unsigned long)bck_toggles,
                   (unsigned long)ws_toggles,
                   (unsigned long)sd_toggles,
                   (unsigned)gpio_get_level(HORN_BCK_PIN),
                   (unsigned)gpio_get_level(s_linein_ws_pin),
                   (unsigned)gpio_get_level(ES8388_ADC_SDOUT_PIN),
                   guimai_linein_fmt_name(s_linein_iis_format)
#if GUIMAI_GTM_DMA_CAPTURE
                   ,
                   (unsigned long)s_gtm_dma_chunks,
                   (unsigned long)s_gtm_dma_words,
                   (unsigned long)s_gtm_dma_samples,
                   (unsigned long)s_gtm_dma_lost,
                   (unsigned)IfxDma_getChannelTransferCount(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL),
                   (unsigned long)MODULE_DMA.CH[GUIMAI_GTM_DMA_CHANNEL].CHCSR.U
#endif
#if GUIMAI_GTM_TBCM_DIAG
                   ,
                   (unsigned long)s_gtm_tbcm_events,
                   (unsigned long)s_gtm_tbcm_newval,
                   (unsigned long)s_gtm_tbcm_gprofl,
                   (unsigned long)s_gtm_tbcm_last_gpr1,
                   (unsigned)s_gtm_tbcm_last_ecnt,
                   (unsigned long)s_gtm_tbcm_ws_one,
                   (unsigned long)s_gtm_tbcm_sd_one,
                   (unsigned long)s_gtm_tbcm_last_irq
#endif
#if GUIMAI_QSPI1_SLAVE_RX
                   ,
                   (unsigned)IfxQspi_getReceiveFifoLevel(&MODULE_QSPI1),
                   (unsigned long)s_qspi1_last_error,
                   (unsigned long)MODULE_QSPI1.STATUS.U,
                   (unsigned)s_qspi1_sync_ok
#endif
                   );
#if GUIMAI_BCK_PORT_DIAG
            if (s_audio_zero_diag_count < 3U)
            {
                s_audio_zero_diag_count++;
                guimai_bck_input_mode_probe("audio0");
            }
#endif
        }
#endif
        return;
    }

    for (i = 0; i < samples; i++)
    {
        int16 pcm_sample = guimai_transform_sample(s_linein_buf[i], GUIMAI_RECORD_PCM_TRANSFORM_MODE);
#if GUIMAI_RECORD_DEFER_TX
#if GUIMAI_RECORD_STREAM_TX
        s_record_samples++;
#else
        if (s_record_samples < GUIMAI_RECORD_STORE_SAMPLES)
        {
            s_record_store[s_record_samples++] = pcm_sample;
        }
        else
        {
            s_record_overflow = 1;
        }
#endif
#endif
        s_mic_voice[s_fill_buf][s_voice_pos] = pcm_sample;
#if GUIMAI_AUDIO_HOTPATH_LOG
        abs_sum += (pcm_sample >= 0) ? pcm_sample : -pcm_sample;
#endif

        s_voice_pos++;
        if (s_voice_pos >= GUIMAI_MIC_BUF_LEN)
        {
            const int16 *ready_buf = &s_mic_voice[s_fill_buf][0];
            s_voice_pos = 0;
            s_fill_buf = (GUIMAI_BUF_0 == s_fill_buf) ? GUIMAI_BUF_1 : GUIMAI_BUF_0;
#if GUIMAI_RECORD_STREAM_TX
            (void)guimai_stream_tx_queue_frame(1U, ready_buf, GUIMAI_MIC_BUF_LEN);
#endif
            s_voice_frame_ready = 1;
        }
    }

    now_ms = system_getval_ms();
    if (0 == s_audio_fs_stat_ms)
    {
        s_audio_fs_stat_ms = now_ms;
    }
    s_audio_fs_stat_samples += samples;
    if ((now_ms - s_audio_fs_stat_ms) >= 1000U)
    {
        uint32 elapsed = now_ms - s_audio_fs_stat_ms;
        if (elapsed > 0U)
        {
            s_audio_fs_est = (s_audio_fs_stat_samples * 1000U) / elapsed;
        }
        s_audio_fs_stat_ms = now_ms;
        s_audio_fs_stat_samples = 0;
    }

#if GUIMAI_AUDIO_HOTPATH_LOG
    if ((0 == s_audio_level_last_ms) || ((now_ms - s_audio_level_last_ms) >= 1000U))
    {
        uint32 bck_toggles = 0;
        uint32 ws_toggles = 0;
        uint32 sd_toggles = 0;
#if 0
        bck_toggles = guimai_pin_toggle_count(HORN_BCK_PIN, GUIMAI_IIS_SCAN_LOOPS);
        ws_toggles = guimai_pin_toggle_count(s_linein_ws_pin, GUIMAI_IIS_SCAN_LOOPS);
        sd_toggles = guimai_sdout_toggle_count(GUIMAI_IIS_SCAN_LOOPS);
#endif
        s_audio_level_last_ms = now_ms;
        printf("[AUDIO] n=%u first=%d avg_abs=%ld state=%u rec=%lu pos=%u tog=%lu/%lu/%lu lvl=%u/%u/%u ws=P%d_%d fmt=%s fs_est=%lu"
#if GUIMAI_GTM_DMA_CAPTURE
               " gtm_chunks=%lu gtm_words=%lu gtm_samples=%lu gtm_lost=%lu"
#endif
#if GUIMAI_QSPI1_SLAVE_RX
               " qspi_words=%lu qspi_fifo=%u qspi_err=0x%lX qspi_sync=%u"
#endif
               "\r\n",
               (unsigned)samples,
               s_linein_buf[0],
               (long)(abs_sum / (int32)samples),
               (unsigned)es8388_record_get_state(),
               (unsigned long)s_record_samples,
               (unsigned)s_voice_pos,
               (unsigned long)bck_toggles,
               (unsigned long)ws_toggles,
               (unsigned long)sd_toggles,
               (unsigned)gpio_get_level(HORN_BCK_PIN),
               (unsigned)gpio_get_level(s_linein_ws_pin),
               (unsigned)gpio_get_level(ES8388_ADC_SDOUT_PIN),
               (unsigned)(s_linein_ws_pin / 32),
               (unsigned)(s_linein_ws_pin % 32),
               guimai_linein_fmt_name(s_linein_iis_format),
               (unsigned long)s_audio_fs_est
#if GUIMAI_GTM_DMA_CAPTURE
               ,
               (unsigned long)s_gtm_dma_chunks,
               (unsigned long)s_gtm_dma_words,
               (unsigned long)s_gtm_dma_samples,
               (unsigned long)s_gtm_dma_lost
#endif
#if GUIMAI_QSPI1_SLAVE_RX
               ,
               (unsigned long)s_qspi1_rx_words,
               (unsigned)IfxQspi_getReceiveFifoLevel(&MODULE_QSPI1),
               (unsigned long)s_qspi1_last_error,
               (unsigned)s_qspi1_sync_ok
#endif
               );
    }
#endif
}

#else
static void guimai_linein_init(void)
{
    s_linein_ready = guimai_adc_audio_init();
    if (!s_linein_ready)
    {
        guimai_record_ui_status("ADC ERR");
        printf("[GUIMAI_ADC] init failed error=%u\r\n",
               (unsigned)guimai_adc_audio_error());
    }
}

static void guimai_linein_start(void)
{
    if (s_linein_running)
    {
        return;
    }
    if (!s_linein_ready)
    {
        guimai_linein_init();
    }
    if (!s_linein_ready)
    {
        return;
    }
    guimai_audio_buffer_reset();
    if (!guimai_adc_audio_start())
    {
        guimai_record_ui_status("ADC ERR");
        printf("[GUIMAI_ADC] start failed error=%u\r\n",
               (unsigned)guimai_adc_audio_error());
        return;
    }
    s_record_start_ms = system_getval_ms();
    s_audio_fs_stat_ms = s_record_start_ms;
    s_linein_running = 1U;
    guimai_record_ui_status("REC ON ");
}

static void guimai_linein_stop(void)
{
    if (!s_linein_running)
    {
        return;
    }
    guimai_adc_audio_stop();
    s_record_elapsed_ms = guimai_elapsed_ms(s_record_start_ms, system_getval_ms());
    s_linein_running = 0U;
    guimai_record_ui_status(guimai_adc_audio_error() ? "ADC ERR" : "REC OFF");
    printf("[GUIMAI_ADC] stop samples=%lu elapsed=%lu error=%u\r\n",
           (unsigned long)s_record_samples, (unsigned long)s_record_elapsed_ms,
           (unsigned)guimai_adc_audio_error());
}

static void guimai_linein_task(void)
{
    uint32 samples;
    uint32 i;
    uint32 now_ms;
    if (!s_linein_running || s_record_overflow)
    {
        return;
    }
    samples = guimai_adc_audio_read(s_linein_buf, GUIMAI_LINEIN_READ_BATCH);
    if (guimai_adc_audio_error())
    {
        s_record_core_stop_req = 1U;
        guimai_record_ui_status("ADC ERR");
        return;
    }
    for (i = 0U; i < samples; i++)
    {
        s_mic_voice[s_fill_buf][s_voice_pos++] = s_linein_buf[i];
        s_record_samples++;
        if (s_voice_pos == GUIMAI_MIC_BUF_LEN)
        {
            const int16 *ready_buf = &s_mic_voice[s_fill_buf][0];
            s_voice_pos = 0U;
            s_fill_buf = (GUIMAI_BUF_0 == s_fill_buf) ? GUIMAI_BUF_1 : GUIMAI_BUF_0;
            if (!guimai_stream_tx_queue_frame(1U, ready_buf, GUIMAI_MIC_BUF_LEN))
            {
                s_record_overflow = 1U;
                s_record_core_stop_req = 1U;
                guimai_adc_audio_stop();
                guimai_record_ui_status("TX FULL");
                printf("[GUIMAI_ADC] audio TX queue unavailable, capture stopped\r\n");
                break;
            }
        }
    }
    now_ms = system_getval_ms();
    s_audio_fs_stat_samples += samples;
    if ((now_ms - s_audio_fs_stat_ms) >= 1000U)
    {
        s_audio_fs_est = s_audio_fs_stat_samples * 1000U / (now_ms - s_audio_fs_stat_ms);
        s_audio_fs_stat_ms = now_ms;
        s_audio_fs_stat_samples = 0U;
    }
}

/* Freeze DMA first, then deliver the remaining samples before the end frame. */
static void guimai_adc_drain_tail(void)
{
    uint32 i;
    guimai_adc_audio_stop();
    for (i = 0U; i < (2048U + GUIMAI_LINEIN_READ_BATCH - 1U) / GUIMAI_LINEIN_READ_BATCH; i++)
    {
        guimai_linein_task();
    }
}
#endif

#if GUIMAI_RECORD_DEFER_TX && GUIMAI_RECORD_TIGHT_LOOP
static void guimai_linein_capture_until_release(void)
{
    uint32 last_key_ms = 0;
    uint32 last_log_ms = 0;
    uint8 key_level = 1U;

    printf("[GUIMAI_TIGHT] begin\r\n");
    while (s_linein_running)
    {
        guimai_linein_task();
#if GUIMAI_GTM_TBCM_DIAG
        system_delay_ms(1);
#endif

        if ((s_record_samples >= GUIMAI_RECORD_STORE_SAMPLES) || s_record_overflow)
        {
            printf("[GUIMAI_TIGHT] buffer full\r\n");
            break;
        }

        {
            uint32 now_ms = system_getval_ms();
            if ((last_key_ms == 0U) || ((now_ms - last_key_ms) >= 20U))
            {
                last_key_ms = now_ms;
                key_level = guimai_switch_read_level();
                if (key_level == s_switch_origin_level)
                {
                    break;
                }
            }

            if ((last_log_ms == 0U) || ((now_ms - last_log_ms) >= 1000U))
            {
                last_log_ms = now_ms;
                printf("[GUIMAI_TIGHT] samples=%lu fs_est=%lu\r\n",
                       (unsigned long)s_record_samples,
                       (unsigned long)s_audio_fs_est);
            }
        }
    }

    guimai_linein_stop();
    guimai_send_stored_audio();
    s_streaming = 0;
    s_switch_event = 0;
    s_switch_state = s_switch_origin_level;
    printf("[GUIMAI_TIGHT] end\r\n");
}
#endif

#if GUIMAI_RECORD_DEFER_TX && GUIMAI_RECORD_TIGHT_LOOP && GUIMAI_RECORD_USE_DEDICATED_CORE
static void guimai_record_core_capture_loop(void)
{
    uint32 last_log_ms = 0;
    uint8 no_dma_diag_printed = 0U;
#if GUIMAI_GTM_TBCM_DIAG
    uint32 no_event_diag_printed = 0U;
#endif

    s_record_core_running = 1U;
    s_record_core_done = 0U;
    printf("[GUIMAI_REC] capture begin\r\n");

    while (!s_record_core_stop_req)
    {
        guimai_linein_task();
#if GUIMAI_GTM_TBCM_DIAG
        system_delay_ms(1);
#endif

#if !GUIMAI_RECORD_STREAM_TX
        if ((s_record_samples >= GUIMAI_RECORD_STORE_SAMPLES) || s_record_overflow)
        {
            printf("[GUIMAI_REC] buffer full\r\n");
            break;
        }
#endif

        {
            uint32 now_ms = system_getval_ms();
            if ((last_log_ms == 0U) || ((now_ms - last_log_ms) >= 1000U))
            {
                last_log_ms = now_ms;
                printf("[GUIMAI_REC] samples=%lu fs_est=%lu"
#if GUIMAI_GTM_DMA_CAPTURE
                       " gtm_chunks=%lu gtm_words=%lu gtm_samples=%lu gtm_lost=%lu dma_tcnt=%u dma_chcsr=0x%08lX"
#endif
#if GUIMAI_GTM_TBCM_DIAG
                       " tbcm_polls=%lu tbcm_events=%lu new=%lu gprofl=%lu gpr1=0x%06lX ecnt=%u ws1=%lu sd1=%lu irq=0x%02lX cnts=0x%08lX ctrl=0x%08lX ectrl=0x%08lX tduv=0x%08lX tduc=0x%08lX"
#if GUIMAI_GTM_ARU_FIFO_DIAG
                       " f2a_en=0x%08lX"
#if GUIMAI_GTM_AFD_DMA_DIAG
                       " afd_dma=%lu/%lu/%lu/%lu/%lu/0x%08lX"
#endif
                       " f0=%lu/%lu/%lu/%lu/%lu/0x%lX/%lu/0x%08lX/0x%03lX"
                       " f1=%lu/%lu/%lu/%lu/%lu/0x%lX/%lu/0x%08lX/0x%03lX"
                       " f2=%lu/%lu/%lu/%lu/%lu/0x%lX/%lu/0x%08lX/0x%03lX"
#endif
#endif
#if GUIMAI_QSPI1_SLAVE_RX
                       " qspi_words=%lu qspi_fifo=%u qspi_err=0x%lX qspi_status=0x%08lX qspi_sync=%u"
#endif
                       "\r\n",
                       (unsigned long)s_record_samples,
                       (unsigned long)s_audio_fs_est
#if GUIMAI_GTM_DMA_CAPTURE
                       ,
                       (unsigned long)s_gtm_dma_chunks,
                       (unsigned long)s_gtm_dma_words,
                       (unsigned long)s_gtm_dma_samples,
                       (unsigned long)s_gtm_dma_lost,
                       (unsigned)IfxDma_getChannelTransferCount(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL),
                       (unsigned long)MODULE_DMA.CH[GUIMAI_GTM_DMA_CHANNEL].CHCSR.U
#endif
#if GUIMAI_GTM_TBCM_DIAG
                       ,
                       (unsigned long)s_gtm_tbcm_polls,
                       (unsigned long)s_gtm_tbcm_events,
                       (unsigned long)s_gtm_tbcm_newval,
                       (unsigned long)s_gtm_tbcm_gprofl,
                       (unsigned long)s_gtm_tbcm_last_gpr1,
                       (unsigned)s_gtm_tbcm_last_ecnt,
                       (unsigned long)s_gtm_tbcm_ws_one,
                       (unsigned long)s_gtm_tbcm_sd_one,
                       (unsigned long)s_gtm_tbcm_last_irq,
                       (unsigned long)s_gtm_tbcm_last_cnts,
                       (unsigned long)((s_gtm_tbcm_ch != NULL_PTR) ? s_gtm_tbcm_ch->CTRL.U : 0U),
                       (unsigned long)((s_gtm_tbcm_ch != NULL_PTR) ? s_gtm_tbcm_ch->ECTRL.U : 0U),
                       (unsigned long)((s_gtm_tbcm_ch != NULL_PTR) ? s_gtm_tbcm_ch->TDUV.U : 0U),
                       (unsigned long)((s_gtm_tbcm_ch != NULL_PTR) ? s_gtm_tbcm_ch->TDUC.U : 0U)
#if GUIMAI_GTM_ARU_FIFO_DIAG
                       ,
                       (unsigned long)s_gtm_aru_fifo_enable,
#if GUIMAI_GTM_AFD_DMA_DIAG
                       (unsigned long)s_gtm_afd_dma_chunks,
                       (unsigned long)s_gtm_afd_dma_words_total,
                       (unsigned long)s_gtm_afd_dma_lost,
                       (unsigned long)s_gtm_afd_dma_tcnt,
                       (unsigned long)s_gtm_afd_dma_last_word,
                       (unsigned long)s_gtm_afd_dma_chcsr,
#endif
                       (unsigned long)s_gtm_aru_fifo_fill[0],
                       (unsigned long)s_gtm_aru_fifo_max_fill[0],
                       (unsigned long)s_gtm_aru_fifo_wr[0],
                       (unsigned long)s_gtm_aru_fifo_rd[0],
                       (unsigned long)s_gtm_aru_fifo_status[0],
                       (unsigned long)s_gtm_aru_fifo_irq[0],
                       (unsigned long)s_gtm_aru_fifo_stream_state[0],
                       (unsigned long)s_gtm_aru_fifo_str_cfg[0],
                       (unsigned long)s_gtm_aru_fifo_aru_addr[0],
                       (unsigned long)s_gtm_aru_fifo_fill[1],
                       (unsigned long)s_gtm_aru_fifo_max_fill[1],
                       (unsigned long)s_gtm_aru_fifo_wr[1],
                       (unsigned long)s_gtm_aru_fifo_rd[1],
                       (unsigned long)s_gtm_aru_fifo_status[1],
                       (unsigned long)s_gtm_aru_fifo_irq[1],
                       (unsigned long)s_gtm_aru_fifo_stream_state[1],
                       (unsigned long)s_gtm_aru_fifo_str_cfg[1],
                       (unsigned long)s_gtm_aru_fifo_aru_addr[1],
                       (unsigned long)s_gtm_aru_fifo_fill[2],
                       (unsigned long)s_gtm_aru_fifo_max_fill[2],
                       (unsigned long)s_gtm_aru_fifo_wr[2],
                       (unsigned long)s_gtm_aru_fifo_rd[2],
                       (unsigned long)s_gtm_aru_fifo_status[2],
                       (unsigned long)s_gtm_aru_fifo_irq[2],
                       (unsigned long)s_gtm_aru_fifo_stream_state[2],
                       (unsigned long)s_gtm_aru_fifo_str_cfg[2],
                       (unsigned long)s_gtm_aru_fifo_aru_addr[2]
#endif
#endif
#if GUIMAI_QSPI1_SLAVE_RX
                       ,
                       (unsigned long)s_qspi1_rx_words,
                       (unsigned)IfxQspi_getReceiveFifoLevel(&MODULE_QSPI1),
                       (unsigned long)s_qspi1_last_error,
                       (unsigned long)MODULE_QSPI1.STATUS.U,
                       (unsigned)s_qspi1_sync_ok
#endif
                       );
#if GUIMAI_GTM_DMA_CAPTURE
                if ((no_dma_diag_printed == 0U) && (s_gtm_dma_chunks == 0U))
                {
                    no_dma_diag_printed = 1U;
                    printf("[GUIMAI_DMA_NOEVT] raw bck=%lu ws=%lu sd=%lu mclk=%lu lvl=%u/%u/%u/%u dadr=0x%08lX tcnt=%u chcsr=0x%08lX tim_ctrl=0x%08lX irq_en=0x%08lX notify=0x%08lX\r\n",
                           (unsigned long)guimai_pin_toggle_count(HORN_BCK_PIN, GUIMAI_IIS_SCAN_LOOPS),
                           (unsigned long)guimai_pin_toggle_count(HORN_WS_PIN, GUIMAI_IIS_SCAN_LOOPS),
                           (unsigned long)guimai_pin_toggle_count(ES8388_ADC_SDOUT_PIN, GUIMAI_IIS_SCAN_LOOPS),
                           (unsigned long)guimai_pin_toggle_count(HORN_MCLK_PIN, GUIMAI_IIS_SCAN_LOOPS),
                           (unsigned)gpio_get_level(HORN_BCK_PIN),
                           (unsigned)gpio_get_level(HORN_WS_PIN),
                           (unsigned)gpio_get_level(ES8388_ADC_SDOUT_PIN),
                           (unsigned)gpio_get_level(HORN_MCLK_PIN),
                           (unsigned long)IfxDma_getChannelDestinationAddress(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL),
                           (unsigned)IfxDma_getChannelTransferCount(&MODULE_DMA, GUIMAI_GTM_DMA_CHANNEL),
                           (unsigned long)MODULE_DMA.CH[GUIMAI_GTM_DMA_CHANNEL].CHCSR.U,
                           (unsigned long)((s_gtm_dma_tim_ch != NULL_PTR) ? s_gtm_dma_tim_ch->CTRL.U : 0U),
                           (unsigned long)((s_gtm_dma_tim_ch != NULL_PTR) ? s_gtm_dma_tim_ch->IRQ.EN.U : 0U),
                           (unsigned long)((s_gtm_dma_tim_ch != NULL_PTR) ? s_gtm_dma_tim_ch->IRQ.NOTIFY.U : 0U));
                }
#endif
#if GUIMAI_GTM_TBCM_DIAG
                if ((no_event_diag_printed == 0U) &&
                    (s_gtm_tbcm_polls > 500U) &&
                    (s_gtm_tbcm_events == 0U))
                {
                    no_event_diag_printed = 1U;
                    guimai_gtm_tbcm_no_event_diag();
                }
#endif
            }
        }
    }

#if GUIMAI_AUDIO_USE_ADC
    guimai_adc_drain_tail();
#endif
    {
#if GUIMAI_RECORD_STREAM_TX
        uint16 tail_samples = s_voice_pos;
#endif
        guimai_linein_stop();
#if GUIMAI_RECORD_STREAM_TX
        if (tail_samples > 0U)
        {
            const int16 *tail_buf = (GUIMAI_BUF_0 == s_fill_buf) ? &s_mic_voice[GUIMAI_BUF_0][0] : &s_mic_voice[GUIMAI_BUF_1][0];
            (void)guimai_stream_tx_queue_frame(1U, tail_buf, tail_samples);
        }
        (void)guimai_stream_tx_queue_frame(2U, NULL_PTR, 0U);
#endif
    }
    s_record_core_running = 0U;
    s_record_core_req = 0U;
    s_record_core_stop_req = 0U;
    s_record_core_done = 1U;
    printf("[GUIMAI_REC] capture end\r\n");
}
#endif

static uint8 guimai_trigger_enabled(void)
{
    return ((ui_task_get() == GUIMAI_TRIGGER_TASK_ID) && (Page_Num == GUIMAI_TRIGGER_PAGE_ID)) ? 1 : 0;
}

static uint8 guimai_switch_read_level(void)
{
#if GUIMAI_TRIGGER_USE_XL9555_KEY3
    return xl9555_app_get_key(GUIMAI_XL9555_KEY_NUM) ? GPIO_HIGH : GPIO_LOW;
#else
    return gpio_get_level(GUIMAI_SWITCH_IO);
#endif
}

static void guimai_switch_init(void)
{
#if GUIMAI_TRIGGER_USE_XL9555_KEY3
    xl9555_app_init();
#else
    gpio_init(GUIMAI_SWITCH_IO, GPI, 1, GPI_PULL_UP);
#endif
    system_delay_ms(50);
    s_switch_origin_level = guimai_switch_read_level();
    s_switch_state = s_switch_origin_level;
}

static void guimai_switch_scan(void)
{
    static uint8 last_switch = GPIO_HIGH;

    if (!guimai_trigger_enabled())
    {
        // Keep last_switch aligned to physical key level while trigger is disabled,
        // so entering Task2/Page1 does not create a fake edge event.
        s_switch_state = guimai_switch_read_level();
        last_switch = s_switch_state;
        s_switch_event = 0;
        return;
    }

    s_switch_state = guimai_switch_read_level();

    if (s_switch_state != last_switch)
    {
        system_delay_ms(20);
        s_switch_state = guimai_switch_read_level();
        if (s_switch_state != last_switch)
        {
            s_switch_event = 1;
        }
    }
    last_switch = s_switch_state;
}

static void guimai_lora_init(void)
{
    uart_init(GUIMAI_LORA_UART, GUIMAI_LORA_BAUD, GUIMAI_LORA_RX_PIN, GUIMAI_LORA_TX_PIN);
    s_lora_ready = 1;
}

static void guimai_voice_init(void)
{
#if GUIMAI_AUDIO_USE_ADC
    printf("[GUIMAI] voice source: TC387 ADC ch=%u, %lu Hz/16bit/mono PCM\r\n",
           (unsigned)GUIMAI_ADC_CHANNEL, (unsigned long)GUIMAI_AUDIO_SAMPLE_RATE);
#else
    printf("[GUIMAI] voice source: ES8388 LINEIN (21/22).\r\n");
#endif
}

static void guimai_wifi_init(void)
{
    uint8 i = 0;

    guimai_wifi_ui_status("WIFI INIT");
    printf("[GUIMAI] wifi target: %s:%s\r\n", GUIMAI_WIFI_TARGET_IP, GUIMAI_WIFI_TARGET_PORT);

    for (i = 0; i < GUIMAI_WIFI_INIT_RETRY; i++)
    {
        printf("[GUIMAI] wifi init try %u/%u ssid=%s\r\n",
               (unsigned)(i + 1U),
               (unsigned)GUIMAI_WIFI_INIT_RETRY,
               GUIMAI_WIFI_SSID);
        if (0 == wifi_uart_init((char *)GUIMAI_WIFI_SSID, (char *)GUIMAI_WIFI_PASSWORD, WIFI_UART_STATION))
        {
            printf("[GUIMAI] wifi init ok.\r\n");
            break;
        }
        printf("[GUIMAI] wifi init try failed.\r\n");
        system_delay_ms(300);
    }

    if (i >= GUIMAI_WIFI_INIT_RETRY)
    {
        s_wifi_ready = 0;
        guimai_wifi_ui_status("WIFI FAIL");
        printf("[GUIMAI] wifi_uart_init failed.\r\n");
        return;
    }

    if (1 != WIFI_UART_AUTO_CONNECT)
    {
        for (i = 0; i < GUIMAI_WIFI_INIT_RETRY; i++)
        {
            printf("[GUIMAI] wifi tcp try %u/%u\r\n",
                   (unsigned)(i + 1U),
                   (unsigned)GUIMAI_WIFI_INIT_RETRY);
            if (0 == wifi_uart_connect_tcp_servers((char *)GUIMAI_WIFI_TARGET_IP, (char *)GUIMAI_WIFI_TARGET_PORT, WIFI_UART_SERIANET))
            {
                s_wifi_ready = 1;
                guimai_wifi_ui_status("TCP OK  ");
                printf("[GUIMAI] wifi tcp connected: %s:%s\r\n", GUIMAI_WIFI_TARGET_IP, GUIMAI_WIFI_TARGET_PORT);
                return;
            }
            system_delay_ms(300);
        }
        s_wifi_ready = 0;
        guimai_wifi_ui_status("TCP FAIL");
        printf("[GUIMAI] wifi tcp connect failed.\r\n");
    }
    else
    {
        s_wifi_ready = 1;
        guimai_wifi_ui_status("WIFI OK ");
    }
}

static void guimai_fill_audio_payload(uint8 flag)
{
    const int16 *src = NULL;

    s_frame[4] = flag;

    if (3 == flag)
    {
        memset(&s_frame[5], 0xDD, GUIMAI_FRAME_PAYLOAD_SIZE);
        return;
    }

    if (0 == flag)
    {
        memset(&s_frame[5], 0, GUIMAI_FRAME_PAYLOAD_SIZE);
#if GUIMAI_RECORD_DEFER_TX
        s_frame[5] = 'G';
        s_frame[6] = 'M';
        s_frame[7] = 'D';
        s_frame[8] = '1';
        s_frame[9] = (uint8)(GUIMAI_AUDIO_SAMPLE_RATE & 0xFFU);
        s_frame[10] = (uint8)((GUIMAI_AUDIO_SAMPLE_RATE >> 8U) & 0xFFU);
        s_frame[11] = (uint8)((GUIMAI_AUDIO_SAMPLE_RATE >> 16U) & 0xFFU);
        s_frame[12] = (uint8)((GUIMAI_AUDIO_SAMPLE_RATE >> 24U) & 0xFFU);
        s_frame[13] = (uint8)(s_record_samples & 0xFFU);
        s_frame[14] = (uint8)((s_record_samples >> 8U) & 0xFFU);
        s_frame[15] = (uint8)((s_record_samples >> 16U) & 0xFFU);
        s_frame[16] = (uint8)((s_record_samples >> 24U) & 0xFFU);
        s_frame[17] = (uint8)(s_record_elapsed_ms & 0xFFU);
        s_frame[18] = (uint8)((s_record_elapsed_ms >> 8U) & 0xFFU);
        s_frame[19] = (uint8)((s_record_elapsed_ms >> 16U) & 0xFFU);
        s_frame[20] = (uint8)((s_record_elapsed_ms >> 24U) & 0xFFU);
#endif
        return;
    }

    src = (GUIMAI_BUF_0 == s_fill_buf) ? &s_mic_voice[GUIMAI_BUF_1][0] : &s_mic_voice[GUIMAI_BUF_0][0];
    memcpy(&s_frame[5], (const uint8 *)src, GUIMAI_FRAME_PAYLOAD_SIZE);
}

#if GUIMAI_RECORD_STREAM_TX
static void guimai_fill_stream_frame(uint8 *frame, uint8 flag, const int16 *samples, uint32 sample_count)
{
    uint32 copy_samples = sample_count;

    frame[0] = GUIMAI_FRAME_HEADER_0;
    frame[1] = GUIMAI_FRAME_HEADER_1;
    frame[2] = GUIMAI_FRAME_HEADER_2;
    frame[3] = GUIMAI_FRAME_HEADER_3;
    frame[4] = flag;
    memset(&frame[5], 0, GUIMAI_FRAME_PAYLOAD_SIZE);

    if ((0U == flag) || ((2U == flag) && (samples == NULL_PTR)))
    {
        frame[5] = 'G';
        frame[6] = 'M';
        frame[7] = 'D';
        frame[8] = '1';
        frame[9] = (uint8)(GUIMAI_AUDIO_SAMPLE_RATE & 0xFFU);
        frame[10] = (uint8)((GUIMAI_AUDIO_SAMPLE_RATE >> 8U) & 0xFFU);
        frame[11] = (uint8)((GUIMAI_AUDIO_SAMPLE_RATE >> 16U) & 0xFFU);
        frame[12] = (uint8)((GUIMAI_AUDIO_SAMPLE_RATE >> 24U) & 0xFFU);
        frame[13] = (uint8)(s_record_samples & 0xFFU);
        frame[14] = (uint8)((s_record_samples >> 8U) & 0xFFU);
        frame[15] = (uint8)((s_record_samples >> 16U) & 0xFFU);
        frame[16] = (uint8)((s_record_samples >> 24U) & 0xFFU);
        frame[17] = (uint8)(s_record_elapsed_ms & 0xFFU);
        frame[18] = (uint8)((s_record_elapsed_ms >> 8U) & 0xFFU);
        frame[19] = (uint8)((s_record_elapsed_ms >> 16U) & 0xFFU);
        frame[20] = (uint8)((s_record_elapsed_ms >> 24U) & 0xFFU);
        return;
    }

    if ((samples != NULL_PTR) && (copy_samples > 0U))
    {
        if (copy_samples > GUIMAI_MIC_BUF_LEN)
        {
            copy_samples = GUIMAI_MIC_BUF_LEN;
        }
        memcpy(&frame[5], (const uint8 *)samples, copy_samples * sizeof(int16));
    }
}

static uint8 *guimai_stream_tx_frame_ptr(uint8 index)
{
    uint32 addr = (uint32)&s_stream_tx_queue[index][0];
#if GUIMAI_STREAM_TX_UNCACHED_ALIAS
    if ((addr >= 0x90000000UL) && (addr < 0xA0000000UL))
    {
        addr += 0x20000000UL;
    }
#endif
    return (uint8 *)addr;
}

static volatile uint8 *guimai_uncached_u8_ptr(volatile uint8 *ptr)
{
    uint32 addr = (uint32)ptr;
#if GUIMAI_STREAM_TX_UNCACHED_ALIAS
    if ((addr >= 0x90000000UL) && (addr < 0xA0000000UL))
    {
        addr += 0x20000000UL;
    }
#endif
    return (volatile uint8 *)addr;
}

#define GUIMAI_STREAM_TX_HEAD      (*guimai_uncached_u8_ptr(&s_stream_tx_head))
#define GUIMAI_STREAM_TX_TAIL      (*guimai_uncached_u8_ptr(&s_stream_tx_tail))
#define GUIMAI_STREAM_TX_ACTIVE    (*guimai_uncached_u8_ptr(&s_stream_tx_active))
#define GUIMAI_STREAM_TX_OVERFLOW  (*guimai_uncached_u8_ptr(&s_stream_tx_overflow))

static void guimai_stream_tx_reset(void)
{
    GUIMAI_STREAM_TX_HEAD = 0U;
    GUIMAI_STREAM_TX_TAIL = 0U;
    GUIMAI_STREAM_TX_ACTIVE = 0U;
    GUIMAI_STREAM_TX_OVERFLOW = 0U;
    __dsync();
}

static uint8 guimai_stream_tx_queue_frame(uint8 flag, const int16 *samples, uint32 sample_count)
{
    uint8 head = GUIMAI_STREAM_TX_HEAD;
    uint8 next_head = (uint8)((head + 1U) % GUIMAI_STREAM_TX_QUEUE_DEPTH);
#if GUIMAI_STREAM_TX_DEBUG
    static uint32 q_count = 0U;
#endif

    if (!s_wifi_ready)
    {
#if GUIMAI_STREAM_TX_DEBUG
        printf("[GUIMAI_STREAM_Q] drop no_wifi flag=%u samples=%lu\r\n",
               (unsigned)flag,
               (unsigned long)sample_count);
#endif
        return 0U;
    }

    if (next_head == GUIMAI_STREAM_TX_TAIL)
    {
        GUIMAI_STREAM_TX_OVERFLOW = 1U;
        if (2U == flag)
        {
            uint8 old_tail = GUIMAI_STREAM_TX_TAIL;
            GUIMAI_STREAM_TX_TAIL = (uint8)((old_tail + 1U) % GUIMAI_STREAM_TX_QUEUE_DEPTH);
            guimai_record_ui_status("TX END ");
#if GUIMAI_STREAM_TX_DEBUG
            printf("[GUIMAI_STREAM_Q] force last drop tail=%u->%u head=%u\r\n",
                   (unsigned)old_tail,
                   (unsigned)GUIMAI_STREAM_TX_TAIL,
                   (unsigned)GUIMAI_STREAM_TX_HEAD);
#endif
        }
        else
        {
            guimai_record_ui_status("TX FULL");
#if GUIMAI_STREAM_TX_DEBUG
            printf("[GUIMAI_STREAM_Q] full flag=%u head=%u tail=%u\r\n",
                   (unsigned)flag,
                   (unsigned)GUIMAI_STREAM_TX_HEAD,
                   (unsigned)GUIMAI_STREAM_TX_TAIL);
#endif
            return 0U;
        }
    }

    guimai_fill_stream_frame(guimai_stream_tx_frame_ptr(head), flag, samples, sample_count);
    __dsync();
    GUIMAI_STREAM_TX_HEAD = next_head;
    GUIMAI_STREAM_TX_ACTIVE = 1U;
#if GUIMAI_STREAM_TX_DEBUG
    q_count++;
    if ((flag != 1U) || ((q_count % 20U) == 1U))
    {
        printf("[GUIMAI_STREAM_Q] flag=%u samples=%lu head=%u tail=%u count=%lu\r\n",
               (unsigned)flag,
               (unsigned long)sample_count,
               (unsigned)GUIMAI_STREAM_TX_HEAD,
               (unsigned)GUIMAI_STREAM_TX_TAIL,
               (unsigned long)q_count);
    }
#endif
    return 1U;
}

static void guimai_stream_tx_send_task(void)
{
    uint8 tail = GUIMAI_STREAM_TX_TAIL;
    uint8 *frame = guimai_stream_tx_frame_ptr(tail);
    uint32 remain = 0U;
#if GUIMAI_STREAM_TX_DEBUG
    static uint32 tx_count = 0U;
#endif

    if (tail == GUIMAI_STREAM_TX_HEAD)
    {
        if (GUIMAI_STREAM_TX_ACTIVE && !s_streaming && !s_linein_running && !s_record_core_running)
        {
            GUIMAI_STREAM_TX_ACTIVE = 0U;
            if (GUIMAI_STREAM_TX_OVERFLOW)
            {
                guimai_record_ui_status("TX LOST");
            }
            else
            {
                guimai_record_ui_status("SENT   ");
            }
        }
        return;
    }

    if (!s_wifi_ready)
    {
        return;
    }

    __dsync();
#if GUIMAI_STREAM_TX_DIRECT_UART
    uart_write_buffer(WIFI_UART_INDEX, frame, GUIMAI_FRAME_SIZE);
    remain = 0U;
#else
    remain = wifi_uart_send_buffer(frame, GUIMAI_FRAME_SIZE);
#endif
    __dsync();
    if (remain != 0U)
    {
        GUIMAI_STREAM_TX_OVERFLOW = 1U;
        guimai_record_ui_status("TX WAIT");
#if GUIMAI_STREAM_TX_DEBUG
        printf("[GUIMAI_STREAM_TX] remain=%lu flag=%u head=%u tail=%u\r\n",
               (unsigned long)remain,
               (unsigned)frame[4],
               (unsigned)GUIMAI_STREAM_TX_HEAD,
               (unsigned)GUIMAI_STREAM_TX_TAIL);
#endif
        return;
    }
    GUIMAI_STREAM_TX_TAIL = (uint8)((tail + 1U) % GUIMAI_STREAM_TX_QUEUE_DEPTH);
#if GUIMAI_STREAM_TX_DEBUG
    tx_count++;
    if ((frame[4] != 1U) || ((tx_count % 20U) == 1U))
    {
        printf("[GUIMAI_STREAM_TX] flag=%u sent=%u head=%u tail=%u count=%lu first=%02X %02X %02X %02X\r\n",
               (unsigned)frame[4],
               (unsigned)GUIMAI_FRAME_SIZE,
               (unsigned)GUIMAI_STREAM_TX_HEAD,
               (unsigned)GUIMAI_STREAM_TX_TAIL,
               (unsigned long)tx_count,
               (unsigned)frame[0],
               (unsigned)frame[1],
               (unsigned)frame[2],
               (unsigned)frame[3]);
    }
#endif

    if (GUIMAI_STREAM_TX_TAIL == GUIMAI_STREAM_TX_HEAD)
    {
        if (!s_streaming && !s_linein_running && !s_record_core_running)
        {
            GUIMAI_STREAM_TX_ACTIVE = 0U;
            if (GUIMAI_STREAM_TX_OVERFLOW)
            {
                guimai_record_ui_status("TX LOST");
            }
            else
            {
                guimai_record_ui_status("SENT   ");
            }
        }
    }
}
#endif

#if GUIMAI_RECORD_DEFER_TX
static void guimai_fill_audio_payload_from_store(uint8 flag, uint32 frame_index)
{
    uint32 sample_offset = frame_index * GUIMAI_MIC_BUF_LEN;
    uint32 remain = 0;
    uint32 copy_samples = 0;

    s_frame[4] = flag;
    memset(&s_frame[5], 0, GUIMAI_FRAME_PAYLOAD_SIZE);

    if (sample_offset >= s_record_samples)
    {
        return;
    }

    remain = s_record_samples - sample_offset;
    copy_samples = (remain > GUIMAI_MIC_BUF_LEN) ? GUIMAI_MIC_BUF_LEN : remain;
    memcpy(&s_frame[5], (const uint8 *)&s_record_store[sample_offset], copy_samples * sizeof(int16));
}

static void guimai_send_stored_audio(void)
{
    uint32 frame_count = 0;
#if GUIMAI_RECORD_TRIM_LEADING_ZERO
    uint32 trim_samples = 0;
#endif

    if (!s_wifi_ready)
    {
        guimai_record_ui_status("NO WIFI");
        s_record_tx_active = 0U;
        return;
    }

#if GUIMAI_RECORD_TRANSFORM_DIAG
    guimai_record_transform_diag();
#endif

#if GUIMAI_GTM_TBCM_DIAG
    printf("[GUIMAI_STORE] skip tx in TBCM diag: samples=%lu elapsed_ms=%lu tbcm_events=%lu gpr1=0x%06lX ws1=%lu sd1=%lu\r\n",
           (unsigned long)s_record_samples,
           (unsigned long)s_record_elapsed_ms,
           (unsigned long)s_gtm_tbcm_events,
           (unsigned long)s_gtm_tbcm_last_gpr1,
           (unsigned long)s_gtm_tbcm_ws_one,
           (unsigned long)s_gtm_tbcm_sd_one);
#if GUIMAI_GTM_ARU_FIFO_DIAG
    printf("[GUIMAI_STORE] fifo diag: polls=%lu f2a_en=0x%08lX f0=%lu/%lu/%lu/%lu/%lu/0x%lX/%lu/0x%08lX/0x%03lX f1=%lu/%lu/%lu/%lu/%lu/0x%lX/%lu/0x%08lX/0x%03lX f2=%lu/%lu/%lu/%lu/%lu/0x%lX/%lu/0x%08lX/0x%03lX\r\n",
           (unsigned long)s_gtm_aru_fifo_polls,
           (unsigned long)s_gtm_aru_fifo_enable,
           (unsigned long)s_gtm_aru_fifo_fill[0],
           (unsigned long)s_gtm_aru_fifo_max_fill[0],
           (unsigned long)s_gtm_aru_fifo_wr[0],
           (unsigned long)s_gtm_aru_fifo_rd[0],
           (unsigned long)s_gtm_aru_fifo_status[0],
           (unsigned long)s_gtm_aru_fifo_irq[0],
           (unsigned long)s_gtm_aru_fifo_stream_state[0],
           (unsigned long)s_gtm_aru_fifo_str_cfg[0],
           (unsigned long)s_gtm_aru_fifo_aru_addr[0],
           (unsigned long)s_gtm_aru_fifo_fill[1],
           (unsigned long)s_gtm_aru_fifo_max_fill[1],
           (unsigned long)s_gtm_aru_fifo_wr[1],
           (unsigned long)s_gtm_aru_fifo_rd[1],
           (unsigned long)s_gtm_aru_fifo_status[1],
           (unsigned long)s_gtm_aru_fifo_irq[1],
           (unsigned long)s_gtm_aru_fifo_stream_state[1],
           (unsigned long)s_gtm_aru_fifo_str_cfg[1],
           (unsigned long)s_gtm_aru_fifo_aru_addr[1],
           (unsigned long)s_gtm_aru_fifo_fill[2],
           (unsigned long)s_gtm_aru_fifo_max_fill[2],
           (unsigned long)s_gtm_aru_fifo_wr[2],
           (unsigned long)s_gtm_aru_fifo_rd[2],
           (unsigned long)s_gtm_aru_fifo_status[2],
           (unsigned long)s_gtm_aru_fifo_irq[2],
           (unsigned long)s_gtm_aru_fifo_stream_state[2],
           (unsigned long)s_gtm_aru_fifo_str_cfg[2],
           (unsigned long)s_gtm_aru_fifo_aru_addr[2]);
#endif
    return;
#endif

#if GUIMAI_RECORD_TRIM_LEADING_ZERO
    while ((trim_samples < s_record_samples) && (s_record_store[trim_samples] == 0))
    {
        trim_samples++;
    }
    if ((trim_samples > 0U) && (trim_samples < s_record_samples))
    {
        uint32 remain_samples = s_record_samples - trim_samples;
        memmove(s_record_store, &s_record_store[trim_samples], remain_samples * sizeof(int16));
        if ((s_record_elapsed_ms > 0U) && (s_record_samples > 0U))
        {
            uint32 trim_ms = (trim_samples * s_record_elapsed_ms) / s_record_samples;
            s_record_elapsed_ms = (s_record_elapsed_ms > trim_ms) ? (s_record_elapsed_ms - trim_ms) : s_record_elapsed_ms;
        }
        s_record_samples = remain_samples;
        printf("[GUIMAI_STORE] trim leading_zero=%lu remain=%lu elapsed_ms=%lu\r\n",
               (unsigned long)trim_samples,
               (unsigned long)s_record_samples,
               (unsigned long)s_record_elapsed_ms);
    }
    else if ((trim_samples > 0U) && (trim_samples >= s_record_samples))
    {
        printf("[GUIMAI_STORE] trim skipped all_zero=%lu\r\n",
               (unsigned long)s_record_samples);
    }
#endif

    frame_count = (s_record_samples + GUIMAI_MIC_BUF_LEN - 1U) / GUIMAI_MIC_BUF_LEN;
    printf("[GUIMAI_STORE] send samples=%lu elapsed_ms=%lu fs=%lu frames=%lu overflow=%u\r\n",
           (unsigned long)s_record_samples,
           (unsigned long)s_record_elapsed_ms,
           (s_record_elapsed_ms > 0U) ? (unsigned long)((s_record_samples * 1000U) / s_record_elapsed_ms) : 0UL,
           (unsigned long)frame_count,
           (unsigned)s_record_overflow);

    s_record_tx_frame_count = frame_count;
    s_record_tx_frame_index = 0U;
    s_record_tx_sent_first = 0U;
    s_record_tx_active = 1U;
    guimai_record_ui_status("SEND   ");
}

static void guimai_send_stored_audio_task(void)
{
    uint8 flag = 1U;
    uint32 sent = 0U;
    uint32 start_ms = 0U;
    uint32 elapsed = 0U;

    if (!s_record_tx_active)
    {
        return;
    }

    if (!s_wifi_ready)
    {
        s_record_tx_active = 0U;
        guimai_record_ui_status("NO WIFI");
        return;
    }

    if (!s_record_tx_sent_first)
    {
        s_record_tx_sent_first = 1U;
        guimai_send_frame(0);
        if (0U != s_record_tx_frame_count)
        {
            return;
        }
    }

    if (0U == s_record_tx_frame_count)
    {
        guimai_send_frame(2);
        s_record_tx_active = 0U;
        guimai_record_ui_status("SENT   ");
        return;
    }

    flag = ((s_record_tx_frame_index + 1U) >= s_record_tx_frame_count) ? 2U : 1U;
    guimai_fill_audio_payload_from_store(flag, s_record_tx_frame_index);
    start_ms = system_getval_ms();
    sent = wifi_uart_send_buffer((uint8 *)s_frame, GUIMAI_FRAME_SIZE);
    elapsed = guimai_elapsed_ms(start_ms, system_getval_ms());
    if ((s_record_tx_frame_index < 3U) || (elapsed > 20U) || (flag == 2U))
    {
        printf("[GUIMAI_STORE_TX] idx=%lu/%lu flag=%u sent=%lu ms=%lu\r\n",
               (unsigned long)(s_record_tx_frame_index + 1U),
               (unsigned long)s_record_tx_frame_count,
               (unsigned)flag,
               (unsigned long)sent,
               (unsigned long)elapsed);
    }

    s_record_tx_frame_index++;
    if (s_record_tx_frame_index >= s_record_tx_frame_count)
    {
        s_record_tx_active = 0U;
        guimai_record_ui_status("SENT   ");
    }
}
#endif

static void guimai_send_frame(uint8 flag)
{
    static uint32 s_continue_tx_count = 0;
    uint32 sent = 0;

    if (!s_wifi_ready)
    {
        return;
    }

    guimai_fill_audio_payload(flag);
    sent = wifi_uart_send_buffer((uint8 *)s_frame, GUIMAI_FRAME_SIZE);

#if GUIMAI_TX_DEBUG
    if (1 == flag)
    {
        s_continue_tx_count++;
        if ((s_continue_tx_count % 20U) == 1U)
        {
            printf("[GUIMAI_TX] flag=1 sent=%lu count=%lu\r\n",
                   (unsigned long)sent,
                   (unsigned long)s_continue_tx_count);
        }
    }
    else
    {
        printf("[GUIMAI_TX] flag=%u sent=%lu\r\n",
               (unsigned)flag,
               (unsigned long)sent);
    }
#endif
}

static void guimai_handle_stream_state(void)
{
    if (!guimai_trigger_enabled())
    {
        if (s_streaming)
        {
            guimai_voice_end_record();
        }
        s_switch_event = 0;
        return;
    }

    if (!s_switch_event)
    {
        return;
    }

    s_switch_event = 0;
    if (s_switch_state != s_switch_origin_level)
    {
        (void)guimai_voice_begin_record();
    }
    else
    {
        guimai_voice_end_record();
    }
}

static uint8 guimai_voice_begin_record(void)
{
#if GUIMAI_ASR_COMMAND_WAIT_ENABLE
    if (guimai_asr_command_wait_active())
    {
        guimai_record_ui_status("WAIT ASR");
        printf("[GUIMAI_ASR] reject record while waiting command elapsed=%lu\r\n",
               (unsigned long)guimai_asr_command_wait_elapsed(system_getval_ms()));
        return 0U;
    }
#endif
#if GUIMAI_HORN_ENABLE
    if (s_horn_active)
    {
        printf("[GUIMAI_HORN] stop for record\r\n");
        guimai_horn_stop();
    }
#endif
#if GUIMAI_RECORD_DEFER_TX
    if (s_record_tx_active)
    {
        guimai_record_ui_status("SEND   ");
        return 0U;
    }
#if GUIMAI_RECORD_STREAM_TX
    if (GUIMAI_STREAM_TX_ACTIVE)
    {
        guimai_record_ui_status("SEND   ");
        return 0U;
    }
#endif
#endif
#if GUIMAI_MUSIC_ENABLE
    if (s_music_playing)
    {
        s_record_pending_after_music = 1U;
#if GUIMAI_VOICE_EXTERNAL_TRIGGER
        s_record_pending_external = 1U;
#else
        s_record_pending_external = 0U;
#endif
        music_player_stop();
        printf("[GUIMAI] record start requested, stopping music first.\r\n");
        return 0U;
    }
#endif

#if GUIMAI_WIFI_INIT_ON_RECORD
    if (!s_wifi_ready)
    {
        printf("[GUIMAI] wifi lazy init before record...\r\n");
        guimai_wifi_init();
    }
#endif

    if (s_streaming || s_linein_running || s_record_core_running)
    {
        return 1U;
    }

#if GUIMAI_HORN_ENABLE
#if GUIMAI_AUDIO_USE_ADC
    printf("[GUIMAI_ADC] record begin ready=%u\r\n", (unsigned)s_linein_ready);
#else
    printf("[GUIMAI_AUDIO] reuse record path reason=record_begin ready=%u es=%u\r\n",
           (unsigned)s_linein_ready,
           (unsigned)s_es8388_ready);
#endif
#endif

    guimai_linein_start();
    if (!s_linein_running)
    {
        s_streaming = 0U;
        return 0U;
    }

    s_streaming = 1U;
#if GUIMAI_RECORD_DEFER_TX && GUIMAI_RECORD_STREAM_TX
    (void)guimai_stream_tx_queue_frame(0U, NULL_PTR, 0U);
#elif !GUIMAI_RECORD_DEFER_TX
    guimai_send_frame(0);
#endif
#if GUIMAI_RECORD_DEFER_TX && GUIMAI_RECORD_TIGHT_LOOP && GUIMAI_RECORD_USE_DEDICATED_CORE
    s_record_core_done = 0U;
    s_record_core_stop_req = 0U;
    s_record_core_req = 1U;
#elif GUIMAI_RECORD_DEFER_TX && GUIMAI_RECORD_TIGHT_LOOP
    guimai_linein_capture_until_release();
#endif
    return 1U;
}

static void guimai_voice_end_record(void)
{
    uint16 tail_samples = 0U;

    if (!(s_streaming || s_linein_running || s_record_core_running))
    {
        return;
    }

#if GUIMAI_RECORD_DEFER_TX
#if GUIMAI_RECORD_TIGHT_LOOP && GUIMAI_RECORD_USE_DEDICATED_CORE
    s_record_core_stop_req = 1U;
#else
#if GUIMAI_RECORD_STREAM_TX
    tail_samples = s_voice_pos;
#endif
    guimai_linein_stop();
#if GUIMAI_RECORD_STREAM_TX
    if (tail_samples > 0U)
    {
        const int16 *tail_buf = (GUIMAI_BUF_0 == s_fill_buf) ? &s_mic_voice[GUIMAI_BUF_0][0] : &s_mic_voice[GUIMAI_BUF_1][0];
        (void)guimai_stream_tx_queue_frame(1U, tail_buf, tail_samples);
    }
    (void)guimai_stream_tx_queue_frame(2U, NULL_PTR, 0U);
#else
    guimai_send_stored_audio();
#endif
#endif
#else
    guimai_send_frame(2);
    s_voice_frame_ready = 0;
    guimai_linein_stop();
#endif
    s_streaming = 0U;
#if GUIMAI_ASR_COMMAND_WAIT_ENABLE
    if (s_record_samples > 0U)
    {
        guimai_asr_command_wait_begin();
    }
    else
    {
        guimai_asr_command_wait_clear("empty");
    }
#endif
}

static void guimai_handle_audio_stream(void)
{
#if GUIMAI_RECORD_DEFER_TX
#if !GUIMAI_RECORD_STREAM_TX
    guimai_send_stored_audio_task();
    if (s_record_core_done)
    {
        s_record_core_done = 0U;
        guimai_send_stored_audio();
        s_streaming = 0;
    }
#else
    if (s_record_core_done)
    {
        s_record_core_done = 0U;
        s_streaming = 0;
    }
#endif
    s_voice_frame_ready = 0;
#else
    if (s_streaming && s_voice_frame_ready)
    {
        guimai_send_frame(1);
        s_voice_frame_ready = 0;
    }
#endif
}

static void guimai_record_auto_stop_task(void)
{
#if GUIMAI_RECORD_AUTO_STOP_ENABLE && GUIMAI_RECORD_DEFER_TX
    uint32 elapsed_ms;

    if ((GUIMAI_RECORD_AUTO_STOP_MS == 0U) || (!guimai_voice_is_recording()) || (s_record_start_ms == 0U))
    {
        return;
    }

#if GUIMAI_RECORD_TIGHT_LOOP && GUIMAI_RECORD_USE_DEDICATED_CORE
    if (s_record_core_stop_req != 0U)
    {
        return;
    }
#endif

    elapsed_ms = guimai_elapsed_ms(s_record_start_ms, system_getval_ms());
    if (elapsed_ms >= (uint32)GUIMAI_RECORD_AUTO_STOP_MS)
    {
        printf("[GUIMAI] record auto stop elapsed=%lu/%lu ms\r\n",
               (unsigned long)elapsed_ms,
               (unsigned long)GUIMAI_RECORD_AUTO_STOP_MS);
        guimai_record_ui_status("AUTO   ");
        guimai_voice_end_record();
    }
#endif
}

void guimai_voice_start_record(void)
{
    guimai_record_ui_status("START  ");
    (void)guimai_voice_begin_record();
}

void guimai_voice_stop_record(void)
{
    guimai_record_ui_status("STOP   ");
    guimai_voice_end_record();
}

uint8 guimai_voice_is_recording(void)
{
    return (s_streaming || s_linein_running || s_record_core_running) ? 1U : 0U;
}

uint8 guimai_music_play_first(void)
{
#if GUIMAI_MUSIC_ENABLE
    if (guimai_voice_is_recording())
    {
        printf("[GUIMAI_MUSIC] ignore play request while recording.\r\n");
        return 0U;
    }

    if (s_music_playing)
    {
        return 1U;
    }

    s_music_play_req = 1U;
    return 1U;
#else
    return 0U;
#endif
}

void guimai_music_stop(void)
{
#if GUIMAI_MUSIC_ENABLE
    s_music_play_req = 0U;
    s_record_pending_after_music = 0U;
    s_record_pending_external = 0U;
    if (s_music_playing)
    {
        music_player_stop();
        printf("[GUIMAI_MUSIC] stop requested\r\n");
    }
#endif
}

uint8 guimai_music_is_playing(void)
{
#if GUIMAI_MUSIC_ENABLE
    return (s_music_playing || s_music_play_req) ? 1U : 0U;
#else
    return 0U;
#endif
}

uint8 guimai_music_toggle_first(void)
{
#if GUIMAI_MUSIC_ENABLE
    if (guimai_music_is_playing())
    {
        guimai_music_stop();
        return 1U;
    }
    return guimai_music_play_first();
#else
    return 0U;
#endif
}

uint8 guimai_voice_get_command(uint8 *cmd)
{
    if ((NULL == cmd) || (s_voice_cmd_head == s_voice_cmd_tail))
    {
        return 0U;
    }

    *cmd = s_voice_cmd_queue[s_voice_cmd_tail];
    s_voice_cmd_tail = (uint8)((s_voice_cmd_tail + 1U) % GUIMAI_COMMAND_QUEUE_SIZE);
    return 1U;
}

static void guimai_send_to_lora(uint8 *data, uint8 len)
{
    if (!s_lora_ready || (NULL == data) || (0 == len))
    {
        return;
    }
    uart_write_buffer(GUIMAI_LORA_UART, data, len);
}

static void guimai_parse_rx_byte(uint8 value)
{
    uint8 i = 0;

    if (guimai_try_consume_text_byte(value))
    {
        return;
    }

    if (170 == value)
    {
        guimai_send_frame(3);
        return;
    }

    if (value >= 16)
    {
        s_normal_command = value;
#if GUIMAI_ASR_COMMAND_WAIT_ENABLE
        guimai_asr_command_wait_clear("cmd");
#endif
        guimai_voice_queue_command(s_normal_command);
        if (s_lora_ready)
        {
            uart_write_byte(GUIMAI_LORA_UART, s_normal_command);
        }
        return;
    }

    if ((0 == value) && s_command_in_progress)
    {
#if GUIMAI_ASR_COMMAND_WAIT_ENABLE
        guimai_asr_command_wait_clear("cmd_list");
#endif
        for (i = 0; i < s_command_count; i++)
        {
            guimai_voice_queue_command(s_command_buffer[i]);
        }
        guimai_send_to_lora(s_command_buffer, s_command_count);
        if (s_lora_ready)
        {
            uart_write_byte(GUIMAI_LORA_UART, 0);
        }
        s_command_in_progress = 0;
        s_command_count = 0;
        for (i = 0; i < (uint32)sizeof(s_command_buffer); i++)
        {
            s_command_buffer[i] = 0;
        }
        return;
    }

    if ((0 != value) && (!s_command_in_progress))
    {
        s_command_in_progress = 1;
        s_command_count = 0;
    }

    if ((0 != value) && s_command_in_progress)
    {
        if (s_command_count < (uint8)sizeof(s_command_buffer))
        {
            s_command_buffer[s_command_count++] = value;
        }
    }
}

static void guimai_wifi_parse(void)
{
    uint8 buf[64];
    uint32 got = 0U;
    uint32 i = 0U;

    if (!s_wifi_ready)
    {
        return;
    }

    got = wifi_uart_read_buffer(buf, (uint32)sizeof(buf));
    for (i = 0U; i < got; i++)
    {
        s_rx_byte = buf[i];
#if GUIMAI_RX_DEBUG
        if ((s_rx_byte >= 32) && (s_rx_byte <= 126))
        {
            printf("[WIFI_RX] 0x%02X '%c'\r\n", s_rx_byte, s_rx_byte);
        }
        else
        {
            printf("[WIFI_RX] 0x%02X '.'\r\n", s_rx_byte);
        }
#endif
        guimai_parse_rx_byte(s_rx_byte);
    }
}

void guimai_board_init(void)
{
    printf("[GUIMAI] fw=%s auto_probe=%u\r\n", GUIMAI_FW_TAG, (unsigned)GUIMAI_LINEIN_AUTO_PROBE);
#if GUIMAI_AUDIO_USE_ADC
    printf("[GUIMAI_BUILD] input=ADC channel=%u gain=%d fs=%lu horn=buzzer\r\n",
           (unsigned)GUIMAI_ADC_CHANNEL, (int)GUIMAI_ADC_PCM_GAIN,
           (unsigned long)GUIMAI_AUDIO_SAMPLE_RATE);
#else
    printf("[GUIMAI_BUILD] record_clock=%s horn=%u blocking=%u recover=8 mclk_ratio=256 relock=keep_mclk stop=keep_clock record_begin=reuse health=%u/%u/%u/%u horn_tx=%s realtime=1 bclkdiv=8 block_recover=defer bckmiss=%u release_iis=1 pre_mclk=%u/%u clock_pump=%u/%u tone_fs=%u asr_wait=%u auto_stop=%u/%u\r\n",
           ES8388_RECORD_MCU_MASTER ? "MCU_MASTER" : "CODEC_MASTER",
           (unsigned)GUIMAI_HORN_ENABLE,
           (unsigned)GUIMAI_HORN_BLOCKING_PLAY,
           (unsigned)GUIMAI_RECORD_START_HEALTH_ENABLE,
           (unsigned)GUIMAI_RECORD_START_HEALTH_SAMPLES,
           (unsigned)GUIMAI_RECORD_START_HEALTH_MIN_AVG,
           (unsigned)GUIMAI_RECORD_START_HEALTH_MIN_PEAK,
           GUIMAI_BUILD_HORN_TX,
           (unsigned)GUIMAI_BCK_CLOCK_MISS_DIAG,
           (unsigned)GUIMAI_RECORD_PRE_MCLK_ENABLE,
           (unsigned)GUIMAI_RECORD_PRE_MCLK_MS,
           (unsigned)GUIMAI_RECORD_CLOCK_PUMP_ENABLE,
           (unsigned)GUIMAI_RECORD_CLOCK_PUMP_MAX_FRAMES,
           (unsigned)GUIMAI_BUILD_TONE_SAMPLE_RATE,
           (unsigned)GUIMAI_ASR_COMMAND_WAIT_MS,
           (unsigned)GUIMAI_RECORD_AUTO_STOP_ENABLE,
           (unsigned)GUIMAI_RECORD_AUTO_STOP_MS);
#endif
#if !GUIMAI_VOICE_EXTERNAL_TRIGGER
    printf("[GUIMAI] init: switch...\r\n");
    guimai_switch_init();
#else
    printf("[GUIMAI] init: external voice trigger.\r\n");
#endif

#if GUIMAI_VOICE_INIT_LORA
    printf("[GUIMAI] init: lora...\r\n");
    guimai_lora_init();
#else
    s_lora_ready = 0U;
    printf("[GUIMAI] init: lora skipped.\r\n");
#endif

    printf("[GUIMAI] init: voice source...\r\n");
    guimai_voice_init();
#if GUIMAI_HORN_ENABLE && GUIMAI_HORN_USE_BUZZER
    guimai_horn_buzzer_init();
    printf("[GUIMAI_HORN_PWM] armed pin=P33_10 channel=ATOM3_CH0 duty=%lu/10000\r\n",
           (unsigned long)GUIMAI_HORN_PWM_DUTY);
#endif
    guimai_horn_pwm_test_init();

    guimai_audio_buffer_reset();
    s_linein_running = 0;
#if GUIMAI_LINEIN_INIT_ON_BOOT
    printf("[GUIMAI] init: linein warmup...\r\n");
    guimai_linein_init();
#else
    s_es8388_ready = 0;
    s_linein_ready = 0;
    printf("[GUIMAI] init: linein deferred.\r\n");
#endif

#if GUIMAI_WIFI_INIT_ON_BOOT
    printf("[GUIMAI] init: wifi...\r\n");
    guimai_wifi_init();
#else
    s_wifi_ready = 0U;
    printf("[GUIMAI] init: wifi deferred.\r\n");
#endif
    printf("[GUIMAI] init: done.\r\n");
}

void guimai_board_task(void)
{
#if GUIMAI_HORN_ENABLE
    guimai_horn_task(system_getval_ms());
#endif
#if GUIMAI_MUSIC_ENABLE
    guimai_music_handle_task();

    if (s_record_pending_after_music && (!s_music_playing))
    {
        uint8 should_start = s_record_pending_external;
#if !GUIMAI_VOICE_EXTERNAL_TRIGGER
        should_start = (guimai_trigger_enabled() && (guimai_switch_read_level() != s_switch_origin_level)) ? 1U : 0U;
#endif
        s_record_pending_after_music = 0U;
        s_record_pending_external = 0U;
        if (should_start)
        {
            (void)guimai_voice_begin_record();
        }
    }
#endif
#if !GUIMAI_VOICE_EXTERNAL_TRIGGER
    guimai_switch_scan();
    guimai_handle_stream_state();
#endif
#if !(GUIMAI_RECORD_DEFER_TX && GUIMAI_RECORD_TIGHT_LOOP && GUIMAI_RECORD_USE_DEDICATED_CORE)
    guimai_linein_task();
#endif
    guimai_record_auto_stop_task();
    guimai_handle_audio_stream();
#if GUIMAI_RECORD_STREAM_TX
    if (!GUIMAI_STREAM_TX_ACTIVE)
#endif
    {
        guimai_wifi_parse();
    }
    guimai_text_rx_timeout_task();
}

void guimai_board_record_core_task(void)
{
#if GUIMAI_RECORD_DEFER_TX && GUIMAI_RECORD_TIGHT_LOOP && GUIMAI_RECORD_USE_DEDICATED_CORE
    if (s_record_core_req && (!s_record_core_running))
    {
        guimai_record_core_capture_loop();
        return;
    }
#endif
#if GUIMAI_MUSIC_ENABLE
    music_player_core_task();
#endif
}

void guimai_board_tx_core_task(void)
{
#if GUIMAI_RECORD_STREAM_TX
    guimai_stream_tx_send_task();
#endif
}

void guimai_board_voice_isr(void)
{
    // Reserved for compatibility; audio acquisition runs through its DMA backend.
}

uint8 guimai_board_is_recording(void)
{
    return guimai_voice_is_recording();
}

const char *guimai_board_record_status_text(void)
{
    return s_record_status_text;
}

const char *guimai_board_wifi_status_text(void)
{
    return s_wifi_status_text;
}

uint8 guimai_board_is_music_playing(void)
{
    return guimai_music_is_playing();
}
