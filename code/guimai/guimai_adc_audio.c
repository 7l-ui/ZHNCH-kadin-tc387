#include "guimai_adc_audio.h"

#if GUIMAI_AUDIO_USE_ADC
#include "Evadc/Adc/IfxEvadc_Adc.h"
#include "Gtm/Atom/Pwm/IfxGtm_Atom_Pwm.h"
#include "Gtm/Std/IfxGtm_Cmu.h"
#include "Dma/Dma/IfxDma_Dma.h"
#include "Scu/Std/IfxScuWdt.h"
#include "Stm/Std/IfxStm.h"

#define AUDIO_DMA_CHANNEL IfxDma_ChannelId_20
#define AUDIO_RING_WORDS  2048U
#define AUDIO_RING_MASK   (AUDIO_RING_WORDS - 1U)
#define AUDIO_POLL_MAX_MS ((AUDIO_RING_WORDS * 1000U / GUIMAI_AUDIO_SAMPLE_RATE) - 8U)

static IfxEvadc_Adc s_adc;
static IfxEvadc_Adc_Group s_group;
static IfxEvadc_Adc_Channel s_channel;
static IfxGtm_Atom_Pwm_Driver s_timer;
static IfxDma_Dma s_dma;
static IfxDma_Dma_Channel s_dma_channel;
#pragma section all "lmubss"
IFX_ALIGN(8192) static volatile uint32 s_ring[AUDIO_RING_WORDS];
#pragma section all restore
static volatile uint32 *s_ring_uncached;
static uint32 s_ring_address;
static uint32 s_read_pos;
static uint32 s_write_pos;
static uint32 s_available;
static uint32 s_poll_ms;
static uint32 s_last_sample_ms;
static uint32 s_stm_ticks_per_ms;
static int32 s_dc_q16;
static uint8 s_dc_ready;
static uint8 s_ready;
static uint8 s_running;
static uint8 s_error;

/* Start runs on CPU0 and capture on CPU2: use one hardware time base. */
static uint32 audio_now_ms(void)
{
    return (uint32)(IfxStm_get(&MODULE_STM0) / s_stm_ticks_per_ms);
}

static uint8 audio_dma_configure(void)
{
    IfxDma_Dma_ChannelConfig config;
    uint32 wait;
    IfxDma_disableChannelTransaction(&MODULE_DMA, AUDIO_DMA_CHANNEL);
    IfxDma_resetChannel(&MODULE_DMA, AUDIO_DMA_CHANNEL);
    for (wait = 0U; wait < 100U; wait++)
    {
        if (IfxDma_isChannelReset(&MODULE_DMA, AUDIO_DMA_CHANNEL))
        {
            break;
        }
        system_delay_us(1U);
    }
    if (wait == 100U)
    {
        s_error = 4U;
        return 0U;
    }
    IfxDma_Dma_initChannelConfig(&config, &s_dma);
    config.channelId = AUDIO_DMA_CHANNEL;
    config.sourceAddress = (uint32)&MODULE_EVADC.G[0].RES[(uint32)GUIMAI_ADC_CHANNEL].U;
    config.destinationAddress = s_ring_address;
    config.sourceCircularBufferEnabled = TRUE;
    config.sourceAddressCircularRange = IfxDma_ChannelIncrementCircular_4;
    config.destinationCircularBufferEnabled = TRUE;
    config.destinationAddressCircularRange = IfxDma_ChannelIncrementCircular_8192;
    config.destinationAddressIncrementStep = IfxDma_ChannelIncrementStep_1;
    config.moveSize = IfxDma_ChannelMoveSize_32bit;
    config.transferCount = AUDIO_RING_WORDS;
    config.requestMode = IfxDma_ChannelRequestMode_oneTransferPerRequest;
    config.operationMode = IfxDma_ChannelOperationMode_continuous;
    config.blockMode = IfxDma_ChannelMove_1;
    config.busPriority = IfxDma_ChannelBusPriority_high;
    config.hardwareRequestEnabled = FALSE;
    config.channelInterruptEnabled = FALSE;
    config.transactionRequestLostInterruptEnabled = TRUE;
    IfxDma_Dma_initChannel(&s_dma_channel, &config);
    IfxDma_clearChannelTransactionRequestLost(&MODULE_DMA, AUDIO_DMA_CHANNEL);
    IfxDma_Dma_clearChannelInterrupt(&s_dma_channel);
    return 1U;
}

uint8 guimai_adc_audio_init(void)
{
    IfxEvadc_Adc_GroupConfig group_config;
    IfxEvadc_Adc_ChannelConfig channel_config;
    IfxGtm_Atom_Pwm_Config timer_config;
    IfxDma_Dma_Config dma_config;
    float32 clock_hz;
    uint32 period;
    uint16 password;

    if (s_ready)
    {
        return 1U;
    }
    /* Group 0 is reserved for audio; pedal acquisition owns group 2. */
    if ((uint32)GUIMAI_ADC_CHANNEL > 7U)
    {
        s_error = 1U;
        return 0U;
    }
    s_stm_ticks_per_ms = (uint32)IfxStm_getFrequency(&MODULE_STM0) / 1000U;
    if (s_stm_ticks_per_ms == 0U)
    {
        s_error = 2U;
        return 0U;
    }
    password = IfxScuWdt_getCpuWatchdogPassword();
    IfxScuWdt_clearCpuEndinit(password);
    IfxGtm_enable(&MODULE_GTM);
    IfxScuWdt_setCpuEndinit(password);
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0);
    clock_hz = IfxGtm_Cmu_getClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, TRUE);
    period = (uint32)(clock_hz / (float32)GUIMAI_AUDIO_SAMPLE_RATE + 0.5f);
    if ((period < 2U) || (period > 0xFFFFFFU))
    {
        s_error = 2U;
        return 0U;
    }

    /* Reuse the shared clock without changing motor/buzzer PWM frequencies. */
    IfxGtm_Atom_Pwm_initConfig(&timer_config, &MODULE_GTM);
    timer_config.atom = IfxGtm_Atom_1;
    timer_config.atomChannel = IfxGtm_Atom_Ch_4;
    timer_config.period = period;
    timer_config.dutyCycle = period / 2U;
    timer_config.synchronousUpdateEnabled = TRUE;
    timer_config.immediateStartEnabled = FALSE;
    IfxGtm_Atom_Pwm_init(&s_timer, &timer_config);
    IfxGtm_Atom_Ch_setClockSource(s_timer.atom, s_timer.atomChannel, IfxGtm_Cmu_Clk_0);
    IfxGtm_Atom_Pwm_stop(&s_timer, TRUE);
    /* TC38A iLLD EVADC trigger table: group0/trigger0 <- ATOM1_CH4 (0x9). */
    __ldmst_c(&MODULE_GTM.ADCTRIG[0].OUT0.U, 0xFU, 0x9U);

    /* Let the existing ADC driver initialize the module only once. */
    adc_init(GUIMAI_ADC_CHANNEL, ADC_12BIT);
    IfxEvadc_clearQueue(&MODULE_EVADC.G[0], TRUE, IfxEvadc_RequestSource_queue0);
    system_delay_us(10U);
    s_adc.evadc = &MODULE_EVADC;
    IfxEvadc_Adc_initGroupConfig(&group_config, &s_adc);
    group_config.groupId = IfxEvadc_GroupId_0;
    group_config.master = IfxEvadc_GroupId_0;
    group_config.arbiter.requestSlotQueue0Enabled = TRUE;
    group_config.queueRequest[0].triggerConfig.gatingMode = IfxEvadc_GatingMode_always;
    group_config.queueRequest[0].triggerConfig.triggerMode = IfxEvadc_TriggerMode_uponRisingEdge;
    group_config.queueRequest[0].triggerConfig.triggerSource = IfxEvadc_TriggerSource_8;
    group_config.inputClass[0].sampleTime = 1.0e-6f;
    if (IfxEvadc_Adc_initGroup(&s_group, &group_config) != IfxEvadc_Status_noError)
    {
        s_error = 3U;
        return 0U;
    }
    IfxEvadc_Adc_initChannelConfig(&channel_config, &s_group);
    channel_config.channelId = (IfxEvadc_ChannelId)GUIMAI_ADC_CHANNEL;
    channel_config.resultRegister = (IfxEvadc_ChannelResult)GUIMAI_ADC_CHANNEL;
    channel_config.rightAlignedStorage = FALSE;
    channel_config.resultPriority = (uint16)AUDIO_DMA_CHANNEL;
    channel_config.resultServProvider = IfxSrc_Tos_dma;
    channel_config.resultSrcNr = IfxEvadc_SrcNr_group0;
    if (IfxEvadc_Adc_initChannel(&s_channel, &channel_config) != IfxEvadc_Status_noError)
    {
        s_error = 3U;
        return 0U;
    }
    IfxSrc_disable(IfxEvadc_getSrcAddress(IfxEvadc_GroupId_0, IfxEvadc_SrcNr_group0));
    s_ring_address = (uint32)&s_ring[0];
    if ((s_ring_address & 0xF0000000U) == 0x90000000U)
    {
        s_ring_address += 0x20000000U;
    }
    s_ring_uncached = (volatile uint32 *)s_ring_address;
    IfxDma_Dma_initModuleConfig(&dma_config, &MODULE_DMA);
    IfxDma_Dma_initModule(&s_dma, &dma_config);
    if (!audio_dma_configure())
    {
        return 0U;
    }
    s_ready = 1U;
    s_error = 0U;
    printf("[GUIMAI_ADC] ready ch=%u fs=%lu period=%lu DMA20 ATOM1_CH4\r\n",
           (unsigned)GUIMAI_ADC_CHANNEL, (unsigned long)GUIMAI_AUDIO_SAMPLE_RATE,
           (unsigned long)period);
    return 1U;
}

void guimai_adc_audio_stop(void)
{
    uint32 wait;
    if (!s_running)
    {
        return;
    }
    IfxGtm_Atom_Pwm_stop(&s_timer, TRUE);
    if (!s_error && ((audio_now_ms() - s_poll_ms) >= AUDIO_POLL_MAX_MS))
    {
        s_error = 5U;
    }
    /* Finish an in-flight conversion before freezing the DMA write cursor. */
    system_delay_us(10U);
    IfxSrc_disable(IfxEvadc_getSrcAddress(IfxEvadc_GroupId_0, IfxEvadc_SrcNr_group0));
    for (wait = 0U; wait < 100U; wait++)
    {
        if (!IfxDma_isChannelTransactionPending(&MODULE_DMA, AUDIO_DMA_CHANNEL))
        {
            break;
        }
        system_delay_us(1U);
    }
    IfxDma_disableChannelTransaction(&MODULE_DMA, AUDIO_DMA_CHANNEL);
    if (wait == 100U)
    {
        s_error = 4U;
    }
    __dsync();
    s_running = 0U;
}

uint8 guimai_adc_audio_start(void)
{
    volatile Ifx_SRC_SRCR *src;
    if (!guimai_adc_audio_init())
    {
        return 0U;
    }
    guimai_adc_audio_stop();
    if (IfxDma_isChannelTransactionPending(&MODULE_DMA, AUDIO_DMA_CHANNEL))
    {
        s_error = 4U;
        return 0U;
    }
    src = IfxEvadc_getSrcAddress(IfxEvadc_GroupId_0, IfxEvadc_SrcNr_group0);
    IfxSrc_disable(src);
    IfxEvadc_clearQueue(s_group.group, TRUE, IfxEvadc_RequestSource_queue0);
    (void)IfxEvadc_getResult(s_group.group, s_channel.resultreg);
    IfxEvadc_clearAllResultRequests(s_group.group);
    IfxSrc_clearRequest(src);
    if (!audio_dma_configure())
    {
        return 0U;
    }
    s_read_pos = 0U;
    s_write_pos = 0U;
    s_available = 0U;
    s_dc_ready = 0U;
    s_dc_q16 = 0;
    s_error = 0U;
    s_poll_ms = audio_now_ms();
    s_last_sample_ms = s_poll_ms;
    /* Refilled queue entries must wait for a new timer edge, not free-run. */
    IfxEvadc_Adc_addToQueue(&s_channel, IfxEvadc_RequestSource_queue0,
                          IFXEVADC_QUEUE_REFILL | (1U << IFX_EVADC_G_Q_QINR_EXTR_OFF));
    IfxGtm_Atom_Ch_setCounterValue(s_timer.atom, s_timer.atomChannel, 0U);
    __dsync();
    IfxDma_enableChannelTransaction(&MODULE_DMA, AUDIO_DMA_CHANNEL);
    IfxSrc_enable(src);
    s_running = 1U;
    IfxGtm_Atom_Pwm_start(&s_timer, TRUE);
    return 1U;
}

uint32 guimai_adc_audio_read(int16 *dst, uint32 capacity)
{
    uint32 now_ms;
    uint32 write_pos;
    uint32 added;
    uint32 count;
    uint32 i;
    if (!s_ready || s_error || (dst == NULL_PTR) || (capacity == 0U))
    {
        return 0U;
    }
    now_ms = audio_now_ms();
    if (s_running && ((now_ms - s_poll_ms) >= AUDIO_POLL_MAX_MS))
    {
        s_error = 5U;
    }
    if (IfxDma_getChannelTransactionRequestLost(&MODULE_DMA, AUDIO_DMA_CHANNEL))
    {
        s_error = 6U;
    }
    s_poll_ms = now_ms;
    write_pos = ((IfxDma_getChannelDestinationAddress(&MODULE_DMA, AUDIO_DMA_CHANNEL)
                  - s_ring_address) / sizeof(uint32)) & AUDIO_RING_MASK;
    __dsync();
    added = (write_pos - s_write_pos) & AUDIO_RING_MASK;
    s_write_pos = write_pos;
    s_available += added;
    if (added != 0U)
    {
        s_last_sample_ms = now_ms;
    }
    if (s_available >= (AUDIO_RING_WORDS - GUIMAI_LINEIN_READ_BATCH))
    {
        s_error = 5U;
    }
    if (s_running && ((now_ms - s_last_sample_ms) >= 500U))
    {
        s_error = 7U;
    }
    if (s_error)
    {
        guimai_adc_audio_stop();
        s_available = 0U;
        return 0U;
    }
    count = (s_available < capacity) ? s_available : capacity;
    for (i = 0U; i < count; i++)
    {
        int32 raw = (int32)(s_ring_uncached[s_read_pos] & 0xFFFU);
        int32 raw_q16 = raw * 65536;
        int32 pcm;
        s_read_pos = (s_read_pos + 1U) & AUDIO_RING_MASK;
        if (!s_dc_ready)
        {
            s_dc_q16 = raw_q16;
            s_dc_ready = 1U;
        }
        s_dc_q16 += (raw_q16 - s_dc_q16) / GUIMAI_ADC_DC_FILTER_DIV;
        pcm = ((raw_q16 - s_dc_q16) / 65536) * GUIMAI_ADC_PCM_GAIN;
        if (pcm > 32767) { pcm = 32767; }
        if (pcm < -32768) { pcm = -32768; }
        dst[i] = (int16)pcm;
    }
    s_available -= count;
    return count;
}

uint8 guimai_adc_audio_error(void)
{
    return s_error;
}
#endif
