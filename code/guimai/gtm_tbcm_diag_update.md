# GTM TBCM Diagnostic Update

Date: 2026-05-08

This note supplements `gtm_i2s_status.md`.

## Code Change

Changed application code only:

`TC387_kd1  guimai/code/guimai/guimai_board.c`

No iLLD or library function was modified.

Current firmware tag:

`2026-05-08-gtm-tbcm-diag`

## Build Switches

The previous GPIO snapshot DMA experiment is disabled:

`GUIMAI_GTM_DMA_CAPTURE = 0`

The first-stage TIM Bit Compression Mode diagnostic is enabled:

`GUIMAI_GTM_TBCM_DIAG = 1`

## Diagnostic Path

Current diagnostic path:

`P11_6/BCK -> TIM2_CH3 edge event -> TIM2 TBCM samples TIM2 inputs -> local GPR1 polling`

Configured TIM input pins:

- `P11_2 / WS / TIM2_CH1`
- `P11_6 / BCK / TIM2_CH3`
- `P11_9 / SDOUT / TIM2_CH4`

Diagnostic TIM channel:

- `TIM2_CH4`
- `TIM_MODE = IfxGtm_Tim_Mode_bitCompression`
- `ARU_EN = 0`
- `CNTS = 1 << 3` for BCK rising-edge sampling

This build does not output valid PCM. Xfyun recognition is not meaningful in this build.

## Serial Log Fields

Watch the `[GUIMAI_TBCM]` init line first:

- `PISEL`: confirms TIM2 input selection after PinMap setup
- `CTRL`: confirms TBCM control register configuration
- `cnts`: confirms selected BCK sampling event

Watch the `[GUIMAI_CPU1]` line during capture:

- `tbcm_polls`: CPU polling count
- `tbcm_events`: observed `IRQ_NOTIFY.NEWVAL` count
- `new`: NEWVAL count
- `gprofl`: GPR overflow count
- `gpr1`: last sampled parallel input word
- `ecnt`: TIM edge counter field
- `ws1`: count where expected WS bit was 1
- `sd1`: count where expected SDOUT bit was 1
- `irq`: last `IRQ_NOTIFY` value
- `cnts`: last `CNTS` register value

## How To Interpret

If `tbcm_events` stays 0:

- TIM2 input selection or TBCM trigger configuration is still wrong.
- Check BCK routing to `TIM2_CH3`.
- Check whether another module overwrote `TIMINSEL`.

If `tbcm_events` increases but `ws1` and `sd1` never change:

- TBCM is triggering, but the assumed `GPR1` bit mapping may be wrong.
- Or WS/SDOUT are not on the expected TIM2 inputs.

If `tbcm_events`, `ws1`, and `sd1` all change:

- The first hardware sampling step is alive.
- Next step should be `TIM TBCM -> ARU -> PSM FIFO` diagnostic.
- Only after FIFO behavior is confirmed should DMA and PCM decode be added.

## 2026-05-08 ARU/FIFO Diagnostic

Current firmware tag:

`2026-05-08-gtm-aru-fifo-diag`

Application code only was changed. No iLLD/library function was modified.

New diagnostic path:

`TIM2_CH4 TBCM -> ARU -> PSM0 F2A stream0 -> FIFO0 channel0`

Current configuration:

- `GUIMAI_GTM_TBCM_DIAG = 1`
- `GUIMAI_GTM_ARU_FIFO_DIAG = 1`
- `TIM2_CH4 CTRL.ARU_EN = 1`
- `F2A = PSM0`
- `F2A stream = 0`
- `FIFO = PSM0 FIFO channel0`
- `ARU read source address = 0x015` for `TIM2_CH4`
- `F2A direction = aruToFifo`
- `F2A transfer mode = transferLowWord`
- `FIFO start = 0`
- `FIFO size = 128`

This build is still not a valid PCM capture build. It still skips Xfyun audio TX.

Watch these serial lines:

- `[GUIMAI_ARU_FIFO] init ...`
- `[GUIMAI_ARU_FIFO] start ...`
- `[GUIMAI_CPU1] ... fifo_fill=... fifo_wr=... f2a_state=...`
- `[GUIMAI_ARU_FIFO] stop ...`
- `[GUIMAI_STORE] fifo diag ...`

Interpretation:

- If `tbcm_events` increases but `fifo_fill`, `fifo_wr`, and `fifo_max` stay 0, TBCM local sampling still works but ARU/F2A/FIFO routing is not correct yet. First suspects are `ARU read source address`, F2A stream/channel mapping, or transfer mode.
- If `fifo_fill` or `fifo_wr` increases, `TIM -> ARU -> PSM FIFO` is alive. FIFO may quickly reach full because this diagnostic intentionally does not drain it with DMA.
- If `f2a_state` is not enabled after `[GUIMAI_ARU_FIFO] start`, the F2A enable sequence or PSM clock/state needs checking.
- If FIFO fills, the next step is not PCM decode yet; next step is `PSM FIFO/AFD -> DMA -> RAM`, then inspect raw FIFO words and only then reconstruct I2S/PCM.

## 2026-05-08 ARU/FIFO 3-Source Diagnostic

Current firmware tag:

`2026-05-08-gtm-aru-fifo-3src`

Application code only was changed. No iLLD/library function was modified.

Reason for this step:

- Local TBCM polling has already shown `tbcm_events`, `GPR1`, `ws1`, and `sd1` changing.
- The previous single-source ARU/FIFO diagnostic was not enough to distinguish wrong ARU source address from wrong PSM/F2A/FIFO routing.
- This build probes three likely TIM2 ARU sources in one run before adding DMA.

Configured streams:

- `f0`: `F2A stream0 -> FIFO channel0`, ARU source `0x015` (`TIM2_CH4`, SD/TBCM diagnostic channel)
- `f1`: `F2A stream1 -> FIFO channel1`, ARU source `0x014` (`TIM2_CH3`, BCK trigger channel)
- `f2`: `F2A stream2 -> FIFO channel2`, ARU source `0x012` (`TIM2_CH1`, WS input channel)

Each FIFO channel uses:

- PSM FIFO: `PSM0 FIFO`
- F2A: `PSM0 F2A`
- FIFO size: `64`
- Transfer direction: `aruToFifo`
- Transfer mode: `transferLowWord`

Important serial lines:

- `[GUIMAI_ARU_FIFO] init idx=...`
- `[GUIMAI_ARU_FIFO] init done ...`
- `[GUIMAI_ARU_FIFO] start ...`
- `[GUIMAI_CPU1] ... f0=... f1=... f2=...`
- `[GUIMAI_ARU_FIFO] stop idx=...`
- `[GUIMAI_STORE] fifo diag ...`

The compact `f0/f1/f2` format is:

`fill/max_fill/wr/rd/status/irq/state/str_cfg/aru_addr`

Interpretation:

- If `tbcm_events` grows and any of `f0/f1/f2` has non-zero `fill`, `max_fill`, or moving `wr`, then `TIM -> ARU -> F2A/FIFO` is at least partly alive for that source.
- If all three remain empty while `tbcm_events` grows, the likely issue is PSM/F2A enable/routing/configuration rather than the basic TIM TBCM sampling.
- If a FIFO fills, the next code step is `PSM FIFO/AFD -> DMA -> RAM`; only after raw FIFO words are captured should PCM reconstruction be added.

## 2026-05-08 AFD FIFO0 Dump Diagnostic

Observed result from `2026-05-08-gtm-aru-fifo-3src`:

- `f0` became full: `fill=64`, `max_fill=64`, `status=1`
- `f0` used ARU source `0x015`, which corresponds to `TIM2_CH4`
- `f1` and `f2` stayed empty

Conclusion:

- `TIM2_CH4 -> ARU source 0x015 -> PSM0 F2A stream0 -> FIFO0` is alive.
- The next risk is not the ARU source address anymore; it is the FIFO read path and raw word layout.

Current firmware tag:

`2026-05-08-gtm-afd-dump`

New diagnostic action:

- Keep the three-source diagnostic.
- On capture stop, read up to 16 words from `PSM0 AFD channel0 BUF_ACC`.
- Print FIFO0 `fill`, `rd`, and `wr` before and after the reads.
- Print raw words as `[GUIMAI_AFD] w0-7=...` and `[GUIMAI_AFD] w8-15=...`.

What to check next:

- If `fill` decreases and/or `rd` advances after the AFD reads, `BUF_ACC` is a valid FIFO read outlet.
- If all dumped words are constant or zero while TBCM `GPR1` changes, the F2A transfer mode or TIM output word selection still needs adjustment.
- If dumped words change with WS/SD/BCK activity, the next step is decoding the 29-bit TBCM word layout into bits, then moving the same read path to DMA.

Observed result:

- `BUF_ACC` read is effective: FIFO0 changed from `fill=64` to `fill=50`, and `rd=0` to `rd=16`.
- All 16 dumped words were `00000000`.
- At the same time, TBCM local `GPR1`, `ws1`, and `sd1` were still changing.

Conclusion:

- The FIFO/AFD read outlet is valid.
- `transferLowWord` is probably reading ARU bits `23:0`, which do not carry the useful TIM TBCM value in this configuration.
- Next build switches F2A transfer mode to `transferHighWord`.

## 2026-05-08 AFD FIFO0 High-Word Dump Diagnostic

Current firmware tag:

`2026-05-08-gtm-afd-high`

Only application code changed. No iLLD/library function was modified.

Change from the previous AFD dump:

- F2A transfer mode changed from `transferLowWord` to `transferHighWord`.
- Other wiring stays the same: `TIM2_CH4 -> ARU source 0x015 -> PSM0 F2A stream0 -> FIFO0 -> AFD BUF_ACC`.

Expected serial lines:

- `[GUIMAI_ARU_FIFO] init ... str=...`
- `[GUIMAI_AFD] fifo0 dump ...`
- `[GUIMAI_AFD] w0-7=...`
- `[GUIMAI_AFD] w8-15=...`

Interpretation:

- If the dump now contains changing non-zero words, the next step is to decode which bit positions map to WS and SD.
- If the dump is still all zero but FIFO fills, try `transferBothWords` next to confirm word ordering.
- If FIFO no longer fills, `transferHighWord` is not compatible with this stream and should be reverted or replaced by `transferBothWords`.

Observed result:

- FIFO0 still fills, so `transferHighWord` is accepted by PSM/F2A.
- Stop-time AFD dump reads valid FIFO entries, but all printed words were `0000000A`.
- The stream configuration changed to `str=0x00010000`, confirming high-word mode.

Important interpretation:

- This does not yet prove high-word mode is useless.
- FIFO0 fills immediately at capture start; a stop-time dump reads the oldest FIFO entries.
- Those first entries can be constant because capture started before stable I2S activity or before the changing TBCM pattern reached FIFO.
- The next diagnostic must drain FIFO while recording is running and print the last words observed, not only the first stale FIFO words at stop.

## 2026-05-08 AFD FIFO0 High-Word Live Drain Diagnostic

Current firmware tag:

`2026-05-08-gtm-afd-high-live`

Only application code changed. No iLLD/library function was modified.

Change:

- Keep `transferHighWord`.
- During each TBCM diagnostic poll, drain up to 64 words from FIFO0 through `AFD BUF_ACC`.
- Keep a 16-word ring of the last drained words.
- On capture stop, print:
  - `[GUIMAI_AFD_LIVE] drained=... nonzero=... changed=... last=...`
  - `[GUIMAI_AFD_LIVE] last0-7=...`
  - `[GUIMAI_AFD_LIVE] last8-15=...`

Interpretation:

- If `drained` grows and `changed` grows, AFD high-word data is changing and can be decoded next.
- If `drained` grows but `changed=0` and all last words are `0000000A`, the ARU/F2A stream is not delivering the useful TBCM data field.
- If `drained` stays 0 while FIFO0 fills, the live drain path is not being called or AFD read timing needs adjustment.

Observed result:

- Live drain worked and drained many words from FIFO0 through `AFD BUF_ACC`.
- `drained=133568`, `nonzero=133568`, `changed=40760`.
- Last observed examples included `0000000A`, `0100001A`, `00000008`, and `02000008`.

Conclusion:

- `transferHighWord` is carrying changing TBCM-like data.
- `AFD BUF_ACC` is confirmed as the readable FIFO outlet.
- CPU polling/live drain is only a diagnostic. It is too slow and intrusive for final capture.
- The next required step is `PSM FIFO upper watermark interrupt -> DMA reads AFD BUF_ACC -> RAM`.

## User Manual Findings Added 2026-05-08

AFD/FIFO access:

- `AFD[i]_CH[x]_BUF_ACC` is the access register for the matching FIFO channel.
- `BUF_ACC.DATA[28:0]` contains the FIFO data field.
- Reading `BUF_ACC` accesses and advances the corresponding FIFO channel.
- Reading an empty FIFO returns `0`.

FIFO interrupt bits:

- `FIFO[i]_CH[z]_IRQ_NOTIFY` bit0: empty
- bit1: full
- bit2: lower watermark
- bit3: upper watermark
- Write `1` clears the corresponding notify bit.
- `FIFO[i]_CH[z]_IRQ_EN` enables the same events to be visible outside GTM.

FIFO IRQ mode / DMA:

- `IRQ_MODE = 0`: level
- `IRQ_MODE = 1`: pulse
- `IRQ_MODE = 2`: pulse-notify
- `IRQ_MODE = 3`: single-pulse
- `DMA_HYSTERESIS` can suppress repeated DMA requests until the FIFO crosses the opposite watermark.
- `DMA_HYST_DIR = 0` means read direction.
- For DMA reading FIFO, the intended trigger is usually upper watermark; with hysteresis enabled, the next request is delayed until FIFO drops to lower watermark.

Useful iLLD APIs confirmed in `IfxGtm_Psm.h`:

- `IfxGtm_Psm_Fifo_getChannelSrcPointer`
- `IfxGtm_Psm_Afd_getChannelPointer`
- `IfxGtm_Psm_Fifo_setChannelInterruptMode`
- `IfxGtm_Psm_Fifo_setChannelDmaHystMode`
- `IfxGtm_Psm_Fifo_enableChannelInterrupt`
- `IfxGtm_Psm_Fifo_disableChannelInterrupt`
- `IfxGtm_Psm_Fifo_clearAllChannelInterrupts`
- `IfxGtm_Psm_F2a_setTransferMode`
- `IfxGtm_Psm_F2a_setTransferDirection`
- `IfxGtm_Psm_F2a_setAruReadAddress`

## 2026-05-08 AFD BUF_ACC DMA Diagnostic

Current firmware tag:

`2026-05-08-gtm-afd-dma-diag`

Only application code changed. No iLLD/library function was modified.

Purpose:

- Verify that FIFO0 upper-watermark interrupt can trigger DMA.
- Verify that DMA can read `PSM0 AFD CH0 BUF_ACC` into RAM.
- Do not decode PCM yet. This is still a transport diagnostic.

Current path:

`TIM2_CH4 TBCM -> ARU source 0x015 -> PSM0 F2A stream0 -> FIFO0 -> AFD CH0 BUF_ACC -> DMA CH20 -> RAM`

Current DMA diagnostic setup:

- DMA channel: `20`
- DMA words: `1024`
- DMA source: `AFD0_CH0_BUF_ACC`
- DMA destination: `s_gtm_afd_dma_words`
- DMA request source: `PSM0 FIFO channel0 SRC`
- FIFO0 interrupt: upper watermark
- FIFO0 IRQ mode: pulse
- DMA hysteresis: disabled for the first trigger-path test
- CPU live drain and stop-time AFD dump are disabled while this DMA diagnostic is enabled, so DMA is the only reader of FIFO0.

Expected serial lines:

- `[GUIMAI_AFD_DMA] init ...`
- `[GUIMAI_AFD_DMA] start ...`
- `[GUIMAI_AFD_DMA] done=... lost=... nonzero=... changed=...`
- `[GUIMAI_AFD_DMA] w0-7=...`
- `[GUIMAI_AFD_DMA] w1016-1023=...`

How to interpret the next test:

- Good sign: `done=1`, `nonzero>0`, `changed>0`, and words resemble the live-drain values such as `0000000A`, `0100001A`, `00000008`, `02000008`.
- Trigger failed: `done=0`, DMA `tcnt` stays high, buffer is all zero, and FIFO0 remains full. Check FIFO IRQ_EN/IRQ_MODE/SRC/TOS DMA setup.
- DMA read wrong/empty path: `done=1` but `nonzero=0`. Check whether DMA source should read `BUF_ACC.U` or `BUF_ACC.B.DATA`, and check FIFO start order.
- DMA cannot keep up: `lost>0` or FIFO stays full. Next adjustment is request mode, block move count, or DMA hysteresis with upper/lower watermark.
- If RAM overflows during build, reduce `GUIMAI_GTM_AFD_DMA_WORDS` from `1024` to `512`.

Observed result:

- DMA init and start both succeeded.
- `done=1`, `nonzero=1024`, `changed=63`.
- Example words:
  - start: `00000008`
  - end: `0000000A`
- FIFO0 was still full at stop and `lost=4622`.

Conclusion:

- The hardware request chain is valid:
  `FIFO0 upper watermark -> FIFO SRC -> DMA CH20 -> AFD BUF_ACC -> RAM`.
- `lost` is expected in this one-shot test because DMA stopped after the first 1024-word transaction while F2A continued producing FIFO data.
- The next build must re-arm DMA after every completed transaction before attempting PCM reconstruction.

## 2026-05-08 AFD BUF_ACC DMA Loop Diagnostic

Current firmware tag:

`2026-05-08-gtm-afd-dma-loop`

Change:

- Keep the same `AFD BUF_ACC -> DMA` path.
- When DMA completes one 1024-word transaction, count non-zero/changing words, clear the buffer, and immediately re-arm DMA.
- FIFO0 IRQ mode is changed from pulse to level so a new DMA transaction can be requested while FIFO is already above upper watermark.
- Periodic CPU1 log now prints:
  - `afd_dma=chunks/words/lost/tcnt/last/chcsr`

Expected result:

- `chunks` and `words` should grow during capture.
- `last` should change between values such as `0x00000008`, `0x0000000A`, `0x0100001A`, or similar.
- `lost` should be much lower than the one-shot build. If it still grows quickly, DMA request mode, block size, or FIFO hysteresis must be adjusted.

Observed result:

- `level` mode only moved one word: `tcnt=1023`, FIFO `rd=1`, and DMA buffer had only `w0=00000008`.
- `chunks` stayed `0`, so no complete 1024-word DMA transaction occurred.

Conclusion:

- `level + oneTransferPerRequest` is not useful for this FIFO SRC path.
- The previous `pulse` mode proved that repeated FIFO upper-watermark events can drive DMA to complete a transaction.
- Next build returns to `pulse` and reduces the DMA transaction size to one FIFO-sized chunk.

## 2026-05-08 AFD BUF_ACC DMA Pulse64 Diagnostic

Current firmware tag:

`2026-05-08-gtm-afd-dma-pulse64`

Change:

- `GUIMAI_GTM_AFD_DMA_WORDS = 64`.
- FIFO IRQ mode returns to `pulse`.
- DMA still re-arms after each completed transaction.

Expected result:

- Each FIFO-full burst should be able to complete one 64-word DMA transaction quickly.
- `chunks` should grow during capture.
- If `chunks` grows but `lost` is still high, the next step is double buffering or DMA linked-list style service.
- If `chunks` stays 0 again, the next test should use `completeTransactionPerRequest` to see whether one FIFO request can drain an entire 64-word DMA transaction.

Observed result:

- Continuous DMA works.
- Example stop line: `chunks=5169`, `words=330816`, `nonzero=330816`, `changed=86176`.
- FIFO remains full and `lost` grows because this is still a CPU re-arm diagnostic, not a final lossless DMA design.

Conclusion:

- DMA transport is good enough to move to TBCM word decoding.
- Do not send audio to Xfyun yet.

## 2026-05-08 TBCM Decode Diagnostic

Current firmware tag:

`2026-05-08-gtm-decode-diag`

Purpose:

- Determine which bits in the DMA word correspond to WS and SD.
- Determine whether the word stream can be decoded into plausible 16-bit PCM.

Added diagnostics:

- `[GUIMAI_DEC_BITS]`: counts `ones` and `toggles` for bits `0..15`.
- `[GUIMAI_DEC]`: tries several candidate mappings and prints:
  - `words`
  - `ws_edges`
  - decoded `samples`
  - `L/R`
  - `bad_slots`
  - `max_slot`
  - `sd_ones`
  - `peak`
  - `avg_abs`
  - last decoded sample

Current candidates:

- `ws1_sd4_l0`
- `ws1_sd4_l1`
- `ws1_sd3_l0`
- `ws1i_sd4_l0`

How to interpret:

- A good candidate should have many samples, balanced L/R counts, low `bad_slots`, and plausible `peak/avg_abs`.
- If all candidates have huge `bad_slots`, the TBCM word is not one BCK sample per DMA word; we must decode the compressed TBCM layout differently.
- If bit statistics show another bit has stronger SD-like toggling, add it as a candidate.

Observed no-event run:

- `tbcm_events=0`, `new=0`, `gprofl=0`, `gpr1=0x000000`, `irq=0x00`.
- FIFO0 stayed empty: `f0=0/0/0/0/0/...`.
- AFD DMA did not start: `chunks=0`, `words=0`, `chcsr=0x00000000`.
- Decode had no input: all `[GUIMAI_DEC]` candidates reported `words=0`, `samples=0`.

Conclusion:

- This run did not reach the decode stage.
- The immediate fault is upstream of ARU/FIFO/DMA: `TIM2_CH4 TBCM` did not receive or did not report BCK-triggered events.
- Earlier `pulse64` logs proved that TBCM events, FIFO fill, and DMA transfer can work, so this looks like a clock/start/routing timing issue rather than a final decode issue.

## 2026-05-08 TBCM No-Event Clock Diagnostic

Current firmware tag:

`2026-05-08-gtm-decode-clockdiag`

Purpose:

- When capture has been polling for a while but `tbcm_events` is still 0, print one extra diagnostic line:
  - `[GUIMAI_TBCM_NOEVT] raw bck=... ws=... sd=... mclk=...`
  - GPIO levels for BCK/WS/SD/MCLK
  - TIM2 input select and TIM2_CH4 registers: `TIMINSEL`, `CTRL`, `ECTRL`, `CNTS`, `IRQ`, `GPR0`, `GPR1`

How to interpret the next log:

- If raw `mclk`, `bck`, `ws`, and `sd` toggles are all 0, focus on codec clock output/startup. ES8388 may not be producing I2S clocks in that run.
- If raw `bck` toggles but `tbcm_events` remains 0, focus on TIM input selection, TIM start order, and TBCM trigger configuration.
- If `tbcm_events` returns and DMA chunks grow again, continue with `[GUIMAI_DEC_BITS]` and `[GUIMAI_DEC]` candidate selection.

Observed working decode-diagnostic run:

- `tbcm_events` increased normally: example `tbcm_events=3203`.
- DMA worked again: example `chunks=3204`, `words=205056`, `nonzero=205056`, `changed=46758`.
- Bit statistics indicate:
  - bit1 behaves like WS: `tog=15554`.
  - bit4 behaves like SD: many ones and toggles.
  - bit3 is always 1 in this run, not SD.
- The raw one-word-per-bit decoder produced plausible activity but too many bad slots:
  - `ws1_sd4_l0 samples=9742`, `L/R=4868/4874`, `bad_slots=5817`, `max_slot=32`.

Conclusion:

- The hardware path is alive: `TIM2_CH4 -> ARU -> FIFO0 -> AFD BUF_ACC -> DMA` is proven again.
- The remaining issue is decode format, not transport.
- The current word stream is likely TBCM-compressed. Treating each DMA word as one BCK bit causes bad slot boundaries.

## 2026-05-08 TBCM RLE Decode Diagnostic

Current firmware tag:

`2026-05-08-gtm-decode-rle-diag`

Change:

- Keep existing raw `[GUIMAI_DEC]` output.
- Add `[GUIMAI_DEC_RLE]` output for two possible compressed-count interpretations:
  - `mode=ecnt`: high byte `ECNT` is the run length.
  - `mode=ecnt1`: high byte `ECNT + 1` is the run length.
- Each mode reuses the same WS/SD candidates and prints:
  - expanded total length
  - zero-count words
  - maximum run length
  - decoded samples
  - L/R balance
  - bad slot count
  - peak and average absolute value

How to interpret next log:

- Pick the RLE mode/candidate with low `bad_slots`, balanced `L/R`, and plausible sample count.
- If neither `ecnt` nor `ecnt1` reduces `bad_slots`, the high byte is not a simple run length and we need the exact TBCM word layout from the manual.
- If one mode works, next code step is to convert that decoder from diagnostic-only into `guimai_gtm_tbcm_read_samples()` output and re-enable network send.

Observed RLE result:

- `mode=ecnt` produced `total=98786`, `zero=85265`, `max=3`.
- `mode=ecnt1` produced `total=278562`, `max=4`.
- Both RLE modes had higher `bad_slots` than the raw decoder.
- CPU-side direct TBCM polling still showed large high-byte `ECNT` values in `GPR1`, for example `ecnt=185`, `ecnt=187`, `ecnt=196`.
- AFD/DMA words only showed small high-byte values, for example `01000018`, `0200000A`, `0100001A`.

Conclusion:

- The high byte in the AFD/DMA word is not the same high byte seen in TIM `GPR1`.
- The current F2A transfer selection is not exposing the complete TIM TBCM word needed for decoding.

## 2026-05-08 F2A Low-Word Diagnostic

Current firmware tag:

`2026-05-08-gtm-f2a-low-diag`

Change:

- Switch F2A stream transfer mode from `transferHighWord` to `transferLowWord`.
- Init log now prints `tmode`.
- Bit statistics now include bits `16..31`, not only `0..15`.

Purpose:

- Check whether the complete useful TIM word is in ARU bits `23:0` instead of bits `47:24`.
- A good sign would be DMA words resembling CPU-polled `GPR1` values such as `BA000018`, or bit statistics showing WS/SD/ECNT in a more consistent location.

How to interpret next log:

- If `tmode=0` and `w0-7` now contain high-byte values like `xx000018` with `xx` much larger than 3, low word is the right half.
- If low word becomes all zero or worse, return to high word and the next test should use `transferBothWords`.
- If both halves are partial, we need to pair the two F2A words from `transferBothWords` or consult the exact ARU word layout.

Observed low-word result:

- `tmode=0` still allowed FIFO/DMA movement, but it did not expose the useful WS/SD fields.
- Example DMA dump:
  - `w0-7=01000000 03000000 00000000 ...`
  - bits `0..23` were all zero.
  - only bits `24/25` changed.
- Decode produced only one zero sample:
  - `ws_edges=0`
  - `samples=1`
  - `peak=0`

Conclusion:

- `transferLowWord` is worse than the previous high-word path.
- The next useful test is not low-word; it is to keep high-word and change what TIM sends into ARU.

## 2026-05-08 GPR CNTS High-Word Diagnostic

Current firmware tag:

`2026-05-08-gtm-gpr-cnts-high-diag`

Change:

- Switch F2A stream transfer mode back to `transferHighWord`.
- Set TIM2_CH4 TBCM:
  - `CNTS_SEL = IfxGtm_Tim_CntsSel_cntReg`
  - `GPR0_SEL = IfxGtm_Tim_GprSel_cnts`
  - `GPR1_SEL = IfxGtm_Tim_GprSel_cnts`
- Keep `ARU_EN=1`, FIFO0, and AFD DMA CH20.
- Init log now prints `gpr=.../...` and `cnts_sel=...` so the effective GPR selection is visible in UART logs.

Reason:

- ARU routes TIM `GPR0/GPR1`.
- The earlier useful high-word data was collected while `GPR0/GPR1` selected `tbuTs0`, so the high byte was timestamp-related, not reliable `CNTS/ECNT`.
- This test verifies whether sending `CNTS` through ARU produces a decodable TBCM stream.

Expected next log:

- Keep these lines:
  - `[GUIMAI_TBCM] init ... gpr=.../... cnts_sel=...`
  - `[GUIMAI_ARU_FIFO] init ... tmode=...`
  - `[GUIMAI_CPU1] ... gpr1=... cnts=... afd_dma=...`
  - `[GUIMAI_AFD_DMA] w0-7=...`
  - all `[GUIMAI_DEC_BITS]`
  - all `[GUIMAI_DEC]` and `[GUIMAI_DEC_RLE]`

How to judge:

- If `CNTS` high-word exposes stable WS/SD and reduces `bad_slots`, convert the decoder into real PCM output.
- If it still only exposes partial fields, the next controlled test is `transferBothWords` and pair the two F2A words before decoding.

Observed result:

- `GPR0_SEL/GPR1_SEL` change is effective:
  - log showed `gpr=3/3 cnts_sel=0`
  - `CTRL=0x00000F28`
- F2A was high-word:
  - `tmode=1`
  - `str=0x00010000`
- Transport stayed alive:
  - `tbcm_events=8209`
  - `chunks=8210`
  - `words=525440`
- Bit layout remained like the previous high-word test:
  - bit1 behaves like WS: `tog=36837`
  - bit4 behaves like SD: `tog=76590`
  - bits 24/25 also changed, but did not solve slot reconstruction.
- Decode did not improve:
  - raw candidate `ws1_sd4_l0`: `samples=27307`, `bad_slots=12407`
  - RLE modes were worse.

Conclusion:

- The earlier high byte was not fixed by switching `GPR_SEL` to `CNTS`.
- A single high-word FIFO stream still does not provide enough context to reconstruct clean 16-bit I2S slots.
- Next controlled test is `transferBothWords` and split FIFO words by even/odd position.

## 2026-05-08 Both-Words Pair Diagnostic

Current firmware tag:

`2026-05-08-gtm-bothwords-pair-diag`

Change:

- F2A transfer mode changed to `IfxGtm_Psm_F2aTransferMode_transferBothWords`.
- Keep `GPR0/GPR1 = cnts`.
- Keep DMA CH20 reading FIFO0 AFD `BUF_ACC`.
- Add paired diagnostics:
  - `[GUIMAI_DEC_PAIR_BITS] h0 ...`
  - `[GUIMAI_DEC_PAIR_BITS] h1 ...`
  - `[GUIMAI_DEC_PAIR] h0 ...`
  - `[GUIMAI_DEC_PAIR] h1 ...`

Purpose:

- Determine whether F2A emits low/high ARU halves as alternating FIFO words.
- Identify which half contains WS/SD and which half contains count/context.
- Avoid judging `transferBothWords` by the mixed full stream only, because low/high words interleaved together can make slot decoding look worse.

Expected next log:

- Confirm:
  - `fw=2026-05-08-gtm-bothwords-pair-diag`
  - `[GUIMAI_ARU_FIFO] init ... tmode=2`
- Paste:
  - `[GUIMAI_AFD_DMA] w0-7=...`
  - all `[GUIMAI_DEC_BITS]`
  - all `[GUIMAI_DEC_PAIR_BITS]`
  - all `[GUIMAI_DEC_PAIR]`
  - all `[GUIMAI_DEC]` and `[GUIMAI_DEC_RLE]`

How to judge:

- If one half has bit1/bit4 activity and the other half has count-like high bits, pair those halves for a real TBCM decoder.
- If one half alone has much lower `bad_slots`, decode only that half.
- If both halves are still partial and no pair relation is obvious, the next needed input is the GTM TIM TBCM ARU word layout from the user manual.

Observed result:

- `transferBothWords` is active:
  - `tmode=2`
  - `str=0x00020000`
- FIFO/DMA transport still works:
  - `chunks=1945`
  - `words=124480`
  - `lost=1945`
- The paired diagnostics show the F2A stream is alternating two halves:
  - `h0` has no usable WS/SD: bit3 is fixed high, bit4 is zero, `ws_edges=0`, `samples=1`.
  - `h1` has the same useful WS/SD as the previous high-word stream:
    - bit1 behaves like WS.
    - bit4 behaves like SD.
    - `ws1_sd4_l0 samples=2399`, `bad_slots=2621`, `max_slot=31`.

Conclusion:

- `h1` is the useful half; `h0` is not directly decodable as I2S data.
- `bad_slots` is still high because the current DMA diagnostic loses requests between CPU re-arms:
  - `lost` is approximately equal to `chunks`.
- Next change should reduce DMA request loss before spending more effort on PCM reconstruction.

## 2026-05-08 Both-Words DMA32 Complete-Transaction Diagnostic

Current firmware tag:

`2026-05-08-gtm-bothwords-dma32-complete`

Change:

- Keep `transferBothWords`.
- Reduce DMA transaction size from 64 words to 32 words.
- Change AFD DMA request mode to `IfxDma_ChannelRequestMode_completeTransactionPerRequest`.

Reason:

- In the previous diagnostic, `oneTransferPerRequest` needed many FIFO service requests to finish one transaction, and lost requests matched chunk count.
- `completeTransactionPerRequest` tests whether a single upper-watermark request can drain a full diagnostic block from `AFD BUF_ACC`.
- 32 words is still even, so h0/h1 pairing remains aligned, and it avoids deliberately reading more words than are guaranteed available at the 62-word upper watermark.

Expected next log:

- Confirm:
  - `fw=2026-05-08-gtm-bothwords-dma32-complete`
  - `[GUIMAI_AFD_DMA] init ... words=32 ...`
  - `CHCFGR` should differ from the previous one-transfer build.
- Paste:
  - `[GUIMAI_AFD_DMA] done=... chunks=... words=... lost=...`
  - `[GUIMAI_DEC_PAIR] h1 ...`
  - `[GUIMAI_DEC_PAIR_BITS] h1 ...`

How to judge:

- Good sign: `chunks` grows and `lost` is much smaller than `chunks`.
- Bad sign: `chunks=0` or only one partial transaction; then this FIFO SRC likely cannot drive complete transactions, and we should revert to one-transfer mode and move toward DMA buffering/linking or lower-overhead CPU service.

Observed result:

- The intended config was active:
  - `words=32`
  - `CHCFGR=0x00480020`
  - `tmode=2`
  - `str=0x00020000`
- It did not reduce request loss:
  - `chunks=3849`
  - `words=123168`
  - `lost=3849`
- The useful half was still `h1`, but decode got worse:
  - `h1` bit1 still behaves like WS.
  - `h1` bit4 still behaves like SD.
  - `h1 ws1_sd4_l0 samples=1149`
  - `h1 bad_slots=5291`
  - `h1 max_slot=70`

Conclusion:

- `completeTransactionPerRequest` is not useful for this AFD FIFO diagnostic.
- Do not keep tuning this direction for PCM reconstruction.
- Revert DMA request mode to `oneTransferPerRequest` for later diagnostics.
- The bigger issue is still the TIM output format: TBCM produces a compressed/partial stream, not a clean serial-bit window.

## 2026-05-08 TSSM Shift Diagnostic

Current firmware tag:

`2026-05-08-gtm-tssm-shift-diag`

Change:

- Keep the verified transport path:
  - `TIM2_CH4 -> ARU source 0x015 -> PSM0 F2A stream0 -> FIFO0 -> AFD BUF_ACC -> DMA CH20 -> RAM`
- Keep `transferBothWords`.
- Keep `GUIMAI_GTM_AFD_DMA_WORDS = 32`.
- Revert DMA request mode to `IfxDma_ChannelRequestMode_oneTransferPerRequest`.
- Change the TIM channel mode directly through the register:
  - `TIM_MODE = 6U`, which is `0b110 / TSSM`.
- Do not modify iLLD/library code. The local iLLD enum lacks TSSM, but its comments document:
  - `TIM_MODE=0b110 (TSSM)`
  - `DSL` defines shift direction.
  - `ECNT_RESET` defines shift-register initial polarity.

Current TSSM probe settings:

- `DSL = 0`, shift left.
- `ECNT_RESET = 0`, initial polarity 0.
- `TDUV = 0`
- `TDUC = 0`
- `ECTRL = 0`
- `CNTS` still selects BCK rising edge as the sampling event.

Expected next log:

- Confirm:
  - `fw=2026-05-08-gtm-tssm-shift-diag`
  - `[GUIMAI_TBCM] init ... mode=6 tssm=1 shift=left init_pol=0 ...`
  - `CTRL`, `ECTRL`, `TDUV`, `TDUC`
- Paste:
  - first two `[GUIMAI_CPU1] ...` lines
  - `[GUIMAI_AFD_DMA] done=...`
  - `[GUIMAI_AFD_DMA] w0-7=...`
  - all `[GUIMAI_DEC_BITS]`
  - all `[GUIMAI_DEC_PAIR_BITS]`
  - all `[GUIMAI_DEC_PAIR]`

How to judge:

- If `tbcm_events` and FIFO/DMA still grow, TSSM is producing ARU data and we compare bit layout against the TBCM logs.
- If `tbcm_events=0` and FIFO stays empty, plain TSSM needs TDU/ECTRL setup before it can emit windows.
- If words become richer than the old `0x01000008/0x0100001A` pattern and one decode candidate has low `bad_slots`, the next step is converting that candidate into real PCM.

Observed result:

- `mode=6` was active:
  - `CTRL=0x00000F2C`
  - `ECTRL=0`
  - `TDUV=0`
  - `TDUC=0`
- TSSM still emits data through the verified path:
  - `tbcm_events=2060`
  - FIFO0 stays active and full.
  - `chunks=2061`
  - `words=65952`
- The output is clearly different from TBCM:
  - `GPR1` now has full-width shift-like patterns such as `0xB8000000`, `0x09FFFFFF`, `0x5DFFFFFF`.
  - Full-stream bits `b0-23` all toggle heavily.
  - In `transferBothWords`, `h0` is still mostly metadata/fixed pattern, while `h1` carries the changing low/mid bits.
- The old TBCM bit decoder is no longer a valid model:
  - `h1 ws1_sd4_l0 samples=653`
  - `h1 bad_slots=3973`
  - `h1 max_slot=272`

Conclusion:

- TSSM is alive and reaches ARU/FIFO/DMA.
- Do not revert to TBCM yet.
- The next diagnostic should inspect TSSM words as shift-register windows, not as one WS bit plus one SD bit.

## 2026-05-08 TSSM History Diagnostic

Current firmware tag:

`2026-05-08-gtm-tssm-history-diag`

Change:

- Keep the same TSSM configuration:
  - `TIM_MODE=6`
  - `DSL=0`
  - `ECNT_RESET=0`
  - `TDUV/TDUC/ECTRL=0`
- Add a 32-word DMA history ring.
- Print `[GUIMAI_AFD_DMA_HIST]` with the last continuous 32 DMA words.
- Expand pair bit statistics to include:
  - `h0/h1 b8-15`
  - `h0/h1 b16-23`

Expected next log:

- Confirm:
  - `fw=2026-05-08-gtm-tssm-history-diag`
- Paste:
  - `[GUIMAI_AFD_DMA] done=...`
  - all `[GUIMAI_AFD_DMA_HIST] ...`
  - all `[GUIMAI_DEC_PAIR_BITS] ...`
  - all `[GUIMAI_DEC_BITS] ...`

How to judge:

- Use `[GUIMAI_AFD_DMA_HIST]` to see whether h1 is a sliding shift-register window.
- Use the expanded `h1 b8-23` statistics to decide which 16-bit slice should be tried as PCM first.
- If the history shows a stable window but bit order is reversed, the next build should flip `GUIMAI_GTM_TIM_TSSM_SHIFT_RIGHT`.

Observed result:

- `h0` is not PCM data:
  - Typical history words are `03000808`, `02000808`.
  - `h0 b0-23` are mostly fixed.
  - `h0 bit3` and `h0 bit11` are fixed high.
  - `h0 bit24/25` carry count/state-like information.
- `h1` is the useful TSSM shift window:
  - Typical history words are `03FFFFFF`, `02FFFFFE`, `02000000`, `03000003`, `030FFFFF`.
  - `h1 b0-23` are active and each bit toggles at a similar rate.
  - `h1 bit25` is fixed high, and `h1 bit24` is state/count-like.
- The old decoder is still the wrong model:
  - It treats bits as independent WS/SD signals.
  - TSSM output is a multi-bit shift window, so PCM candidates must be evaluated as 16-bit slices of h1.

Conclusion:

- Continue with TSSM.
- Stop using `ws1_sd4` as the primary success/failure metric.
- Add direct h1 16-bit slice statistics before trying to store/send PCM.

## 2026-05-08 TSSM Slice Diagnostic

Current firmware tag:

`2026-05-08-gtm-tssm-slice-diag`

Change:

- Keep TSSM and DMA history diagnostics.
- Add `[GUIMAI_TSSM_SLICE]` statistics for h1 candidate 16-bit windows:
  - `h1_b0_15`
  - `h1_b4_19`
  - `h1_b8_23`
  - bit-reversed versions of those three.

Expected next log:

- Confirm:
  - `fw=2026-05-08-gtm-tssm-slice-diag`
- Paste:
  - `[GUIMAI_AFD_DMA] done=...`
  - all `[GUIMAI_AFD_DMA_HIST]`
  - all `[GUIMAI_TSSM_SLICE]`
  - all `[GUIMAI_DEC_PAIR_BITS] h1 ...`

How to judge:

- A usable PCM slice should have many changed samples, limited zero/fullscale counts, and a plausible `avg_abs`.
- If normal slices look saturated but reversed slices look reasonable, flip the TSSM shift direction or use bit-reverse during PCM extraction.
- If all slices look like long runs of all-zero/all-one, TSSM needs different `DSL`, `ECNT_RESET`, or TDU/ECTRL setup.

Observed result:

- Left-shift TSSM is not a usable PCM window yet.
- `h1` still carries the active TSSM shift data, but all candidate slices are dominated by `0x0000` and `0xFFFF`.
- Example from the left-shift slice log:
  - `h1_b0_15 words=42080 changed=8950 zero=13134 full=25580`
  - `h1_b4_19 words=42080 changed=8927 zero=13160 full=25568`
  - `h1_b8_23 words=42080 changed=8900 zero=13165 full=25579`
- Reversed slices are similar, so bit reversal alone is not the fix.

Conclusion:

- Flip TSSM shift direction first, keeping the rest of the transport unchanged.
- Also split the old `full` statistic into:
  - `neg1`: raw `0xFFFF`
  - `sat`: raw `0x7FFF` or `0x8000`

## 2026-05-08 TSSM Right-Shift Slice Diagnostic

Current firmware tag:

`2026-05-08-gtm-tssm-right-slice`

Change:

- Set `GUIMAI_GTM_TIM_TSSM_SHIFT_RIGHT=1`.
- Keep:
  - `TIM_MODE=6`
  - `ECNT_RESET=0`
  - `transferBothWords`
  - `DMA words=32`
- Print `[GUIMAI_TSSM_SLICE]` with `zero/neg1/sat`.

Expected next log:

- Confirm:
  - `fw=2026-05-08-gtm-tssm-right-slice`
  - `[GUIMAI_TBCM] init ... shift=right ...`
- Paste:
  - `[GUIMAI_AFD_DMA] done=...`
  - all `[GUIMAI_AFD_DMA_HIST]`
  - all `[GUIMAI_TSSM_SLICE]`
  - all `[GUIMAI_DEC_PAIR_BITS] h1 ...`

How to judge:

- Good sign: one slice has high `changed`, much lower `zero + neg1`, low `sat`, and plausible `avg_abs`.
- If right shift is still mostly `zero/neg1`, try `GUIMAI_GTM_TIM_TSSM_INIT_POL=1` next.
- If both shift directions and both initial polarities fail, TSSM likely needs TDU/ECTRL configuration rather than only `TIM_MODE/DSL`.

Observed result:

- Right-shift with `ECNT_RESET=0` is still not usable.
- The DMA history is still dominated by words like `02000000`, `03FFFFFF`, and short partial windows such as `03FFFC00`.
- All h1 candidate slices are dominated by zero and `0xFFFF`.
- Example:
  - `h1_b0_15 words=60048 changed=7938 zero=19081 neg1=38081 sat=187`
  - `h1_b4_19 words=60048 changed=7929 zero=19069 neg1=38088 sat=216`
  - `h1_b8_23 words=60048 changed=8006 zero=19031 neg1=38082 sat=208`
- `avg_abs` is only about 200, so this is mostly fill state, not audio PCM.

Conclusion:

- Test the remaining simple TSSM polarity combination before moving to TDU/ECTRL:
  - `GUIMAI_GTM_TIM_TSSM_SHIFT_RIGHT=1`
  - `GUIMAI_GTM_TIM_TSSM_INIT_POL=1`

## 2026-05-08 TSSM Right-Shift Polarity-1 Diagnostic

Current firmware tag:

`2026-05-08-gtm-tssm-right-pol1`

Change:

- Set `GUIMAI_GTM_TIM_TSSM_INIT_POL=1`.
- Keep:
  - `GUIMAI_GTM_TIM_TSSM_SHIFT_RIGHT=1`
  - `TIM_MODE=6`
  - `transferBothWords`
  - `DMA words=32`

Expected next log:

- Confirm:
  - `fw=2026-05-08-gtm-tssm-right-pol1`
  - `[GUIMAI_TBCM] init ... shift=right init_pol=1 ...`
- Paste:
  - `[GUIMAI_AFD_DMA] done=...`
  - all `[GUIMAI_AFD_DMA_HIST]`
  - all `[GUIMAI_TSSM_SLICE]`
  - all `[GUIMAI_DEC_PAIR_BITS] h1 ...`

How to judge:

- If `zero + neg1` drops sharply in one slice, use that slice as the PCM extraction candidate.
- If it remains mostly `zero/neg1`, stop changing `DSL/ECNT_RESET`; the next work should configure TSSM TDU/ECTRL or abandon TSSM for a different capture path.

Observed result:

- Right-shift with `ECNT_RESET=1` is also not usable.
- Example:
  - `h1_b0_15 words=16720 changed=3202 zero=3928 neg1=11484 sat=84`
  - `h1_b4_19 words=16720 changed=3192 zero=3931 neg1=11498 sat=94`
  - `h1_b8_23 words=16720 changed=3205 zero=3903 neg1=11508 sat=100`
- `zero + neg1` is still the majority of every slice, so the data is still fill/state-like rather than PCM.

Conclusion:

- Stop testing `DSL` and `ECNT_RESET`; all simple shift direction and initial polarity combinations have been tried.
- The failure is not ES8388 init, I2C, ARU/FIFO/DMA transport, or `transferBothWords`.
- The remaining issue is TSSM configuration: GPR source selection, `CNTS` bit count, and shift clock/TDU source.

## 2026-05-08 Manual Findings For TSSM

From the GTM TIM Serial Shift Mode documentation:

- In TSSM, the shift register is `TIM[i]_CH[x]_CNT[23:0]`.
- `TIM[i]_CH[x]_CNT[23:0]` means:
  - `TPWM`: duration
  - `TPIM`: pulse sum
  - `TIEM`: edge count
  - `TIPM`: received edges
  - `TGPS`: elapsed time
  - `TSSM`: shift data
- `GPR0/GPR1/CNTS[31:24]` include `ECNT[7:0]`, so high byte values such as `0x02` or `0x03` are edge-counter/status metadata, not audio.
- Current `transferBothWords` history is best interpreted as:
  - `h0 = GPR0`
  - `h1 = GPR1`
- With the current `CTRL` setting:
  - `GPR0_SEL=3`, `EGPR0_SEL=0`: `GPR0` uses `CNTS`
  - `GPR1_SEL=3`, `EGPR1_SEL=0`: `GPR1` uses `CNT`
- Therefore `h1` is still the right place to inspect TSSM shift data, and `h0` is mostly `CNTS`/status.

Useful manual details:

- `TIM_MODE=110b` selects TSSM.
- `DSL` in TSSM selects shift direction:
  - `0`: shift left
  - `1`: shift right
- `ECNT_RESET` in TSSM defines the initial polarity for the shift register.
- `CNTS_SEL` in TSSM selects the source signal for registered or latched shift-out operation:
  - `0`: use `F_OUTx`
  - `1`: use `TIM_INx`
- `CNTS[7:0]` defines the amount of bits stored inside `CNT`.
- Each shift clock increments `CNTS[15:8]`.
- When `CNTS[15:8] >= CNTS[7:0]`, a capture is performed, `NEWVAL_IRQ` is issued, and `CNTS[15:8]` is reset.
- `CNTS[17:16]` selects the TSSM shift clock:
  - `00b`: source selected by `USE_TDU_CLK_SRC`; can be a CMU clock source or local `tdu_sample_evt`
  - `01b`: `tdu_word_evt` is used as shift clock
  - `10b`: source selected by `USE_TDU_CLK_SRC`, gated with `tdu_word_evt=1`
  - `11b`: source selected by `USE_TDU_CLK_SRC`, gated with `tdu_word_evt=0`
- `CNTS[19:18]` selects the external capture source in TSSM:
  - `00b`: source selected by `EXT_CAP_SRC`
  - `01b`: `tdu_word_evt`
  - `10b`: `tdu_frame_evt`
  - `11b`: reserved
- `CNTS[21:20]` selects `TSSM_OUTx` behavior:
  - `00b`: constant output `0`
  - `10b`: shift output
  - `01b`: latched output
  - `11b`: registered output
- `CNTS[22]` enables `GPR0` as a shadow register for `CNT`; update occurs on a trigger after CPU writes `GPR0`.
- `CNTS[23]` is related to `SHIFT_OUT`/`TSSM_OUT` behavior.

Current configuration problems:

- Current log shows `cnts=0x000008`, so:
  - `CNTS[7:0] = 8`: only 8 shift clocks before capture, too short for 16-bit PCM.
  - `CNTS[17:16] = 00`: shift clock is the `USE_TDU_CLK_SRC`/local `tdu_sample_evt` path, not proven to be BCK.
- A plausible next TSSM test should not change `DSL` or `ECNT_RESET` again.
- The next test should instead configure:
  - `GPR1` remains `CNT`
  - `CNTS[7:0] = 16` first, possibly `32` if I2S slot alignment requires it
  - `CNTS[17:16]` and TDU source so the shift clock is actually BCK
  - If external capture is needed, `CNTS[19:18]` should be set according to whether `tdu_word_evt` or `tdu_frame_evt` marks word/frame boundaries

Open items to look up before further code changes:

- `USE_TDU_CLK_SRC` bit/location and how to select local `tdu_sample_evt`.
- `TDUV`, `TDUC`, and `ECTRL` fields required to generate `tdu_sample_evt`, `tdu_word_evt`, and `tdu_frame_evt`.
- Whether a TIM channel can use another pin/channel, such as BCK, directly as TSSM shift clock.
- Whether WS can be used as `tdu_word_evt` or `tdu_frame_evt` for 16/32-bit I2S alignment.

## 2026-05-08 TDU_RESYNC Manual Tables

User provided GTM manual Table 46 and Table 47 for `TDU_RESYNC`.

Table 46: Behavior of `TDU_RESYNC` with `SLICING != 0b11`.

Useful entries:

- `TDU_RESYNC=1000b`:
  - reset `TO_CNT`, `TO_CNT1`, and `TO_CNT2` on the event selected by `EXT_CAP_SRC`.
  - This is the most useful candidate if WS can be routed into `EXT_CAP_SRC`; it can make the TDU counters restart on the I2S word/frame boundary.
- `TDU_RESYNC=1--b`:
  - if `SLICING != 0b00`, reset `TO_CNT2` on `tdu_sample_evt`.
  - This may be useful for local sample-event cleanup, but it does not directly solve BCK/WS alignment.
- `TDU_RESYNC=1-1b`:
  - reset `TO_CNT` on `tdu_word_evt`.
- `TDU_RESYNC=11--b`:
  - reset `TO_CNT1` on `tdu_frame_evt`.
  - if `SLICING=0b01`, also reset `TO_CNT` on `tdu_frame_evt`.

Table 47: Behavior of `TDU_RESYNC` with `SLICING = 0b11`.

Useful entries:

- `TDU_RESYNC=1000b`:
  - load `TO_CNT` with `TOV2`.
  - reset `TO_CNT1` on the event selected by `EXT_CAP_SRC`.
  - This is different from Table 46 and is less directly useful unless `SLICING=0b11` is intentionally used.
- `TDU_RESYNC=1-1b`:
  - reset `TO_CNT` on `tdu_word_evt`.
- `TDU_RESYNC=-1--b`:
  - reset `TO_CNT1` on `tdu_frame_evt`.

Decision from these tables:

- Do not continue changing `DSL` or `ECNT_RESET`.
- The next technically meaningful TSSM direction is TDU/EXT_CAP configuration:
  - make TIM2_CH4 TSSM shift on BCK, most likely through the local TDU sample/word event path;
  - make WS reset or align the TDU counters, likely through `EXT_CAP_SRC` plus `TDU_RESYNC=1000b`;
  - set `CNTS[7:0]` to `16` first, because `8` captures too few bits for 16-bit PCM.
- Before changing code, still need the bitfield definitions for:
  - `ECTRL.EXT_CAP_SRC`
  - `ECTRL.TDU_RESYNC`
  - `TDUV.TCS_USE_SAMPLE_EVT`
  - `TDUV.TDU_SAME_CNT_CLK`
  - `TDUV.TCS`
  - `TDUC.TO_CTRL` / `TOCTRL` and any `TOV/TOV2` fields
  - `USE_PREV_TDU_IN` or equivalent field that lets TIM2_CH4 TDU use previous channel TIM2_CH3/BCK input

Search keywords for the manual:

- `TDU_RESYNC`
- `EXT_CAP_SRC`
- `USE_PREV_TDU_IN`
- `TCS_USE_SAMPLE_EVT`
- `TDU_SAME_CNT_CLK`
- `TO_CTRL`
- `TOCTRL`
- `TDUV`
- `TDUC`
- `TIM ECTRL`

## 2026-05-08 TDUV/TDUC/ECTRL Register Fields

User provided screenshots for:

- `TIM[i]_CH[x]_TDUV` at offset `001018H + i*800H + x*80H`
- `TIM[i]_CH[x]_TDUC` at offset `001014H + i*800H + x*80H`
- `TIM[i]_CH[x]_ECNT` at offset `00100CH + i*800H + x*80H`
- `TIM[i]_CH[x]_ECTRL` at offset `001028H + i*800H + x*80H`

`TDUV` confirmed fields:

- `TOV` bits `7:0`: timeout compare value slice0 for `TO_CNT`.
- `TOV1` bits `15:8`: timeout compare value slice1 for `TO_CNT1`.
- `TOV2` bits `23:16`: timeout compare value slice2 for `TO_CNT2`; if `SLICING=0b11`, `TOV2` operates as a shadow register for `TO_CNT`.
- `SLICING` bits `25:24`: cascading of counter slices.
  - `00b`: combine slice2/slice1/slice0 to one 24-bit counter, or reserved depending on LUT settings.
  - `01b`: combine slice1/slice0 to one 16-bit counter; slice2 as 8-bit counter, or combine slice1/slice0 and slice2 unusable depending on LUT settings.
  - `10b`: use slice2/slice1/slice0 as three 8-bit counters, or use slice1/slice0 as two 8-bit counters and slice2 unusable depending on LUT settings.
  - `11b`: use slice1/slice0 as two 8-bit counters.
- `TCS_USE_SAMPLE_EVT` bit `26`:
  - `0b`: CMU clock selected by `TCS` is used by `TO_CNT` and `TO_CNT2`.
  - `1b`: CMU clock selected by `TCS` is used by `TO_CNT2`; `tdu_sample_evt` is used by `TO_CNT`.
- `TDU_SAME_CNT_CLK` bit `27`:
  - `0b`: `TO_CNT` clock selected by `TCS/TCS_USE_SAMPLE_EVT`; `TO_CNT1` clocked on `tdu_word_evt`.
  - `1b`: `TO_CNT1` uses same clock as `TO_CNT`.
- `TCS` bits `30:28`: timeout clock selection.
  - `000b`..`111b`: select `CMU_CLK0`..`CMU_CLK7`.
- bit `31`: reserved, read as zero and write as zero.

`TDUC` confirmed fields:

- Register is writable if timeout unit is disabled (`TOCTRL=0b00`).
- If `USE_LUT != 0b00`, input signal generation is by lookup table; `TO_CNT2` is writable at any time, `TO_CNT` and `TO_CNT1` will not be changed.
- `TO_CNT` bits `7:0`: current timeout value slice0.
  - If `SLICING != 0b11`, counter resets to `0x00` on `TDU_RESYNC`.
  - If `SLICING = 0b11`, counter loads `TOV2` on `TDU_RESYNC`.
- `TO_CNT1` bits `15:8`: current timeout value slice1; resets to `0x00` on `TDU_RESYNC`.
- `TO_CNT2` bits `23:16`: current timeout value slice2; resets to `0x00` on `TDU_RESYNC`.
- bits `31:24`: reserved, read as zero and write as zero.

`ECNT` confirmed fields:

- `ECNT` bits `15:0`: SMU edge counter.
- If TIM channel is disabled, `ECNT` content gets frozen.
- A read auto-clears bits `15:1`; further reads show bit0 as actual input signal value of the channel.
- bits `31:16`: reserved.

`ECTRL` confirmed fields:

- `EXT_CAP_SRC` bits `3:0`: selected source for triggering `EXT_CAPTURE`.
  - `0x0`: `NEW_VAL_IRQ` of following channel.
  - `0x1`: `AUX_IN`.
  - `0x2`: `CNTOFL_IRQ` of following channel.
  - `0x3`: if `CICTRL=1`, use `TIM_IN(x)` as input for channel x; if `CICTRL=0`, use `TIM_IN(x-1)` for channel x, or `TIM_IN(m-1)` if x is 0.
  - `0x4`: `ECNTOFL_IRQ` of following channel.
  - `0x5`: `TODET_IRQ` of following channel.
  - `0x6`: `GLITCHDET_IRQ` of following channel.
  - `0x7`: `GPROFL_IRQ` of following channel.
  - `0x8`: `cmu_clk` selected by `CLK_SEL` of following channel.
  - `0x9`: `REDGE_DET` of following channel.
  - `0xA`: `FEDGE_DET` of following channel.
  - `0xB`: logical OR of `FEDGE_DET` and `REDGE_DET` of following channel.
  - `0xC`: `tdu_sample_evt` of local TDU.
  - `0xD`: `tdu_word_evt` of local TDU.
  - `0xE`: `tdu_frame_evt` of local TDU.
  - `0xF`: reserved.
- `USE_PREV_TDU_IN` bit `5`: select input data source for TDU.
  - `0b`: use input data of local filter for TDU.
  - `1b`: use input data of previous channel, after filter unit, for TDU.
- `TODET_IRQ_SRC` bits `7:6`: source for `TODET_IRQ`.
  - `00b`: use `tdu_timeout_evt`.
  - `01b`: use `tdu_word_evt`.
  - `10b`: use `tdu_frame_evt`.
  - `11b`: use `tdu_sample_evt`.
- `TDU_START` bits `10:8`: position confirmed, detailed encoding still needed.
- `TDU_STOP` bits `14:12`: position confirmed, detailed encoding still needed.
- `TDU_RESYNC` bits `19:16`: position confirmed; use together with Table 46/47 behavior.
- `USE_LUT` bits `22:21`: position confirmed, detailed encoding still needed.
- `EFLT_CTR_RE` bit `24`: position confirmed, detailed meaning still needed.
- `EFLT_CTR_FE` bit `25`: position confirmed, detailed meaning still needed.
- `SWAP_CAPTURE` bit `28`: position confirmed, detailed meaning still needed.
- `IMM_START` bit `29`: position confirmed, detailed meaning still needed.
- `CLK_SEL` bit `30`: position confirmed, detailed meaning still needed.
- `USE_PREV_CH_IN` bit `31`: position confirmed, detailed meaning still needed.

Immediate coding implication:

- For TIM2_CH4, setting `ECTRL.USE_PREV_TDU_IN=1` should let the local TDU use previous channel input after filter, i.e. TIM2_CH3/BCK, as the TDU input source.
- `TDUV.TCS_USE_SAMPLE_EVT=1` can make `TO_CNT` use `tdu_sample_evt` instead of CMU clock.
- `TDUV.TDU_SAME_CNT_CLK=1` can make `TO_CNT1` use the same clock as `TO_CNT`; otherwise `TO_CNT1` is clocked on `tdu_word_evt`.
- `ECTRL.EXT_CAP_SRC=0x3` is a candidate for using a TIM input as external capture. With `CICTRL=0`, for channel x it selects `TIM_IN(x-1)`, which would be TIM2_CH3/BCK for TIM2_CH4. With `CICTRL=1`, it selects local TIM2_CH4/SD.
- For WS alignment, `EXT_CAP_SRC=0x9/0xA/0xB` of the following channel may be relevant only if the "following channel" can be arranged to be WS; with current TIM2_CH4, following channel would not obviously be TIM2_CH1/WS, so this is not yet proven.
- `TDU_RESYNC=1000b` can now be written as `ECTRL[19:16] = 0x8`, but the right `EXT_CAP_SRC` source must be chosen first.

## 2026-05-08 TDU_START/TDU_STOP Encodings

User provided the `ECTRL.TDU_START` and partial `ECTRL.TDU_STOP` field descriptions.

`TDU_START` bits `10:8`: condition which starts the TDU unit.

Note:

- If `SLICING=0b11`, every start/restart loads `TO_CNT` with `TOV2`.
- `tdu_start_000_event` is defined as each write of `TOCTRL != 0`, independent of current `TOCTRL`, while `TDU_START=0b000` and TDU is stopped. It lasts one system clock cycle.

Confirmed `TDU_START` encodings:

- `000b`: start once immediately on `tdu_start_000_event`.
- `001b`: start once with occurrence of first `cmu_clk` selected by `CLK_SEL` when measure unit is enabled by `TIM_EN=1`.
- `010b`: start once with occurrence of first active edge selected by `TOCTRL`; restart on `tdu_frame_evt` if TDU is stopped.
- `011b`: start once with occurrence of first active edge selected by `TOCTRL`.
- `100b`: start/restart with occurrence of external capture event; if TDU is stopped, restart again.
- `101b`: start/restart with occurrence of first `cmu_clk` selected by `CLK_SEL` when measure unit is enabled by `TIM_EN=1`; if TDU is stopped, restart again.
- `110b`: start once with occurrence of external capture event; restart on `tdu_frame_evt` if TDU is stopped.
- `111b`: start/restart with occurrence of first active edge selected by `TOCTRL`; if TDU is stopped, restart again.

`TDU_STOP` bits `14:12`: condition which stops the TDU unit.

Note:

- `tdu_toctrl_0_event` is each write of `TOCTRL=0`, independent of current `TOCTRL`, while TDU is started. It lasts one system clock cycle.

Confirmed visible `TDU_STOP` encodings:

- `000b`: immediate stop counting of TDU on `tdu_toctrl_0_event`.
- `001b`: stop counting of TDU on `tdu_word_evt` or on `tdu_toctrl_0_event`.
- `010b`: stop counting of TDU on `tdu_frame_evt` or on `tdu_toctrl_0_event`.
- `011b`: stop counting of TDU on `tdu_timeout_evt` or on `tdu_toctrl_0_event`.
- `100b`: stop counting of TDU on external capture event or on `tdu_toctrl_0_event`.

Still missing:

- `TDU_STOP=101b`, `110b`, and `111b` descriptions, if the manual defines them below the current screenshot.
- If no further rows exist in the manual, do not use `TDU_STOP=101b/110b/111b`; keep the next diagnostic on the confirmed encodings only.

## 2026-05-08 TDU Operation Notes

User provided the TDU functional description.

Confirmed behavior:

- Each TDU slice has its own start/stop control.
- `TO_CNT`, `TO_CNT1`, and `TO_CNT2` are reloaded depending on the counter/compare configuration.
- Compare results from the three TDU slices and the selected resolution generate:
  - `tdu_sample_evt`
  - `tdu_word_evt`
  - `tdu_frame_evt`
- Primary TDU resolution is selected by `TDUV.TCS`; the selected `CMU_CLKx` clocks the TDU.
- Individual timeout/compare values are selected by `TDUV.TOV`, `TOV1`, and `TOV2`.
- When bit slices are cascaded through `SLICING`, `TCS_USE_SAMPLE_EVT`, and `TDU_SAME_CLK`, the counting resolution can switch to:
  - `tdu_sample_evt`
  - `tdu_word_evt`
- Counter compare units start on the first start event configured by `TDU_START`.
- They continue until the first stop event configured by `TDU_STOP`.
- If a start event and compare/count resolution event occur in the same clock cycle, counters increment or reload/reset according to `TDU_RESYNC` immediately. No `tdu_sample_evt`, `tdu_word_evt`, or `tdu_frame_evt` is generated in that cycle.
- If a stop event occurs, counters do not change their values.
- If a stop event and compare/count resolution event occur in the same clock cycle, the corresponding TDU event is generated.
- If start and stop events occur in the same clock cycle, counters do not change their values and no TDU events are generated.
- The TDU function configured with `TDU_RESYNC=0000b` and `TDU_START=000b` can be started/stopped through `TIM[i]_CH[x]_CTRL.TOCTRL`.
- Timeout detection sensitivity to falling/rising/both edges is configured by `CTRL.TOCTRL`.
- When timeout is detected:
  - internal `TIM_TIMEOUTx_IRQ` is generated;
  - `TODET` bit is set in `IRQ_NOTIFY`;
  - if ARU access is enabled, actual `GPR0/GPR1` values and latest signal level are sent to ARU on `TDU_TIMEOUT_EVT`.

Useful conclusion:

- `TOCTRL` is in `TIM[i]_CH[x]_CTRL`, not in `ECTRL/TDUV/TDUC`.
- To make TDU react to BCK input edges, the next required manual item is the `CTRL.TOCTRL` bitfield and encoding.
- This is important because `TDU_START` modes `010/011/111` and many `TDU_RESYNC` Table 46 rows depend on "active edge selected by `TOCTRL`".

Still needed:

- `TIM[i]_CH[x]_CTRL.TOCTRL` bit position and values.
- `CTRL.TOCTRL=0` behavior is already implied: writing `TOCTRL=0` creates `tdu_toctrl_0_event` while TDU is started.

## 2026-05-08 CTRL Register Field Positions

User provided `TIM[i]_CH[x]_CTRL` bit layout.

Confirmed `CTRL` field positions and descriptions:

- `TIM_EN` bit `0`:
  - `0b`: channel disabled.
  - `1b`: channel enabled.
  - Enabling resets `ECNT`, `CNT`, `GPR0`, and `GPR1`.
- `TIM_MODE` bits `3:1`:
  - `000b`: TPWM.
  - `001b`: TPIM.
  - `010b`: TIEM.
  - `011b`: TIPM.
  - `100b`: TBCM.
  - `101b`: TGPS.
  - `110b`: TSSM.
- `OSM` bit `4`:
  - `0b`: continuous operation.
  - `1b`: one-shot mode.
- `ARU_EN` bit `5`:
  - `0b`: `GPR0/GPR1` not routed.
  - `1b`: `GPR0/GPR1` routed to ARU.
- `CICTRL` bit `6`:
  - `0b`: use `TIM_IN(x)` as input for channel x.
  - `1b`: use `TIM_IN(x-1)` as input for channel x, or `TIM_IN(m-1)` if x is 0.
- `TBU0_SEL` bit `7`: TBU_TS0 bits input select for `TIM0_CH[x]_GPRz`, only applicable to TIM0.
- `GPR0_SEL` bits `9:8`:
  - If `EGPR0_SEL=0` / `EGPR0_SEL=1`:
  - `00b`: use `TBU_TS0` / use `ECNT`.
  - `01b`: use `TBU_TS1` / use `TIM_INP_VAL`.
  - `10b`: use `TBU_TS2` / reserved.
  - `11b`: use `CNTS`; if TGPS mode in channel 0 is selected, use TIM filter `F_OUT` / reserved.
- `GPR1_SEL` bits `11:10`:
  - If `EGPR1_SEL=0` / `EGPR1_SEL=1`:
  - `00b`: use `TBU_TS0` / use `ECNT`.
  - `01b`: use `TBU_TS1` / use `TIM_INP_VAL`.
  - `10b`: use `TBU_TS2` / reserved.
  - `11b`: use `CNT` / reserved.
  - Note: in TBCM mode, `EGPR1_SEL=1` and `GPR1_SEL=01b` selects `TIM_INP_VAL`; in all other cases, TIM filter `F_OUT` is used.
- `CNTS_SEL` bit `12`:
  - In TSSM, source for registered or latched shift-out operation.
  - `0b`: use `F_OUTx`.
  - `1b`: use `TIM_INx`.
  - Other mode meanings are mode dependent; functionality is disabled in TIPM/TGPS/TBCM.
- `DSL` bit `13`:
  - In TSSM, `0b` shift left, `1b` shift right.
  - Other mode meanings are signal-level control.
- `ISL` bit `14`: ignore signal level; mode-dependent.
- `ECNT_RESET` bit `15`.
- `FLT_EN` bit `16`
- `FLT_CNT_FRQ` bits `18:17`
- `EXT_CAP_EN` bit `19`
- `FLT_MODE_FE` bit `20`
- `FLT_CTR_RE` bit `21`
- `FLT_MODE_RE` bit `22`
- `FLT_CTR_FE` bit `23`
- `CLK_SEL` bits `26:24`
- `FR_ECNT_OFL` bit `27`
- `EGPR0_SEL` bit `28`
- `EGPR1_SEL` bit `29`
- `TOCTRL` bits `31:30`

`CTRL.TOCTRL` bits `31:30`: timeout control.

- Writing `TOCTRL=0` stops TDU every time, independent of previous `TOCTRL` state.
- `00b`: timeout feature disabled.
- `01b`: timeout feature enabled for rising edge only.
- `10b`: timeout feature enabled for falling edge only.
- `11b`: timeout feature enabled for both edges.

Coding implication:

- For BCK-driven TDU experiments, use `TOCTRL=01b` for rising-edge-only first, matching the previous BCK-rising capture attempts.
- Use `TOCTRL=11b` only if we intentionally want both BCK edges and account for a doubled event rate.

## 2026-05-08 TSSM TDU BCK-16 Diagnostic Build

Current firmware tag:

`2026-05-08-gtm-tssm-tdu-bck16`

Code direction:

- Keep TSSM enabled on TIM2_CH4.
- Keep `GPR1=CNT`, so `h1` remains the TSSM shift-register data path.
- Stop changing `DSL` and `ECNT_RESET`; keep the last tested values:
  - `shift=right`
  - `init_pol=1`
- Change TSSM shadow counter from the old TBCM-style BCK bit mask to a TSSM bit-count setup:
  - `CNTS[7:0] = 16`
  - `CNTS[17:16] = 00`
  - `CNTS[19:18] = 00`
- Configure local TDU for the first BCK-driven experiment:
  - `USE_PREV_TDU_IN=1`, so TIM2_CH4 TDU uses previous channel TIM2_CH3/BCK after filter.
  - `TOCTRL=01b` is written at capture start, enabling rising-edge timeout detection.
  - `TDU_START=000b`, started by writing `TOCTRL != 0` after `TIM_EN=1`.
  - `TDU_STOP=000b`, stopped by writing `TOCTRL=0`.
  - `TDU_RESYNC=0000b`.
  - `TOV/TOV1/TOV2=1/16/32`.
  - `SLICING=00b` for this first scoped check.

Important limitation:

- This build deliberately does not try WS alignment yet.
- `EXT_CAP_SRC` is still left at reset because with current TIM2_CH4 routing it is not yet proven how to make `EXT_CAP_SRC` see TIM2_CH1/WS. First verify that the TDU can be driven from TIM2_CH3/BCK and that `h1` changes from fill-like `0x0000/0xFFFF` runs into a plausible 16-bit window.

Expected init log:

- `fw=2026-05-08-gtm-tssm-tdu-bck16`
- `[GUIMAI_TBCM] init ... tdu=1 ... cnts=0x000010 bits=16 shift_clk=0 ext_cap=0 toctrl=0 prev_tdu=1 tdu_start=0 tdu_stop=0 tdu_resync=0 slicing=0 tov=1/16/32 ...`
- During capture, `toctrl` in the live `CTRL` value should become rising-edge enabled because `guimai_gtm_tbcm_start()` writes `TOCTRL=01b` after `TIM_EN=1`.

Next log to paste:

- `[GUIMAI] fw=...`
- `[GUIMAI_TBCM] init ...`
- the first two `[GUIMAI_CPU1] samples=0 ...` lines
- `[GUIMAI_AFD_DMA] done=...`
- all `[GUIMAI_AFD_DMA_HIST]`
- all `[GUIMAI_TSSM_SLICE]`
- all `[GUIMAI_DEC_PAIR_BITS] h1 ...`

How to judge:

- If `tbcm_events`/DMA stop, TDU start/clocking is wrong; next change should focus on `TDU_START` or `TCS_USE_SAMPLE_EVT`.
- If DMA continues but `h1` is still dominated by `zero/neg1`, the TDU is not yet feeding the TSSM shift clock correctly; next likely change is `CNTS[17:16]` or `TDUV.SLICING/TCS_USE_SAMPLE_EVT`.
- If one `h1` slice has much lower `zero+neg1` and a realistic `avg_abs`, promote that slice to PCM extraction candidate.

Observed result:

- The TDU BCK-16 build initialized as intended:
  - `CNTS=0x00000010`
  - `prev_tdu=1`
  - `TDUV=0x00201001`
  - `ECTRL=0x00000020`
- DMA and ARU/FIFO transport still work.
- TSSM now reports 16-bit capture setup in h0, for example `0x001010`.
- However, `h1` is still mostly fill-like:
  - `h1_b0_15 words=48864 changed=8557 zero=22563 neg1=22885`
  - `h1_b4_19 words=48864 changed=8503 zero=22554 neg1=22936`
  - `h1_b8_23 words=48864 changed=8424 zero=22586 neg1=22967`
- `zero + neg1` is still the majority, so this is not PCM.
- `tbcm_events=3053` over about `5844 ms`, which is only hundreds of TSSM windows per second, not a BCK/16 rate. TDU/shift-clock routing is therefore still not right.

Next build:

- Firmware tag: `2026-05-08-gtm-tssm-tdu-timin16`.
- Keep TDU settings from BCK-16.
- Change only TSSM `CNTS_SEL`:
  - previous: `CNTS_SEL=0`, use `F_OUTx`
  - next: `CNTS_SEL=1`, use `TIM_INx`
- Add live `CTRL/ECTRL/TDUV/TDUC` fields to `[GUIMAI_CPU1]` logs.
- Purpose: verify whether the shift input source was the reason `h1` remained dominated by `0x0000/0xFFFF`.

Observed result:

- The `tdu-timin16` build initialized as intended:
  - `cnts_sel=1 tim_in=1`
  - `CTRL=0x0000BF2C`
  - `ECTRL=0x00000020`
  - `TDUV=0x00201001`
- During capture, live `CTRL` became `0x4000BF2D`, so `TIM_EN=1` and `TOCTRL=01b` were active.
- `TDUC` changed during capture (`0x16`, `0x1B`, `0x0E` in the pasted run), so the TDU is not dead; it is counting.
- But `h1` is still not PCM:
  - `h1_b0_15 words=22576 changed=3990 zero=10250 neg1=10741`
  - `h1_b4_19 words=22576 changed=3969 zero=10246 neg1=10766`
  - `h1_b8_23 words=22576 changed=3919 zero=10257 neg1=10798`
- `tbcm_events=1410` over about `3040 ms`, still only about `464` windows per second.
- Conclusion: changing `CNTS_SEL` from `F_OUTx` to `TIM_INx` did not fix the TSSM window rate or the fill-like `0x0000/0xFFFF` samples. Stop testing `CNTS_SEL` for now.

Next build:

- Firmware tag: `2026-05-08-gtm-tssm-tdu-slice2`.
- Change only `TDUV.SLICING`:
  - previous: `SLICING=0`
  - next: `SLICING=2`
- Keep the proven/known current settings unchanged:
  - `mode=6`
  - `tssm=1`
  - `tdu=1`
  - `shift=right`
  - `init_pol=1`
  - `cnts_sel=1`
  - `tim_in=1`
  - `bits=16`
  - `prev_tdu=1`
  - `toctrl=01b` during capture
- Purpose: verify whether TDU slicing was leaving us on the wrong 24-bit/timeout-window behavior instead of producing usable 16-bit TSSM windows.

For the next paste, only these keywords are needed:

- `[GUIMAI] fw=`
- `[GUIMAI_TBCM] init`
- first two `[GUIMAI_CPU1] samples=0`
- `[GUIMAI_AFD_DMA] done=`
- all `[GUIMAI_TSSM_SLICE]`
- `[GUIMAI_DEC_PAIR_BITS] h1`

Observed result:

- The `shiftclk1` build initialized as intended:
  - `CNTS=0x00010010`
  - `cnts=0x010010`
  - `shift_clk=1`
  - `slicing=2`
  - `TDUV=0x02201001`
- Runtime still shows the same slow event rate:
  - `tbcm_events=1998` over `4092 ms`, about `488` windows per second.
- `h1` improved only slightly but is still not PCM:
  - `h1_b0_15 words=31984 changed=5372 zero=14155 neg1=13364`
  - `h1_b4_19 words=31984 changed=5411 zero=14109 neg1=13354`
  - `h1_b8_23 words=31984 changed=5386 zero=14048 neg1=13413`
- `zero + neg1` is still about `86%`, so this is not a usable 16 kHz PCM stream. `shift_clk=1` alone is not the missing setting.

Next build:

- Firmware tag: `2026-05-08-gtm-tssm-tcs-sample`.
- Change only `TDUV.TCS_USE_SAMPLE_EVT`:
  - previous: `0`
  - next: `1`
- Keep:
  - `shift_clk=1`
  - `slicing=2`
  - `cnts_sel=1`
  - `tim_in=1`
  - `prev_tdu=1`
  - `bits=16`
- Purpose: check whether the TDU counter needs the TIM sample event as its clock/control source to produce proper TSSM word windows.

For the next paste, same short set:

- `[GUIMAI] fw=`
- `[GUIMAI_TBCM] init`
- first two `[GUIMAI_CPU1] samples=0`
- `[GUIMAI_AFD_DMA] done=`
- all `[GUIMAI_TSSM_SLICE]`
- `[GUIMAI_DEC_PAIR_BITS] h1`

## 2026-05-08 Ring DMA RAM Adjustment

The first continuous ring DMA build used:

- `chunk=4096`
- `ring=8192 words`
- `ring_bytes=32768`
- `IFX_ALIGN(32768)`
- destination circular range `32768`

Build failed with linker RAM placement errors:

- `tc E112: cannot locate ... sections`
- requirement about `218K` RAM in range `0x70000000-0x7003c000`
- contiguous group restriction

Interpretation:

- The 32KB-aligned ring buffer likely created too much placement pressure in the available contiguous RAM group.

Adjusted build:

- Firmware tag: `2026-05-08-gtm-dma-ring2048-d0`.
- `GUIMAI_GTM_DMA_CHUNK_WORDS = 2048`.
- `GUIMAI_GTM_DMA_RING_WORDS = 4096`.
- Ring buffer size is now `16384` bytes.
- Ring buffer alignment changed to `IFX_ALIGN(16384)`.
- DMA destination circular range changed to `IfxDma_ChannelIncrementCircular_16384`.
- Init log now prints `ring_bytes`.

Expected init signature:

- `fw=2026-05-08-gtm-dma-ring2048-d0`
- `[GUIMAI_GTM_DMA] init ... chunk=2048 ring=4096 ring_bytes=16384 delay=0 left_ws=0 mode=ring ...`

Observed result:

- Init matched:
  - `chunk=2048`
  - `ring=4096`
  - `ring_bytes=16384`
  - `delay=0`
  - `left_ws=0`
  - `mode=ring`
  - `dst=0x70004000`
- DMA continuity fixed:
  - `gtm_lost=0` throughout the run.
  - `gtm_chunks` and `gtm_words` increased continuously.
- PCM production is stable:
  - `samples=15488`, then `30976`, `46528`, `62016`, `77504`
  - store: `samples=80000 elapsed_ms=5180 fs=15444`
- Server received all frames and wrote dumps:
  - metadata `samples=80000 elapsed_ms=5180`
  - observed source rate `15444.0`
  - raw peak `32768`
  - raw RMS `22862.7`
  - raw ZCR `0.486`
  - resampled to `16000`, peak `32768`, RMS `20653.5`, ZCR `0.245`

Interpretation:

- Ring DMA solved the main `gtm_lost` problem.
- The capture chain now reliably produces samples from BCK/WS/SD GPIO snapshots.
- Effective rate is about `15.44 kHz`, not exact `16 kHz`; server-side observed-rate resampling compensates this.
- The raw audio is still extremely hot/noisy. Since the silicon microphone was not connected, empty Xfyun text is expected. The high RMS/ZCR means the ADC/input is likely floating or the selected analog source has no valid signal.

Next phase:

- Connect a real mic/line source before judging ASR.
- If speech is still distorted after connecting the source, tune in this order:
  1. `GUIMAI_GTM_DMA_SAMPLE_LEADING_EDGE`
  2. `GUIMAI_GTM_DMA_LEFT_WS_LEVEL`
  3. `GUIMAI_GTM_DMA_I2S_DELAY_BITS`
  4. ES8388 ADC gain/input selection

## 2026-05-08 Manual Page: TDU Sub-unit Architecture Text

User provided the TDU architecture diagram and the following text around `28.13.3.3`.

Confirmed from the diagram:

- The TDU has three 8-bit slices:
  - slice0 associated with `WORD_EVT`
  - slice1 associated with `FRAME_EVT`
  - slice2 associated with `SAMPLE_EVT`
- Each slice has its own start/stop and reset/load path.
- The compare event logic consumes `GT_EVT` and `EQ_EVT` from the three slices and generates:
  - `tdu_sample_evt`
  - `tdu_word_evt`
  - `tdu_frame_evt`
  - `tdu_timeout_evt`
- `TCS_USE_SAMPLE_EVT` and `TDU_SAME_CNT_CLK` sit in the clock/control path between the slice/event cascade, not in the external pin path.

Confirmed from the text:

- Each `TDU_slice` has its own start/stop control.
- Reset/load control decides whether `TO_CNT`, `TO_CNT1`, and `TO_CNT2` are reloaded based on configuration and compare results.
- The primary resolution is selected by `TDUV.TCS`; the selected `CMU_CLKx` clocks the TDU.
- Timeout/compare values are selected by:
  - `TOV`
  - `TOV1`
  - `TOV2`
- With cascading by `SLICING`, `TCS_USE_SAMPLE_EVT`, and `TDU_SAME_CLK`, the counter resolution can be switched to:
  - `tdu_sample_evt`
  - `tdu_word_evt`
- Counters start on the first start event configured by `TDU_START`.
- Counters run until the first stop event configured by `TDU_STOP`.
- If a start event and compare/count resolution event occur in the same clock cycle, counters increment or reload/reset according to `TDU_RESYNC`, and no sample/word/frame event is generated in that cycle.
- If a stop event and compare/count resolution event occur in the same clock cycle, the corresponding sample/word/frame event is generated.
- If start and stop occur in the same clock cycle, counters do not change and no sample/word/frame event is generated.
- The function configured with `TDU_RESYNC=0000b` and `TDU_START=000b` can be started/stopped inside `TIM[i]_CH[x]_CTRL` by setting/resetting the `TOCTRL` bit.

What this changes:

- The diagram/text confirms that `TOV=16/16/32` cannot be judged without the missing cascade table.
- The phrase "More details see table above" is the next required manual item. That table should define how `SLICING`, `TCS_USE_SAMPLE_EVT`, and `TDU_SAME_CNT_CLK` select the slice counter clocks and how `WORD_EVT/FRAME_EVT/SAMPLE_EVT` are produced.

Next manual page needed:

- The table immediately above/before this text that mentions:
  - `SLICING`
  - `TCS_USE_SAMPLE_EVT`
  - `TDU_SAME_CLK` / `TDU_SAME_CNT_CLK`
  - `tdu_sample_evt`
  - `tdu_word_evt`
  - `tdu_frame_evt`
  - counter resolution / cascading

## 2026-05-08 Manual Table 30: TDU Slicing Cascade

User provided Table 30, "Which of the available 8 bit resources are cascaded with a chosen SLICING".

Confirmed counter modes:

- `24 bit`:
  - `CNT` counts on `TCS`.
  - `CNT >= TCMP` generates `tdu_sample_evt`.
  - `tdu_timeout_evt = tdu_sample_evt`.
  - `tdu_frame_evt = 0`.
  - `tdu_word_evt = 0`.
- `3x8 bit`:
  - `TO_CNT2 >= TOV2` generates `tdu_sample_evt`.
  - `TO_CNT >= TOV` generates `tdu_word_evt`.
  - `TO_CNT1 >= TOV1` generates `tdu_frame_evt`.
  - `TO_CNT2` always counts on `TCS`.
  - `TO_CNT` counts on:
    - `tdu_sample_evt` when `TCS_USE_SAMPLE_EVT=1`
    - `TCS` when `TCS_USE_SAMPLE_EVT=0`
  - `TO_CNT1` counts on:
    - `tdu_word_evt` when `TDU_SAME_CNT_CLK=0`
    - same clock as `TO_CNT` when `TDU_SAME_CNT_CLK=1`
  - `tdu_timeout_evt = tdu_word_evt` or `tdu_frame_evt` depending on the exact 3x8 row.
- `2x8 bit`:
  - `TO_CNT >= TOV` generates `tdu_word_evt`.
  - `TO_CNT1 >= TOV1` generates `tdu_frame_evt`.
  - `tdu_sample_evt = 0`.
- `1x8 bit + 1x16 bit`:
  - `TO_CNT2 >= TOV2` generates `tdu_sample_evt`.
  - `CNT >= TCMP` generates `tdu_frame_evt`.
  - `tdu_word_evt = 0`.
  - `CNT` counts on `TCS` when `TCS_USE_SAMPLE_EVT=0`, or on `tdu_sample_evt` when `TCS_USE_SAMPLE_EVT=1`.

Important implication:

- The TDU counter resolution is `TCS` / selected `CMU_CLKx` or an internal TDU event cascade. Table 30 does not show a mode where the TDU counters directly count external BCK edges.
- `USE_PREV_TDU_IN=1` and `TOCTRL=risingEdge` can make the TDU observe the previous channel input for timeout edge handling, but Table 30 says the slice counters themselves still count on `TCS` or internal TDU events.
- Therefore the previous idea "TOV=16 means 16 BCK edges" is not supported by Table 30 unless another manual page proves that `TCS`/`USE_TDU_CLK_SRC` can be sourced from the BCK-derived sample event.

Current failing baseline interpreted through Table 30:

- Current code uses:
  - `SLICING=2`
  - `TCS_USE_SAMPLE_EVT=0`
  - `TDU_SAME_CNT_CLK=0`
  - `TOV/TOV1/TOV2=16/16/32`
- In Table 30 this is the 3x8 row:
  - `TO_CNT2` counts on `TCS`
  - `TO_CNT` counts on `TCS`
  - `TO_CNT1` counts on `tdu_word_evt`
- That explains why the events are not locked to I2S BCK/WS boundaries and why the observed capture rate stays in the hundreds of windows per second.

Next technical decision:

- Either find the missing manual definition of `USE_TDU_CLK_SRC` / TSSM shift clock source and prove it can select a BCK-derived clock, or stop spending tests on TDU/TSSM and switch to the existing port-sampling DMA path that samples `MODULE_P11.IN.U` on BCK timing and decodes WS/SD in software.

## 2026-05-08 Manual Pages: TIM Serial Shift Mode

User provided the TSSM block diagram and Table 39/40 text.

Confirmed TSSM input and shift behavior:

- On each shift-clock event, the current `TSSM_INx` value is shifted into `TIM[i]_CH[x]_CNT`.
- If `ISL=0`, `FOUTx` is used as the shift-in value `TSSM_INx`.
- If `ISL=1`, `ECNT_RESET` defines the shift-in value for `TSSM_INx`.
- With `DSL=0`:
  - `TSSM_INx` is stored into `CNT[0]`.
  - `CNT[23:1] = CNT[22:0]`, i.e. shift left.
- With `DSL=1`:
  - `TSSM_INx` is stored into `CNT[23]`.
  - `CNT[22:0] = CNT[23:1]`, i.e. shift right.
- `CNTS[7:0]` defines how many bits are stored in `CNT`.
- Each shift clock increments `CNTS[15:8]`.
- When `CNTS[15:8] >= CNTS[7:0]`, TSSM performs capture, issues `NEWVAL_IRQ`, and clears `CNTS[15:8]`.
- With `ISL=1`, a capture also sets `CNT[23:0]` to `ECNT_RESET`.
- With `ISL=0`, capture does not reset `CNT`.

Confirmed capture/update behavior:

- With each capture event, `GPR0` and `GPR1` are updated according to `GPR0_SEL`, `GPR1_SEL`, `EGPR0_SEL`, and `EGPR1_SEL`.
- If `CNTS[22]=1`, `GPR1` operates as a shadow register for `CNTS`; this allows the bit count for sampling to be changed on a trigger after the CPU writes `GPR1`.
- External capture for TSSM requires `EXT_CAP_EN=1`.
- If external capture is enabled, external capture events capture the `GPRx`, reset `CNTS[15:8]`, optionally reset `CNT` depending on `ISL`, and issue `NEWVAL_IRQ`.

Confirmed TSSM shift clock selection, `CNTS[17:16]`:

- `00b`: source selected by `USE_TDU_CLK_SRC`; can be set to any `CMU_CLK` source or to the local TDU sample clock `tdu_sample_evt`.
- `01b`: `tdu_word_evt` is used as shift clock.
- `10b`: source selected by `USE_TDU_CLK_SRC`, gated with `tdu_word_evt`; if `tdu_word_evt=0`, shift clock is 0.
- `11b`: source selected by `USE_TDU_CLK_SRC`, gated with `tdu_word_evt`; if `tdu_word_evt=1`, shift clock is 0.

Confirmed TSSM external capture source selection, `CNTS[19:18]`:

- `00b`: source selected by `EXT_CAP_SRC`.
- `01b`: `tdu_word_evt`.
- `10b`: `tdu_frame_evt`.
- `11b`: reserved.

Important implication:

- The TSSM pages confirm that a clean I2S capture needs two distinct rates:
  - shift clock should be BCK-derived, either directly or through `USE_TDU_CLK_SRC` selecting `tdu_sample_evt`;
  - external capture should be word/slot boundary, likely `tdu_word_evt` or an `EXT_CAP_SRC` signal aligned to WS.
- Current failing logs with `shift_clk=0` still do not prove the source, because the location/configuration of `USE_TDU_CLK_SRC` is still missing.
- The pages also explain why `ISL=1` produced fill-like results: with `ISL=1`, capture resets `CNT` to `ECNT_RESET`, so a bad/slow capture rhythm causes repeated `0x0000` or `0xFFFFFF` style windows.

Next manual item still needed:

- The register/field definition for `USE_TDU_CLK_SRC`.
- It must show how TSSM selects between:
  - CMU clock source
  - local TDU sample clock `tdu_sample_evt`
- It is not `TIM[i]_CH[x]_ECTRL[12]` in this TC38A project, because local headers/manual show that bit belongs to `TDU_STOP`.

## 2026-05-08 Manual Page: CTRL CLK_SEL / TOCTRL

User provided the `TIM[i]_CH[x]_CTRL` field table around `CLK_SEL` and `TOCTRL`.

Confirmed:

- `CTRL.CLK_SEL` bits `26:24`: CMU clock source select for channel.
- The table explicitly describes each `CLK_SEL` value as:
  - if `ECLK_SEL=0` / `ECLK_SEL=1`
  - `000b`: `CMU_CLK0 selected` / `tdu_sample_evt of TDU selected`
  - `001b`: `CMU_CLK1 selected` / reserved
  - `010b`: `CMU_CLK2 selected` / reserved
  - `011b`: `CMU_CLK3 selected` / reserved
  - `100b`: `CMU_CLK4 selected` / reserved
  - `101b`: `CMU_CLK5 selected` / reserved
  - `110b`: `CMU_CLK6 selected` / reserved
  - `111b`: `CMU_CLK7 selected` / reserved
- `CTRL.TOCTRL` bits `31:30`:
  - `00b`: timeout feature disabled
  - `01b`: timeout feature enabled for rising edge only
  - `10b`: timeout feature enabled for falling edge only
  - `11b`: timeout feature enabled for both edges

This resolves the `USE_TDU_CLK_SRC` ambiguity:

- In the TSSM text, `USE_TDU_CLK_SRC` is the source selected by `CTRL.CLK_SEL`, extended by `ECTRL.ECLK_SEL`.
- For this project, selecting the local TDU sample clock means:
  - `CTRL.CLK_SEL = 000b`
  - `ECTRL.ECLK_SEL = 1`
- Selecting `CMU_CLK0` means:
  - `CTRL.CLK_SEL = 000b`
  - `ECTRL.ECLK_SEL = 0`

Immediate coding implication:

- Our current TSSM/TDU code sets `CTRL.CLK_SEL=IfxGtm_Cmu_Clk_0` but does not set `ECTRL.ECLK_SEL=1`.
- Therefore `CNTS[17:16]=00` has been using `CMU_CLK0`, not local `tdu_sample_evt`.
- This explains the previous non-I2S behavior: the TSSM shift clock source was not the local TDU sample event path.
- Next narrow build should set only:
  - `ECTRL.ECLK_SEL = 1`
  - keep `CTRL.CLK_SEL = 0`
  - keep `CNTS[17:16] = 00`
  - keep `CNTS[19:18] = 01` for `tdu_word_evt` external capture
  - keep `ISL=0`
- Expected effect: TSSM shifts on local `tdu_sample_evt` instead of CMU clock, while captures on `tdu_word_evt`.

Implemented next narrow build:

- Firmware tag: `2026-05-08-gtm-tssm-eclk-sample`.
- Added macro:
  - `GUIMAI_GTM_TIM_TSSM_ECLK_SEL = 1`
- Set:
  - `tim_ch->ECTRL.B.ECLK_SEL = 1`
- Added `eclk_sel=%u` to the `[GUIMAI_TBCM] init` log.
- Kept the rest of the current TSSM/TDU configuration unchanged:
  - `shift_clk=0`
  - `ext_cap=1`
  - `slicing=2`
  - `tcs_sample=0`
  - `sameclk=0`
  - `tdu_resync=2`
  - `tov=16/16/32`
  - `ISL=0`

Expected init signature:

- `fw=2026-05-08-gtm-tssm-eclk-sample`
- `shift_clk=0`
- `ext_cap=1`
- `eclk_sel=1`
- `ECTRL` should have bit 30 set compared with the previous `0x00020020`; expected around `0x40020020` before TIM enable and `0x40020020` plus runtime enable/status effects after start.

For the next paste:

- `[GUIMAI] fw=`
- `[GUIMAI_TBCM] init`
- first two `[GUIMAI_CPU1] samples=0`
- `[GUIMAI_AFD_DMA] done=`
- all `[GUIMAI_TSSM_SLICE]`
- `[GUIMAI_DEC_PAIR_BITS] h1`

Observed result:

- The `eclk-sample` build initialized as intended:
  - `eclk_sel=1`
  - `ECTRL=0x40020020`
  - `TDUV=0x02201010`
  - `shift_clk=0`
  - `ext_cap=1`
  - `tov=16/16/32`
- TDU counters were active immediately:
  - first live `TDUC=0x00254D08`
- Still not PCM:
  - `tbcm_events=352` over `1186 ms`, about `297 windows/s`.
  - `h1_b0_15 words=5648 changed=966 zero=3056 neg1=2202`
  - `h1_b4_19 words=5648 changed=981 zero=3055 neg1=2188`
  - `h1_b8_23 words=5648 changed=980 zero=3048 neg1=2210`
  - `h1` pair decode has only `peak=2 avg_abs=0`, i.e. no useful PCM amplitude.
- This confirms that setting `ECLK_SEL=1` did route `CNTS[17:16]=00` through the TDU sample clock path, but the local `tdu_sample_evt` itself is still not BCK/I2S-bit aligned.

Interpretation:

- Table 30 already indicated why: in the current `SLICING=2`, `TCS_USE_SAMPLE_EVT=0`, `TDU_SAME_CNT_CLK=0` setup, `tdu_sample_evt` is generated from `TO_CNT2` counting on `TCS`, not from external BCK edges.
- Therefore `ECLK_SEL=1` was necessary to resolve the manual ambiguity, but it is not sufficient to make TSSM count BCK bits.
- Continuing to tune only `TOV/TOV1/TOV2` is unlikely to produce a stable I2S capture unless another mechanism makes `TCS` or `tdu_sample_evt` BCK-derived.

Next direction:

- Stop treating TDU `TOV` values as BCK dividers.
- Either find the actual KBA implementation detail for "TIM_1 tracks clock and triggers TIM_2 on rising clock edge", or switch to the existing port-sampling DMA path that samples `MODULE_P11.IN.U` and decodes WS/SD in software.

## 2026-05-08 KBA-Style TIM1 Trigger TIM2 Suggestion Review

User provided a proposed explanation for "TIM_1 triggers TIM_2 on BCK rising edge".

Useful concept:

- The high-level KBA direction is plausible:
  - one TIM channel observes WS/LRCLK;
  - one TIM channel observes BCK edges;
  - one TIM channel samples SD and sends data through ARU/FIFO/DMA.
- For TC38A headers, the actual TIM channel-linking fields found locally are:
  - `CTRL.CICTRL`
  - `ECTRL.EXT_CAP_SRC`
  - `ECTRL.USE_PREV_TDU_IN`
  - `ECTRL.USE_PREV_CH_IN`
- The manual/source confirmed `EXT_CAP_SRC` can select internal events from the following channel, including edge-detect and TDU events.

Unverified or likely wrong details in the proposed explanation:

- Local TC38A headers do not show TIM fields named:
  - `TIM_IN_SEL`
  - `AUX_IN_SEL`
- Therefore the suggestion "set TIM_2_AUX_IN_SEL to TIM_1 FEDGE_DET" is not directly actionable in this codebase without another GTM routing register/page proving such a path.
- The suggestion says `TIM_2` can choose `TIM_1.FEDGE_DET` through `AUX_IN`; in the confirmed `ECTRL.EXT_CAP_SRC` table, `EXT_CAP_SRC=0x9/0xA/0xB` selects edge-detect of the *following* channel, not arbitrary previous channel `TIM_1`.
- Current physical mapping is non-consecutive:
  - WS = `TIM2_CH1`
  - BCK = `TIM2_CH3`
  - SD = `TIM2_CH4`
- That means "following channel" from `TIM2_CH4` is not `TIM2_CH3/BCK`, so the simple following-channel `EXT_CAP_SRC` route cannot directly use BCK for SD capture with the current mapping.

Actionable implication:

- Do not implement the proposed `AUX_IN_SEL/TIM_IN_SEL` code as written.
- If continuing with TIM/TSSM, the next useful manual search is for the exact `CICTRL` / `USE_PREV_CH_IN` behavior and whether TSSM input/shift can be sourced from previous channel events.
- Otherwise, switch to the already-existing `GUIMAI_GTM_DMA_CAPTURE` port-sampling path, which avoids dependence on unclear internal TIM channel routing.

## 2026-05-08 Switch Back To GTM DMA Port Capture

User asked to switch to the GTM TIM + DMA GPIO snapshot path.

Implemented build:

- Firmware tag: `2026-05-08-gtm-dma-port4096`.
- Enabled:
  - `GUIMAI_GTM_DMA_CAPTURE = 1`
- Disabled:
  - `GUIMAI_GTM_TBCM_DIAG = 0`
- Increased DMA chunk size:
  - `GUIMAI_GTM_DMA_CHUNK_WORDS: 2048 -> 4096`
  - Purpose: reduce CPU/DMA re-arm frequency and reduce the chance of BCK requests being lost during re-arm.
- Kept known-good trigger mode:
  - TIM2_CH3 / P11_6 BCK
  - `IfxGtm_IrqMode_pulse`
  - `IfxDma_ChannelRequestMode_oneTransferPerRequest`
  - source `MODULE_P11.IN.U`
- Removed the explicit `IfxDma_disableChannelTransaction()` at the beginning of `guimai_gtm_dma_start_chunk()`.
  - Purpose: avoid an extra disable window during chunk re-arm.
  - `guimai_gtm_dma_stop()` still disables the DMA channel.
- Added I2S decode phase macro:
  - `GUIMAI_GTM_DMA_I2S_DELAY_BITS = 1`
  - Purpose: standard I2S skips one BCK after WS changes before capturing the 16 data bits.
- Init log now prints:
  - chunk size
  - I2S delay bits
  - selected left WS level

Expected init signature:

- `fw=2026-05-08-gtm-dma-port4096`
- `[GUIMAI_GTM_DMA] init ... edge=leading irq=pulse chunk=4096 delay=1 left_ws=0 ...`
- No `[GUIMAI_TBCM] init` lines.

Primary success criteria:

- `gtm_chunks` continuously increments.
- `gtm_lost` stays `0` or very low.
- `gtm_words / elapsed_s` is close to BCK rate.
- `gtm_samples / elapsed_s` and `fs_est` are close to `16000`.
- `[GUIMAI_STORE] send ... fs=...` appears instead of `skip tx in TBCM diag`.

If capture runs but audio is distorted, next phase knobs are:

- `GUIMAI_GTM_DMA_SAMPLE_LEADING_EDGE`: rising vs falling BCK.
- `GUIMAI_GTM_DMA_LEFT_WS_LEVEL`: left/right channel selection.
- `GUIMAI_GTM_DMA_I2S_DELAY_BITS`: `0` for left-justified-like, `1` for standard I2S.

Observed result with initial `delay=1`:

- DMA path initialized correctly:
  - `[GUIMAI_GTM_DMA] init ... chunk=4096 delay=1 left_ws=0`
- `gtm_words` increased at BCK scale, proving TIM2_CH3 -> DMA -> `MODULE_P11.IN.U` capture is active.
- But `gtm_samples=0` and only one stored sample was sent.
- Interpretation: with the observed codec framing, skipping one BCK after WS transition leaves only 15 captured bits in the 16-BCK half-frame, so the decoder cannot assemble 16-bit samples.

Changed:

- Firmware tag: `2026-05-08-gtm-dma-port4096-d0`.
- `GUIMAI_GTM_DMA_I2S_DELAY_BITS: 1 -> 0`.

Observed result with `delay=0`:

- DMA path initialized correctly:
  - `[GUIMAI_GTM_DMA] init ... chunk=4096 delay=0 left_ws=0`
- Capture now produces PCM-rate samples:
  - after about 1s: `samples=15306`, `gtm_samples=15306`
  - after about 2s: `samples=30736`, `fs_est=15513`
  - after about 3s: `samples=46168`, `fs_est=15416`
  - store: `samples=51778 elapsed_ms=3382 fs=15309`
- This proves the GTM DMA GPIO snapshot approach is viable for producing near-16 kHz PCM under current hardware.

Remaining issues:

- `gtm_lost` remains high:
  - `gtm_lost=99/120 chunks`
  - `203/241 chunks`
  - `304/362 chunks`
- The effective sample rate is about `15.3 kHz`, around 4.3% lower than target `16 kHz`.
- `GUIMAI_STORE_TX` reports `sent=4294967295`, which is likely a send failure return value (`-1`) and is separate from capture quality.

Next priorities:

1. Fix or reduce DMA lost requests / timing loss.
2. Confirm network send path, because ASR cannot work if every frame send returns `-1`.
3. After timing and send are stable, tune BCK edge, WS level, and bit delay for audio quality.

## 2026-05-08 Server Result And Ring DMA Build

User ran `server_xfyun/main.py` with PCM dump enabled.

Observed server result from the `port4096-d0` build:

- Packet metadata:
  - `sample_rate=16000`
  - `samples=51778`
  - `elapsed_ms=3382`
- Server calculated:
  - observed duration `3.382s`
  - source rate `15309.9`
  - raw peak `32768`
  - raw RMS `22564.2`
  - raw ZCR `0.481`
  - after resample: peak `32768`, RMS `20297.0`, ZCR `0.244`
- Xfyun returned final empty text.

Interpretation:

- Because the silicon microphone was not connected, empty ASR text is expected.
- However, the very high peak/RMS/ZCR means the captured data is not quiet silence. It is likely floating input noise, phase/bit alignment error, or saturation-like wrong data.
- Capture path viability is still proven, because the board produced near-16 kHz frames and the PC server received/dumped them correctly.

Implemented next build to reduce `gtm_lost`:

- Firmware tag: `2026-05-08-gtm-dma-ring4096-d0`.
- Changed DMA destination from CPU re-armed ping-pong chunks to continuous circular ring:
  - `GUIMAI_GTM_DMA_CHUNK_WORDS = 4096`
  - `GUIMAI_GTM_DMA_RING_WORDS = 8192`
  - buffer aligned with `IFX_ALIGN(32768)` because 8192 words x 4 bytes = 32768 bytes and DMA circular destination uses low address bits for wrap.
  - `destinationCircularBufferEnabled = TRUE`
  - `destinationAddressCircularRange = IfxDma_ChannelIncrementCircular_32768`
  - `operationMode = IfxDma_ChannelOperationMode_continuous`
  - `transferCount = GUIMAI_GTM_DMA_RING_WORDS`
- CPU now polls `DADR` and processes whichever 4096-word half is no longer being written.
- Removed per-chunk destination/transfer-count re-arm from the hot path.

Expected result:

- `gtm_lost` should drop significantly because DMA no longer disables/re-arms every 4096 BCK requests.
- Init signature:
  - `fw=2026-05-08-gtm-dma-ring4096-d0`
  - `[GUIMAI_GTM_DMA] init ... chunk=4096 ring=8192 delay=0 left_ws=0 mode=ring32k ...`

If `gtm_samples` drops to zero in this build:

- The DADR half detection may not match the DMA circular update behavior; inspect `dma_tcnt`, `dma_chcsr`, and `gtm_words`.
- Fallback is ping-pong with larger chunks or DMA linked list/double destination buffering.

## 2026-05-08 Manual Page: TSSM External Capture Table 40

User provided the continuation of the TSSM external capture section and Table 40.

Confirmed:

- The external capture event source for TSSM is selected by `TIM[i]_CH[x]_CNTS[19:18]`:
  - `00b`: source selected by `EXT_CAP_SRC`.
  - `01b`: `tdu_word_evt`.
  - `10b`: `tdu_frame_evt`.
  - `11b`: reserved.
- If `EXT_CAP_EN=1`, an external capture event captures `GPRx`, resets `CNTS[15:8]`, and issues `NEWVAL_IRQ`.
- If `ISL=1`, the same capture also resets `CNT[23:0]` to `ECNT_RESET`.
- If `ISL=0`, capture resets only the bit counter `CNTS[15:8]`; `CNT` is not forcibly filled.
- If `shift clock = 1` and `tssm_ext_capture = 1` happen in the same cycle, Table 40 applies the shift operation first and then performs capture.

Important implication:

- The old diagnostic combination `CNTS[17:16]=01` and `CNTS[19:18]=01` made `tdu_word_evt` act as both shift clock and external capture event.
- Table 40 confirms that this causes a shift and capture in the same event cycle. That is not a clean I2S serial capture boundary, because the boundary event also shifts one bit into `CNT` before latching.
- Keep rejecting that configuration for production direction.
- A coherent TSSM setup needs:
  - shift clock from BCK-derived/sample event path;
  - external capture from word/frame boundary path;
  - `ISL=0` unless a proven reset/fill behavior is required.

Observed later results:

- `EXT_CAP_EN=1` is still worth keeping because the manual says external capture selection only works when `CTRL.EXT_CAP_EN=1`, and the captured AFD word layout changed after enabling it.
- `ISL=1` failed:
  - `CTRL=0x0000FF2C`
  - h1 became constant `0xFFFF`
  - `changed=0`, `neg1=29760`
  - This proves reset-on-capture is active, but the shift/capture window is still wrong.
- `TDU_SAME_CNT_CLK=1` failed:
  - Firmware tag: `2026-05-08-gtm-tssm-sameclk1`
  - `TDUV=0x0A201001`
  - live `tduc=0x00010101`
  - `tbcm_events=624` over `1703 ms`, about `366/s`
  - `h1_b0_15 words=10000 changed=1770 zero=4367 neg1=4967`
  - still about 93% zero/neg1, not PCM.
- Reverted `TDU_SAME_CNT_CLK` to `0`.
- Current baseline tag: `2026-05-08-gtm-tssm-extcap-en`.
- Current intended TSSM/TDU baseline:
  - `shift_clk=0`
  - `ext_cap=1`
  - `EXT_CAP_EN=1`
  - `ISL=0`
  - `sameclk=0`
  - `slicing=2`
  - `prev_tdu=1`

Next manual target:

- Find the register/field that controls `USE_TDU_CLK_SRC`.
- `CNTS[17:16]=00` depends on that source. Until that source is known, the TSSM shift clock is effectively ambiguous and the output keeps looking like slow fill/window data instead of BCK-shifted I2S bits.

## 2026-05-08 Manual Page: TSSM Shift Direction

Useful fact from the TSSM operation table:

- `DSL=0`:
  - `CNT[23:1]=CNT[22:0]`
  - `CNT[0]=value`
  - This shifts left and appends the new input bit into bit 0.
- `DSL=1`:
  - `CNT[22:0]=CNT[23:1]`
  - `CNT[23]=value`
  - This shifts right and appends the new input bit into bit 23.

Implication for I2S:

- ES8388 I2S data is MSB first.
- With `DSL=0`, after 16 shift clocks the lower 16 bits contain the received sample in normal MSB-to-LSB order.
- With the previous `DSL=1`, the sample lands toward the high side of `CNT` and bit order is not what the current `h1_b0_15` / `h1_b4_19` / `h1_b8_23` diagnostics expect.

New code change:

- Firmware tag: `2026-05-08-gtm-tssm-dsl0`.
- Change one variable only:
  - `DSL: 1 -> 0`
- Keep:
  - `EXT_CAP_EN=1`
  - `ISL=0`
  - `TDU_SAME_CNT_CLK=0`
  - `shift_clk=0`
  - `ext_cap=1`
  - `slicing=2`
  - `prev_tdu=1`
  - `bits=16`

Expected init signature:

- `fw=2026-05-08-gtm-tssm-dsl0`
- `shift=left`
- `ext_cap_en=1`
- `isl=0`
- `sameclk=0`
- `cnts=0x040010`

For the next paste:

- `[GUIMAI] fw=`
- `[GUIMAI_TBCM] bck prev ch`
- `[GUIMAI_TBCM] init`
- first two `[GUIMAI_CPU1] samples=0`
- `[GUIMAI_AFD_DMA] done=`
- all `[GUIMAI_TSSM_SLICE]`
- `[GUIMAI_DEC_PAIR_BITS] h1`

Observed result:

- The `resync2` build initialized as intended:
  - `fw=2026-05-08-gtm-tssm-resync2`
  - `tdu_resync=2`
  - `ECTRL=0x00020020`
  - `TDUV=0x02201001`
- TDU counters became much more visibly active:
  - live `tduc=0x00048200`
  - later `tduc=0x00954A01`
  - later `tduc=0x005C2E00`
- This supports the idea that `TDU_RESYNC=0000` was harmful for `SLICING=2`.
- But the result is still not PCM:
  - `tbcm_events=1404` over `3058 ms`, about `459/s`
  - `h1_b0_15 words=22480 changed=3994 zero=10584 neg1=10287`
  - `h1_b4_19 words=22480 changed=3970 zero=10594 neg1=10290`
  - `h1_b8_23 words=22480 changed=3966 zero=10602 neg1=10298`
  - `zero + neg1` is still about 93%.

New observation:

- Current `TOV/TOV1/TOV2` is `1/16/32`.
- External capture source is `tdu_word_evt` through `CNTS[19:18]=01`.
- With `TOV=1`, `tdu_word_evt` is likely far too short/early for a 16-bit I2S word boundary.

New code change:

- Firmware tag: `2026-05-08-gtm-tssm-tov16`.
- Change only the TDU compare value for word event:
  - `TOV: 1 -> 16`
- Keep:
  - `TOV1=16`
  - `TOV2=32`
  - `TDU_RESYNC=2`
  - `TDU_START=0`
  - `TDU_STOP=0`
  - `EXT_CAP_EN=1`
  - `shift_clk=0`
  - `ext_cap=1`
  - `slicing=2`
  - `prev_tdu=1`

Expected init signature:

- `fw=2026-05-08-gtm-tssm-tov16`
- `tdu_resync=2`
- `tov=16/16/32`
- `TDUV` should differ from previous `0x02201001`.

For the next paste:

- `[GUIMAI] fw=`
- `[GUIMAI_TBCM] init`
- first two `[GUIMAI_CPU1] samples=0`
- `[GUIMAI_AFD_DMA] done=`
- all `[GUIMAI_TSSM_SLICE]`
- `[GUIMAI_DEC_PAIR_BITS] h1`

Observed result:

- The `tov16` build initialized as intended:
  - `fw=2026-05-08-gtm-tssm-tov16`
  - `tdu_resync=2`
  - `tov=16/16/32`
  - `TDUV=0x02201010`
- TDU counters were active:
  - `tduc=0x001A4C0E`
  - later `tduc=0x0024A70D`
  - later `tduc=0x00BFBF10`
- It still did not produce PCM:
  - `tbcm_events=1450` over `3132 ms`, about `463/s`
  - `h1_b0_15 words=23216 changed=4317 zero=11039 neg1=10466`
  - `h1_b4_19 words=23216 changed=4357 zero=10979 neg1=10487`
  - `h1_b8_23 words=23216 changed=4310 zero=11006 neg1=10489`
  - `zero + neg1` is still about 92.6%.
- Pair decode is still not aligned:
  - `h1 ws_edges=2679`, `samples=473`, `bad_slots=2600`, `avg_abs=9`.

Conclusion:

- Changing `TOV` from `1` to `16` did not fix the event/window source.
- The remaining missing manual detail is not the `ECTRL` bit layout; that is now known.
- Need the TDU architecture details for how each 8-bit slice chooses:
  - `INC`
  - `LOAD_VAL`
  - `GT_EVT`
  - `EQ_EVT`
  - cascade wiring for `SLICING=0b10`
  - generation of `tdu_sample_evt`, `tdu_word_evt`, and `tdu_frame_evt`

## 2026-05-08 Local Manual TOC Check

User added `目录.md`, which is a table of contents for the TC3xx user manual.

Useful target sections from the TOC:

- `28.13.3 Timeout Detection Unit (TDU)`
- `28.13.3.3 Architecture of the TDU Sub-unit`
- `28.13.4.2.7 TIM Serial Shift Mode (TSSM)`
- `28.13.8 TIM Configuration Registers Description`

The TOC itself does not contain the register field definitions needed for the next code change.

Need manual pages/screenshots for:

- `TIM[i]_CH[x]_CTRL` fields `CLK_SEL`, `DSL`, `ISL`, `EXT_CAP_EN`, `ECNT_RESET`
- `TIM[i]_CH[x]_ECTRL` fields `ECLK_SEL`, `EXT_CAP_SRC`, `USE_PREV_TDU_IN`, `TDU_START`, `TDU_STOP`, `TDU_RESYNC`
- `TIM[i]_CH[x]_TDUV` fields `TCS`, `TCS_USE_SAMPLE_EVT`, `TDU_SAME_CNT_CLK`, `SLICING`, `TOV/TOV1/TOV2`
- Any diagram/table in `28.13.3.3` that names `USE_TDU_CLK_SRC` or shows the path from `CLK_SEL/ECLK_SEL/TCS/tdu_sample_evt` into the TSSM shift clock.

## 2026-05-08 USE_TDU_CLK_SRC Claim Check

User provided a generated-looking summary claiming:

- `TIMi_CHx_ECTRL[12] = USE_TDU_CLK_SRC`
- `0 = CMU_CLK`
- `1 = TBU_TS0`

This does not match the local TC38A SFR headers used by this project.

Local official header check:

- File: `libraries/infineon_libraries/Infra/Sfr/TC38A/_Reg/IfxGtm_regdef.h`
- `Ifx_GTM_TIM_CH_ECTRL_Bits` says:
  - `EXT_CAP_SRC[3:0]`
  - `USE_PREV_TDU_IN[5]`
  - `TDU_START[10:8]`
  - `TDU_STOP[14:12]`
  - `TDU_RESYNC[19:16]`
  - `ECLK_SEL[30]`
  - `USE_PREV_CH_IN[31]`
- File: `IfxGtm_bf.h` confirms:
  - `IFX_GTM_TIM_CH_ECTRL_TDU_STOP_OFF (12u)`
  - `IFX_GTM_TIM_CH_ECTRL_ECLK_SEL_OFF (30u)`

Conclusion:

- For this TC38A/TC387 project, do not write `ECTRL bit12` as `USE_TDU_CLK_SRC`.
- Writing bit 12 changes `TDU_STOP[0]`, which can accidentally alter TDU stop behavior.
- The provided claim is likely from a different GTM revision, a mismatched manual, or an AI-generated hallucination.
- Continue using local headers/manual screenshots as the authority for this project.

Useful screenshot fact:

- The manual text for `28.13.3 TDU` says each TIM channel has its own TDU and the timeout event can be set up on the filtered input signal.
- This supports the idea that TDU clock/input routing depends on TIM input/filter path plus `USE_PREV_TDU_IN`, not an `ECTRL[12]` clock-source bit in this TC38A header.

User later provided the official `28.13.8.18 Register TIM[i]_CH[x]_ECTRL` field layout.

This screenshot confirms the local header:

- `EXT_CAP_SRC[3:0]`
- bit 4 reserved
- `USE_PREV_TDU_IN[5]`
- `TODET_IRQ_SRC[7:6]`
- `TDU_START[10:8]`
- bit 11 reserved
- `TDU_STOP[14:12]`
- bit 15 reserved
- `TDU_RESYNC[19:16]`
- `USE_LUT[23:22]`
- `EFLT_CTR_RE[24]`
- `EFLT_CTR_FE[25]`
- bits 27:26 reserved
- `SWAP_CAPTURE[28]`
- `IMM_START[29]`
- `ECLK_SEL[30]`
- `USE_PREV_CH_IN[31]`

Therefore, `ECTRL[12]` must not be used as `USE_TDU_CLK_SRC`; it is `TDU_STOP[0]`.

## 2026-05-08 Manual Page: TDU_START / STOP / RESYNC

User provided the official field descriptions for `TDU_START`, `TDU_STOP`, and `TDU_RESYNC`, plus Table 46/47.

Important facts:

- `TDU_START=000b` starts once on `tdu_start_000_event`, which is generated by each write of `TOCTRL != 0` while `TDU_START=000b` and TDU is stopped.
- `TDU_STOP=000b` stops counting on `tdu_toctrl_0_event`, generated by each write of `TOCTRL=0` while TDU is started.
- For `SLICING != 0b11`, Table 46 says `TDU_RESYNC=0000b` can reset:
  - `TO_CNT2` on each active edge selected by `TOCTRL`;
  - `TO_CNT`/`TO_CNT1` on timeout/start events;
  - and for `SLICING=0b10`, also reset counters on active input edges selected by `TOCTRL`.

Current important implication:

- Our current TDU setup uses:
  - `SLICING=0b10`
  - `TOCTRL=risingEdge`
  - `TDU_RESYNC=0000b`
- That means the TDU counters can be reset on the same BCK active edges that should be counted.
- This is a strong explanation for why the observed TDU/TSSM window rate stayed in the hundreds per second instead of building clean 16-bit word windows.

New code change:

- Firmware tag: `2026-05-08-gtm-tssm-resync2`.
- Revert the untested `DSL=0` change back to `DSL=1`, so this test has one functional variable.
- Change one TDU variable:
  - `TDU_RESYNC: 0000b -> 0010b`
- Reason:
  - In Table 46, `0x1-` means reset `TO_CNT` on `tdu_word_evt`.
  - `0010b` matches that pattern and avoids the `0000b` behavior that resets counters on each `TOCTRL` active edge for `SLICING=0b10`.
- Keep:
  - `EXT_CAP_EN=1`
  - `ISL=0`
  - `TDU_STOP=000b`
  - `TDU_START=000b`
  - `TDU_SAME_CNT_CLK=0`
  - `TCS_USE_SAMPLE_EVT=0`
  - `shift_clk=0`
  - `ext_cap=1`
  - `slicing=2`
  - `prev_tdu=1`
  - `bits=16`

Expected init signature:

- `fw=2026-05-08-gtm-tssm-resync2`
- `shift=right`
- `tdu_start=0`
- `tdu_stop=0`
- `tdu_resync=2`
- `slicing=2`
- `cnts=0x040010`

For the next paste:

- `[GUIMAI] fw=`
- `[GUIMAI_TBCM] bck prev ch`
- `[GUIMAI_TBCM] init`
- first two `[GUIMAI_CPU1] samples=0`
- `[GUIMAI_AFD_DMA] done=`
- all `[GUIMAI_TSSM_SLICE]`
- `[GUIMAI_DEC_PAIR_BITS] h1`

## 2026-05-08 BCK Previous Channel Result

Observed result:

- Firmware tag: `2026-05-08-gtm-tssm-bck-prevch`.
- Init matched the intended build:
  - `bck prev ch TIM2_3 enabled CTRL=0x00003F05`
  - `cnts=0x040010`
  - `shift_clk=0`
  - `ext_cap=1`
  - `tcs_sample=0`
  - `slicing=2`
  - `TDUV=0x02201001`
- Runtime still did not produce PCM:
  - `tbcm_events=318` over `1144 ms`, about `278` windows per second.
  - `h1_b0_15 words=5104 changed=934 zero=1859 neg1=2866`
  - `h1_b4_19 words=5104 changed=924 zero=1853 neg1=2874`
  - `h1_b8_23 words=5104 changed=912 zero=1869 neg1=2873`
  - `h1` remains dominated by `0x0000` / `0xFFFF` style fill windows.

Conclusion:

- Enabling TIM2_CH3 as a running previous channel does not make TIM2_CH4 TSSM shift correctly on BCK.
- `USE_PREV_TDU_IN=1` plus a configured previous channel is not sufficient, or the TDU source/event is not the one we need.
- Do not keep changing `DSL`, `ECNT_RESET`, `CNTS_SEL`, `SLICING`, `shift_clk=1`, or `TCS_USE_SAMPLE_EVT` blindly. Those paths have already failed or produced logically bad sample/word coupling.

Local code/header facts:

- `IfxGtm_Tim_In.c` shows TIM input routing can select current/adjacent/aux through `IN_SRC` and `CICTRL`.
- That routing changes the TIM channel input signal. It is not safe to point TIM2_CH4 at adjacent/BCK because TIM2_CH4 must still sample SDOUT.
- Local register headers confirm these bits exist:
  - `ECTRL.EXT_CAP_SRC[3:0]`
  - `ECTRL.USE_PREV_TDU_IN`
  - `ECTRL.ECLK_SEL`
  - `ECTRL.USE_PREV_CH_IN`
  - `TDUV.TCS_USE_SAMPLE_EVT`
  - `TDUV.TDU_SAME_CNT_CLK`
- The local headers do not explain where `USE_TDU_CLK_SRC` is configured.

Need manual before the next code change. Search keywords:

- `USE_TDU_CLK_SRC`
- `TSSM USE_TDU_CLK_SRC`
- `CNTS[17:16] source selected by USE_TDU_CLK_SRC`
- `TIM ECTRL ECLK_SEL`
- `TIM ECTRL USE_PREV_CH_IN`
- `TDU sample event generation`
- `TDU word event generation`
- `TDU_SAME_CNT_CLK`
- `USE_PREV_TDU_IN`

Most important manual questions:

- Where is `USE_TDU_CLK_SRC` configured?
- Does `ECLK_SEL` affect the TDU/TSSM shift clock source, or only extend `CLK_SEL` for CMU clocks?
- Can TSSM shift on BCK while the channel input remains SDOUT?
- What exact `TDU_START`, `TDU_STOP`, `TDU_RESYNC`, `TCS_USE_SAMPLE_EVT`, and `TDU_SAME_CNT_CLK` settings produce a word event every 16 BCK edges?

## 2026-05-08 Manual Page: TSSM Shift Clock And Capture Reset

Useful facts from the manual page:

- Every shift clock increments `TIM[i]_CH[x]_CNTS[15:8]`.
- When `CNTS[15:8] >= CNTS[7:0]`, a capture event is raised and `NEWVAL` is asserted.
- On capture, `GPR0` and `GPR1` are updated according to `GPR0_SEL/GPR1_SEL/EGPR0_SEL/EGPR1_SEL`.
- If `ISL=1`, then on capture event `TIM[i]_CH[x]_CNT` is set to the value defined by `ECNT_RESET`.
- In TSSM, `GPR1` can operate as a shadow register for `CNTS`, but the update only happens when `GPR1` was written by CPU. Our current build does not use this CPU-written dynamic shadow update path.
- `CNTS[17:16]` selects shift clock:
  - `00b`: source selected by `USE_TDU_CLK_SRC`; can be CMU clock or local TDU `tdu_sample_evt`.
  - `01b`: `tdu_word_evt` is used as shift clock.
  - `10b`: `USE_TDU_CLK_SRC` gated with `tdu_word_evt=1`.
  - `11b`: `USE_TDU_CLK_SRC` gated with `tdu_word_evt=0`.
- TSSM output generation uses `CNTS[21:20]` and `CNTS[23]`; registered/latched output can use `SHIFT_OUT_INx`.
- For registered/latched output, `CNTS_SEL=0` uses `FOUTx`; `CNTS_SEL=1` uses `TIM_INx` as `SHIFT_OUT_INx`.
- External capture mode says if `EXT_CAP_EN=1`, external capture events capture GPRx, reset CNT depending on `ISL`, and issue `NEWVAL`.

New code change:

- Firmware tag: `2026-05-08-gtm-tssm-isl-reset`.
- Change one variable only:
  - `ISL: 0 -> 1`
- Reason:
  - The manual explicitly says `ISL=1` resets `CNT` on capture to `ECNT_RESET`.
  - Current failing logs show `h1/CNT` dominated by long `0x0000` / `0xFFFF` fill windows. Resetting the shift register on each capture is a plausible missing piece and is narrower than changing TDU routing again.
- Keep:
  - `shift_clk=0`
  - `ext_cap=1`
  - `slicing=2`
  - `tcs_sample=0`
  - `prev_tdu=1`
  - `bck prev ch enabled`
  - `bits=16`
  - `shift=right`
  - `init_pol=1`

Expected init signature:

- `fw=2026-05-08-gtm-tssm-isl-reset`
- `[GUIMAI_TBCM] bck prev ch TIM2_3 enabled ...`
- `cnts=0x040010`
- `shift_clk=0`
- `ext_cap=1`
- `isl=1`
- `TDUV=0x02201001`

For the next paste:

- `[GUIMAI] fw=`
- `[GUIMAI_TBCM] bck prev ch`
- `[GUIMAI_TBCM] init`
- first two `[GUIMAI_CPU1] samples=0`
- `[GUIMAI_AFD_DMA] done=`
- all `[GUIMAI_TSSM_SLICE]`
- `[GUIMAI_DEC_PAIR_BITS] h1`

Observed result:

- The `sameclk1` build initialized as intended:
  - `sameclk=1`
  - `TDUV=0x0A201001`
  - live `tduc=0x00010101`
- It still did not produce PCM:
  - `tbcm_events=624` over `1703 ms`, about `366` events/s.
  - `h1_b0_15 words=10000 changed=1770 zero=4367 neg1=4967`
  - `h1_b4_19 words=10000 changed=1748 zero=4391 neg1=4969`
  - `h1_b8_23 words=10000 changed=1730 zero=4402 neg1=4970`
  - `zero + neg1` is still about 93%.
- Pair-bit diagnostics still show only about `1100` toggles over `10000` h1 words, far below a valid 16-bit audio stream.

Conclusion:

- `TDU_SAME_CNT_CLK=1` changes the live TDU counters but does not fix the window rate or h1 contents.
- Revert `TDU_SAME_CNT_CLK` to `0` for the next baseline.
- Keep `EXT_CAP_EN=1`, because that was a real missing external-capture gate.
- The next manual search must find the actual shift-clock source selector for `CNTS[17:16]=00`, not more TSSM operation table rows.

Observed result:

- The `extcap-en` build initialized as intended:
  - `ext_cap_en=1`
  - `isl=0`
  - `cnts=0x040010`
  - `shift_clk=0`
  - `ext_cap=1`
  - `TDUV=0x02201001`
  - `CTRL=0x0008BF2C`, live `CTRL=0x4008BF2D`
- Enabling `EXT_CAP_EN` changed the captured word layout:
  - AFD history now includes `06040110/07040110` in h0 instead of only the previous `06041010/07041010` style.
  - This confirms `CTRL.EXT_CAP_EN` is a real part of the TSSM external capture path.
- It still did not produce PCM:
  - `tbcm_events=613` over `1667 ms`, about `368` events/s.
  - `h1_b0_15 words=9824 changed=1795 zero=4997 neg1=4154`
  - `h1_b4_19 words=9824 changed=1821 zero=4977 neg1=4148`
  - `h1_b8_23 words=9824 changed=1755 zero=5018 neg1=4174`
  - `zero + neg1` remains about 93%, so h1 is still mostly fill-like.
- Pair-bit diagnostics show h1 is no longer fully stuck, but still not a stable 16-bit audio word:
  - h1 bit toggles are only about `1110..1138` over `9824` words.
  - decoded pair candidates still have many bad slots and tiny `avg_abs`.

Conclusion:

- `EXT_CAP_EN=1` was a valid correction, but it is not sufficient.
- TSSM capture can now react through the external capture path, but the shift clock/window source is still wrong.
- Do not revert `EXT_CAP_EN`; keep it enabled for future TSSM tests.
- The remaining blocker is still `USE_TDU_CLK_SRC` / TDU sample-clock routing:
  - `shift_clk=0` means TSSM shift clock comes from the source selected by `USE_TDU_CLK_SRC`.
  - Current output rate is hundreds of events/s, not a 16 kHz word stream.
  - The shift register is not accumulating one valid 16-bit I2S sample before capture.

Next manual target:

- Find where `USE_TDU_CLK_SRC` is configured.
- Search also:
  - `USE_TDU_CLK_SRC`
  - `TDU clock source`
  - `tdu_sample_evt`
  - `TDU sample event`
  - `ECLK_SEL`
  - `CLK_SEL`
  - `TDU_SAME_CNT_CLK`
  - `TCS_USE_SAMPLE_EVT`

## 2026-05-08 Manual Page: TDU SLICING And Counter Clocks

Useful facts from Table 29 and Table 30:

- The local TDU can generate:
  - `tdu_sample_evt`
  - `tdu_word_evt`
  - `tdu_frame_evt`
  - `tdu_timeout_evt`
- For `3x8 bit` counter mode:
  - `TO_CNT2` counts on selected `TCS`.
  - `TO_CNT2 >= TOV2` generates `tdu_sample_evt`.
  - `TO_CNT >= TOV` generates `tdu_word_evt`.
  - `TO_CNT1 >= TOV1` generates `tdu_frame_evt`.
- Clock selection in the `3x8 bit` mode depends on `TCS_USE_SAMPLE_EVT` and `TDU_SAME_CNT_CLK`:
  - `TO_CNT2`: selected `TCS`.
  - `TO_CNT`: selected by `TCS_USE_SAMPLE_EVT`.
  - `TO_CNT1`: selected by `TDU_SAME_CNT_CLK`.
- The table explicitly shows:
  - with `TDU_SAME_CNT_CLK=0`, `TO_CNT1` can be clocked by `tdu_word_evt`.
  - with `TDU_SAME_CNT_CLK=1`, `TO_CNT1` can use the same selected TCS path.

Implication:

- Our current `SLICING=2` / `3x8 bit` TDU setup had `TDU_SAME_CNT_CLK=0`, which may leave part of the word/frame cascade dependent on `tdu_word_evt` instead of the selected clock path.
- Since the measured event rate is only hundreds per second, the TDU event cascade is still not producing a clean BCK-derived 16-bit word event.

New code change:

- Firmware tag: `2026-05-08-gtm-tssm-sameclk1`.
- Change one variable only:
  - `TDUV.TDU_SAME_CNT_CLK: 0 -> 1`
- Keep:
  - `EXT_CAP_EN=1`
  - `ISL=0`
  - `shift_clk=0`
  - `ext_cap=1` / `CNTS[19:18]=01`, so external capture source is `tdu_word_evt`
  - `slicing=2`
  - `tcs_sample=0`
  - `prev_tdu=1`
  - `bck prev ch enabled`
  - `bits=16`
  - `shift=right`
  - `init_pol=1`

Expected init signature:

- `fw=2026-05-08-gtm-tssm-sameclk1`
- `ext_cap_en=1`
- `isl=0`
- `sameclk=1`
- `cnts=0x040010`
- `shift_clk=0`
- `ext_cap=1`
- `TDUV` should have bit 27 set compared with the previous `0x02201001`.

For the next paste:

- `[GUIMAI] fw=`
- `[GUIMAI_TBCM] bck prev ch`
- `[GUIMAI_TBCM] init`
- first two `[GUIMAI_CPU1] samples=0`
- `[GUIMAI_AFD_DMA] done=`
- all `[GUIMAI_TSSM_SLICE]`
- `[GUIMAI_DEC_PAIR_BITS] h1`

Observed result:

- The `isl-reset` build initialized as intended:
  - `fw=2026-05-08-gtm-tssm-isl-reset`
  - `cnts=0x040010`
  - `shift_clk=0`
  - `ext_cap=1`
  - `isl=1`
  - `CTRL=0x0000FF2C`
  - live `CTRL=0x4000FF2D`
- Result got worse but is diagnostically useful:
  - AFD history alternates almost perfectly: `07000000 07FFFFFF ...`
  - `h1_b0_15 words=29760 changed=0 zero=0 neg1=29760 raw=0xFFFF`
  - all h1 slice candidates are constant `0xFFFF`.
  - `h1` pair bits show all data bits fixed high: `ones=29760/... tog=0/...`
- `tbcm_events=1859` over `3988 ms`, still only about `466` events/s.

Conclusion:

- `ISL=1` confirms that TSSM capture/reset is active, but it does not produce PCM.
- With `ISL=1` and `ECNT_RESET=1`, each capture resets `CNT` to all ones. Since h1 is constant all ones, a valid shift window is not being accumulated before capture.
- Revert `ISL` to `0`. Do not test `ISL=1` further unless the shift-clock source is fixed first.
- The remaining root problem is still shift-clock/event routing:
  - `CNTS[17:16]=00` depends on `USE_TDU_CLK_SRC`.
  - We still do not know where `USE_TDU_CLK_SRC` is configured.
  - Current TDU/capture path produces only hundreds of events per second, not BCK/16 or 16 kHz sample windows.

## 2026-05-08 Manual Recheck: EXT_CAP_EN Gate

Manual recheck from the TSSM external capture section:

- The section begins with: if external capture is enabled, `EXT_CAP_EN=1`, the external capture events capture GPRx, reset CNT depending on `ISL`, and issue `NEWVAL_IRQ`.
- `CNTS[19:18]` selects the external capture event source, but the mode still depends on `CTRL.EXT_CAP_EN`.

Important code correction:

- Previous TSSM builds selected `CNTS[19:18]=01` (`tdu_word_evt`) but left `CTRL.EXT_CAP_EN=0`.
- That means the TSSM external capture path may not have actually been enabled even though the source field was set.
- This is a more direct manual-supported missing bit than more `DSL/ISL/shift_clk` experiments.

New code change:

- Firmware tag: `2026-05-08-gtm-tssm-extcap-en`.
- Change one variable only:
  - `CTRL.EXT_CAP_EN: 0 -> 1`
- Keep:
  - `ISL=0`
  - `shift_clk=0`
  - `ext_cap=1` / `CNTS[19:18]=01`, so external capture source is `tdu_word_evt`
  - `slicing=2`
  - `tcs_sample=0`
  - `prev_tdu=1`
  - `bck prev ch enabled`
  - `bits=16`
  - `shift=right`
  - `init_pol=1`

Expected init signature:

- `fw=2026-05-08-gtm-tssm-extcap-en`
- `ext_cap_en=1`
- `isl=0`
- `cnts=0x040010`
- `shift_clk=0`
- `ext_cap=1`
- `TDUV=0x02201001`

For the next paste:

- `[GUIMAI] fw=`
- `[GUIMAI_TBCM] bck prev ch`
- `[GUIMAI_TBCM] init`
- first two `[GUIMAI_CPU1] samples=0`
- `[GUIMAI_AFD_DMA] done=`
- all `[GUIMAI_TSSM_SLICE]`
- `[GUIMAI_DEC_PAIR_BITS] h1`

## 2026-05-08 Manual Page: TSSM Operation Table 40

Useful facts from Table 40:

- TSSM external capture source is selected by `CNTS[19:18]`:
  - `00b`: `EXT_CAP_SRC`
  - `01b`: `tdu_word_evt`
  - `10b`: `tdu_frame_evt`
  - `11b`: reserved
- Operation depends on:
  - input signal `F_OUTx`
  - shift clock
  - `tssm_ext_capture`
  - `ISL`
  - `DSL`
- If `shift clock=0`, `tssm_ext_capture=1`, `ISL=1`:
  - capture happens;
  - `NEWVAL_IRQ` is issued;
  - `CNTS[15:8]=0`;
  - `CNT[23:0]=ECNT_RESET`.
- If `shift clock=0`, `tssm_ext_capture=1`, `ISL=0`:
  - capture happens;
  - `NEWVAL_IRQ` is issued;
  - `CNTS[15:8]=0`;
  - `CNT` is not reset by this table row.
- If `shift clock=1`, `tssm_ext_capture=1`, `ISL=1`, `DSL=1`:
  - `CNT[22:0]=CNT[23:1]`;
  - `CNT[23]=value`;
  - capture happens;
  - `NEWVAL_IRQ` is issued;
  - `CNTS[15:8]=0`;
  - `CNT[23:0]=ECNT_RESET`.

Implication for the current `isl-reset` build:

- `ISL=1` is still a valid test because it prevents the shift register from carrying long stale fill runs after capture.
- But the result depends on whether the external capture event aligns with a shift-clock active cycle.
- If the next log still shows very low event rate or mostly `0x0000/0xFFFF`, the remaining suspect is not `ISL`; it is the event routing/timing:
  - `USE_TDU_CLK_SRC` source for shift clock;
  - whether `tdu_word_evt` is really generated every 16 BCK edges;
  - whether capture and shift happen in the intended same cycle.

## 2026-05-08 Manual Page Check: TSSM External Capture Source

User-provided manual page says:

- In TSSM mode, external capture event source is selected by `TIM[i]_CH[x]_CNTS[19:18]`.
- Meaning:
  - `00b`: source selected by `EXT_CAP_SRC`.
  - `01b`: `tdu_word_evt` is used as source.
  - `10b`: `tdu_frame_evt` is used as source.
  - `11b`: reserved.
- Table 40 also shows that TSSM capture/new-value generation depends on:
  - `shift clock`
  - `tssm_ext_capture`
  - `ISL`
  - `DSL`
  - input signal value

Correction from this page:

- Our previous builds had `CNTS[19:18]=00`, so TSSM external capture source stayed on `EXT_CAP_SRC`, not local `tdu_word_evt`.
- This explains the persistent slow/fill-like output even though local TDU counters were active.
- The next more justified single-variable test is therefore `CNTS[19:18]=01`, not another direction/polarity sweep.

Next build:

- Firmware tag: `2026-05-08-gtm-tssm-extcap-word`.
- Change only TSSM external capture source:
  - previous: `ext_cap=0` / `CNTS[19:18]=00`
  - next: `ext_cap=1` / `CNTS[19:18]=01`, use local `tdu_word_evt`
- Revert the not-yet-tested `TCS_USE_SAMPLE_EVT` change back to `0` for a clean single-variable test.
- Keep:
  - `shift_clk=1`
  - `slicing=2`
  - `cnts_sel=1`
  - `tim_in=1`
  - `prev_tdu=1`
  - `bits=16`
- Expected init signature:
  - `fw=2026-05-08-gtm-tssm-extcap-word`
  - `cnts=0x050010`
  - `shift_clk=1`
  - `ext_cap=1`
  - `tcs_sample=0`

For the next paste:

- `[GUIMAI] fw=`
- `[GUIMAI_TBCM] init`
- first two `[GUIMAI_CPU1] samples=0`
- `[GUIMAI_AFD_DMA] done=`
- all `[GUIMAI_TSSM_SLICE]`
- `[GUIMAI_DEC_PAIR_BITS] h1`

Observed result:

- The `sample-word` build initialized as intended:
  - `CNTS=0x00040010`
  - `cnts=0x040010`
  - `shift_clk=0`
  - `ext_cap=1`
  - `tcs_sample=1`
  - `TDUV=0x06201001`
- Runtime showed `TDUC` mostly in the high byte only:
  - `tduc=0x000E0000`
  - `tduc=0x000C0000`
  - `tduc=0x001A0000`
- This means `TO_CNT2` is moving, but `TO_CNT`/`TO_CNT1` are not running as needed for sample/word event generation.
- Result is still not PCM:
  - `tbcm_events=1439` over `3112 ms`, about `462` windows per second.
  - `h1_b0_15 words=23040 changed=3813 zero=11076 neg1=10448`
  - `h1_b4_19 words=23040 changed=3801 zero=11078 neg1=10449`
  - `h1_b8_23 words=23040 changed=3794 zero=11080 neg1=10451`

Important correction:

- `TCS_USE_SAMPLE_EVT=1` appears to be the wrong direction here: it can make `TO_CNT` wait on `tdu_sample_evt`, while `tdu_sample_evt` itself depends on the TDU compare/counter path. The observed `TDUC` supports this self-dependency suspicion.
- Also, manual text says `USE_PREV_TDU_IN=1` uses the previous channel input after filter. In the TBCM/TSSM path we configured the TIM2_CH3 pin but did not enable/configure TIM2_CH3 itself as a running channel. The previous-channel filter path may therefore not be valid.

Next build:

- Firmware tag: `2026-05-08-gtm-tssm-bck-prevch`.
- Enable TIM2_CH3 as a simple BCK input-event channel before configuring TIM2_CH4.
- Keep TIM2_CH4 as the TSSM capture channel.
- Revert `tcs_sample` to `0` to avoid the suspected self-dependency.
- Keep:
  - `shift_clk=0`
  - `ext_cap=1`
  - `slicing=2`
  - `cnts_sel=1`
  - `tim_in=1`
  - `prev_tdu=1`
  - `bits=16`
- Expected init signature:
  - `fw=2026-05-08-gtm-tssm-bck-prevch`
  - `[GUIMAI_TBCM] bck prev ch TIM2_3 enabled ...`
  - `cnts=0x040010`
  - `shift_clk=0`
  - `ext_cap=1`
  - `tcs_sample=0`
  - `TDUV=0x02201001`

For the next paste:

- `[GUIMAI] fw=`
- `[GUIMAI_TBCM] bck prev ch`
- `[GUIMAI_TBCM] init`
- first two `[GUIMAI_CPU1] samples=0`
- `[GUIMAI_AFD_DMA] done=`
- all `[GUIMAI_TSSM_SLICE]`
- `[GUIMAI_DEC_PAIR_BITS] h1`

Observed result:

- The `extcap-word` build initialized as intended:
  - `CNTS=0x00050010`
  - `cnts=0x050010`
  - `shift_clk=1`
  - `ext_cap=1`
  - `tcs_sample=0`
- Runtime result:
  - `tbcm_events=683` over `1801 ms`, about `379` windows per second.
  - `h1_b0_15 words=10944 changed=1859 zero=4959 neg1=4464`
  - `h1_b4_19 words=10944 changed=1848 zero=4945 neg1=4454`
  - `h1_b8_23 words=10944 changed=1853 zero=4953 neg1=4411`
- This is still not PCM. It got slower, not better.

Important correction:

- The manual page shows two independent concepts:
  - `CNTS[17:16]`: shift clock selection.
  - `CNTS[19:18]`: external capture source selection.
- `shift_clk=1` means `tdu_word_evt` is the shift clock.
- `ext_cap=1` means `tdu_word_evt` is also the capture source.
- That combination makes the word event do both jobs. For serial audio this is wrong: BCK/sample events should shift bits, while a word/slot event should define the capture boundary.

Next build:

- Firmware tag: `2026-05-08-gtm-tssm-sample-word`.
- Change to the more coherent sample/word split:
  - `shift_clk=0`: use the `USE_TDU_CLK_SRC`/sample-event path for shift clock.
  - `tcs_sample=1`: make the TDU counter path use `tdu_sample_evt`.
  - `ext_cap=1`: keep `tdu_word_evt` as external capture boundary.
- Keep:
  - `slicing=2`
  - `cnts_sel=1`
  - `tim_in=1`
  - `prev_tdu=1`
  - `bits=16`
- Expected init signature:
  - `fw=2026-05-08-gtm-tssm-sample-word`
  - `cnts=0x040010`
  - `shift_clk=0`
  - `ext_cap=1`
  - `tcs_sample=1`

For the next paste:

- `[GUIMAI] fw=`
- `[GUIMAI_TBCM] init`
- first two `[GUIMAI_CPU1] samples=0`
- `[GUIMAI_AFD_DMA] done=`
- all `[GUIMAI_TSSM_SLICE]`
- `[GUIMAI_DEC_PAIR_BITS] h1`

Observed result:

- The `tdu-slice2` build initialized as intended:
  - `slicing=2`
  - `TDUV=0x02201001`
  - live `tduv=0x02201001`
- TDU is still active:
  - live `tduc` changed between `0x00010001` and `0x00000000`
  - IRQ became `0x19/0x1D`, so the slicing-related timeout status is visible.
- But the result is still not PCM:
  - `tbcm_events=1989` over `4079 ms`, about `488` windows per second.
  - `h1_b0_15 words=31840 changed=5203 zero=14626 neg1=15174`
  - `h1_b4_19 words=31840 changed=5200 zero=14611 neg1=15177`
  - `h1_b8_23 words=31840 changed=5228 zero=14591 neg1=15184`
- `zero + neg1` is still about `94%`, and the window rate is still only hundreds per second. `SLICING=2` alone is not the missing setting.

Next build:

- Firmware tag: `2026-05-08-gtm-tssm-shiftclk1`.
- Change only `CNTS[17:16]` / `shift_clk`:
  - previous: `shift_clk=0`
  - next: `shift_clk=1`
- Keep:
  - `slicing=2`
  - `cnts_sel=1`
  - `tim_in=1`
  - `shift=right`
  - `init_pol=1`
  - `prev_tdu=1`
  - `bits=16`
- Purpose: check whether TSSM needs the alternate shift-clock selection to consume the TDU/window event instead of the current slow/fill-like path.

For the next paste, same short set:

- `[GUIMAI] fw=`
- `[GUIMAI_TBCM] init`
- first two `[GUIMAI_CPU1] samples=0`
- `[GUIMAI_AFD_DMA] done=`
- all `[GUIMAI_TSSM_SLICE]`
- `[GUIMAI_DEC_PAIR_BITS] h1`
## 2026-05-08 23:06 GTM-DMA ring stable, add ES8388 source scan

Latest observed log:

- Firmware path: `GUIMAI_GTM_DMA_CAPTURE=1`, `chunk=2048`, `ring=4096`, `delay=0`, `left_ws=0`.
- Runtime:
  - `gtm_lost=0`.
  - `samples=80000 elapsed_ms=5180 fs=15444`.
  - Server receives all frames and writes WAV dumps.
- Server analysis:
  - `src_rate=15444.0`.
  - `peak=32768`.
  - `rms` about `22894` raw / `20620` after resample.
  - `zcr` about `0.486` raw / `0.250` after resample.
  - Xfyun final text is empty.

Conclusion:

- The GTM TIM + DMA GPIO snapshot path is now viable as a transport:
  - BCK-triggered DMA is continuous.
  - Ring polling no longer loses requests.
  - Effective sample rate is stable enough for server-side resampling.
- The current audio is not sane speech:
  - Full-scale peaks and very high RMS/ZCR look like wrong ADC input, floating input, disconnected mic, or analog front-end/bias issue.
  - Xfyun empty result cannot prove the capture algorithm is bad while the silicon mic connection is uncertain.

Code change:

- Keep the stable DMA path unchanged.
- Add `GUIMAI_LINEIN_SRC_SCAN_ON_START=1`.
- Firmware tag changed to `2026-05-08-gtm-dma-ring2048-srcscan`.
- On capture start, after GTM-DMA starts, scan ES8388 ADC input sources:
  - `SRC_A(0x0A=0x00)`
  - `SRC_B(0x0A=0x50)`
  - `SRC_C(0x0A=0xA0)`
  - `SRC_D(0x0A=0xF0)`
- Each source prints:
  - `n`
  - `avg_abs`
  - `zc`
  - `clip`
  - `nonzero`
  - `min/max/last`
  - `sd_tog`
  - `reg0A`
- After scan, the code restores the configured source and resets GTM-DMA runtime counters so formal recording logs are not polluted by the scan.

Next paste needed:

- `[GUIMAI] fw=`
- all `[GUIMAI_SRC_SCAN]` lines
- first two `[GUIMAI_CPU1] samples=...`
- `[GUIMAI_STORE] send ...`
- server `buffered audio ...`

How to read the scan:

- A disconnected/floating source usually has high `avg_abs`, high `zc`, many `clip`, and full-scale `min/max`.
- A quiet valid mic source should have lower `avg_abs`, low or moderate `zc`, few clips, and should change when speaking/tapping the mic.
- If all four sources look clipped/noisy, check mic wiring/bias/power before tuning I2S phase.

## 2026-05-15 16:53 Source scan result and next change

Observed source scan:

- `SRC_A`: `avg_abs=9455`, `zc=1158`, `clip=1182`, `min=-32768`, `max=32767`.
- `SRC_B`: `avg_abs=16313`, `zc=1980`, `clip=1954`, `min=-32768`, `max=32767`.
- `SRC_C`: `avg_abs=16903`, `zc=2007`, `clip=2113`, `min=-32763`, `max=18`.
- `SRC_D`: `avg_abs=16568`, `zc=2004`, `clip=2071`, `min=-32768`, `max=32767`.
- Runtime still stable:
  - `gtm_lost=0`.
  - `samples=17600 elapsed_ms=1147 fs=15344`.
- Server dump still bad:
  - `peak=32768`.
  - raw `rms=22662.2`, `zcr=0.502`.
  - Xfyun empty text.

Interpretation:

- `SRC_B/C/D` are clearly unusable in the current decode/source setup.
- `SRC_A` is less bad but still has too many clips for normal silence or speech.
- Because every source has clipping and the WAV contains many near-`326xx/-327xx` values mixed with tiny values, the remaining risk is not only analog input selection. I2S half-frame (`WS`) and 1-bit delay may also be wrong.

Code change:

- Firmware tag changed to `2026-05-15-gtm-dma-src-phase-scan`.
- The scan now tests 16 combinations:
  - source: `SRC_A/B/C/D`
  - decoded WS half-frame: `ws=0/1`
  - I2S delay: `delay=0/1`
- Each scan line now prints `score`.
- The firmware automatically selects the lowest-score combination and prints:
  - `[GUIMAI_SRC_SCAN] selected src=... ws=... delay=... score=...`
- Formal capture logs now include the selected `ws` and `delay`.

Next paste needed:

- `[GUIMAI] fw=`
- all `[GUIMAI_SRC_SCAN] src=... ws=... delay=... score=...` lines
- `[GUIMAI_SRC_SCAN] selected ...`
- `[GUIMAI] linein capture start...`
- first two `[GUIMAI_CPU1] samples=...`
- server `buffered audio ...`

If the best score is still high and server RMS/ZCR remain similar, stop tuning GTM-DMA and check ES8388 analog input wiring/bias/mic power.

## 2026-05-15 17:01 Phase scan result and UI issue

Observed phase scan:

- The scanner incorrectly selected `SRC_A ws=0 delay=0` because it was all zero:
  - `n=1536 avg_abs=0 clip=0 nonzero=0 score=0`.
  - This is not a valid audio path.
- All `delay=1` combinations produced `n=0`; delay=1 is not usable with the current decoder/window logic.
- The only clearly better nonzero combination was:
  - `SRC_D ws=1 delay=0 n=1599 avg_abs=2808 zc=725 clip=137 nonzero=1054 score=4495`.
- Formal recording after the scan lasted only `44 ms` / `320 samples` because the long scan ran before the code entered the recording state. This also explains why the screen did not show recording: `guimai_board_is_recording()` depends on `s_streaming` / `s_record_core_running`, and those are set after `guimai_linein_start()` returns.

Code change:

- Disable normal startup scan:
  - `GUIMAI_LINEIN_SRC_SCAN_ON_START = 0`.
- Fix scanner scoring for future diagnostics:
  - `n=0` and `nonzero=0` combinations now get invalid score `0xFFFFFFFF` and cannot be selected.
- Use the best observed nonzero combination as the fixed runtime path:
  - source: `SRC_D(0x0A=0xF0)`
  - `ws=1`
  - `delay=0`
- Firmware tag changed to `2026-05-15-gtm-dma-srcD-ws1-d0`.

Next paste needed:

- `[GUIMAI] fw=`
- `[GUIMAI] linein record ready ...`
- `[GUIMAI_GTM_DMA] init ...`
- `[GUIMAI] linein capture start. (src=SRC_D... ws=1 delay=0)`
- two or more `[GUIMAI_CPU1] samples=...` lines while holding the key
- `[GUIMAI_STORE] send ...`
- server `buffered audio ...`

Expected behavior:

- Screen should show recording while the key is held because scan no longer blocks the state transition.
- Audio may still not be recognizable; `SRC_D/ws=1/delay=0` is only the least-bad current combination. If RMS/ZCR/clip are still high after this, the next target is ES8388 analog input wiring/bias/mic power, not DMA timing.

## 2026-05-15 17:09 Fixed SRC_D result, try trailing BCK edge

Observed fixed-path result:

- Runtime path was correct:
  - `src=SRC_D(0x0A=0xF0)`
  - `ws=1`
  - `delay=0`
  - `edge=leading`
- DMA path remains stable:
  - `gtm_lost=0`
  - `samples=70775 elapsed_ms=4643 fs=15243`
- Server output is still invalid audio:
  - raw `peak=32768`
  - raw `rms=23011.7`
  - raw `zcr=0.480`
  - 16k `rms=20486.8`
  - 16k `zcr=0.266`

WAV value distribution:

- Very frequent `-1`, `0`, `32767`, `32766`, `-32768`.
- This shape is more consistent with sampling SDOUT at/near the bit transition, or a serial bit boundary/word interpretation error, than with a normal microphone waveform.

Code change:

- Keep source/channel/decode unchanged:
  - `SRC_D`
  - `ws=1`
  - `delay=0`
- Change only BCK trigger edge:
  - `GUIMAI_GTM_DMA_SAMPLE_LEADING_EDGE: 1 -> 0`
- Firmware tag changed to `2026-05-15-gtm-dma-srcD-ws1-d0-fall`.

Next paste needed:

- `[GUIMAI] fw=`
- `[GUIMAI_GTM_DMA] init ... edge=trailing ...`
- `[GUIMAI] linein capture start...`
- `[GUIMAI_CPU1] samples=... gtm_lost=...`
- server `buffered audio ...`

Interpretation:

- If RMS/ZCR/clip drop substantially, the issue is BCK sampling phase.
- If the result stays full-scale/noisy, add a GPIO snapshot decoder diagnostic for bit offsets/word slicing, then check analog input only after confirming the serial framing.

## 2026-05-15 17:13 Trailing edge result, add transform diagnostic

Observed trailing-edge result:

- Runtime path:
  - `edge=trailing`
  - `SRC_D`
  - `ws=1`
  - `delay=0`
- DMA still stable:
  - `gtm_lost=0`
  - `samples=68863 elapsed_ms=4457 fs=15450`
- Server improved only slightly:
  - raw `rms=21524.6`, `zcr=0.482`
  - 16k `rms=19356.4`, `zcr=0.269`
  - `peak=32768` still full-scale.

Offline transform check on the WAV:

- `byteswap` reduces clip and RMS somewhat, but still does not look like normal voice.
- Arithmetic right shift removes clipping but only scales the bad waveform; it is not a real fix.
- This means the problem is not solved by BCK edge alone, and not obviously a simple endian/sign inversion only.

Code change:

- Keep current runtime audio path unchanged.
- Add board-side transform diagnostic before stored audio is sent:
  - `[GUIMAI_XFORM] mode=orig`
  - `[GUIMAI_XFORM] mode=byteswap`
  - `[GUIMAI_XFORM] mode=invert`
  - `[GUIMAI_XFORM] mode=bitrev`
  - `[GUIMAI_XFORM] mode=shr1`
  - `[GUIMAI_XFORM] mode=shr2`
- Firmware tag changed to `2026-05-15-gtm-dma-transform-diag`.
- This does not change the audio sent to the server; it only prints statistics from `s_record_store`.

Next paste needed:

- `[GUIMAI] fw=`
- `[GUIMAI_GTM_DMA] init ...`
- all `[GUIMAI_XFORM]` lines
- `[GUIMAI_STORE] send ...`
- server `buffered audio ...`

Interpretation:

- If a transform has much lower `clip`, lower `avg_abs`, and reasonable `zc`, we can apply that transform to the sent PCM.
- If no transform is plausible, add a raw GPIO word/bit-offset decoder diagnostic on the DMA snapshots, because the 16-bit sample boundary is likely wrong.

## 2026-05-15 17:22 Transform diagnostic result, apply byteswap

Observed transform diagnostic:

- `orig`:
  - `avg_abs=13378`
  - `clip=31590`
  - `min=-32768 max=32767`
  - invalid full-scale audio.
- `byteswap`:
  - `avg_abs=228`
  - `clip=0`
  - `min=-1793 max=1536`
  - this is the only plausible PCM interpretation.
- `shr1` / `shr2` remove clipping only by scaling the bad data and still have high ZC.
- `bitrev` / `invert` remain bad.

Conclusion:

- The GTM-DMA GPIO decoder is producing the correct 16 bits but in the opposite byte order for the server/PCM path.
- Apply byteswap to the captured samples before storing and sending.

Code change:

- Add `GUIMAI_RECORD_PCM_TRANSFORM_MODE = 1U`.
- Apply `guimai_transform_sample(..., GUIMAI_RECORD_PCM_TRANSFORM_MODE)` in `guimai_linein_task()` before writing to:
  - `s_record_store`
  - `s_mic_voice`
- Firmware tag changed to `2026-05-15-gtm-dma-byteswap`.
- Keep transform diagnostic enabled temporarily. After byteswap is applied, `mode=orig` in `[GUIMAI_XFORM]` means the already-corrected stored PCM.

Next paste needed:

- `[GUIMAI] fw=2026-05-15-gtm-dma-byteswap`
- `[GUIMAI_XFORM] mode=orig ...`
- `[GUIMAI_STORE] send ...`
- server `buffered audio ...`

Expected server result:

- `peak` should no longer be near `32768`.
- `rms` should drop from about `20000` to the low hundreds or low thousands depending on actual mic/input level.
- If Xfyun is still empty after PCM becomes sane, then check whether the active analog source `SRC_D` is connected to the actual microphone and whether the microphone is powered/biasing correctly.

## 2026-05-15 17:26 Byteswap verified, raise analog gain

Observed byteswap result:

- Board-side corrected PCM:
  - `mode=orig avg_abs=287 clip=0 min=-1921 max=1536`
- Server-side corrected PCM:
  - raw `peak=1921`
  - raw `rms=411.9`
  - raw `zcr=0.088`
  - 16k `peak=1735`
  - 16k `rms=409.2`
  - 16k `zcr=0.076`
- Xfyun is still empty.

Conclusion:

- Digital capture, framing, DMA, and byteswap are now sane.
- Current remaining issue is likely analog/input level or actual microphone source, not GTM-DMA.
- RMS around `400` is low for speech recognition unless the user spoke very close/loudly into the active microphone.

Code change:

- Raise ES8388 PGA gain:
  - `ES8388_MIC_GAIN: 0 -> 4` (`12 dB`).
- Keep byteswap enabled:
  - `GUIMAI_RECORD_PCM_TRANSFORM_MODE = 1U`.
- Disable transform diagnostic spam:
  - `GUIMAI_RECORD_TRANSFORM_DIAG = 0`.
- Firmware tag changed to `2026-05-15-gtm-dma-byteswap-gain12`.

Next paste needed:

- `[GUIMAI] fw=2026-05-15-gtm-dma-byteswap-gain12`
- ES8388 regs line, especially `0x09`
- `[GUIMAI_STORE] send ...`
- server `buffered audio ...`
- Xfyun result if any

Expected result:

- `0x09` should become `0x44`.
- Server RMS should rise compared with `~400`, without `peak` hitting `32768`.
- If still empty with reasonable RMS, verify `SRC_D` wiring and speak/tap directly on the active microphone while recording.

## 2026-05-15 17:30 Gain 12dB result, trim leading silence

Observed gain result:

- ES8388 gain register is correct:
  - `0x09=0x44`.
- Server audio is now sane and stronger:
  - raw `peak=9601`
  - raw `rms=1783.6`
  - raw `zcr=0.053`
  - 16k `peak=9216`
  - 16k `rms=1775.9`
  - 16k `zcr=0.048`
- No clipping.
- WAV starts with a short run of zero samples before real energy appears.
- Xfyun result was still empty.

Conclusion:

- Digital path and gain are now usable.
- Remaining likely causes for empty ASR:
  - first frame starts with pure silence/zero warmup;
  - recording content was not clear speech into the active mic;
  - active source `SRC_D` may be line/mic hardware different from the intended silicon mic.

Code change:

- Add `GUIMAI_RECORD_TRIM_LEADING_ZERO = 1`.
- Before sending stored audio, trim leading zero samples and adjust `s_record_elapsed_ms` proportionally.
- Firmware tag changed to `2026-05-15-gtm-dma-byteswap-gain12-trim`.

Next paste needed:

- `[GUIMAI] fw=2026-05-15-gtm-dma-byteswap-gain12-trim`
- `[GUIMAI_STORE] trim ...` if printed
- `[GUIMAI_STORE] send ...`
- server `buffered audio ...`
- Xfyun result

Test instruction:

- Hold the key for 4-5 seconds.
- Speak clearly near the actual connected microphone during the whole capture.
- If Xfyun remains empty with `rms` around `1000-5000` and no clipping, confirm the physical input route for `SRC_D` and test with a known audio signal into that input.

## 2026-05-15 17:34 Zero-sample run, re-arm TIM notification on start

Observed zero-sample run:

- Server metadata:
  - `samples=0 elapsed_ms=4672`.
- Firmware runtime:
  - repeated `[GUIMAI_CPU1] samples=0 ... gtm_chunks=0 gtm_words=0 gtm_samples=0 gtm_lost=0 dma_tcnt=0 dma_chcsr=0x00000000`.
- This is not an audio-level issue. DMA/TIM did not start producing BCK-triggered requests.

Likely cause:

- `guimai_gtm_dma_stop()` disables `s_gtm_dma_tim_ch->IRQ.EN.U`.
- `guimai_gtm_dma_start_chunk()` re-enabled DMA and `TIM_EN`, but did not explicitly re-enable TIM NEWVAL notification after a stop/start cycle.

Code change:

- In `guimai_gtm_dma_start_chunk()`, re-arm TIM notification every start:
  - `IfxGtm_Tim_Ch_setNotificationMode(... pulse)`
  - `IfxGtm_Tim_Ch_setNotification(... newVal)`
- Add `[GUIMAI_GTM_DMA] start ...` log with:
  - DMA `tcnt`
  - DMA `chcsr`
  - TIM `CTRL`
  - TIM `IRQ.EN`
  - TIM `IRQ.NOTIFY`
- Keep byteswap/gain/trim settings.
- If trimming sees all-zero stored audio, it now skips trimming and prints `trim skipped all_zero=...` instead of sending `samples=0` metadata.
- Firmware tag changed to `2026-05-15-gtm-dma-byteswap-gain12-rearm`.

Next paste needed:

- `[GUIMAI] fw=2026-05-15-gtm-dma-byteswap-gain12-rearm`
- `[GUIMAI_GTM_DMA] start ...`
- first two `[GUIMAI_CPU1] samples=...`
- `[GUIMAI_STORE] trim ...` or `trim skipped ...`
- server `buffered audio ...`

## 2026-05-15 17:45 UI status and Xfyun empty-text diagnosis

Current judgment:

- GTM-DMA capture path remains the selected path. Do not return to TBCM/TSSM unless DMA regresses.
- The latest good logs show `gtm_lost=0`, byteswap applied, `0x09=0x44`, no clipping, and server-side PCM around `peak=7809 rms=1591.8 zcr=0.055`.
- Xfyun still returns empty text. At this point the likely remaining causes are input content/source, not BCK loss:
  - capture may be mostly a tonal signal/noise instead of speech;
  - the active ES8388 source `SRC_D(0x0A=0xF0)` may not be the intended microphone route;
  - actual silicon mic wiring/power is still unconfirmed.

Code changes:

- Firmware tag changed to `2026-05-15-gtm-dma-byteswap-gain12-ui-xfyun-diag`.
- Firmware now directly updates the display status line from the recording state machine:
  - `REC ON` when capture starts;
  - `REC OFF` when capture stops;
  - `SEND` while buffered PCM is sent;
  - `SENT` after all PCM frames are sent;
  - `NO WIFI` if cached audio cannot be sent.
- Server now prints deeper audio diagnosis after resampling:
  - `audio diag avg_abs=... mean=... min=... max=... clip=... zero=... abs_p50/p90/p99=... win_rms=... dom=...Hz ratio=...`.
- Server now logs the last Xfyun JSON when final text is empty.
- Added server option `--debug-xfyun` for raw Xfyun response snippets.
- Fixed command keyword table from mojibake to valid Chinese keywords, so recognition text can map to commands once ASR returns text.

Next paste needed:

- `[GUIMAI] fw=2026-05-15-gtm-dma-byteswap-gain12-ui-xfyun-diag`
- `[GUIMAI_GTM_DMA] start ... irq_en=0x00000001 ...`
- first two `[GUIMAI_CPU1] samples=... gtm_lost=...`
- `[GUIMAI_STORE] trim ...` and `[GUIMAI_STORE] send ...`
- server `buffered audio ...`
- server `audio diag ...`
- server `xfyun last json: ...` if final is still empty

Test instruction:

- Start the server with `--debug-xfyun` for the next run.
- Hold key 3 for 4-5 seconds and speak a simple command continuously near the actual connected microphone, for example `你好` or `打开双闪`.
- If the new `audio diag` warns `audio looks tonal/non-speech`, prioritize checking ES8388 source routing and microphone wiring/power over more DMA changes.

## 2026-05-15 17:58 End-to-end recognition success

Observed result:

- Firmware capture remains stable:
  - `SRC_D(0x0A=0xF0)`
  - `0x09=0x44`
  - `edge=trailing delay=0 left_ws=1`
  - `gtm_lost=0`
  - `trim leading_zero=2174 remain=31617 elapsed_ms=2057`
- Server PCM is usable:
  - `samples=31617 observed=2.057s src_rate=15370.4`
  - after resample `peak=11251 rms=1050.8 zcr=0.112`
  - `audio diag ... clip=0 ... win_rms=111.6..1963.9 dom=109.4Hz ratio=0.019`
- Xfyun succeeded:
  - `xfyun partial: 你好你好你好。`
  - `xfyun final: 你好你好你好。`
- Text return succeeded:
  - `send_text_to_board len=14 encoding=gbk text=你好你好你好。`
  - `send_text_to_board done`
- Command matching succeeded:
  - `send commands: [25]`

Conclusion:

- GTM-DMA GPIO snapshot + software I2S decode is confirmed viable for this hardware setup.
- Current production baseline should stay on:
  - `GUIMAI_GTM_DMA_CAPTURE = 1`
  - `SRC_D(0x0A=0xF0)`
  - `edge=trailing`
  - `left_ws=1`
  - `delay=0`
  - `byteswap`
  - `ES8388_MIC_GAIN = 4`.
- Remaining work is polish and robustness, not core capture bring-up.

Recommended next checks:

- Test action commands such as `打开双闪`, `打开左转`, `打开右转` and verify physical command behavior.
- Decide whether to keep `--debug-xfyun` during development or run normal mode for less log noise.
- If latency matters, evaluate streaming to Xfyun during recording instead of buffered replay after key release.
