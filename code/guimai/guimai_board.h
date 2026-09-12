#ifndef CODE_GUIMAI_BOARD_H_
#define CODE_GUIMAI_BOARD_H_

#include "zf_common_headfile.h"
#include "guimai_voice_config.h"

// -------------------- ?????? --------------------
void guimai_board_init(void);
void guimai_board_task(void);
void guimai_board_record_core_task(void);
void guimai_board_tx_core_task(void);
void guimai_board_voice_isr(void);
uint8 guimai_board_is_recording(void);
const char *guimai_board_record_status_text(void);
const char *guimai_board_wifi_status_text(void);
uint8 guimai_board_is_music_playing(void);

// Portable control-facing API. Existing projects can call these directly
// instead of using the built-in KEY3/UI trigger path.
void guimai_voice_start_record(void);
void guimai_voice_stop_record(void);
uint8 guimai_voice_is_recording(void);
uint8 guimai_voice_get_command(uint8 *cmd);
uint8 guimai_voice_handle_horn_command(uint8 cmd);

uint8 guimai_music_play_first(void);
void guimai_music_stop(void);
uint8 guimai_music_is_playing(void);
uint8 guimai_music_toggle_first(void);

#endif /* CODE_GUIMAI_BOARD_H_ */
