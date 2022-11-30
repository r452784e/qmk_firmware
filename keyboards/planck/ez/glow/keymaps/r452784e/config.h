#pragma once

#ifdef AUDIO_ENABLE
#define STARTUP_SONG SONG(PLANCK_SOUND)
#endif

#define MIDI_BASIC

#define ENCODER_RESOLUTION 4

/*
  Set any config.h overrides for your specific keymap here.
  See config.h options at https://docs.qmk.fm/#/config_options?id=the-configh-file
*/
#define ORYX_CONFIGURATOR
#define USB_SUSPEND_WAKEUP_DELAY 0
#define IGNORE_MOD_TAP_INTERRUPT
#define FIRMWARE_VERSION u8"dnx6Z/bBd0g"
#define RAW_USAGE_PAGE 0xFF60
#define RAW_USAGE_ID 0x61
#define LAYER_STATE_8BIT
#define RGB_MATRIX_STARTUP_SPD 60

#define MK_VARIANT MK_TYPE_KINETIC

#define MK_KINETIC_MOUSE_MAXS 32
#define MK_KINETIC_MOUSE_ACCN 40
#define MK_KINETIC_MOUSE_FRIC 18
#define MK_KINETIC_MOUSE_DRAG 18

#define MK_KINETIC_WHEEL_MAXS 1
#define MK_KINETIC_WHEEL_ACCN 32
