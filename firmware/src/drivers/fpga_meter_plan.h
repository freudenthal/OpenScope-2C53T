#ifndef FPGA_METER_PLAN_H
#define FPGA_METER_PLAN_H

#include <stdbool.h>
#include <stdint.h>

#define FPGA_METER_LOCAL_SUBMODE_COUNT 11u
#define FPGA_METER_STOCK_MODE_COUNT    8u   /* formatter families, not wire words */
#define FPGA_METER_LOGICAL_FUNCTION_COUNT 13u
#define FPGA_METER_TRANSITION_DISCARD_FRAMES 2u
#define FPGA_METER_TRANSITION_SETTLE_MS      20u
#define FPGA_METER_INVALID_STOCK_MODE        0xFFu
#define FPGA_METER_INVALID_LOCAL_SUBMODE     0xFFu
#define FPGA_METER_INVALID_SELECTOR_WORD     0x0000u
#define FPGA_METER_CONFIGURE_WORD            0x0508u
#define FPGA_METER_START_WORD                0x0509u

typedef enum {
    FPGA_METER_FUNCTION_DCV = 0,
    FPGA_METER_FUNCTION_ACV = 1,
    FPGA_METER_FUNCTION_DC_UA = 2,
    FPGA_METER_FUNCTION_DC_MA = 3,
    FPGA_METER_FUNCTION_DC_A = 4,
    FPGA_METER_FUNCTION_AC_UA = 5,
    FPGA_METER_FUNCTION_AC_MA = 6,
    FPGA_METER_FUNCTION_AC_A = 7,
    FPGA_METER_FUNCTION_RESISTANCE = 8,
    FPGA_METER_FUNCTION_CONTINUITY = 9,
    FPGA_METER_FUNCTION_DIODE = 10,
    FPGA_METER_FUNCTION_CAPACITANCE = 11,
    FPGA_METER_FUNCTION_TEMPERATURE = 12,
} fpga_meter_logical_function_t;

typedef enum {
    FPGA_METER_FRAME_FAMILY_VOLTAGE = 0,
    FPGA_METER_FRAME_FAMILY_CURRENT = 1,
    FPGA_METER_FRAME_FAMILY_RESISTANCE = 2,
    FPGA_METER_FRAME_FAMILY_CONTINUITY = 3,
    FPGA_METER_FRAME_FAMILY_DIODE = 4,
    FPGA_METER_FRAME_FAMILY_EXTENDED = 5,
    FPGA_METER_FRAME_FAMILY_INVALID = 0xFF,
} fpga_meter_frame_family_t;

typedef struct {
    uint8_t function_selector;  /* Stock DMM mode index used by the raw 0x05xx table. */
    uint8_t range_selector;     /* Low byte from the recovered stock DMM command table. */
    bool    voltage_function_axis;
} fpga_meter_selector_t;

typedef struct {
    uint8_t submode;
    uint8_t stock_mode;
    uint8_t mux_index;          /* Debug alias for the Port C/E mux projection. */
    uint8_t portc_porte_mux;    /* Local projection of stock ms[0x02]. */
    uint8_t porta_portb_mux;    /* Local projection of stock ms[0x03]. */
    uint8_t frame_family;
    uint8_t discard_frames;
    uint16_t settle_ms;
    bool has_command_bank_prefix;
    uint8_t command_bank_first;
    uint8_t command_bank_second;
    bool has_config_word;
    uint16_t config_word;
    uint16_t selector_word;
    bool has_apply_word;
    uint16_t apply_word;
    bool has_probe_detect;
    uint16_t start_word;
    bool voltage_function_axis;
} fpga_meter_transition_plan_t;

typedef struct {
    uint8_t pc12;
    uint8_t pe4;
    uint8_t pe5;
    uint8_t pe6;
    uint8_t pa15;
    uint8_t pa10;
    uint8_t pb10;
    uint8_t pb11;
    uint8_t pb9;
    uint8_t pa6;
} fpga_meter_mux_gpio_state_t;

bool fpga_meter_submode_is_valid(uint8_t submode);
/* stage-272: bench-checked on guest-coldtrace (table and record in the .c). */
bool fpga_meter_submode_is_usable(uint8_t submode);
bool fpga_meter_logical_function_is_valid(uint8_t function);
bool fpga_meter_logical_function_is_supported(uint8_t function);
bool fpga_meter_logical_function_is_unresolved(uint8_t function);
uint8_t fpga_meter_submode_for_logical_function(uint8_t function);
uint8_t fpga_meter_stock_mode_for_submode(uint8_t submode);
uint16_t fpga_meter_stock_cmd_word_for_submode(uint8_t submode);
fpga_meter_frame_family_t fpga_meter_frame_family_for_submode(uint8_t submode);
bool fpga_meter_frame_family_is_recovered(uint8_t family);
bool fpga_meter_frame_family_has_stock_marker(uint8_t family);
bool fpga_meter_frame_family_is_acceptable(uint8_t expected, uint8_t observed);
fpga_meter_transition_plan_t fpga_meter_transition_plan_for_submode(uint8_t submode);
bool fpga_meter_mux_gpio_state_for_stock_mux_arms(
    uint8_t portc_porte_mux,
    uint8_t porta_portb_mux,
    fpga_meter_mux_gpio_state_t *out);
bool fpga_meter_mux_gpio_state_for_submode(uint8_t submode,
                                           fpga_meter_mux_gpio_state_t *out);
bool fpga_meter_rx_frame_should_parse(bool transition_busy,
                                      volatile uint8_t *discard_count,
                                      volatile uint32_t *transition_skip_count);

/*
 * M-1 (stage-270): the meter SoC wakes in its own Auto function (word 0x0514)
 * at every meter entry. In Auto it measures a signal it can detect (3.3 V DC
 * reads 3.256 V), but with no input it shows the glyphs A, U, t, o: segment
 * codes 0x0A 0x0B 0x0C 0x0D, `---` on the screen (bench unit #1, 2026-11-25,
 * V lead open; the DC V word 0x050C then reads -0.0003 V at once). After
 * FPGA_METER_AUTO_RUN_FRAMES such frames in a row the firmware sends the
 * function selector again, at most once per FPGA_METER_AUTO_RESEND_MS.
 */
#define FPGA_METER_AUTO_RUN_FRAMES  8u
#define FPGA_METER_AUTO_RESEND_MS   3000u

typedef struct {
    uint16_t auto_run;        /* Auto frames in a row; 0 after any other frame */
    bool     resent;          /* last_resend_ms is valid */
    uint32_t last_resend_ms;
    uint32_t resends;         /* total, for `meter dump` */
} fpga_meter_auto_recover_t;

bool fpga_meter_digits_show_auto(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void fpga_meter_auto_recover_frame(fpga_meter_auto_recover_t *s, bool shows_auto);
/* True when the caller must send the selector now; records the resend. */
bool fpga_meter_auto_recover_due(fpga_meter_auto_recover_t *s, uint32_t now_ms);

#endif /* FPGA_METER_PLAN_H */
