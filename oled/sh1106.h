#ifndef __SH1106_H__
#define __SH1106_H__

#include "device.h"
#include "font.h"

// OLED slave address
#define SH1106_ADDRESS                  0x7A

// Set Column Address
#define SH1106_LOWER_COLUMN_ADDRESS(X)              ((X) & 0x0F)    // Set Lower Column Address of display RAM: 0x00 ~ 0x0F
#define SH1106_HIGHER_COLUMN_ADDRESS(X)             (((X) | 0x10) & 0x1F) // Higher Column Address: 0x10 ~ 0x1F

// Pump voltage value: 0x30 ~ 0x33
#define SH1106_PUMP_VOLTAGE(X)                      (((X) | 0x30) & 0x33)
#define SH1106_VPP_6_4                              (0x00)
#define SH1106_VPP_7_4                              (0x01)
#define SH1106_VPP_8_0                              (0x02)
#define SH1106_VPP_9_0                              (0x03)

// Display Start Line: 0x40 ~ 0x7F
#define SH1106_DISPLAY_START_LINE(X)                (((X) | 0x40) & 0x7F)

// Contrast control Register
#define SH1106_CONTRAST_CTL_MODE                    (0x81)  // Contrast Control Mode Set
#define SH1106_CONTRAST_DATA_REG(X)                 ((X) & 0xFF)    // Contrast Data Register Set: 0x00 ~ 0xFF

// Segment remap
// Change the relationship between RAM column address and segment driver
#define SH1106_SEG_REMAP_L                          (0xA0)  // Right rotates
#define SH1106_SEG_REMAP_H                          (0xA1)  // Left rotates

// Set Entire Display OFF/ON
#define SH1106_ENTIRE_DISPLAY_OFF                   (0xA4)  // Normal display status
#define SH1106_ENTIRE_DISPLAY_ON                    (0xA5)  // Entire display on status

// Set Normal/Reverse Display
#define SH1106_NORMAL_DISPLAY                       (0xA6)  // Normal display
#define SH1106_REVERSE_DISPLAY                      (0xA7)  // Reverse display

// Mutiplex Ration
#define SH1106_MUL_RATION_MODE                      (0xA8)  // Mutiplex Ration Mode Set
#define SH1106_MUL_RATION_DATA(X)                   ((X) & 0x3F) // Mutiplex Ration Data Set: 0x00 ~ 0x3F

// Set DC-DC OFF/ON
#define SH1106_DC_DC_CTL_MODE                       (0xAD)  // DC-DC Control Mode Set
#define SH1106_DC_DC_MODE_OFF                       (0x8A)  // DC-DC is disable
#define SH1106_DC_DC_MODE_ON                        (0x8B)  // DC-DC will be on when display on

// Display OFF/ON
#define SH1106_DISPLAY_OFF                          (0xAE)  // DC-DC is disable
#define SH1106_DISPLAY_ON                           (0xAF)  // DC-DC will be on when display on

// Set Page Address: 0xB0 ~ 0xB7
#define SH1106_PAGE_ADDRESS(X)                      (((X) | 0xB0) & 0xB7)

// Set Common Output Scan Direction
#define SH1106_OUTPUT_SCAN_DIR_LR                   (0xC0)  // Scan from COM0 to COM[N-1]
#define SH1106_OUTPUT_SCAN_DIR_RL                   (0xC8)  // Scan from COM[N-1] to COM0

// Set Display Offset
#define SH1106_DISPLAY_OFFSET_MODE                  (0xD3)  // Display Offset Mode Set
#define SH1106_DISPLAY_OFFSET(X)                    ((X) & 0x3F)    // Display Offset Data Set

// Set Display Clock Ratio/Oscillattor Frequency
#define SH1106_DIVIDE_RATIO_OSC_FREQ                (0xD5) // Divide Ratio/Oscillattor Frequency Mode Set
// oscliactor freq = x*5% - 25% (0x00 ~ 0x0F)
#define SH1106_OSC_FREQ(X)                          (((X) & 0x0F) << 4)
// divide ration of the display clocks (DCLK)
#define SH1106_DIVIDE_RATION_CLK(X)                 ((X) & 0x0F)

// Set Dis-charge/Pre-charge Period
#define SH1106_CHARGE_PERIOD                        (0xD9) // Divide Ratio/Oscillattor Frequency Mode Set
// Dis-charge Period: 0 < X <= 15 DCLK
#define SH1106_DIS_CHARGE(X)                        (((X) & 0x0F) << 4)
// Pre-charge Period: 0 < X <= 15 DCLK
#define SH1106_PRE_CHARGE(X)                        ((X) & 0x0F)

// Set Common pads hardware configuration
#define SH1106_PAD_CFG_MODE                         (0xDA)
// Sequential Mode: COM31, 30 - 1, 0 | SEG0, 1 - 130, 131 | COM32, 33 - 62, 63
#define SH1106_SEQ_MODE                             (0x02)
// Alternative Mode: COM62, 60 - 2, 0 | SEG0, 1 - 130, 131 | COM1, 3 - 61, 63
#define SH1106_ALT_MODE                             (0x12)

// Set VCOMM Deselect level
// VCOM Deselect Level Mode Set
#define SH1106_VCOM_DESELECT_LVL_MODE               (0xDB)
// VCOM = beta * V_REF
// VCOM Deselect Level Data: if X < 0x20: beta = 0.430; elif X < 0x40: beta = 0.770; else X >= 0x40: beta = 1
#define SH1106_VCOM_DESELECT_LVL(X)                 ((X) & 0xFF)

// A pair of Read-Modify-Write and End commands always be used
#define SH1106_READ_MOD_WRITE                       (0xE0)  // Read-Modify-Write
// END: Cancel Read-Modify-Write mode and returns column address to the original address (when Read-Modify is issued)
#define SH1106_END                                  (0xEE)

// Non-Operation Command
#define SH1106_NOP                                  (0xE3)
// Read SH1106 Status
#define SH1106_BUSY                                 (0x80) // Busy
#define SH1106_OFF                                  (0x40) // OFF

// OLED parameters
#ifndef SH1106_PAGE
#define SH1106_PAGE                     8                   // OLED pages
#endif
#ifndef SH1106_ROW
#define SH1106_ROW                      8 * SH1106_PAGE     // OLED rows
#endif
#ifndef SH1106_COLUMN
#define SH1106_COLUMN                   128                 // OLED columns
#endif

typedef enum {
  SH1106_COLOR_NORMAL = 0xA6, // bg: black, color: white
  SH1106_COLOR_REVERSED = 0xA7    // bg: white, color: black
} SH1106_ColorMode;

typedef struct sh1106_dev sh1106_t;
typedef struct sh1106_dev_attr sh1106_attr_t;
typedef struct sh1106_dev_gpu sh1106_gpu_t;
typedef enum sh1106_ret sh1106_ret_t;

struct sh1106_dev
{
    device_t *dev;
    device_t *reset_pin;

    // dc pin: low: command, hig: data
    device_t *dc_pin;
    sh1106_attr_t *sh1106_attr;
};

struct sh1106_dev_attr
{
    uint16_t page;
    uint16_t row;
    uint16_t column;
    sh1106_gpu_t *sh1106_gpu;
};

struct sh1106_dev_gpu
{
    uint8_t **gpu;
};

enum sh1106_ret
{
    DEVICE_COMMON_RET(SH1106),

    /* Incorrect length parameter */
    RET_ERROR(SH1106, INVALID_LENGTH),

    /* GPU uninitialized */
    RET_ERROR(SH1106, UNINIT_GPU),

    RET_ERROR(SH1106, UNINIT_INTERFACE),

    RET_ERROR(SH1106, HEAP_OVERFLOW),

    RET_ERROR(SH1106, INVALID_PARAM),

    RET_ERROR(SH1106, UNINIT_RESET_PIN),

    RET_ERROR(SH1106, UNINIT_DC_PIN),
    // hardware pixel out of bound
    RET_ERROR(SH1106, HAL_OOB)
};

DEVICE_INTF_RET_TYPE sh1106_null_ptr_check(sh1106_t *sh1106);

DEVICE_INTF_RET_TYPE sh1106_reset(sh1106_t *sh1106);
DEVICE_INTF_RET_TYPE sh1106_init(sh1106_t *sh1106);
DEVICE_INTF_RET_TYPE oled_sh1106_init(sh1106_t *sh1106);
sh1106_attr_t *sh1106_attr_create(uint32_t page, uint32_t column);
void sh1106_attr_destroy(sh1106_attr_t *attr);
sh1106_gpu_t *sh1106_gpu_create(uint32_t page, uint32_t column);
sh1106_gpu_t *sh1106_get_gpu(sh1106_t *sh1106);
void sh1106_gpu_destroy(sh1106_gpu_t *gpu, uint32_t page);
DEVICE_INTF_RET_TYPE sh1106_attr_init(sh1106_t *sh1106, uint32_t page, uint32_t column);
DEVICE_INTF_RET_TYPE sh1106_interface_init(sh1106_t *sh1106, device_t *dev, device_t *reset_pin, device_t *dc_pin, device_type_t dtype, uint32_t page, uint32_t column);

DEVICE_INTF_RET_TYPE sh1106_send_cmd(sh1106_t *sh1106, uint8_t cmd);
DEVICE_INTF_RET_TYPE sh1106_send_data(sh1106_t *sh1106, uint8_t *data, uint16_t len);
DEVICE_INTF_RET_TYPE sh1106_display_on(sh1106_t *sh1106);
DEVICE_INTF_RET_TYPE sh1106_display_off(sh1106_t *sh1106);
DEVICE_INTF_RET_TYPE sh1106_new_frame(sh1106_t *sh1106);
DEVICE_INTF_RET_TYPE sh1106_show_frame(sh1106_t *sh1106);
DEVICE_INTF_RET_TYPE sh1106_set_color_mode(sh1106_t *sh1106, SH1106_ColorMode mode);

DEVICE_INTF_RET_TYPE sh1106_set_pixel(sh1106_t *sh1106, uint8_t x, uint8_t y, SH1106_ColorMode mode);
DEVICE_INTF_RET_TYPE sh1106_set_byte_fine(sh1106_t *sh1106, uint8_t page, uint8_t column, uint8_t data, uint8_t start, uint8_t end, SH1106_ColorMode mode);
DEVICE_INTF_RET_TYPE sh1106_set_byte(sh1106_t *sh1106, uint8_t page, uint8_t column, uint8_t data, SH1106_ColorMode mode);
DEVICE_INTF_RET_TYPE sh1106_set_bits_fine(sh1106_t *sh1106, uint8_t x, uint8_t y, uint8_t data, uint8_t len, SH1106_ColorMode mode);
DEVICE_INTF_RET_TYPE sh1106_set_bits(sh1106_t *sh1106, uint8_t x, uint8_t y, uint8_t data, SH1106_ColorMode mode);
DEVICE_INTF_RET_TYPE sh1106_set_block(sh1106_t *sh1106, uint8_t x, uint8_t y, const uint8_t *data, uint8_t w, uint8_t h, SH1106_ColorMode mode);

DEVICE_INTF_RET_TYPE sh1106_draw_line(sh1106_t *sh1106, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SH1106_ColorMode mode);
DEVICE_INTF_RET_TYPE sh1106_draw_rectangle(sh1106_t *sh1106, uint8_t x, uint8_t y, uint8_t w, uint8_t h, SH1106_ColorMode mode);
DEVICE_INTF_RET_TYPE sh1106_draw_filled_rectangle(sh1106_t *sh1106, uint8_t x, uint8_t y, uint8_t w, uint8_t h, SH1106_ColorMode mode);
DEVICE_INTF_RET_TYPE sh1106_draw_triangle(sh1106_t *sh1106, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, SH1106_ColorMode mode);
DEVICE_INTF_RET_TYPE sh1106_draw_filled_triangle(sh1106_t *sh1106, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, SH1106_ColorMode mode);
DEVICE_INTF_RET_TYPE sh1106_draw_circle(sh1106_t *sh1106, uint8_t x, uint8_t y, uint8_t r, SH1106_ColorMode mode);
DEVICE_INTF_RET_TYPE sh1106_draw_filled_circle(sh1106_t *sh1106, uint8_t x, uint8_t y, uint8_t r, SH1106_ColorMode mode);
DEVICE_INTF_RET_TYPE sh1106_draw_image(sh1106_t *sh1106, uint8_t x, uint8_t y, const image_t *img, SH1106_ColorMode mode);

DEVICE_INTF_RET_TYPE sh1106_print_ascii_char(sh1106_t *sh1106, uint8_t x, uint8_t y, char ch, const ascii_font_t *font, SH1106_ColorMode mode);
DEVICE_INTF_RET_TYPE sh1106_print_ascii_string(sh1106_t *sh1106, uint8_t x, uint8_t y, char *str, const ascii_font_t *font, SH1106_ColorMode mode);
DEVICE_INTF_RET_TYPE sh1106_print_string(sh1106_t *sh1106, uint8_t x, uint8_t y, char *str, const font_t *font, SH1106_ColorMode mode);
#endif
