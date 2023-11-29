#ifndef __MAX30102_H__
#define __MAX30102_H__

#include "device.h"

#define MAX30102_SLAVE_ADDRESS                      (0x57)

#define MAX30102_I2C_WRITE_ADDR                     0xAE
#define MAX30102_I2C_READ_ADDR                      0xAF
#define MAX30102_ID                                 0x15

//register addresses
#define MAX30102_REG_INTR_STATUS_1                  0x00
#define MAX30102_REG_INTR_STATUS_2                  0x01
#define MAX30102_REG_INTR_ENABLE_1                  0x02
#define MAX30102_REG_INTR_ENABLE_2                  0x03
#define MAX30102_REG_FIFO_WR_PTR                    0x04
#define MAX30102_REG_OVF_COUNTER                    0x05
#define MAX30102_REG_FIFO_RD_PTR                    0x06
#define MAX30102_REG_FIFO_DATA                      0x07
#define MAX30102_REG_FIFO_CONFIG                    0x08
#define MAX30102_REG_MODE_CONFIG                    0x09
#define MAX30102_REG_SPO2_CONFIG                    0x0A
#define MAX30102_REG_LED1_RED_PA                    0x0C
#define MAX30102_REG_LED2_IR_PA                     0x0D
#define MAX30102_REG_PILOT_PA                       0x10
#define MAX30102_REG_MULTI_LED_CTRL1                0x11
#define MAX30102_REG_MULTI_LED_CTRL2                0x12
#define MAX30102_REG_TEMP_INTR                      0x1F
#define MAX30102_REG_TEMP_FRAC                      0x20
#define MAX30102_REG_TEMP_CONFIG                    0x21
#define MAX30102_REG_PROX_INT_THRESH                0x30
#define MAX30102_REG_REV_ID                         0xFE
#define MAX30102_REG_PART_ID                        0xFF

/* Status */
// Interrupt Status 1                               R
#define MAX30102_INT1_STATUS_A_FULL                 (1 << 7)    // FIFO Almost Full Flag
#define MAX30102_INT1_STATUS_PPG_RDY                (1 << 6)    // New FIFO Data Ready
#define MAX30102_INT1_STATUS_ALC_OVF                (1 << 5)    // Ambient Light Cancellation Overflow
#define MAX30102_INT1_STATUS_PWR_RDY                (1 << 0)    // Power Ready Flag

// Interrupt Status 2                               R
#define MAX30102_INT2_STATUS_DIE_TEMP_RDY           (1 << 7)    // Internal Temperature Ready Flag

// Interrupt Enable 1                               R/W
#define MAX30102_INT1_A_FULL_EN                     (1 << 7)    // 
#define MAX30102_INT1_PPG_RDY_EN                    (1 << 6)
#define MAX30102_INT1_ALC_OVF_EN                    (1 << 5)

// Interrupt Enable 2                               R
#define MAX30102_INT2_DIE_TMP_RDY                   (1 << 7)

/* FIFO */
#define MAX30102_FIFO_WR_PTR(X)                     ((X) & 0x1F)    // FIFO Write Pointer           R/W
#define MAX30102_OVF_COUNTER(X)                     ((X) & 0x1F)    // FIFO Overflow Counter        R/W
#define MAX30102_FIFO_RD_PTR(X)                     ((X) & 0x1F)    // FIFO Read Pointer            R/W

/* FIFO Configuration */

/* Sample Averaging Definition */
#define MAX30102_FIFO_CFG_SMP_AVE(X)                ((X) << 5)
#define MAX30102_SMP_AVE_1                          (0b000)
#define MAX30102_SMP_AVE_2                          (0b001)
#define MAX30102_SMP_AVE_4                          (0b010)
#define MAX30102_SMP_AVE_8                          (0b011)
#define MAX30102_SMP_AVE_16                         (0b100)
#define MAX30102_SMP_AVE_32                         (0b101)
#define MAX30102_SMP_AVE_32_2                       (0b110)
#define MAX30102_SMP_AVE_32_3                       (0b111)

#define MAX30102_FIFO_CFG_ROLLOVER_EN               (1 << 4)    // FIFO Rolls on Full

/**
 * FIFO Almost Full Value
 * 
 * This register sets the number of data samples (3 bytes/sample)
 * remaining in the FIFO when the interrupt is issued.
 * if this field is set to 0x0, the interrupt is issued
 * when there is 0 data samples remaining in the FIFO (all 32
 FIFO words have unread data).
 * */
#define MAX30102_FIFO_CFG_A_FULL(X)                 ((X) & 0x0F)

/* Mode Configuration */
#define MAX30102_MODE_CFG_SHDN                      (1 << 7)
#define MAX30102_MODE_CFG_RESET                     (1 << 6)
/* Mode Control */
#define MAX30102_MODE_CFG_CTL(X)                    ((X) & 0x07)
#define MAX30102_MODE_HR_RED                        (0b010)         // Heart Rate Mode | Red Only
#define MAX30102_MODE_SPO2_RED_IR                   (0b011)         // SpO2 mode | Red and IR
#define MAX30102_MODE_MLED_RED_IR                   (0b111)         // Multi-LED mode | Red and IR

/* SpO2 Configuration */
/* SpO2 ADC Range Control (18-Bit Resolution)*/
#define MAX30102_SPO2_CFG_ADC_REG(X)                (((X) & 0x03) << 5)
#define MAX30102_SPO2_ADC_RGE_2048                  (0b00)          // LSB Size 7.81 pA
#define MAX30102_SPO2_ADC_RGE_4096                  (0b01)          // LSB Size 15.63 pA
#define MAX30102_SPO2_ADC_RGE_8192                  (0b10)          // LSB Size 31.25 pA
#define MAX30102_SPO2_ADC_RGE_16384                 (0b11)          // LSB Size 62.45 pA

/*  SpO2 Sample Rate Control*/
#define MAX30102_SPO2_SR_50                         (0b000)
#define MAX30102_SPO2_SR_100                        (0b001)
#define MAX30102_SPO2_SR_200                        (0b010)
#define MAX30102_SPO2_SR_400                        (0b011)
#define MAX30102_SPO2_SR_800                        (0b100)
#define MAX30102_SPO2_SR_1000                       (0b101)
#define MAX30102_SPO2_SR_1600                       (0b110)
#define MAX30102_SPO2_SR_1600                       (0b110)
#define MAX30102_SPO2_SR_3200                       (0b111)
#define MAX30102_SPO2_CFG_SR(X)                     (((X) & 0x07) << 2)

/* LED Pulse Width Control and ADC Resolution */
#define MAX30102_SPO2_CFG_LED_PW(X)                 ((X) & 0x03)
#define MAX30102_SPO2_LED_PW_69                     (0b00)      // Pulse Width 68.95 μs  | ADC Resolution(15 bits)
#define MAX30102_SPO2_LED_PW_118                    (0b00)      // Pulse Width 117.78 μs | ADC Resolution(16 bits)
#define MAX30102_SPO2_LED_PW_215                    (0b10)      // Pulse Width 215.44 μs | ADC Resolution(17 bits)
#define MAX30102_SPO2_LED_PW_411                    (0b11)      // Pulse Width 410.75 μs | ADC Resolution(18 bits)

/** Multi-LED Mode Control Registers (0x11–0x12)
 * 
 * Each slot generates a 3-byte output into the FIFO. One sample comprises all active slots, for example if SLOT1 and SLOT2 are non-zero, then one sample is 2 x 3 = 6 bytes.
 * 
 * The slots should be enabled in order (i.e., SLOT1 should not be disabled if SLOT2 is enabled).
 *
 */
#define MAX30102_MLED_MODE_CTL_H(X)                 (((X) & 0x07) << 4)
#define MAX30102_MLED_MODE_CTL_L(X)                 ((X) & 0x07)
#define MAX30102_MLED_MODE_DISABLE                  (0b000)
#define MAX30102_MLED_MODE_LED1_RED                 (0b001)
#define MAX30102_MLED_MODE_LED2_IR                  (0b010)

#define MAX30102_DIE_TEMP_ENABLE                    (0x00)
#define MAX30102_DIE_TEMP_DISABLE                   (0x01)

typedef struct max30102_dev max30102_t;

struct max30102_dev
{
    device_t *dev;
};

typedef enum max30102_ret_type
{
    DEVICE_COMMON_RET(MAX30102),
    MAX30102_E_INVALID_LED_SLOT,
} max30102_ret_type_t;

DEVICE_INTF_RET_TYPE max30102_interface_init(max30102_t *max30102, device_t *dev);
DEVICE_INTF_RET_TYPE max30102_init(device_t *max30102);

/*! Reads the value of a register. */
DEVICE_INTF_RET_TYPE max30102_get_register_value(device_t *max30102,
                                                uint8_t reg,
                                                uint8_t *data,
                                                uint16_t len);

/*! Writes data into a register. */
DEVICE_INTF_RET_TYPE max30102_set_register_value(device_t *max30102,
                                                uint8_t register_address,
                                                uint8_t register_value);

DEVICE_INTF_RET_TYPE max30102_int1_en(device_t *max30102, uint8_t int_ctl);
DEVICE_INTF_RET_TYPE max30102_int2_en(device_t *max30102, uint8_t temp_rdy_en);
DEVICE_INTF_RET_TYPE max30102_get_int1_status(device_t *max30102, uint8_t *status);
DEVICE_INTF_RET_TYPE max30102_get_int2_status(device_t *max30102, uint8_t *status);

DEVICE_INTF_RET_TYPE max30102_get_fifo_wr_ptr(device_t *max30102, uint8_t *wr_ptr);
DEVICE_INTF_RET_TYPE max30102_get_fifo_ovf_counter(device_t *max30102, uint8_t *overflow_counter);
DEVICE_INTF_RET_TYPE max30102_get_fifo_rd_ptr(device_t *max30102, uint8_t *rd_ptr);
DEVICE_INTF_RET_TYPE max30102_get_fifo_size(device_t *max30102, uint8_t *buffer_size);
DEVICE_INTF_RET_TYPE max30102_fifo_read_one(device_t *max30102, uint32_t *red_led, uint32_t *ir_led);
DEVICE_INTF_RET_TYPE max30102_get_fifo_data(device_t *max30102, uint32_t *red_led_data, uint32_t *ir_led_data, uint16_t data_size, uint16_t sample_size, uint16_t data_start_idx);
DEVICE_INTF_RET_TYPE max30102_set_fifo_a_full(device_t *max30102, uint8_t a_full);
DEVICE_INTF_RET_TYPE max30102_fifo_rollover_en(device_t *max30102, uint8_t enable);
DEVICE_INTF_RET_TYPE max30102_set_fifo_smp_ave(device_t *max30102, uint8_t smp_ave);

DEVICE_INTF_RET_TYPE max30102_mode_config(device_t *max30102, uint8_t mode);
DEVICE_INTF_RET_TYPE max30102_mode_reset(device_t *max30102, uint8_t enable);
DEVICE_INTF_RET_TYPE max30102_mode_shutdown(device_t *max30102, uint8_t enable);

DEVICE_INTF_RET_TYPE max30102_spo2_config(device_t *max30102, uint8_t led_pw, uint8_t sample_rate, uint8_t adc_rge);
DEVICE_INTF_RET_TYPE max30102_set_spo2_led_pw(device_t *max30102, uint8_t led_pw);
DEVICE_INTF_RET_TYPE max30102_set_spo2_sample_rate(device_t *max30102, uint8_t sample_rate);
DEVICE_INTF_RET_TYPE max30102_set_spo2_adc_range(device_t *max30102, uint8_t adc_rge);

DEVICE_INTF_RET_TYPE max30102_set_led_red_pa(device_t *max30102, float current);
DEVICE_INTF_RET_TYPE max30102_set_led_ir_pa(device_t *max30102, float current);
/**
 * @brief Set Multi-LED Mode. \n
 * In multi-LED mode, each sample is split into up to four time slots, SLOT1    * through SLOT4. These control registers determine which LED is active in each * time slot, making for a very flexible configuration.
 * 
 * @param[in] max30102: device handler
 * @param[in] slotx[2:0]: see @ref MAX30102_MLED_MODE_x for detail.
*/
DEVICE_INTF_RET_TYPE max30102_set_multi_led_mode(device_t *max30102, uint8_t slot1, uint8_t slot2, uint8_t slot3, uint8_t slot4);

/**
 * @brief Get on board temperature ADC output result.
 * 
 * @param[in, out] temp: temp res.
*/
DEVICE_INTF_RET_TYPE max30102_get_temperature(device_t *max30102, float *temp);
DEVICE_INTF_RET_TYPE max30102_temp_en(device_t *max30102, uint8_t enable);
#endif
