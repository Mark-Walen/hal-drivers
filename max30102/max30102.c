#include <stdio.h>
#include "max30102/max30102.h"

DEVICE_INTF_RET_TYPE max30102_interface_init(max30102_t *max30102, device_t *dev)
{
	if (max30102 == NULL || dev == NULL)
    {
        return DEVICE_E_NULLPTR;
    }

	max30102->dev = dev;
	return DEVICE_OK;
}

DEVICE_INTF_RET_TYPE max30102_init(device_t *max30102)
{
    int32_t status = 0;
    uint8_t chip_id = 0;
    
    if (device_null_ptr_check(max30102) != MAX30102_OK)
    {
        return MAX30102_E_NULLPTR;
    }

    status = max30102_get_register_value(max30102, MAX30102_REG_PART_ID, &chip_id, 1);
	if (status != MAX30102_OK)
		return status;
    
    if (chip_id != MAX30102_ID)
    {
        return MAX30102_E_NOT_FOUND;
    }

	return config_device_info(max30102, "%n%i%c", "max30102", "I2C", chip_id);
}

/*! Reads the value of a register. */
DEVICE_INTF_RET_TYPE max30102_get_register_value(device_t *max30102,
                                                uint8_t reg,
                                                uint8_t *data,
                                                uint16_t len)
{
    DEVICE_INTF_RET_TYPE ret = MAX30102_OK;

    if (device_null_ptr_check(max30102) != MAX30102_OK)
    {
        return MAX30102_E_NULLPTR;
    }

    ret = max30102->write(&reg, 1, max30102->fp, max30102->addr);
    if (ret != DEVICE_OK)
    {
        return ret;
    }
    
    ret = max30102->read(data, len, max30102->fp, max30102->addr);
    return ret;
}

/*! Writes data into a register. */
DEVICE_INTF_RET_TYPE max30102_set_register_value(device_t *max30102,
                                                uint8_t reg,
                                                uint8_t reg_value)
{
    int32_t ret;
    uint8_t buffer[2] = {reg, reg_value};

	ret = device_null_ptr_check(max30102);
	if (ret != DEVICE_OK)
		return ret;

	return max30102->write(buffer, 2, max30102->fp, max30102->addr);
}

/**
 * @brief get interrupt 1 status
 * 
 * @param[in, out] status: a_full, ppg_rdy, alc_ovf.
*/
DEVICE_INTF_RET_TYPE max30102_get_int1_status(device_t *max30102, uint8_t *status)
{
    return max30102_get_register_value(max30102, MAX30102_REG_INTR_STATUS_1, status, 1);
}

/**
 * @brief get interrupt 2 status
 * 
 * @param[in, out] status: a_full, ppg_rdy, alc_ovf.
*/
DEVICE_INTF_RET_TYPE max30102_get_int2_status(device_t *max30102, uint8_t *status)
{
    return max30102_get_register_value(max30102, MAX30102_REG_INTR_STATUS_2, status, 1);
}

/**
 * @brief config interrupt 1 enable
 * 
 * @param[in, out] int_ctl: a_full, ppg_rdy, alc_ovf.
*/
DEVICE_INTF_RET_TYPE max30102_int1_en(device_t *max30102, uint8_t int_ctl)
{
    return max30102_set_register_value(max30102, MAX30102_REG_INTR_ENABLE_1, int_ctl);
}

/**
 * @brief config interrupt 2 enable
 * 
 * @param[in, out] temp_rdy_en: die_temp_rdy.
*/
DEVICE_INTF_RET_TYPE max30102_int2_en(device_t *max30102, uint8_t temp_rdy_en)
{
    return max30102_set_register_value(max30102, MAX30102_REG_INTR_ENABLE_2, temp_rdy_en);
}

/**
 * @brief The FIFO Write Pointer points to the location where the MAX30102 writes the next sample. It can also be changed through the I2C interface when MODE[2:0] is MAX30102_MODE_HR_RED(010), MAX30102_MODE_SPO2_RED_IR(011), or MAX30102_MODE_MLED_RED_IR(111).
 * 
 * @param[in, out] wr_ptr: overflow count.
*/
DEVICE_INTF_RET_TYPE max30102_get_fifo_wr_ptr(device_t *max30102, uint8_t *wr_ptr)
{
    return max30102_get_register_value(max30102, MAX30102_REG_FIFO_WR_PTR, wr_ptr, 1);
}

/**
 * @brief When the FIFO is full, samples are not pushed on to the FIFO, samples are lost. OVF_COUNTER counts the number of samples lost. It saturates at 0x1F.
 * 
 * @param[in, out] overflow_counter: overflow count.
*/
DEVICE_INTF_RET_TYPE max30102_get_fifo_ovf_counter(device_t *max30102, uint8_t *overflow_counter)
{
    return max30102_get_register_value(max30102, MAX30102_REG_OVF_COUNTER, overflow_counter, 1);
}

/**
 * @brief The FIFO Read Pointer points to the location from where the processor gets the next sample from the FIFO through the I2C interface. The processor can also write to this pointer after reading the samples to allow rereading samples from the FIFO if there is a data communication error.
 * 
 * @param[in, out] rd_ptr: read pointer.
*/
DEVICE_INTF_RET_TYPE max30102_get_fifo_rd_ptr(device_t *max30102, uint8_t *rd_ptr)
{
    return max30102_get_register_value(max30102, MAX30102_REG_FIFO_RD_PTR, rd_ptr, 1);
}

/**
 * @brief Get available sample of fifo.
 * 
 * @param[in] max30102: device handler
 * @param[in, out] buffer_size: number of available samples
*/
DEVICE_INTF_RET_TYPE max30102_get_fifo_size(device_t *max30102, uint8_t *buffer_size)
{
    uint8_t int1_status = 0, int2_status = 0;
    uint8_t write_ptr = 0, read_ptr = 0;
    int8_t tmp;
    DEVICE_INTF_RET_TYPE ret = MAX30102_OK;

    ret = max30102_get_fifo_wr_ptr(max30102, &write_ptr);
    if (ret != MAX30102_OK)
    {
        return ret;
    }

    ret = max30102_get_fifo_rd_ptr(max30102, &read_ptr);
    if (ret != MAX30102_OK)
    {
        return ret;
    }

    tmp = write_ptr - read_ptr;
    if (tmp < 0)
    {
        tmp += 32;
    }
    *buffer_size = tmp;
    printf("%d %d, %d\r\n", read_ptr, write_ptr, tmp);
    return ret;
}

/**
 * @brief Get one sample from fifo.
 * 
 * @param[in] max30102: device handler
 * @param[in, out] red_led: red led sample
 * @param[in, out] ir_led: ir led sample
*/
DEVICE_INTF_RET_TYPE max30102_fifo_read_one(device_t *max30102, uint32_t *red_led, uint32_t *ir_led)
{
    DEVICE_INTF_RET_TYPE ret = MAX30102_OK;
    uint8_t buffer[6];
    if (red_led == NULL || ir_led == NULL)
    {
        return MAX30102_E_NULLPTR;
    }
    
    ret = max30102_get_register_value(max30102, MAX30102_REG_FIFO_DATA, buffer, 6);

    *red_led = (buffer[0] << 16 | buffer[1] << 8 | buffer[2]) & 0x0003FFFF;
    *ir_led = (buffer[3] << 16 | buffer[4] << 8 | buffer[5]) & 0x0003FFFF;
    return ret;
}

static void discard_head(uint32_t *data, uint16_t data_size, uint16_t head_n)
{
    if (data_size < head_n)
    {
        memset(data, 0, sizeof(uint32_t) * data_size);
        return;
    }
    
    for (uint16_t i = head_n; i < data_size; i++)
    {
        data[i - head_n] = data[i];
    }
    memset(data + data_size - head_n, 0, sizeof(uint32_t)*head_n);
}

/**
 * @brief Each sample comprises multiple bytes of data, so multiple bytes should be read from this register (in the same transaction) to get one full sample.
 * 
 * @param[in] max30102: device handler
 * @param[in, out] red_led_data: red led samples
 * @param[in, out] ir_led_data: ir led samples
 * @param[in, out] len: sample length
*/
DEVICE_INTF_RET_TYPE max30102_get_fifo_data(device_t *max30102, uint32_t *red_led_data, uint32_t *ir_led_data, uint16_t data_size, uint16_t sample_size, uint16_t data_start_idx)
{
    uint16_t count = sample_size, data_idx = data_start_idx;
    uint8_t fifo_size = 0, discard_size;
    DEVICE_INTF_RET_TYPE ret = MAX30102_OK;

    if (red_led_data == NULL || ir_led_data == NULL || data_size == 0 || (data_idx + sample_size) > data_size)
    {
        return MAX30102_E_NULLPTR;
    }

    while (count > 0)
    {
        ret = max30102_get_fifo_size(max30102, &fifo_size);
        printf("%d\n", fifo_size);
        if (count < fifo_size)
        {
            discard_size = fifo_size - count;
            discard_head(red_led_data, data_size, discard_size);
            discard_head(ir_led_data, data_size, discard_size);
            red_led_data -= discard_size;
            ir_led_data -= discard_size;
        }
        
        while (fifo_size > 0)
        {
            ret = max30102_fifo_read_one(max30102, &red_led_data[data_idx], &ir_led_data[data_idx]);
            data_idx++;
            fifo_size--;
        }
        count -= fifo_size;
    }
    printf("hey");
    return ret;
}

/**
 * @brief Set the number of data samples (3 bytes/sample) remaining in the FIFO when the interrupt is issued.
 * 
 * @param[in] max30102: device handler
 * @param[in] a_full:  If this field is set to 0x0, the interrupt is issued when there is 0 data samples remaining in the FIFO. Furthermore, if this field is set to 0xF, the interrupt is issued when 15 data samples are remaining in the FIFO (17 FIFO data samples have unread data). 0 <= a_full <= 15
*/
DEVICE_INTF_RET_TYPE max30102_set_fifo_a_full(device_t *max30102, uint8_t a_full)
{
    uint8_t old_fifo_cfg, new_fifo_cfg;
    DEVICE_INTF_RET_TYPE ret = MAX30102_OK;

    ret = max30102_get_register_value(max30102, MAX30102_REG_FIFO_CONFIG, &old_fifo_cfg, 1);
    if (ret != MAX30102_OK)
    {
        return ret;
    }

    if (a_full > 0x0F)
    {
        a_full = 0x0F;
    }
    
    old_fifo_cfg = old_fifo_cfg & 0xF0;
    new_fifo_cfg = old_fifo_cfg | a_full;

    ret = max30102_set_register_value(max30102, MAX30102_REG_FIFO_CONFIG, new_fifo_cfg);
    return ret;
}

/**
 * @brief To reduce the amount of data throughput, adjacent samples (in each individual channel) can be averaged and decimated on the chip.
 * 
 * @param[in] max30102: device handler
 * @param[in] enable: 0x00: the FIFO is not updated until FIFO_DATA is read or the WRITE/READ pointer positions are changed.
 *                0x01: the FIFO address rolls over to zero and the FIFO continues to fill with new data
*/
DEVICE_INTF_RET_TYPE max30102_fifo_rollover_en(device_t *max30102, uint8_t enable)
{
    uint8_t old_fifo_cfg, new_fifo_cfg;
    DEVICE_INTF_RET_TYPE ret = MAX30102_OK;
    uint8_t rollover_en = ~MAX30102_FIFO_CFG_ROLLOVER_EN;

    ret = max30102_get_register_value(max30102, MAX30102_REG_FIFO_CONFIG, &old_fifo_cfg, 1);
    if (ret != MAX30102_OK)
    {
        return ret;
    }

    old_fifo_cfg = old_fifo_cfg & rollover_en;
    if (enable)
    {
        rollover_en = MAX30102_FIFO_CFG_ROLLOVER_EN;
    } else {
        rollover_en = 0;
    }
    
    new_fifo_cfg = old_fifo_cfg | rollover_en;

    ret = max30102_set_register_value(max30102, MAX30102_REG_FIFO_CONFIG, new_fifo_cfg);

    return ret;
}

/**
 * @brief To reduce the amount of data throughput, adjacent samples (in each individual channel) can be averaged and decimated on the chip.
 * 
 * @param[in] max30102: device handler
 * @param[in] smp_ave: MAX30102_FIFO_CFG_SMP_AVE(X)
 *              Options: See MAX30102_SMP_AVE_x for more detail.
*/
DEVICE_INTF_RET_TYPE max30102_set_fifo_smp_ave(device_t *max30102, uint8_t smp_ave)
{
    uint8_t old_smp_ave, new_smp_ave;
    DEVICE_INTF_RET_TYPE ret = MAX30102_OK;

    ret = max30102_get_register_value(max30102, MAX30102_REG_FIFO_CONFIG, &old_smp_ave, 1);

    if (ret != MAX30102_OK)
    {
        return ret;
    }

    old_smp_ave = old_smp_ave & 0x1F;
    new_smp_ave = old_smp_ave | smp_ave;

    ret = max30102_set_register_value(max30102, MAX30102_REG_FIFO_CONFIG, new_smp_ave);

    return ret;
}

/**
 * @brief Set operating state of MAX30102. Changing modes does not change any other setting, nor does it erase any previously stored data inside the data registers.
 * 
 * @param[in] max30102: device handler
 * @param[in] mode: MAX30102_MODE_CFG_CTL(X)
 *              Options: See MAX30102_MODE_x for more detail.
*/
DEVICE_INTF_RET_TYPE max30102_mode_config(device_t *max30102, uint8_t mode)
{
    uint8_t old_mode_cfg, new_mode_cfg;
    DEVICE_INTF_RET_TYPE ret = MAX30102_OK;

    ret = max30102_get_register_value(max30102, MAX30102_REG_MODE_CONFIG, &old_mode_cfg, 1);

    if (ret != MAX30102_OK)
    {
        return ret;
    }

    old_mode_cfg = old_mode_cfg & 0xF8;
    new_mode_cfg = old_mode_cfg | mode;

    ret = max30102_set_register_value(max30102, MAX30102_REG_MODE_CONFIG, new_mode_cfg);

    return ret;
}

/**
 * @brief Shutdown mode: power save mode
 * 
 * @param[in] max30102: device handler
 * @param[in] reset: reset device
 *               Example 0x00 - no effect
 *                       0x01 - reset
*/
DEVICE_INTF_RET_TYPE max30102_mode_reset(device_t *max30102, uint8_t enable)
{
    uint8_t old_mode_cfg, new_mode_cfg;
    DEVICE_INTF_RET_TYPE ret = MAX30102_OK;
    uint8_t reset = MAX30102_MODE_CFG_RESET;

    ret = max30102_get_register_value(max30102, MAX30102_REG_MODE_CONFIG, &old_mode_cfg, 1);

    if (ret != MAX30102_OK)
    {
        return ret;
    }

    old_mode_cfg = old_mode_cfg & ~reset;
    if (!enable)
    {
        reset = 0;
    }
    
    new_mode_cfg = reset | old_mode_cfg;
    ret = max30102_set_register_value(max30102, MAX30102_REG_MODE_CONFIG, new_mode_cfg);

    return ret;
}

/**
 * @brief Shutdown mode: power save mode
 * 
 * @param[in] max30102: device handler
 * @param[in] enable: Enable or Disable shutdown
 *                  Example 0x00 - disable
 *                          0x01 - enable
*/
DEVICE_INTF_RET_TYPE max30102_mode_shutdown(device_t *max30102, uint8_t enable)
{
    uint8_t old_mode_cfg, new_mode_cfg;
    DEVICE_INTF_RET_TYPE ret = MAX30102_OK;
    uint8_t shutdown = MAX30102_MODE_CFG_SHDN;

    ret = max30102_get_register_value(max30102, MAX30102_REG_MODE_CONFIG, &old_mode_cfg, 1);

    if (ret != MAX30102_OK)
    {
        return ret;
    }

    old_mode_cfg = old_mode_cfg & ~shutdown;
    if (!enable)
    {
        shutdown = 0;
    }
    
    
    new_mode_cfg = shutdown | old_mode_cfg;
    ret = max30102_set_register_value(max30102, MAX30102_REG_MODE_CONFIG, new_mode_cfg);

    return ret;
}

/**
 * @brief Config spo2
 * 
 * @param[in] max30102: device handler
 * @param[in] led_pw: SpO2 led pulse width - MAX30102_SPO2_CFG_LED_PW(X)
 *                      Options: See MAX30102_SPO2_LED_PW_x for more details
 * 
 * @param[in] sample_rate: SpO2 sample rate - MAX30102_SPO2_CFG_SR(X)
 *                      Options: See MAX30102_SPO2_SR_x for more details
 * 
 * @param[in] adc_rge: ADC range - MAX30102_SPO2_CFG_ADC_REG(X)
 *                  Options: see MAX30102_SPO2_ADC_RGE_x for more detail.
*/
DEVICE_INTF_RET_TYPE max30102_spo2_config(device_t *max30102, uint8_t led_pw, uint8_t sample_rate, uint8_t adc_rge)
{
    uint8_t spo2_config = led_pw | sample_rate | adc_rge;

    return max30102_set_register_value(max30102, MAX30102_REG_SPO2_CONFIG, spo2_config);
}

/**
 * @brief Config spo2 led pulse width
 * 
 * @param[in] max30102: device handler
 * @param[in] led_pw: SpO2 led pulse width - MAX30102_SPO2_CFG_LED_PW(X)
 *                      Options: See MAX30102_SPO2_LED_PW_x for more details
*/
DEVICE_INTF_RET_TYPE max30102_set_spo2_led_pw(device_t *max30102, uint8_t led_pw)
{
    DEVICE_INTF_RET_TYPE ret = MAX30102_OK;
    uint8_t old_led_pw, new_led_pw;

    ret = max30102_get_register_value(max30102, MAX30102_REG_SPO2_CONFIG, &old_led_pw, 1);
    
    if (ret != MAX30102_OK)
    {
        return ret;
    }

    new_led_pw = old_led_pw & 0x7C;
    new_led_pw |= led_pw;

    ret = max30102_set_register_value(max30102, MAX30102_REG_SPO2_CONFIG, new_led_pw);
    
    return ret;
}

/**
 * @brief Config spo2 sample rate
 * 
 * @param[in] max30102: device handler
 * @param[in] sample_rate: SpO2 sample rate - MAX30102_SPO2_CFG_SR(X)
 *                      Options: See MAX30102_SPO2_SR_x for more details
*/
DEVICE_INTF_RET_TYPE max30102_set_spo2_sample_rate(device_t *max30102, uint8_t sample_rate)
{
    DEVICE_INTF_RET_TYPE ret = MAX30102_OK;
    uint8_t old_sr, new_sr;

    ret = max30102_get_register_value(max30102, MAX30102_REG_SPO2_CONFIG, &old_sr, 1);
    
    if (ret != MAX30102_OK)
    {
        return ret;
    }

    new_sr = old_sr & 0x63;
    new_sr |= sample_rate;

    ret = max30102_set_register_value(max30102, MAX30102_REG_SPO2_CONFIG, new_sr);
    
    return ret;
}

/**
 * @brief Config SpO2 ADC range
 * 
 * @param[in] max30102: device handler
 * @param[in] adc_rge: ADC range - MAX30102_SPO2_CFG_ADC_REG(X)
 *                  Options: see MAX30102_SPO2_ADC_RGE_x for more detail.
*/
DEVICE_INTF_RET_TYPE max30102_set_spo2_adc_range(device_t *max30102, uint8_t adc_rge)
{
    DEVICE_INTF_RET_TYPE ret = MAX30102_OK;
    uint8_t old_rge, new_rge;

    ret = max30102_get_register_value(max30102, MAX30102_REG_SPO2_CONFIG, &old_rge, 1);
    if (ret != MAX30102_OK)
    {
        return ret;
    }
    
    new_rge = old_rge & 0x1F;
    new_rge |= adc_rge;
    ret = max30102_set_register_value(max30102, MAX30102_REG_SPO2_CONFIG, new_rge);
    
    return ret;
}

/*! Set LED Pulse Amplitude */
static DEVICE_INTF_RET_TYPE max30102_set_led_pa(device_t *max30102, uint8_t register_address, float current)
{    
    uint8_t reg_val;

    if (current > 51)
    {
        current = 51;
    }
    reg_val  = (uint16_t) (current * 10) / 2;

    return max30102_set_register_value(max30102, register_address, reg_val);
}

/**
 * @brief Set RED LED Pulse Amplitude
 * 
 * @param[in] max30102: device handler
 * @param[in] max30102: LED Pulse Amplitude. Range: 0~51 mA
 * 
*/
DEVICE_INTF_RET_TYPE max30102_set_led_red_pa(device_t *max30102, float current)
{
    return max30102_set_led_pa(max30102, MAX30102_REG_LED1_RED_PA, current);
}

/**
 * @brief Set IR LED Pulse Amplitude
 * 
 * @param[in] max30102: device handler
 * @param[in] max30102: LED Pulse Amplitude. Range: 0~51 mA
 * 
*/
DEVICE_INTF_RET_TYPE max30102_set_led_ir_pa(device_t *max30102, float current)
{
    return max30102_set_led_pa(max30102, MAX30102_REG_LED2_IR_PA, current);
}

/**
 * @brief Set Multi-LED Mode. \n
 * In multi-LED mode, each sample is split into up to four time slots, SLOT1    * through SLOT4. These control registers determine which LED is active in each * time slot, making for a very flexible configuration. \n
 * 
 * Each slot generates a 3-byte output into the FIFO. The slots should be enabled in order.
 * 
 * @param[in] max30102: device handler
 * @param[in] slotx[2:0]: see @ref MAX30102_MLED_MODE_x for detail.
 *                        MAX30102_MLED_MODE_LED1_RED - LED1 LED1_PA
 *                        MAX30102_MLED_MODE_LED2_IR - LED2 LED2_PA
 *                         
*/
DEVICE_INTF_RET_TYPE max30102_set_multi_led_mode(device_t *max30102, uint8_t slot1, uint8_t slot2, uint8_t slot3, uint8_t slot4)
{
    uint8_t time_slot1 = 0, time_slot2 = 0;
    if (slot1 == 0)
    {
        return MAX30102_E_INVALID_LED_SLOT;
    }
    
    time_slot1 = MAX30102_MLED_MODE_CTL_H(slot2) | MAX30102_MLED_MODE_CTL_L(slot1);
    max30102_set_register_value(max30102, MAX30102_REG_MULTI_LED_CTRL1, time_slot1);
    
    if (slot2 != 0 && slot3 != 0)
    {
        time_slot2 = MAX30102_MLED_MODE_CTL_H(slot4) | MAX30102_MLED_MODE_CTL_L(slot3);
        max30102_set_register_value(max30102, MAX30102_REG_MULTI_LED_CTRL1, time_slot2);
    } else {
        return MAX30102_E_INVALID_LED_SLOT;
    }
    
    return MAX30102_OK;
}

/**
 * @brief Get on board temperature ADC output result.
 * 
 * @param[in, out] temp: temp res.
*/
DEVICE_INTF_RET_TYPE max30102_get_temperature(device_t *max30102, float *temp)
{
    uint8_t temp_int, temp_frac;
    DEVICE_INTF_RET_TYPE ret = MAX30102_OK;
    *temp = 0;

    ret = max30102_get_register_value(max30102, MAX30102_REG_TEMP_INTR, &temp_int, 1);
    if (ret != DEVICE_OK)
    {
        return ret;
    }

    ret = max30102_get_register_value(max30102, MAX30102_REG_TEMP_FRAC, &temp_frac, 1);
    if (ret != MAX30102_OK)
    {
        return ret;
    }
    
    *temp = ((int8_t) temp_int) + (0.0625) * temp_frac;

    return MAX30102_OK;
}

/**
 * @brief Enable or Disable temperature measurement
 * 
 * @param enable: Example: MAX30102_DIE_TEMP_ENABLE
 *                         MAX30102_DIE_TEMP_DISABLE
*/
DEVICE_INTF_RET_TYPE max30102_temp_en(device_t *max30102, uint8_t enable)
{   
    if (enable)
    {
        enable = MAX30102_DIE_TEMP_ENABLE;
    }
    
    return max30102_set_register_value(max30102, MAX30102_REG_TEMP_CONFIG, enable);
}
