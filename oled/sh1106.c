#include <stdlib.h>
#include "sh1106.h"

DEVICE_INTF_RET_TYPE sh1106_null_ptr_check(sh1106_t *sh1106)
{
    sh1106_attr_t *attr = sh1106->sh1106_attr;
    sh1106_gpu_t *gpu = attr->sh1106_gpu;

    if (sh1106 == NULL)
    {
        return SH1106_E_NULLPTR;
    }

    if (attr == NULL || gpu == NULL || gpu->gpu == NULL)
    {
        return SH1106_E_UNINIT_GPU;
    }

    if (device_null_ptr_check(sh1106->reset_pin) != SH1106_OK)
    {
        return SH1106_E_UNINIT_RESET_PIN;
    }

    return device_null_ptr_check(sh1106->dev);
}

DEVICE_INTF_RET_TYPE sh1106_reset(sh1106_t *sh1106)
{
    uint8_t pin_level = 0;
    device_t *reset_pin = sh1106->reset_pin;
    platform_t *platform = get_platform();
    DEVICE_INTF_RET_TYPE ret = sh1106_null_ptr_check(sh1106);
    if (ret != SH1106_OK)
    {
        return ret;
    }

    platform->delay_us(100000);
    reset_pin->write(&pin_level, 1, reset_pin->fp, reset_pin->addr);
    platform->delay_us(100000);
    pin_level = 1;
    reset_pin->write(&pin_level, 1, reset_pin->fp, reset_pin->addr);

    return DEVICE_OK;
}

static DEVICE_INTF_RET_TYPE sh1106_dc_pin_write(sh1106_t *sh1106, uint8_t pin_level)
{
    device_t *dc_pin = sh1106->dc_pin;
    platform_t *platform = get_platform();
    DEVICE_INTF_RET_TYPE ret = sh1106_null_ptr_check(sh1106);
    if (ret != SH1106_OK)
    {
        return ret;
    }

    ret = device_null_ptr_check(dc_pin);
    if (ret != SH1106_OK)
    {
        return SH1106_E_UNINIT_DC_PIN;
    }

    dc_pin->write(&pin_level, 1, dc_pin->fp, dc_pin->addr);

    return DEVICE_OK;
}

DEVICE_INTF_RET_TYPE sh1106_init(sh1106_t *sh1106)
{
    device_t *dev = sh1106->dev;
    DEVICE_INTF_RET_TYPE ret = SH1106_OK;

    ret = sh1106_null_ptr_check(sh1106);
    if (ret != SH1106_OK)
    {
        return ret;
    }

    if (dev->info == NULL)
    {
        return SH1106_E_UNINIT_INTERFACE;
    }

    sh1106_reset(sh1106);

    sh1106_send_cmd(sh1106, SH1106_DISPLAY_OFF);

    sh1106_send_cmd(sh1106, SH1106_LOWER_COLUMN_ADDRESS(0x02));  // set lower column address
    sh1106_send_cmd(sh1106, SH1106_HIGHER_COLUMN_ADDRESS(0x10)); // set higher column address

    sh1106_send_cmd(sh1106, SH1106_DISPLAY_START_LINE(0x40)); // set display start line

    sh1106_send_cmd(sh1106, SH1106_CONTRAST_CTL_MODE);       // set contrast
    sh1106_send_cmd(sh1106, SH1106_CONTRAST_DATA_REG(0xCF)); // set contrast value: 128

    sh1106_send_cmd(sh1106, SH1106_SEG_REMAP_H);    // set segment remap
    sh1106_send_cmd(sh1106, SH1106_NORMAL_DISPLAY); // set normal display

    sh1106_send_cmd(sh1106, SH1106_MUL_RATION_MODE);       // set mutiplex ratio
    sh1106_send_cmd(sh1106, SH1106_MUL_RATION_DATA(0x3F)); // duty = 1/64

    sh1106_send_cmd(sh1106, SH1106_DC_DC_CTL_MODE); // set charge pump enable
    sh1106_send_cmd(sh1106, SH1106_DC_DC_MODE_ON);  // set enable DC-DC

    sh1106_send_cmd(sh1106, SH1106_PUMP_VOLTAGE(SH1106_VPP_9_0)); // set charge pump enable

    sh1106_send_cmd(sh1106, SH1106_OUTPUT_SCAN_DIR_RL); // set output scan direction right to left

    sh1106_send_cmd(sh1106, SH1106_DISPLAY_OFFSET_MODE);  // set display offset
    sh1106_send_cmd(sh1106, SH1106_DISPLAY_OFFSET(0x00)); // set display offset: 0

    sh1106_send_cmd(sh1106, SH1106_DIVIDE_RATIO_OSC_FREQ); // set osc frequency
    sh1106_send_cmd(sh1106, SH1106_OSC_FREQ(0x0C));        // osc freq: 35%

    sh1106_send_cmd(sh1106, SH1106_CHARGE_PERIOD);                              // set charge period
    sh1106_send_cmd(sh1106, SH1106_PRE_CHARGE(0x0F) | SH1106_DIS_CHARGE(0x01)); // pre charge: 15 DCLK, dis charge: 1 DCLK

    sh1106_send_cmd(sh1106, SH1106_PAD_CFG_MODE); // set COM pins
    sh1106_send_cmd(sh1106, SH1106_ALT_MODE);

    sh1106_send_cmd(sh1106, SH1106_VCOM_DESELECT_LVL_MODE); // set VCOM level
    sh1106_send_cmd(sh1106, SH1106_VCOM_DESELECT_LVL(0x40));

    sh1106_new_frame(sh1106);
    sh1106_show_frame(sh1106);

    sh1106_send_cmd(sh1106, SH1106_DISPLAY_ON);

    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_send_data(sh1106_t *sh1106, uint8_t *data, uint16_t len)
{
    device_type_t dtype;
    device_t *dev = NULL;
    uint8_t cmd = (data[0] != 0);
    DEVICE_INTF_RET_TYPE ret = sh1106_null_ptr_check(sh1106);

    if (ret != SH1106_OK)
    {
        return ret;
    }

    dev = sh1106->dev;
    get_device_info(dev, "%t", &dtype);

    if (dtype == SPI)
    {
        device_t *nss = (device_t *)dev->addr;
        uint8_t gpio_level = 0;
        ret = device_null_ptr_check(nss);
        if (ret != DEVICE_OK)
        {
            return SH1106_E_NULLPTR;
        }

        data++;
        len--;
        sh1106_dc_pin_write(sh1106, cmd);
        nss->write(&gpio_level, 1, nss->fp, nss->addr);
        ret = dev->write(data, len, dev->fp, dev->addr);
        gpio_level = 1;
        nss->write(&gpio_level, 1, nss->fp, nss->addr);
        sh1106_dc_pin_write(sh1106, 1);
    }
    else
    {
        ret = dev->write(data, len, dev->fp, dev->addr);
    }

    return ret;
}

DEVICE_INTF_RET_TYPE sh1106_send_cmd(sh1106_t *sh1106, uint8_t cmd)
{
    static uint8_t send_buffer[2] = {0};

    send_buffer[1] = cmd;
    return sh1106_send_data(sh1106, send_buffer, 2);
}

DEVICE_INTF_RET_TYPE sh1106_write_data(sh1106_t *sh1106, uint8_t data);
DEVICE_INTF_RET_TYPE sh1106_read_data(sh1106_t *sh1106, uint8_t *data);
DEVICE_INTF_RET_TYPE sh1106_display_on(sh1106_t *sh1106);

sh1106_attr_t *sh1106_attr_create(uint32_t page, uint32_t column)
{
    sh1106_attr_t *attr = (sh1106_attr_t *) malloc(sizeof(sh1106_attr_t));

    if (attr == NULL)
    {
        return NULL;
    }

    attr->page = page;
    attr->row = page * 8;
    attr->column = column;
    attr->sh1106_gpu = NULL;

    return attr;
}

void sh1106_attr_destroy(sh1106_attr_t *attr)
{
    if (attr == NULL)
    {
        return;
    }

    if (attr->sh1106_gpu != NULL)
    {
        sh1106_gpu_destroy(attr->sh1106_gpu, attr->page);
    }
    free(attr);
}

sh1106_gpu_t *sh1106_gpu_create(uint32_t page, uint32_t column)
{
    sh1106_gpu_t *gpu = NULL;
    platform_t *plt = get_platform();

    if (page == 0 || column == 0)
    {
        return NULL;
    }

    gpu = (sh1106_gpu_t *)malloc(sizeof(sh1106_gpu_t));
    if (gpu == NULL)
    {
        return NULL;
    }

    // gpu->gpu = (uint8_t **)malloc(sizeof(uint8_t *) * page);
    // if (gpu->gpu == NULL)
    // {
    //     free(gpu);
    //     return NULL;
    // }

    // for (size_t i = 0; i < page; i++)
    // {
    //     gpu->gpu[i] = (uint8_t *)malloc(sizeof(uint8_t) * column);
    //     plt->println("%d\r\n", i);
    //     if (gpu->gpu[i] == NULL)
    //     {
    //         sh1106_gpu_destroy(gpu, i);
    //         return NULL;
    //     }
    // }

    return gpu;
}

void sh1106_gpu_destroy(sh1106_gpu_t *gpu, uint32_t page)
{
    if (gpu == NULL || page == 0)
    {
        return;
    }

    for (size_t i = 0; i < page; i++)
    {
        if (gpu->gpu[i] != NULL)
        {
            free(gpu->gpu[i]);
        }
    }

    free(gpu->gpu);
    free(gpu);
}

sh1106_gpu_t *sh1106_get_gpu(sh1106_t *sh1106)
{
    return sh1106->sh1106_attr->sh1106_gpu;
}
/**
 * @brief Init sh1106 oled gpu
 */
DEVICE_INTF_RET_TYPE sh1106_attr_init(sh1106_t *sh1106, uint32_t page, uint32_t column)
{
    sh1106_attr_t *attr = NULL;
    sh1106_gpu_t *gpu = NULL;
    platform_t *plt = get_platform();

    if (sh1106 == NULL)
    {
        return SH1106_E_NULLPTR;
    }

    if (page == 0 || column == 0)
    {
        return SH1106_E_INVALID_PARAM;
    }

    attr = sh1106_attr_create(page, column);
    if (attr == NULL)
    {
        return SH1106_E_HEAP_OVERFLOW;
    }
    plt->println("%p\r\n", attr);

    // Init gpu
    gpu = sh1106_gpu_create(attr->page, attr->column);
    if (gpu == NULL)
    {
        sh1106_attr_destroy(attr);
        return SH1106_E_HEAP_OVERFLOW;
    }
    plt->println("%p\r\n", attr);

    sh1106->sh1106_attr->sh1106_gpu = gpu;
    sh1106->sh1106_attr = attr;
    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_interface_init(sh1106_t *sh1106, device_t *dev, device_t *reset_pin, device_t *dc_pin, device_type_t dtype, uint32_t page, uint32_t column)
{
    char *interface = NULL;
    DEVICE_INTF_RET_TYPE ret = sh1106_null_ptr_check(sh1106);
    if (ret == SH1106_E_NULLPTR)
        return ret;

    if (ret == SH1106_E_UNINIT_GPU)
    {
        ret = sh1106_attr_init(sh1106, page, column);
        if (ret != SH1106_OK)
        {
            return ret;
        }
    }

    ret = device_null_ptr_check(reset_pin);
    if (ret != DEVICE_OK)
        return ret;

    ret = device_null_ptr_check(dc_pin);
    if (dtype == SPI && ret != DEVICE_OK)
        return ret;

    switch (dtype)
    {
    case SPI:
        interface = "SPI";
        break;

    case I2C:
    default:
        dtype = I2C;
        interface = "I2C";
        break;
    }

    config_device_info(dev, "%i%t", interface, dtype);
    sh1106->dev = dev;
    sh1106->reset_pin = reset_pin;
    sh1106->dc_pin = dc_pin;

    return DEVICE_OK;
}

DEVICE_INTF_RET_TYPE sh1106_display_on(sh1106_t *sh1106)
{
    sh1106_send_cmd(sh1106, 0x8D);
    sh1106_send_cmd(sh1106, 0x14);
    sh1106_send_cmd(sh1106, SH1106_DISPLAY_ON);

    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_display_off(sh1106_t *sh1106)
{
    sh1106_send_cmd(sh1106, SH1106_DISPLAY_OFF);
    sh1106_send_cmd(sh1106, 0x8D);
    sh1106_send_cmd(sh1106, 0x10);

    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_new_frame(sh1106_t *sh1106)
{
    sh1106_attr_t *attr = sh1106->sh1106_attr;
    sh1106_gpu_t *gpu = attr->sh1106_gpu;
    DEVICE_INTF_RET_TYPE ret = sh1106_null_ptr_check(sh1106);

    if (ret != DEVICE_OK)
    {
        return ret;
    }

    memset(gpu->gpu, 0, sizeof(uint8_t) * attr->page * attr->column);
    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_show_frame(sh1106_t *sh1106)
{
    sh1106_attr_t *attr = sh1106->sh1106_attr;
    sh1106_gpu_t *gpu = attr->sh1106_gpu;
    DEVICE_INTF_RET_TYPE ret = sh1106_null_ptr_check(sh1106);
    static uint8_t *send_buffer = NULL;
    static uint8_t send_buffer_size = 0;

    if (ret != SH1106_OK)
    {
        return ret;
    }

    if (send_buffer == NULL)
    {
        send_buffer_size = attr->column + 1;
        send_buffer = malloc(sizeof(uint8_t) * send_buffer_size);
        send_buffer[0] = 0x40;
    }

    for (size_t i = 0; i < attr->page; i++)
    {
        sh1106_send_cmd(sh1106, 0xB0 + i);
        sh1106_send_cmd(sh1106, SH1106_LOWER_COLUMN_ADDRESS(0x02));  // set lower column address
        sh1106_send_cmd(sh1106, SH1106_HIGHER_COLUMN_ADDRESS(0x10)); // set higher column address
        memcpy(send_buffer + 1, gpu->gpu[i], attr->column);
        sh1106_send_data(sh1106, send_buffer, send_buffer_size);
    }
    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_set_color_mode(sh1106_t *sh1106, SH1106_ColorMode mode)
{
    return sh1106_send_cmd(sh1106, mode);
}

DEVICE_INTF_RET_TYPE sh1106_set_pixel(sh1106_t *sh1106, uint8_t x, uint8_t y, SH1106_ColorMode mode)
{
    sh1106_attr_t *attr = sh1106->sh1106_attr;
    sh1106_gpu_t *gpu = attr->sh1106_gpu;
    DEVICE_INTF_RET_TYPE ret = sh1106_null_ptr_check(sh1106);

    if (ret != DEVICE_OK)
    {
        return ret;
    }

    if (x >= attr->column || y >= attr->row)
    {
        return SH1106_E_HAL_OOB;
    }

    if (mode & 0x01)
    {
        gpu->gpu[y / 8][x] &= ~(1 << (y % 8));
    }
    else
    {
        gpu->gpu[y / 8][x] |= 1 << (y % 8);
    }

    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_set_byte_fine(sh1106_t *sh1106, uint8_t page, uint8_t column, uint8_t data, uint8_t start, uint8_t end, SH1106_ColorMode mode)
{
    sh1106_attr_t *attr = sh1106->sh1106_attr;
    sh1106_gpu_t *gpu = attr->sh1106_gpu;
    DEVICE_INTF_RET_TYPE ret = sh1106_null_ptr_check(sh1106);
    static uint8_t temp;

    if (ret != SH1106_OK)
    {
        return ret;
    }

    if (page >= attr->page || column >= attr->column)
    {
        return SH1106_E_HAL_OOB;
    }

    if (mode & 0x01)
        data = ~data;

    temp = data | (0xff << (end + 1)) | (0xff >> (8 - start));
    gpu->gpu[page][column] &= temp;
    temp = data & ~(0xff << (end + 1)) & ~(0xff >> (8 - start));
    gpu->gpu[page][column] |= temp;

    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_set_byte(sh1106_t *sh1106, uint8_t page, uint8_t column, uint8_t data, SH1106_ColorMode mode)
{
    sh1106_attr_t *attr = sh1106->sh1106_attr;
    sh1106_gpu_t *gpu = attr->sh1106_gpu;
    DEVICE_INTF_RET_TYPE ret = sh1106_null_ptr_check(sh1106);

    if (ret != SH1106_OK)
    {
        return ret;
    }

    if (page >= attr->page || column >= attr->column)
    {
        return SH1106_E_HAL_OOB;
    }

    if (mode & 0x01)
        data = ~data;
    gpu->gpu[page][column] = data;

    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_set_bits_fine(sh1106_t *sh1106, uint8_t x, uint8_t y, uint8_t data, uint8_t len, SH1106_ColorMode mode)
{
    uint8_t page = y / 8;
    uint8_t bit = y % 8;
    DEVICE_INTF_RET_TYPE ret = SH1106_OK;

    if (bit + len > 8)
    {
        ret = sh1106_set_byte_fine(sh1106, page, x, data, bit, 7, mode);
        if (ret != SH1106_OK)
        {
            return ret;
        }

        return sh1106_set_byte_fine(sh1106, page + 1, x, data >> (8 - bit), 0, len + bit - 9, mode);
    }

    return sh1106_set_byte_fine(sh1106, page, x, data << bit, bit, bit + len - 1, mode);
}

DEVICE_INTF_RET_TYPE sh1106_set_bits(sh1106_t *sh1106, uint8_t x, uint8_t y, uint8_t data, SH1106_ColorMode mode)
{
    uint8_t page = y / 8;
    uint8_t bit = y % 8;

    if (bit)
    {
        return sh1106_set_byte_fine(sh1106, page + 1, x, data >> (8 - bit), 0, bit - 1, mode);
    }

    return sh1106_set_byte_fine(sh1106, page, x, data << bit, bit, 7, mode);
}

DEVICE_INTF_RET_TYPE sh1106_set_block(sh1106_t *sh1106, uint8_t x, uint8_t y, const uint8_t *data, uint8_t w, uint8_t h, SH1106_ColorMode mode)
{
    uint8_t full_row = h / 8; // integrity row
    uint8_t part_bit = h % 8; // remain bits
    DEVICE_INTF_RET_TYPE ret = SH1106_OK;

    for (uint8_t i = 0; i < w; i++)
    {
        for (uint8_t j = 0; j < full_row; j++)
        {
            ret = sh1106_set_bits(sh1106, x + i, y + j * 8, data[i + j * w], mode);
            if (ret != SH1106_OK)
                return ret;
        }
    }
    if (part_bit)
    {
        uint16_t full_num = w * full_row; // full bytes
        for (uint8_t i = 0; i < w; i++)
        {
            ret = sh1106_set_bits_fine(sh1106, x + i, y + (full_row * 8), data[full_num + i], part_bit, mode);
            if (ret != SH1106_OK)
                return ret;
        }
    }

    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_draw_line(sh1106_t *sh1106, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SH1106_ColorMode mode)
{
    static uint8_t temp = 0;
    if (x1 == x2)
    {
        if (y1 > y2)
        {
            temp = y1;
            y1 = y2;
            y2 = temp;
        }
        for (uint8_t y = y1; y <= y2; y++)
        {
            sh1106_set_pixel(sh1106, x1, y, mode);
        }
    }
    else if (y1 == y2)
    {
        if (x1 > x2)
        {
            temp = x1;
            x1 = x2;
            x2 = temp;
        }
        for (uint8_t x = x1; x <= x2; x++)
        {
            sh1106_set_pixel(sh1106, x, y1, mode);
        }
    }
    else
    {
        // Bresenham直线算法
        int16_t dx = x2 - x1;
        int16_t dy = y2 - y1;
        int16_t ux = ((dx > 0) << 1) - 1;
        int16_t uy = ((dy > 0) << 1) - 1;
        int16_t x = x1, y = y1, eps = 0;
        dx = abs(dx);
        dy = abs(dy);
        if (dx > dy)
        {
            for (x = x1; x != x2; x += ux)
            {
                sh1106_set_pixel(sh1106, x, y, mode);
                eps += dy;
                if ((eps << 1) >= dx)
                {
                    y += uy;
                    eps -= dx;
                }
            }
        }
        else
        {
            for (y = y1; y != y2; y += uy)
            {
                sh1106_set_pixel(sh1106, x, y, mode);
                eps += dx;
                if ((eps << 1) >= dy)
                {
                    x += ux;
                    eps -= dy;
                }
            }
        }
    }
    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_draw_rectangle(sh1106_t *sh1106, uint8_t x, uint8_t y, uint8_t w, uint8_t h, SH1106_ColorMode mode)
{
    sh1106_draw_line(sh1106, x, y, x + w, y, mode);
    sh1106_draw_line(sh1106, x, y + h, x + w, y + h, mode);
    sh1106_draw_line(sh1106, x, y, x, y + h, mode);
    sh1106_draw_line(sh1106, x + w, y, x + w, y + h, mode);

    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_draw_filled_rectangle(sh1106_t *sh1106, uint8_t x, uint8_t y, uint8_t w, uint8_t h, SH1106_ColorMode mode)
{
    for (uint8_t i = 0; i < h; i++)
    {
        sh1106_draw_line(sh1106, x, y + i, x + w, y + i, mode);
    }

    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_draw_triangle(sh1106_t *sh1106, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, SH1106_ColorMode mode)
{
    sh1106_draw_line(sh1106, x1, y1, x2, y2, mode);
    sh1106_draw_line(sh1106, x2, y2, x3, y3, mode);
    sh1106_draw_line(sh1106, x3, y3, x1, y1, mode);
	
	return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_draw_filled_triangle(sh1106_t *sh1106, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, SH1106_ColorMode mode)
{
    uint8_t a = 0, b = 0, y = 0, last = 0;
    if (y1 > y2)
    {
        a = y2;
        b = y1;
    }
    else
    {
        a = y1;
        b = y2;
    }
    y = a;
    for (; y <= b; y++)
    {
        if (y <= y3)
        {
            sh1106_draw_line(sh1106, x1 + (y - y1) * (x2 - x1) / (y2 - y1), y, x1 + (y - y1) * (x3 - x1) / (y3 - y1), y, mode);
        }
        else
        {
            last = y - 1;
            break;
        }
    }
    for (; y <= b; y++)
    {
        sh1106_draw_line(sh1106, x2 + (y - y2) * (x3 - x2) / (y3 - y2), y, x1 + (y - last) * (x3 - x1) / (y3 - last), y, mode);
    }
    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_draw_circle(sh1106_t *sh1106, uint8_t x, uint8_t y, uint8_t r, SH1106_ColorMode mode)
{
    int16_t a = 0, b = r, di = 3 - (r << 1);
    while (a <= b)
    {
        sh1106_set_pixel(sh1106, x - b, y - a, mode);
        sh1106_set_pixel(sh1106, x + b, y - a, mode);
        sh1106_set_pixel(sh1106, x - a, y + b, mode);
        sh1106_set_pixel(sh1106, x - b, y - a, mode);
        sh1106_set_pixel(sh1106, x - a, y - b, mode);
        sh1106_set_pixel(sh1106, x + b, y + a, mode);
        sh1106_set_pixel(sh1106, x + a, y - b, mode);
        sh1106_set_pixel(sh1106, x + a, y + b, mode);
        sh1106_set_pixel(sh1106, x - b, y + a, mode);
        a++;
        if (di < 0)
        {
            di += 4 * a + 6;
        }
        else
        {
            di += 10 + 4 * (a - b);
            b--;
        }
        sh1106_set_pixel(sh1106, x + a, y + b, mode);
    }
    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_draw_filled_circle(sh1106_t *sh1106, uint8_t x, uint8_t y, uint8_t r, SH1106_ColorMode mode)
{
    int16_t a = 0, b = r, di = 3 - (r << 1);
    while (a <= b)
    {
        for (int16_t i = x - b; i <= x + b; i++)
        {
            sh1106_set_pixel(sh1106, i, y + a, mode);
            sh1106_set_pixel(sh1106, i, y - a, mode);
        }
        for (int16_t i = x - a; i <= x + a; i++)
        {
            sh1106_set_pixel(sh1106, i, y + b, mode);
            sh1106_set_pixel(sh1106, i, y - b, mode);
        }
        a++;
        if (di < 0)
        {
            di += 4 * a + 6;
        }
        else
        {
            di += 10 + 4 * (a - b);
            b--;
        }
    }
    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_draw_image(sh1106_t *sh1106, uint8_t x, uint8_t y, const image_t *img, SH1106_ColorMode mode)
{
    return sh1106_set_block(sh1106, x, y, img->data, img->w, img->h, mode);
}

DEVICE_INTF_RET_TYPE sh1106_print_ascii_char(sh1106_t *sh1106, uint8_t x, uint8_t y, char ch, const ascii_font_t *font, SH1106_ColorMode mode)
{
    return sh1106_set_block(sh1106, x, y, font->chars + (ch - ' ') * (((font->h + 7) / 8) * font->w), font->w, font->h, mode);
}

DEVICE_INTF_RET_TYPE sh1106_print_ascii_string(sh1106_t *sh1106, uint8_t x, uint8_t y, char *str, const ascii_font_t *font, SH1106_ColorMode mode)
{
    uint8_t x0 = x;
    while (*str)
    {
        sh1106_print_ascii_char(sh1106, x0, y, *str, font, mode);
        x0 += font->w;
        str++;
    }

    return SH1106_OK;
}

static uint8_t _sh1106_get_utf8_len(char *string)
{
    if ((string[0] & 0x80) == 0x00)
    {
        return 1;
    }
    else if ((string[0] & 0xE0) == 0xC0)
    {
        return 2;
    }
    else if ((string[0] & 0xF0) == 0xE0)
    {
        return 3;
    }
    else if ((string[0] & 0xF8) == 0xF0)
    {
        return 4;
    }
    return 0;
}

DEVICE_INTF_RET_TYPE sh1106_print_string(sh1106_t *sh1106, uint8_t x, uint8_t y, char *str, const font_t *font, SH1106_ColorMode mode)
{
    uint16_t i = 0;                                       // 字符串索引
    uint8_t oneLen = (((font->h + 7) / 8) * font->w) + 4; // 一个字模占多少字节
    uint8_t found;                                        // 是否找到字模
    uint8_t utf8Len;                                      // UTF-8编码长度
    uint8_t *head;                                        // 字模头指针
    while (str[i])
    {
        found = 0;
        utf8Len = _sh1106_get_utf8_len(str + i);
        if (utf8Len == 0)
            break; // 有问题的UTF-8编码

        // 寻找字符  TODO 优化查找算法, 二分查找或者hash
        for (uint8_t j = 0; j < font->len; j++)
        {
            head = (uint8_t *)(font->chars) + (j * oneLen);
            if (memcmp(str + i, head, utf8Len) == 0)
            {
                sh1106_set_block(sh1106, x, y, head + 4, font->w, font->h, mode);
                // 移动光标
                x += font->w;
                i += utf8Len;
                found = 1;
                break;
            }
        }

        // 若未找到字模,且为ASCII字符, 则缺省显示ASCII字符
        if (found == 0)
        {
            if (utf8Len == 1)
            {
                sh1106_print_ascii_char(sh1106, x, y, str[i], font->ascii, mode);
                // 移动光标
                x += font->ascii->w;
                i += utf8Len;
            }
            else
            {
                sh1106_print_ascii_char(sh1106, x, y, ' ', font->ascii, mode);
                x += font->ascii->w;
                i += utf8Len;
            }
        }
    }
    return SH1106_OK;
}
