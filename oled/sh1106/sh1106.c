#include <stdlib.h>
#include "sh1106.h"

#define sh1106_send_data(sh1106, data, len)  sh1106_write(sh1106, data, len, 1)
#define sh1106_send_cmd_seq(sh1106, data, len)  sh1106_write(sh1106, data, len, 0)

static uint8_t **sh1106_gram_create(uint8_t page, uint8_t column);
static void sh1106_gram_destroy(uint8_t **gram, uint8_t page);

static uint8_t **sh1106_gram_create(uint8_t page, uint8_t column)
{
    uint8_t **gram = (uint8_t **)malloc(sizeof(uint8_t *) * page);

    if (gram == NULL)
    {
        return NULL;
    }
    
    for (size_t i = 0; i < page; i++)
    {
        gram[i] = (uint8_t *)malloc(sizeof(uint8_t) * column);
        platform_log("allocate space for gram[%d]\r\n", i);
        if (gram[i] == NULL)
        {
            sh1106_gram_destroy(gram, i);
            gram = NULL;

            break;
        }
        memset(gram[i], 0, sizeof(uint8_t) * column);
    }
    
    return gram;
}

static void sh1106_gram_destroy(uint8_t **gram, uint8_t page)
{
    if (gram == NULL)
    {
        return;
    }
    
    for (size_t i = 0; i < page; i++)
    {
        if (gram[i])
        {
            free(gram[i]);
            gram[i] = NULL;
        }
    }
    free(gram);
}

DEVICE_INTF_RET_TYPE sh1106_null_ptr_check(sh1106_t *sh1106)
{
    if (sh1106 == NULL)
    {
        return SH1106_E_NULLPTR;
    }

    if (device_null_ptr_check(sh1106->reset_pin) != SH1106_OK)
    {
        return SH1106_E_UNINIT_RESET_PIN;
    }

    return device_null_ptr_check(sh1106->dev);
}

DEVICE_INTF_RET_TYPE sh1106_init(sh1106_t *sh1106, uint8_t gram_page, uint8_t gram_column)
{
    device_t *dev = sh1106->dev;
    DEVICE_INTF_RET_TYPE ret = SH1106_OK;
    uint8_t sh1106_init_sequence[] = {0xAE, 0x00, 0x10, 0x40, 0xB0, 0x81, 0xEF, 0xA1, 0xC8, 0xA8, 0X3F, 0xD3, 0X00, 0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB, 0x30, 0x8D, 0x14, 0xA4, 0xA6};

    ret = sh1106_null_ptr_check(sh1106);
    if (ret != SH1106_OK)
    {
        return ret;
    }

    if (dev->info == NULL)
    {
        return SH1106_E_UNINIT_INTERFACE;
    }

    sh1106->gram_page = gram_page;
    sh1106->gram_column = gram_column;
    // if (sh1106->gram == NULL)
    // {
    //     sh1106->gram = sh1106_gram_create(gram_page, gram_column);
    //     if (sh1106->gram == NULL)
    //     {
    //         return SH1106_E_HEAP_OVERFLOW;
    //     }
    //     sh1106->gram_page = gram_page;
    //     sh1106->gram_column = gram_column;
    // }
    
    // oled config
    sh1106_reset(sh1106->reset_pin);
	sh1106_send_cmd_seq(sh1106, sh1106_init_sequence, sizeof(sh1106_init_sequence));
    sh1106_new_frame(sh1106);
    sh1106_show_frame(sh1106);
    sh1106_send_cmd(sh1106, 0xAF);

	return ret;
}

DEVICE_INTF_RET_TYPE sh1106_interface_init(sh1106_t *sh1106, device_t *dev, device_t *reset_pin, device_t *dc_pin, device_type_t dtype)
{
    char *interface = NULL;
    DEVICE_INTF_RET_TYPE ret = sh1106_null_ptr_check(sh1106);
    if (ret == SH1106_E_NULLPTR)
        return ret;

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

DEVICE_INTF_RET_TYPE sh1106_reset(device_t *reset_pin)
{
    DEVICE_INTF_RET_TYPE ret = device_null_ptr_check(reset_pin);
    if (ret != SH1106_OK)
    {
        return ret;
    }

    platform_delay_ms(100);
    device_write_byte(reset_pin, 0);
    platform_delay_ms(100);
    device_write_byte(reset_pin, 1);

    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_send_cmd(sh1106_t *sh1106, uint8_t data)
{
    return sh1106_write(sh1106, &data, 1, 0);
}

DEVICE_INTF_RET_TYPE sh1106_write(sh1106_t *sh1106, uint8_t *data, uint16_t len, uint8_t cmd)
{
	device_t *dev = sh1106->dev;
	device_t *nss_pin = (device_t *)dev->addr;
	device_t *dc_pin = sh1106->dc_pin;
    DEVICE_INTF_RET_TYPE ret;

    ret = device_write_byte(dc_pin, cmd);
    ret = device_write_byte(nss_pin, 0);
    ret = device_write(dev, data, len);
    ret = device_write_byte(nss_pin, 1);
    ret = device_write_byte(dc_pin, 1);
	
	return ret;
}

void sh1106_new_frame(sh1106_t *sh1106)
{
    memset(sh1106->gram, 0, sizeof(uint8_t)*sh1106->gram_page*sh1106->gram_column);
}

void sh1106_show_frame(sh1106_t *sh1106)
{
    for (size_t i = 0; i < sh1106->gram_page; i++)
    {
        uint8_t cmd_seq[3] = {0xB0 + i, 0x02, 0x10};
        sh1106_send_cmd_seq(sh1106, cmd_seq, 3);
        if(sh1106->gram[i]) {
			sh1106_send_data(sh1106, sh1106->gram[i], sh1106->gram_column);
		} else {
			platform_log("gram[%d] is null\r\n", i);
		}
    }
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

DEVICE_INTF_RET_TYPE sh1106_set_color_mode(sh1106_t *sh1106, SH1106_ColorMode mode)
{
    return sh1106_send_cmd(sh1106, mode);
}

DEVICE_INTF_RET_TYPE sh1106_set_pixel(sh1106_t *sh1106, uint8_t x, uint8_t y, SH1106_ColorMode mode)
{
    DEVICE_INTF_RET_TYPE ret = sh1106_null_ptr_check(sh1106);

    if (ret != DEVICE_OK)
    {
        return ret;
    }

    if (x >= SH1106_COLUMN || y >= SH1106_ROW)
    {
        return SH1106_E_HAL_OOB;
    }

    if (mode & 0x01)
    {
       sh1106->gram[y / 8][x] &= ~(1 << (y % 8));
    }
    else
    {
       sh1106->gram[y / 8][x] |= 1 << (y % 8);
    }

    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_set_byte_fine(sh1106_t *sh1106, uint8_t page, uint8_t column, uint8_t data, uint8_t start, uint8_t end, SH1106_ColorMode mode)
{
    DEVICE_INTF_RET_TYPE ret = sh1106_null_ptr_check(sh1106);
    static uint8_t temp;

    if (ret != SH1106_OK)
    {
        return ret;
    }

    if (page >= SH1106_PAGE || column >= SH1106_COLUMN)
    {
        return SH1106_E_HAL_OOB;
    }

    if (mode & 0x01)
        data = ~data;

    temp = data | (0xff << (end + 1)) | (0xff >> (8 - start));
    sh1106->gram[page][column] &= temp;
    temp = data & ~(0xff << (end + 1)) & ~(0xff >> (8 - start));
    sh1106->gram[page][column] |= temp;

    return SH1106_OK;
}

DEVICE_INTF_RET_TYPE sh1106_set_byte(sh1106_t *sh1106, uint8_t page, uint8_t column, uint8_t data, SH1106_ColorMode mode)
{
    DEVICE_INTF_RET_TYPE ret = sh1106_null_ptr_check(sh1106);

    if (ret != SH1106_OK)
    {
        return ret;
    }

    if (page >= SH1106_PAGE || column >= SH1106_COLUMN)
    {
        return SH1106_E_HAL_OOB;
    }

    if (mode & 0x01)
        data = ~data;
    sh1106->gram[page][column] = data;

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
	DEVICE_INTF_RET_TYPE ret = sh1106_set_byte_fine(sh1106, page, x, data << bit, bit, 7, mode);
    if (bit)
    {
        ret = sh1106_set_byte_fine(sh1106, page + 1, x, data >> (8 - bit), 0, bit - 1, mode);
    }

    return ret;
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
