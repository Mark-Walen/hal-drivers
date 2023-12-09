#ifndef __FONT_H__
#define __FONT_H__

#include "stdint.h"
#include "string.h"

typedef struct ascii_font_s ascii_font_t;
typedef struct font_s font_t;
typedef struct image_s image_t;

struct ascii_font_s {
  uint8_t h;
  uint8_t w;
  uint8_t *chars;
};

extern const ascii_font_t afont8x6;
extern const ascii_font_t afont12x6;
extern const ascii_font_t afont16x8;
extern const ascii_font_t afont24x12;

/**
 * @brief 字体结构体
 * @note  字库前4字节存储utf8编码 剩余字节存储字模数据
 * @note 字库数据可以使用波特律动LED取模助手生成(https://led.baud-dance.com)
 */
struct font_s {
  uint8_t h;              // 字高度
  uint8_t w;              // 字宽度
  const uint8_t *chars;   // 字库 字库前4字节存储utf8编码 剩余字节存储字模数据
  uint8_t len;            // 字库长度 超过 255 则请改为 uint16_t
  const ascii_font_t *ascii; // 缺省ASCII字体 当字库中没有对应字符且需要显示ASCII字符时使用
};

extern const font_t font16x16;

/**
 * @brief 图片结构体
 * @note  图片数据可以使用波特律动LED取模助手生成(https://led.baud-dance.com)
 */
struct image_s{
  uint8_t w;           // 图片宽度
  uint8_t h;           // 图片高度
  const uint8_t *data; // 图片数据
};

extern const image_t bilibili_img;

#endif // __FONT_H
