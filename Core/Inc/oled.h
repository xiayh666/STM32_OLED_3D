#ifndef __OLED_H
#define __OLED_H
#include "i2c.h"
#include "stm32_hal_legacy.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_i2c.h"
#include <stdlib.h>
#include <string.h>

#define OLED_ADDRESS 0x78

void OLED_SendCmd(uint8_t cmd) {
  uint8_t buffer[2];
  buffer[0] = 0x00;
  buffer[1] = cmd;
  HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, buffer, 2, HAL_MAX_DELAY);
}

void OLED_Init() {
  OLED_SendCmd(0xAE); // 1. 显示关闭
  OLED_SendCmd(0xD5); // 2. 设置显示时钟分频/振荡频率
  OLED_SendCmd(0x80);
  OLED_SendCmd(0xA8);
  OLED_SendCmd(0x3F); // 64行 (0x3F)
  OLED_SendCmd(0xD3); // 4. 设置显示偏移
  OLED_SendCmd(0x00);
  OLED_SendCmd(0x40);
  OLED_SendCmd(0x8D); // 6. 电荷泵设置
  OLED_SendCmd(0x14); // 开启电荷泵
  OLED_SendCmd(0x20); // 7. 内存地址模式
  OLED_SendCmd(0x02); // 页地址模式 (Page Mode)
  OLED_SendCmd(0xA1); // 8. 列地址重映射 (A0->A1 左右翻转)
  OLED_SendCmd(0xC8); // 9. 行扫描方向翻转 (上下翻转)
  OLED_SendCmd(0xDA); // 10. COM 引脚配置
  OLED_SendCmd(0x12); // COM配置 (0x12 for 64行)
  OLED_SendCmd(0x81); // 11. 对比度设置
  OLED_SendCmd(0x7F); // 对比度值 (0x7F)
  OLED_SendCmd(0xD9); // 12. 预充电周期
  OLED_SendCmd(0xF1);
  OLED_SendCmd(0xDB);
  OLED_SendCmd(0x40);
  OLED_SendCmd(0xA4);
  OLED_SendCmd(0xA6);
  OLED_SendCmd(0xAF);
}
void OLED_Init2() {
  OLED_SendCmd(0xAE);

  OLED_SendCmd(0x02);
  OLED_SendCmd(0x10);

  OLED_SendCmd(0x40);

  OLED_SendCmd(0xB0);

  OLED_SendCmd(0x81);
  OLED_SendCmd(0xCF);

  OLED_SendCmd(0xA1);

  OLED_SendCmd(0xA6);

  OLED_SendCmd(0xA8);
  OLED_SendCmd(0x3F);

  OLED_SendCmd(0xAD);
  OLED_SendCmd(0x8B);

  OLED_SendCmd(0x33);

  OLED_SendCmd(0xC8);

  OLED_SendCmd(0xD3);
  OLED_SendCmd(0x00);

  OLED_SendCmd(0xD5);
  OLED_SendCmd(0xC0);

  OLED_SendCmd(0xD9);
  OLED_SendCmd(0x1F);

  OLED_SendCmd(0xDA);
  OLED_SendCmd(0x12);

  OLED_SendCmd(0xDB);
  OLED_SendCmd(0x40);

  OLED_SendCmd(0xAF);
}
uint8_t GRAM[8][128];
void newFrame() { memset(GRAM, 0, sizeof(GRAM)); }
void fillFrame() { memset(GRAM, 0xff, sizeof(GRAM)); }

void showFrame() {
  for (char i = 0; i < 8; i++) {
    uint8_t sendBuffer[128 + 1] = {0};
    sendBuffer[0] = 0x40;
    for (uint8_t j = 0; j < 128; j++) {
      sendBuffer[j + 1] = GRAM[i][j];
    }

    OLED_SendCmd(0xB0 + i);
    OLED_SendCmd(0x00);
    OLED_SendCmd(0x10);

    HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, sendBuffer,
                            sizeof(sendBuffer), HAL_MAX_DELAY);
  }
}

void setPixel(int x, int y) {
  if (x < 0 || y < 0 || x >= 128 || y >= 64) {
    return;
  }
  GRAM[y / 8][x] |= 1 << y % 8;
}
void erasePixel(int x, int y) {
  if (x < 0 || y < 0 || x >= 128 || y >= 64) {
    return;
  }
  GRAM[y / 8][x] &= ~(1 << y % 8);
}

int abs(int x) { return x < 0 ? -x : x; }
void drawLine(int x0, int y0, int x1, int y1) {
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (1) {
    setPixel(x0, y0);
    if (x0 == x1 && y0 == y1)
      break;
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
}

void OLED_Test() {
  newFrame();
  fillFrame();
  erasePixel(0, 0);
  // setPixel(0, 0);
  for (int i = 0; i < 64; i++) {
    erasePixel(i, i);
  }
  showFrame();
}

#endif