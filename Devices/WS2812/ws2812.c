#include "ws2812.h"
#include <string.h>

#define WS2812_LowLevel    0xC0
#define WS2812_HighLevel   0xF0
#define WS2812_BITS_PER_LED 24U
#define WS2812_RESET_BYTES 100U
#define WS2812_DATA_OFFSET WS2812_RESET_BYTES

static uint8_t ws2812_txbuf[WS2812_RESET_BYTES + WS2812_BITS_PER_LED + WS2812_RESET_BYTES];

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    memset(ws2812_txbuf, 0, sizeof(ws2812_txbuf));

    for (int i = 0; i < 8; i++)
    {
        ws2812_txbuf[WS2812_DATA_OFFSET + 7 - i]  = (((g >> i) & 0x01U) ? WS2812_HighLevel : WS2812_LowLevel);
        ws2812_txbuf[WS2812_DATA_OFFSET + 15 - i] = (((r >> i) & 0x01U) ? WS2812_HighLevel : WS2812_LowLevel);
        ws2812_txbuf[WS2812_DATA_OFFSET + 23 - i] = (((b >> i) & 0x01U) ? WS2812_HighLevel : WS2812_LowLevel);
    }

    (void)HAL_SPI_Transmit(&WS2812_SPI_UNIT, ws2812_txbuf, sizeof(ws2812_txbuf), HAL_MAX_DELAY);
}
