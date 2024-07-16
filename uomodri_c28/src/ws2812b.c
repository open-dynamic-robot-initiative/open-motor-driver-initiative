/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include "ws2812b.h"

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       Force a reset command on WS2812b led.
 * @param[out]  *p_led_colors   Pointer on the led message created.
 */
uint16_t* ws2812b_resetLed(uint16_t* p_led_colors)
{
    *p_led_colors++ = 0x0000;

    return(p_led_colors);

}

/**
 * @brief       Create a message for WS2812b led control.
 * @param[in]   red             Red brightness (0-255)
 * @param[in]   green           Green brightness (0-255)
 * @param[in]   blue            Blue brightness (0-255)
 * @param[out]  *p_led_colors   Pointer on the led message created.
 */
uint16_t* ws2812b_writeLed(uint16_t red, uint16_t green, uint16_t blue, uint16_t* p_led_colors)
{
//    *p_led_colors++ = 0x0000;
    *p_led_colors++     = ws2812_color_table_r_justified[green][0];
//    *p_led_colors++ = ws2812_color_table_r_justified[green][1] | ws2812_color_table_l_justified[red][0];
    *p_led_colors       = ws2812_color_table_r_justified[green][1];
    *p_led_colors++    |= ws2812_color_table_l_justified[red][0];
    *p_led_colors++     = ws2812_color_table_l_justified[red][1];
    *p_led_colors++     = ws2812_color_table_r_justified[blue][0];
    *p_led_colors++     = ws2812_color_table_r_justified[blue][1];

    return (p_led_colors);
}
