/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include "ws2812b.h"

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       Start \& configure the DRV ICs.
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 */
void ws2812b_writeLed(uint16_t red, uint16_t green, uint16_t blue, uint16_t* p_led_colors)
{
    *p_led_colors++ = 0x0000;
    *p_led_colors++ = ws2812_color_table_r_justified[green][0];
    *p_led_colors++ = ws2812_color_table_r_justified[green][1] | ws2812_color_table_l_justified[red][0];
    *p_led_colors++ = ws2812_color_table_l_justified[red][1];
    *p_led_colors++ = ws2812_color_table_r_justified[blue][0];
    *p_led_colors++ = ws2812_color_table_r_justified[blue][1];

    return;
}
