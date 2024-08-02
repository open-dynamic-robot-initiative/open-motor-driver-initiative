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
 * @param[out]  *p_msg          Pointer on the led message created.
 */
uint16_t* ws2812b_writeLed(uint16_t led_pos, uint16_t red, uint16_t green, uint16_t blue, uint16_t* p_msg)
{
    red        &= 0x00FF;
    green      &= 0x00FF;
    blue       &= 0x00FF;

    if(led_pos % 2)
    {
        *p_msg++   |= ws2812_color_table_l_justified[green][0];
        *p_msg++    = ws2812_color_table_l_justified[green][1];
        *p_msg++    = ws2812_color_table_r_justified[red][0];
        *p_msg      = ws2812_color_table_r_justified[red][1];
        *p_msg++   |= ws2812_color_table_l_justified[blue][0];
        *p_msg++    = ws2812_color_table_l_justified[blue][1];
    }
    else
    {
        *p_msg++    = ws2812_color_table_r_justified[green][0];
        *p_msg      = ws2812_color_table_r_justified[green][1];
        *p_msg++   |= ws2812_color_table_l_justified[red][0];
        *p_msg++    = ws2812_color_table_l_justified[red][1];
        *p_msg++    = ws2812_color_table_r_justified[blue][0];
        *p_msg      = ws2812_color_table_r_justified[blue][1];
    }

    return (p_msg);
}

/**
 * @brief       Define a command according motor status.
 * @param[in]   motor_state     Current motor state.
 * @param[in]   timer_cpt_ms    Current millisecond counter value.
 * @param[out]  *p_msg          Pointer on the led message created.
 */
uint16_t* ws2812b_dimmingLed(uint16_t led_pos, motor_state_e motor_state, uint16_t timer_cpt_ms, uint16_t* p_msg)
{
    float32_t   fade_blink_f32  = (float32_t)((timer_cpt_ms < 500U) ? (timer_cpt_ms): (1000U - timer_cpt_ms)) / 500.0f;
    uint16_t    blink           = (timer_cpt_ms < 500U)             ? (0x00)        : (0xFF);
    uint16_t    fade_blink_u8   = (uint16_t)(0x00FF * fade_blink_f32);


    switch(motor_state)
    {
    case MOTOR_STATE_INIT_FW:
    case MOTOR_STATE_INIT_REV:
    case MOTOR_STATE_INIT_TEST:
    case MOTOR_STATE_INIT_FIX:
        p_msg = ws2812b_writeLed(led_pos, fade_blink_u8, 0, fade_blink_u8, p_msg); //Magenta fade,
        break;
    case MOTOR_STATE_READY:
    case MOTOR_STATE_STOP:
        p_msg = ws2812b_writeLed(led_pos, 0, fade_blink_u8, 0, p_msg);//Green fade, Active control
        break;
    case MOTOR_STATE_ERROR:
        p_msg = ws2812b_writeLed(led_pos, blink, 0, 0, p_msg);//red fade, error control
        break;
    case MOTOR_STATE_IDLE:
    default:
        p_msg = ws2812b_writeLed(led_pos, 0,  0, fade_blink_u8, p_msg);//blue fade
    }

    return (p_msg);
}
