/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include "ws2812b.h"

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
//static bool DRV_Enable(drv8353_t*);
//static bool DRV_SetConfig(drv8353_t*);
//static void DRV_Calibration(drv8353_t*);
//uint16_t    DRV_readOne(drv8353_t*, DRV_Address_e);
//void        DRV_writeOne(drv8353_t*, DRV_Address_e, uint16_t);

//struct led_state ws_led;
/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       Start \& configure the DRV ICs.
 * @param[in]   *p_drv      Pointer on the DRV definition structure
 */
//void LED_ini(void)
//{
//
//
//    if((p_drv != NULL) && (p_drv->p_drvCfgHandler != NULL) && (p_drv->p_drvRegHandler != NULL))
//    {
//        DRV_Enable(p_drv);
//        if(DRV_SetConfig(p_drv))
//            DRV_Calibration(p_drv);
//    }
//
//    return;
//}

//void ws2812_control_init(void); //init the LEDs
//void set_all_leds(0x0f0f0f);
//
//ws2812_write_leds(ws_led);
//
//#define BITS_PER_LED_CMD 24
//#define LED_BUFFER_ITEMS ((NUM_LEDS * BITS_PER_LED_CMD))

void ws2812b_writeLed(uint16_t red, uint16_t green, uint16_t blue, uint16_t* p_led_colors)
{
    *p_led_colors++ = 0x0000;
    *p_led_colors++ = ws2812_color_table_r_justified[green][0];
    *p_led_colors++ = ws2812_color_table_r_justified[green][1] | ws2812_color_table_l_justified[red][0];
    *p_led_colors++ = ws2812_color_table_l_justified[red][1];
    *p_led_colors++ = ws2812_color_table_r_justified[blue][0];
    *p_led_colors++ = ws2812_color_table_r_justified[blue][1];

    /* LEDs */
//    uint16_t ms_cpt = 0;
//    float32_t fade_blink = (ms_cpt % 1000 < 500) ? (ms_cpt % 1000) / 500.0 : (1000 - (ms_cpt % 1000)) / 500.0;
//    bool_t  blink = ((ms_cpt % 1000) > 500) ? (true) : (false);

//    switch (current_state)
//    {
//    case WAITING_FOR_INIT:
//        set_all_leds(RGB(0xff * fade_blink, 0, 0)); //Red fade, Waiting for init
//
//        // Change wifi channel to find the one used by the computer
//        if ((ms_cpt % 100) == 0 && next_state == current_state)
//        {
//            wifi_channel = wifi_channel < 14 ? (wifi_channel + 1) : 1;
//            wifi_change_channel(wifi_channel);
//        }
//
//        wifi_eth_count = 0; // we allow not receiving messages if waiting for init
//        break;
//
//    case SPI_AUTODETECT:
//        set_all_leds(RGB(0xff * fade_blink, 0, 0xff * fade_blink)); //Magenta fade, SPI Autodetect
//
//        if (spi_count > SPI_AUTODETECT_MAX_COUNT)
//        {
//            next_state = SENDING_INIT_ACK;
//        }
//
//        wifi_eth_count = 0; // we allow not receiving messages while checking for spi slaves
//        break;
//
//    case SENDING_INIT_ACK:
//        set_all_leds(RGB(0, 0, 0xff * fade_blink)); //Blue fade, waiting for first commmand
//
//        if (wifi_eth_count > CONFIG_WIFI_ETH_TIMEOUT_ACK)
//        {
//            next_state = WIFI_ETH_ERROR;
//        }
//
//        wifi_eth_count++;
//        break;
//
//    case ACTIVE_CONTROL:
//        set_all_leds(RGB(0, 0xff * fade_blink, 0)); //Green fade, Active control
//
//        if (wifi_eth_count > CONFIG_WIFI_ETH_TIMEOUT_CONTROL)
//        {
//            next_state = WIFI_ETH_ERROR;
//        }
//        else
//        {
//            p_tx = spi_use_a ? spi_tx_packet_a : spi_tx_packet_b;
//        }
//
//        wifi_eth_count++;
//        break;
//
//    case WIFI_ETH_LINK_DOWN:
//        set_all_leds(RGB(0x3f * blink, 0x3f * blink, 0)); //Yellow blink, ethernet link down state awaiting for link up
//
//        wifi_eth_count = 0; // we can't receive any messages if link is down
//        break;
//
//    case WIFI_ETH_ERROR:
//        set_all_leds(RGB(0xff * blink, 0, 0)); //Red blink, error state (communication with PC), awaiting for new init msg
//
//        wifi_eth_count = 0; // we allow not receiving messages if waiting for init in error state
//        break;
//
//    default:
//        set_all_leds(RGB(0xff * blink, 0xff * blink, 0xff * blink)); //White blink, state machine error (should never happen)
//        break;
//    }

    return;
}
