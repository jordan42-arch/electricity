#include "key.h"
#include "board.h"
#include "ti_msp_dl_config.h"
bool red_flag = false;
KeyState get_key_state(void)
{
    if (!DL_GPIO_readPins(KEY_PORT, KEY_K1_PIN)) {
        delay_ms(10);
        if (!DL_GPIO_readPins(KEY_PORT, KEY_K1_PIN)) {
            while (!DL_GPIO_readPins(KEY_PORT, KEY_K1_PIN)); // Wait for release
            return KEY_K1;
        }
    }

    if (!DL_GPIO_readPins(KEY_PORT, KEY_K2_PIN)) {
        delay_ms(10);
        if (!DL_GPIO_readPins(KEY_PORT, KEY_K2_PIN)) {
            while (!DL_GPIO_readPins(KEY_PORT, KEY_K2_PIN)); // Wait for release
            return KEY_K2;
        }
    }

    if (!DL_GPIO_readPins(KEY_PORT, KEY_K3_PIN)) {
        delay_ms(10);
        if (!DL_GPIO_readPins(KEY_PORT, KEY_K3_PIN)) {
            while (!DL_GPIO_readPins(KEY_PORT, KEY_K3_PIN)); // Wait for release
            return KEY_K3;
        }
    }

    return KEY_NONE;
}
//  KeyState get_red_state(void)
// {
//     if (!DL_GPIO_readPins(RED_PORT, RED_red_PIN)) {
//         delay_ms(10);
//         if (!DL_GPIO_readPins(RED_PORT, RED_red_PIN)) {
//             while (!DL_GPIO_readPins(RED_PORT, RED_red_PIN)); // Wait for release
//             return KEY_RED_GET;
//         }
//     }

//     if (DL_GPIO_readPins(RED_PORT, RED_red_PIN)) {
//         delay_ms(10);
//         if (DL_GPIO_readPins(RED_PORT, RED_red_PIN)) {
//             while (DL_GPIO_readPins(RED_PORT, RED_red_PIN)); // Wait for release
//             return KEY_RED_REMOVE;
//         }
//     }
//      return KEY_NONE;
// }
//  void get_drug(void)
// {
//     KeyState red = get_red_state();
//             if(red == KEY_RED_REMOVE)
//                 red_flag = false;
//             else
//                 red_flag = true;
           
// }


