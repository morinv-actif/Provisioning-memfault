/** @file main.c
 *   @brief Milo nRF5340 application core main application
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <dk_buttons_and_leds.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#include "wifi_provisioning/wifi_prov.h"
#include "memfault/memfault.h"

LOG_MODULE_REGISTER(milo_main, CONFIG_MILO_FW_LOG_LEVEL);

int main(void)
{
    /* Sleep 1 seconds to allow initialization of wifi driver. */
    // It is everywhere in Nordic example, we will keep it since we don't have a good reason to remove it.
    k_sleep(K_SECONDS(1));

    start_wifi_provisioning_service();
    k_yield();
    start_memfault();
    while (1)
    {
        LOG_INF("Looping main");
        k_yield();
    }
    return 0;
}