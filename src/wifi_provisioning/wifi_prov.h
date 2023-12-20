/** @file wifi_prov.h
 *  @brief This file expose the start function for the wifi provisioning
 */

#ifndef MILO__WIFI_PROVISIONING_V2__WIFI_PROV__H_
#define MILO__WIFI_PROVISIONING_V2__WIFI_PROV__H_

/**
 * @brief TODO: ADD
 * @defgroup TODO: ADD
 * @ingroup TODO: ADD
 * @{
 */

// Add your include here

#ifdef __cplusplus
extern "C"
{
#endif // end of __cplusplus

    /**
     * @brief Initialize and start the WiFi provisioning service.
     *
     * This function takes care of the initialization steps required for WiFi provisioning.
     * It includes Bluetooth and WiFi initialization, advertisement setup, and attempts to
     * connect to a stored WiFi network if available.
     *
     * This function launches an eternal worker thread that will continue to execute unless
     * explicitly terminated. The worker thread is responsible for handling various tasks,
     * such as monitoring WiFi and Bluetooth statuses, and updating data accordingly.
     *
     * @note This worker thread will live indefinitely after it is launched.
     */
    void start_wifi_provisioning_service(void);

#ifdef __cplusplus
}
#endif // end of __cplusplus

/**
 * @}
 */

#endif // MILO__WIFI_PROVISIONING_V2__WIFI_PROV__H_