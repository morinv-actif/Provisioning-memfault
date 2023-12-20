/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/net/wifi.h>
#include <zephyr/net/wifi_mgmt.h>
#include <net/wifi_credentials.h>
#include <net/wifi_mgmt_ext.h>

#include <bluetooth/services/wifi_provisioning.h>

#include "wifi_prov.h"

LOG_MODULE_REGISTER(applayer_wifi_prov, CONFIG_APPLAYER_WIFI_PROV_LOG_LEVEL);

#ifdef CONFIG_WIFI_PROV_ADV_DATA_UPDATE
#define ADV_DATA_UPDATE_INTERVAL CONFIG_WIFI_PROV_ADV_DATA_UPDATE_INTERVAL
#endif /* CONFIG_WIFI_PROV_ADV_DATA_UPDATE */

#define ADV_PARAM_UPDATE_DELAY 1

#define ADV_DATA_VERSION_IDX (BT_UUID_SIZE_128 + 0)
#define ADV_DATA_FLAG_IDX (BT_UUID_SIZE_128 + 1)
#define ADV_DATA_FLAG_PROV_STATUS_BIT BIT(0)
#define ADV_DATA_FLAG_CONN_STATUS_BIT BIT(1)
#define ADV_DATA_RSSI_IDX (BT_UUID_SIZE_128 + 3)

/**
 * @brief Fast advertising parameters.
 *
 * This sets the advertising parameters for a "fast" advertising mode.
 * The device is set to be connectable.
 *
 * - Minimum advertising interval: 100 ms (BT_GAP_ADV_FAST_INT_MIN_2)
 * - Maximum advertising interval: 150 ms (BT_GAP_ADV_FAST_INT_MAX_2)
 *
 * @see BT_GAP_ADV_FAST_INT_MIN_2
 * @see BT_GAP_ADV_FAST_INT_MAX_2
 */
#define PROV_BT_LE_ADV_PARAM_FAST BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, \
                                                  BT_GAP_ADV_FAST_INT_MIN_2, \
                                                  BT_GAP_ADV_FAST_INT_MAX_2, NULL)

/**
 * @brief Slow advertising parameters.
 *
 * This sets the advertising parameters for a "slow" advertising mode.
 * The device is set to be connectable.
 *
 * - Minimum advertising interval: 1 s (BT_GAP_ADV_SLOW_INT_MIN)
 * - Maximum advertising interval: 1.2 s (BT_GAP_ADV_SLOW_INT_MAX)
 *
 * @see BT_GAP_ADV_SLOW_INT_MIN
 * @see BT_GAP_ADV_SLOW_INT_MAX
 */
#define PROV_BT_LE_ADV_PARAM_SLOW BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, \
                                                  BT_GAP_ADV_SLOW_INT_MIN,   \
                                                  BT_GAP_ADV_SLOW_INT_MAX, NULL)

#define ADV_DAEMON_STACK_SIZE 4096
#define ADV_DAEMON_PRIORITY 5

K_THREAD_STACK_DEFINE(adv_daemon_stack_area, ADV_DAEMON_STACK_SIZE);

static struct k_work_q adv_daemon_work_q;

static uint8_t device_name[] = {'M', 'I', 'L', 'O', 'W', '0', '0', '0', '0', '0', '0'};
static char device_name_str[sizeof(device_name) + 1];

/**
 * @def MILO_BT_SERVICE_UUID
 * @brief UUID for the Bluetooth Service.
 *
 * Currently, this is based on the `BT_UUID_PROV_VAL` from the Nordic nRF example.
 *
 * @section future_dev_note Future Developers Note
 *
 * You are allowed to modify this UUID for specific features or services.
 *
 * @subsection impact_of_modification Impact of Modifying the UUID
 *
 * - @b Device Compatibility: Affect how other devices identify this service.
 * - @b Application Interoperability: Existing applications may need to be updated.
 * - @b Service Discovery: Custom applications that search for this UUID will need updates.
 * - @b Legal Concerns: Ensure no infringement on patented or trademarked UUIDs.
 * - @b Documentation: Update all relevant documentation.
 * - @b Testing: Thoroughly test to ensure no functionalities are broken.
 * - @b Client Updates: Update client-side code that references this UUID.
 * - @b Backward Compatibility: Handle older devices or software looking for the old UUID.
 *
 * @subsection suggested_steps Suggested Steps for Modification
 *
 * 1. Generate a new 128-bit UUID.
 * 2. Update this #define with the new UUID.
 * 3. Update and test client applications.
 *
 * @author Vincent Morin
 * @date 10/17/2023
 */
#define MILO_BT_SERVICE_UUID BT_UUID_PROV_VAL

/**
 * @brief Defines the provisioning service data for Bluetooth advertising.
 *
 * @details The `prov_svc_data` array serves as the storage for:
 * - The WiFi Provisioning Service UUID, which is defined by MILO_BT_SERVICE_UUID.
 * - Additional service-specific data, as described below.
 *
 * The data layout is as follows:
 * - Byte 1: Version information, located at index ADV_DATA_VERSION_IDX.
 * - Byte 2: Flag data (Least Significant Byte), located at index ADV_DATA_FLAG_IDX.
 * - Byte 3: Flag data (Most Significant Byte), located at index ADV_DATA_FLAG_IDX + 1.
 * - Byte 4: RSSI (Received Signal Strength Indicator), located at index ADV_DATA_RSSI_IDX.
 *
 * All index positions are calculated relative to the end of the 128-bit UUID.
 *
 * ## Flag Format
 *
 * The flag is a 16-bit little-endian field. Byte 2 (first byte sent over the air) is the LSB,
 * and Byte 3 (second byte sent over the air) is the MSB.
 *
 * The bits in the flag are used as follows:
 * - **Bit 0**: Provisioning status. Set to 1 if the device is provisioned; 0 otherwise.
 * - **Bit 1**: Connection status. Set to 1 if WiFi is connected; 0 otherwise.
 * - **Bit 2-15**: Reserved for Future Use (RFU).
 */
static uint8_t prov_svc_data[] = {MILO_BT_SERVICE_UUID, 0x00, 0x00, 0x00, 0x00};

/**
 * @brief Advertisement Data Array
 *
 * This array specifies the data that will be included in the advertisement packets.
 *
 * - Flags: Indicates that the device is in general discoverable mode and not compatible with BR/EDR.
 * - UUID128: Includes the 128-bit Bluetooth UUID for the WiFi Provisioning Service.
 * - Complete Name: Sets the complete name of the device, as specified in the device_name variable.
 *
 */
static const struct bt_data advertisement_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_PROV_VAL),
    BT_DATA(BT_DATA_NAME_COMPLETE, device_name, sizeof(device_name)),
};

/**
 * @brief Scan Response Data Array
 *
 * Contains service data that will be sent in scan response packets.
 */
static const struct bt_data service_data[] = {
    BT_DATA(BT_DATA_SVC_DATA128, prov_svc_data, sizeof(prov_svc_data)),
};

static struct k_work_delayable update_adv_param_work;
static struct k_work_delayable update_adv_data_work;

/**
 * @brief Update WiFi connection status and RSSI in Bluetooth advertisement.
 *
 * This function queries the WiFi interface for its current status and RSSI.
 * It updates the 'wifi_connected' and 'rssi' variables based on the information obtained.
 *
 * @param[out] wifi_connected Pointer to the boolean flag indicating WiFi connection status.
 * @param[out] rssi Pointer to the variable to store the Received Signal Strength Indicator (RSSI).
 */
static void update_wifi_status_in_adv(void)
{
    int rc;
    struct net_if *iface = net_if_get_default();
    struct wifi_iface_status status = {0};

    prov_svc_data[ADV_DATA_VERSION_IDX] = PROV_SVC_VER;

    /* If no config, mark it as unprovisioned. */
    if (!bt_wifi_prov_state_get())
    {
        prov_svc_data[ADV_DATA_FLAG_IDX] &= ~ADV_DATA_FLAG_PROV_STATUS_BIT;
    }
    else
    {
        prov_svc_data[ADV_DATA_FLAG_IDX] |= ADV_DATA_FLAG_PROV_STATUS_BIT;
    }

    rc = net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status,
                  sizeof(struct wifi_iface_status));
    /* If WiFi is not connected or error occurs, mark it as not connected. */
    if ((rc != 0) || (status.state < WIFI_STATE_ASSOCIATED))
    {
        prov_svc_data[ADV_DATA_FLAG_IDX] &= ~ADV_DATA_FLAG_CONN_STATUS_BIT;
        prov_svc_data[ADV_DATA_RSSI_IDX] = INT8_MIN;
    }
    else
    {
        /* WiFi is connected. */
        prov_svc_data[ADV_DATA_FLAG_IDX] |= ADV_DATA_FLAG_CONN_STATUS_BIT;
        /* Currently cannot retrieve RSSI. Use a dummy number. */
        prov_svc_data[ADV_DATA_RSSI_IDX] = status.rssi;
    }
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if (err)
    {
        printk("BT Connection failed (err 0x%02x).\n", err);
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("BT Connected: %s", addr);

    k_work_cancel_delayable(&update_adv_data_work);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("BT Disconnected: %s (reason 0x%02x).\n", addr, reason);

    k_work_reschedule_for_queue(&adv_daemon_work_q, &update_adv_param_work,
                                K_SECONDS(ADV_PARAM_UPDATE_DELAY));
    k_work_reschedule_for_queue(&adv_daemon_work_q, &update_adv_data_work, K_NO_WAIT);
}

static void identity_resolved(struct bt_conn *conn, const bt_addr_le_t *rpa,
                              const bt_addr_le_t *identity)
{
    char addr_identity[BT_ADDR_LE_STR_LEN];
    char addr_rpa[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(identity, addr_identity, sizeof(addr_identity));
    bt_addr_le_to_str(rpa, addr_rpa, sizeof(addr_rpa));

    printk("BT Identity resolved %s -> %s.\n", addr_rpa, addr_identity);
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
                             enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err)
    {
        printk("BT Security changed: %s level %u.\n", addr, level);
    }
    else
    {
        printk("BT Security failed: %s level %u err %d.\n", addr, level,
               err);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .identity_resolved = identity_resolved,
    .security_changed = security_changed,
};

/**
 * @brief Handles Bluetooth authentication cancellation events.
 *
 * This function is called when Bluetooth authentication is cancelled.
 *
 * @param[in] conn Pointer to the Bluetooth connection object.
 */
static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("BT Pairing cancelled: %s.\n", addr);
}

/**
 * @brief Bluetooth Connection Authentication Callback Structure.
 *
 * In Zephyr's Bluetooth stack, this structure is used for handling
 * various Bluetooth authentication events.
 *
 * @details The callback is generally triggered under the following scenarios:
 *  - Pairing Request
 *  - PIN Code Request
 *  - Passkey Display
 *  - Passkey Entry
 *  - Passkey Confirm
 *  - Authentication Status
 *  - User Confirm
 *  - LE Secure Connections
 *  - Authorization
 *
 * Currently, only the 'cancel' callback is defined. All other callbacks default to NULL.
 */
static struct bt_conn_auth_cb auth_cb_display = {
    .cancel = auth_cancel,
};

/**
 * @brief Handles Bluetooth pairing completion events.
 *
 * This function is triggered when Bluetooth pairing is successfully completed.
 *
 * @param[in] conn Pointer to the Bluetooth connection object.
 * @param bonded Flag indicating if the devices are bonded.
 */
static void pairing_complete(struct bt_conn *conn, bool bonded)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("BT pairing completed: %s, bonded: %d\n", addr, bonded);
}

/**
 * @brief Handles Bluetooth pairing failure events.
 *
 * This function is triggered when Bluetooth pairing fails.
 * It will call bt_conn_disconnect on the bt_conn object
 *
 * @param[in] conn Pointer to the Bluetooth connection object.
 * @param reason Enum value indicating the reason for pairing failure.
 */
static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    printk("BT Pairing Failed (%d). Disconnecting.\n", reason);
    bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
}

/**
 * @brief Bluetooth Connection Authentication Information Callback Structure.
 *
 * In Zephyr's Bluetooth stack, this structure is used for handling
 * Bluetooth pairing success and failure events.
 *
 * @details The callback is generally triggered under the following scenarios:
 *  - Pairing is completed
 *  - Pairing failed
 *  - Bond deleted
 *
 * Here, "bond" refers to the stored security keys and authentication
 * information that allow two devices to reconnect securely without requiring
 * to pair again.
 *
 * Currently, only the 'pairing complete' and 'pairing failed' events are implemented.
 */
static struct bt_conn_auth_info_cb auth_info_cb_display = {

    .pairing_complete = pairing_complete,
    .pairing_failed = pairing_failed,
};

static void update_adv_data_task(struct k_work *item)
{
    int rc;

    update_wifi_status_in_adv();
    rc = bt_le_adv_update_data(advertisement_data, ARRAY_SIZE(advertisement_data), service_data, ARRAY_SIZE(service_data));
    if (rc != 0)
    {
        printk("Cannot update advertisement data, err = %d\n", rc);
    }
#ifdef CONFIG_WIFI_PROV_ADV_DATA_UPDATE
    k_work_reschedule_for_queue(&adv_daemon_work_q, &update_adv_data_work,
                                K_SECONDS(ADV_DATA_UPDATE_INTERVAL));
#endif /* CONFIG_WIFI_PROV_ADV_DATA_UPDATE */
}

static void update_adv_param_task(struct k_work *item)
{
    int rc;

    rc = bt_le_adv_stop();
    if (rc != 0)
    {
        printk("Cannot stop advertisement: err = %d\n", rc);
        return;
    }

    rc = bt_le_adv_start(prov_svc_data[ADV_DATA_FLAG_IDX] & ADV_DATA_FLAG_PROV_STATUS_BIT ? PROV_BT_LE_ADV_PARAM_SLOW : PROV_BT_LE_ADV_PARAM_FAST,
                         advertisement_data, ARRAY_SIZE(advertisement_data), service_data, ARRAY_SIZE(service_data));
    if (rc != 0)
    {
        printk("Cannot start advertisement: err = %d\n", rc);
    }
}

/**
 * @brief Convert a byte to its hexadecimal representation.
 *
 * This static function converts a single byte into its two-character hexadecimal representation.
 * The converted hex characters are stored in the location pointed to by 'ptr'.
 *
 * @param[out] ptr Pointer to the location where the resulting hex characters will be stored.
 *                 It's expected that 'ptr' points to a buffer of at least 2 characters.
 * @param[in] byte The byte to be converted to hexadecimal.
 * @param[in] base The base character for alphabetic hex digits ('A' for uppercase, 'a' for lowercase).
 */
static void byte_to_hex(char *ptr, uint8_t byte, char base)
{
    int i, val;

    for (i = 0, val = (byte & 0xf0) >> 4; i < 2; i++, val = byte & 0x0f)
    {
        if (val < 10)
        {
            *ptr++ = (char)(val + '0');
        }
        else
        {
            *ptr++ = (char)(val - 10 + base);
        }
    }
}

/**
 * @brief Update the device name using a MAC address.
 *
 * This static function updates the global 'device_name' based on the provided MAC address.
 * It specifically uses the last 3 bytes of the MAC address and converts them to hexadecimal
 * to form a part of the device name.
 *
 * @note The 'device_name' global variable should be defined with sufficient size to hold
 *       the updated name. This function assumes that 'device_name' has at least 11 characters.
 *
 * @param[in] mac_addr Pointer to the structure holding the MAC address. The last 3 bytes of the
 *                     MAC address are used to update the device name.
 */
static void update_dev_name(const struct net_linkaddr *mac_addr)
{
    byte_to_hex(&device_name[5], mac_addr->addr[3], 'A');
    byte_to_hex(&device_name[7], mac_addr->addr[4], 'A');
    byte_to_hex(&device_name[9], mac_addr->addr[5], 'A');
}

/**
 * @brief Set the Access Point (AP) Name for the Bluetooth Device
 *
 * This function sets the Bluetooth GAP (Generic Access Profile) Device Name,
 * which is the name that other Bluetooth devices will see when scanning for
 * devices to pair with.
 *
 * @note Ensure to update the advertising data by calling either
 * `bt_le_adv_update_data` or `bt_le_ext_adv_set_data` after using this function.
 *
 * @return Zero on success, or (negative) error code otherwise.
 */
static int bt_set_ap_name(const struct net_linkaddr *mac_addr)
{
    if (mac_addr)
    {
        update_dev_name(mac_addr);
        memcpy(device_name_str, device_name, sizeof(device_name));
        device_name_str[sizeof(device_name_str) - 1] = '\0';
    }
    else
    {
        LOG_ERR("Mac addresses pointer was NULL, default BT name");
    }

    return bt_set_name(device_name_str);
}

/**
 * @brief Starts Bluetooth Advertising
 *
 * Logs the scan response data and initiates Bluetooth advertising with either fast or slow
 * advertising parameters, based on the provisioning status.
 *
 * @return Status of advertising start operation.
 */
static int bt_start_advertising()
{
    const struct bt_le_adv_param *advertising_speed = prov_svc_data[ADV_DATA_FLAG_IDX] & ADV_DATA_FLAG_PROV_STATUS_BIT ? PROV_BT_LE_ADV_PARAM_SLOW : PROV_BT_LE_ADV_PARAM_FAST;

    return bt_le_adv_start(advertising_speed, advertisement_data, ARRAY_SIZE(advertisement_data), service_data, ARRAY_SIZE(service_data));
}

/**
 * @brief Initializes and starts the work queue for Bluetooth tasks.
 *
 * This function sets up the Advertising Daemon work queue and initializes
 * delayable work items for advertisement updates.
 */
static void bt_init_and_start_workqueue()
{
    k_work_queue_init(&adv_daemon_work_q);
    k_work_queue_start(&adv_daemon_work_q, adv_daemon_stack_area, K_THREAD_STACK_SIZEOF(adv_daemon_stack_area), ADV_DAEMON_PRIORITY, NULL);

    k_work_init_delayable(&update_adv_param_work, update_adv_param_task);
    k_work_init_delayable(&update_adv_data_work, update_adv_data_task);
#ifdef CONFIG_WIFI_PROV_ADV_DATA_UPDATE
    k_work_schedule_for_queue(&adv_daemon_work_q, &update_adv_data_work, K_SECONDS(CONFIG_WIFI_PROV_ADV_DATA_UPDATE_INTERVAL));
#endif /* CONFIG_WIFI_PROV_ADV_DATA_UPDATE */
}

void start_wifi_provisioning_service(void)
{
    struct net_if *iface = net_if_get_default();
    struct net_linkaddr *mac_addr = net_if_get_link_addr(iface);

    /* Sleep 1 seconds to allow initialization of wifi driver. */
    k_sleep(K_SECONDS(1));

    bt_conn_auth_cb_register(&auth_cb_display);
    bt_conn_auth_info_cb_register(&auth_info_cb_display);

    if (bt_enable(NULL))
    {
        LOG_ERR("Bluetooth init failed.\n");
        return;
    }

    LOG_INF("Bluetooth initialized.\n");

    if (bt_wifi_prov_init())
    {
        LOG_ERR("Error occurs when initializing Wi-Fi provisioning service.\n");
        return;
    }

    LOG_INF("Wi-Fi provisioning service starts successfully.\n");

    if (bt_set_ap_name(mac_addr))
    {
        LOG_ERR("Error occurred when providing BT AP name\n");
        return;
    }

    if (bt_start_advertising())
    {
        LOG_ERR("BT Advertising failed to start\n");
        return;
    }
    LOG_INF("BT Advertising successfully started.\n");

    update_wifi_status_in_adv();

    bt_init_and_start_workqueue();

    if (net_mgmt(NET_REQUEST_WIFI_CONNECT_STORED, iface, NULL, 0))
    {
        LOG_WRN("There's no stored WiFi credential found on the device\n");
        return;
    }

    LOG_INF("Exiting applayer_wifi_prov, the work queue is properly started\n");
    return;
}
