#ifndef __LL_IFC_SYMPHONY_H
#define __LL_IFC_SYMPHONY_H

#include <stdint.h>
#include "ll_ifc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup Link_Labs_Interface_Library
 * @{
 */

/**
 * @defgroup Symphony_Interface Symphony Link
 *
 * @brief Communicate with a Symphony Link network.
 *
 * Symphony Link mode allows the external host to communicate with Symphony
 * Link networks, a dynamic IEEE 802.15.4 based LoRa wireless system built
 * specifically for low power, wide-area connectivity.
 *
 * @{
 */

enum ll_downlink_mode {
    LL_DL_OFF = 0,
    LL_DL_ALWAYS_ON = 1,
    LL_DL_MAILBOX = 2,
};

enum ll_state {
    LL_STATE_IDLE_CONNECTED = 1,
    LL_STATE_IDLE_DISCONNECTED = 2,
    LL_STATE_INITIALIZING = 3,
    LL_STATE_ERROR = -1,
};

enum ll_tx_state {
    LL_TX_STATE_TRANSMITTING = 1,
    LL_TX_STATE_SUCCESS = 2,
    LL_TX_STATE_ERROR = -1,
};

enum ll_rx_state {
    LL_RX_STATE_NO_MSG = 0,
    LL_RX_STATE_RECEIVED_MSG = 1,
};

/**
 * @brief
 *   Set the current configuration of the module
 *
 * @details
 *   Sets the configuration values for the module. The module
 *   will only respond to values that have changed. For example,
 *   if the app token is set to the app token that the module already
 *   has, it will not try to re-register the app token.
 *
 *   The user should call this function whenever the module is powered on.
 *
 * @param[in] net_token
 *   The network token set for this module. This determines what gateways the
 *   module will connect to. Use `OPEN_NET_TOKEN` if you don't have a particular
 *   network token.
 *
 * @param[in] app_token
 *   The app token set for this module. This is what registers your uplink
 *   messages with your Conductor account.
 *
 * @param[in] dl_mode
 *   The downlink mode for this module.
 *
 * @param[in] qos
 *   The quality of service level [0-15] for this module. The higher the
 *   quality of service, the faster uplink messages will be transmitted.
 *   Note that too many modules with a high quality of service may adversely
 *   affect network capacity.
 *
 * @return
 *   0 - success, negative otherwise.
 */
int32_t ll_config_set(uint32_t net_token, const uint8_t app_token[APP_TOKEN_LEN],
                      enum ll_downlink_mode dl_mode, uint8_t qos);

/**
 * @brief
 *   Get the current configuration of the module
 *
 * @details
 *   Returns the configuration set by the user (or defaults
 *   if the user didn't set config yet). The config parameters
 *   may not be in effect yet. For example, this function will return
 *   the downlink mode that the user requested even though
 *   the module may not have entered that downlink mode yet.
 *
 * @param[out] net_token
 *   The network token set for this module.
 *
 * @param[out] app_token
 *   The app token set for this module.
 *
 * @param[out] dl_mode
 *   The downlink mode requested for this module.
 *
 * @param[out] qos
 *   The quality of service level [0-15] requested for this module.
 *
 * @return
 *   0 - success, negative otherwise.
 */
int32_t ll_config_get(uint32_t *net_token, uint8_t app_token[APP_TOKEN_LEN],
                      enum ll_downlink_mode * dl_mode, uint8_t *qos);

/**
 * @brief
 *   Gets the state of the module.
 *
 * @details
 *   Returns the state of the module as three separate states: The general
 *   connection state, the state of the current uplink message, and the
 *   state of the current downlink message.
 *
 * @param[out] state
 *   The state of the connection. If the state is `LL_STATE_ERROR`, then this
 *   invalidates any of the other state variables.
 *
 * @param[out] tx_state
 *   The state of the transmission. If no messages have been sent by the user since the
 *   last reboot, then this variable is invalid. Otherwise, it returns the result of
 *   the last transmission (or LL_TX_STATE_TRANSMITTING if the message is in progress).
 *
 * @param[out] rx_state
 *   The state of the received message. Either the module has a downlink message ready for
 *   the user to pull out with `ll_retrieve_message`, or not. Once the user pulls out the
 *   message using `ll_retrieve_message`, this state will be reset.
 *
 * @return
 *   0 - success, negative otherwise.
 */
int32_t ll_get_state(enum ll_state * state, enum ll_tx_state * tx_state, enum ll_rx_state * rx_state);

/**
 *@brief
 *  Request
 *
 *@return
 *   0 - success, negative otherwise (Fails if module is not in MAILBOX mode)
 */
int32_t ll_mailbox_request(void);

/**
 *@brief
 *  Get the end node's application token's registration status
 *
 *@param[out] is_registered
 *  1=>registered, 0=>otherwise
 *
 *@return
 *   0 - success, negative otherwise
 */
int32_t ll_app_reg_get(uint8_t *is_registered);

/**
 *@brief
 *  Request a new key exchange between end node and gateway
 *
 *@return
 *   0 - success, negative otherwise
 */
int32_t ll_encryption_key_exchange_request(void);

/**
 * @brief
 *   Get the Network Info
 *
 * @param[out] net_info
 *   Network Info:
 *   All multi byte values are sent over in little-endian mode.
 *   Byte [0-3]   : uint32_t network id (node)
 *   Byte [4-7]   : uint32_t network id (gateway)
 *   Byte [8]     : int8_t gateway channel
 *   Byte [9-12]  : uint32_t gateway frequency
 *   Byte [13-16] : uint32_t Seconds elapsed since last beacon Rx'd
 *   Byte [17-18] : int16_t Downlink RSSI [dBm]
 *   Byte [19]    : uint8_t Downlink SNR [dB]
 *   Byte [20-23] : uint32_t Connection Status (0=Unknown, 1=Disconnected, 2=Connected)
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_net_info_get(llabs_network_info_t *net_info);

/**
 * @brief
 *   Get the Network Communication Statistics
 *
 * @param[out] s
 *   The stats struct sent by the module
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_stats_get(llabs_stats_t *s);

/**
 * @brief
 *   Get the Downlink Band Configuration
 *
 * @param[out] p_dl_band_cfg
 *   The band cfg struct sent by the module
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_dl_band_cfg_get(llabs_dl_band_cfg_t *p_dl_band_cfg);


/**
 * @brief
 *   Set the Downlink Band Configuration
 *
 * @param[in] p_dl_band_cfg
 *   Same param as ll_dl_band_cfg_get()
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_dl_band_cfg_set(const llabs_dl_band_cfg_t *p_dl_band_cfg);

/**
 * @brief
 *   Get the system time as number of seconds since the
 *   UNIX epoch 00:00:00 UTC on 1 January 1970
 *   and the time the module last synchronized time with the gateway
 *
 * @param[out]
 *   The time_info struct sent by the module
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_system_time_get(llabs_time_info_t *time_info);

/**
 * @brief
 *   Force the module to synchronize system time from the gateway
 *
 * @note
 *   worse-case delay for synchronization is the Info Block period
 *   which defaults to 8 seconds
 *
 * @param[in] sync_mode
 *   0 - Time sync only when requested
 *   1 - Time sync opportunistically
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_system_time_sync(uint8_t sync_mode);

/**
 * @brief
 *   Request Symphony MAC to send an acknowledged uplink message
 *
 * @details
 *   When the message finishes transmission the module will return to the idle state.
 *
 * @param[in] buf
 *   byte array containing the data payload
 *
 * @param[in] len
 *   length of the input buffer in bytes
 *
 * @return
 *   positive number of bytes queued,
 *   negative if an error
 */
int32_t ll_message_send_ack(uint8_t buf[], uint16_t len);

/**
 * @brief
 *   Request Symphony MAC to send an unacknowledged uplink message
 *
 * @details
 *   When the message finishes transmission the module will return to the idle state.
 *
 * @param[in] buf
 *   byte array containing the data payload
 *
 * @param[in] len
 *   length of the input buffer in bytes
 *
 * @return
 *   positive number of bytes queued,
 *   negative if an error
 */
int32_t ll_message_send_unack(uint8_t buf[], uint16_t len);


/**
 * @brief
 *   Retrieves a downlink message received by the module.
 *
 * @details
 *   This function should be called when the `rx_state` variable from the `get_state`
 *   function is `LL_RX_STATE_RECEIVED_MSG'.
 *
 * @param[out] buf
 *   The buffer into which the received message will be placed. This buffer must
 *   be at least (MAX_RX_MSG_LEN + 3) bytes in size (3 bytes will be used to get the RSSI and SNR).
 *
 * @param[out] size
 *   The size of the received message.
 *
 * @param[out] rssi
 *   The rssi of the received message.
 *
 * @param[out] snr
 *   The snr of the received message.
 *
 * @return
 *   0 - success, negative otherwise.
 */
int32_t ll_retrieve_message(uint8_t *buf, uint8_t *size, int16_t *rssi, uint8_t *snr);

/** @} (end defgroup Symphony_Interface) */

/** @} (end addtogroup Link_Labs_Interface_Library) */


#ifdef __cplusplus
}
#endif

#endif
