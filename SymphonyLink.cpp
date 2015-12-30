#include "arduino.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "SymphonyLink.h"

extern const uint32_t OPEN_NET_TOKEN;

uint8_t read_uint8(const uint8_t buff[], size_t * index) {
    return buff[(*index)++];
}

uint16_t read_uint16(const uint8_t buff[], size_t * index) {
    uint16_t msb = read_uint8(buff, index);
    uint16_t lsb = read_uint8(buff, index);
    return (msb << 8) | lsb;
}

uint32_t read_uint32(const uint8_t buff[], size_t * index) {
    uint32_t msb = read_uint16(buff, index);
    uint32_t lsb = read_uint16(buff, index);
    return (msb << 16) | lsb;
}

uint64_t read_uint64(const uint8_t buff[], size_t *index) {
    uint64_t msb = read_uint32(buff, index);
    uint64_t lsb = read_uint32(buff, index);
    return (msb << 32) | lsb;
}

void write_uint8(uint8_t x, uint8_t **buff) {
    *((*buff)++) = x;
}

void write_uint16(uint16_t x, uint8_t **buff) {
    write_uint8(x >> 8, buff);
    write_uint8(x, buff);
}

void write_uint32(uint32_t x, uint8_t **buff) {
    write_uint16(x >> 16, buff);
    write_uint16(x, buff);
}

void write_uint64(uint64_t x, uint8_t **buff) {
    write_uint32(x >> 32, buff);
    write_uint32(x, buff);
}

// Parse a serialized llabs_network_info_t struct.
void ll_net_info_deserialize(const uint8_t buff[NET_INFO_BUFF_SIZE], llabs_network_info_t *net_info) {
    size_t i = 0;
    net_info->network_id_node = read_uint32(buff, &i);
    net_info->network_id_gw = read_uint32(buff, &i);
    net_info->gateway_channel = read_uint8(buff, &i);
    net_info->gateway_frequency = read_uint32(buff, &i);
    net_info->last_rx_tick = read_uint32(buff, &i);
    net_info->rssi = read_uint16(buff, &i);
    net_info->snr = read_uint8(buff, &i);
    net_info->connection_status = (llabs_connect_status_t)read_uint8(buff, &i);
    net_info->is_scanning_gateways = read_uint8(buff, &i);
    net_info->gateway_id = read_uint64(buff, &i);
}

// Serializes an llabs_network_info_t struct into a buffer to be sent over the host interface.
// Returns the size of the serialized struct in the buffer.
uint16_t ll_net_info_serialize(const llabs_network_info_t *net_info, uint8_t buff[NET_INFO_BUFF_SIZE]) {
    uint8_t * buff_cpy = buff;
    write_uint32(net_info->network_id_node, &buff_cpy);
    write_uint32(net_info->network_id_gw, &buff_cpy);
    write_uint8(net_info->gateway_channel, &buff_cpy);
    write_uint32(net_info->gateway_frequency, &buff_cpy);
    write_uint32(net_info->last_rx_tick, &buff_cpy);
    write_uint16(net_info->rssi, &buff_cpy);
    write_uint8(net_info->snr, &buff_cpy);
    write_uint8(net_info->connection_status, &buff_cpy);
    write_uint8(net_info->is_scanning_gateways, &buff_cpy);
    write_uint64(net_info->gateway_id, &buff_cpy);
    return buff_cpy - buff;
}

void ll_dl_band_cfg_deserialize(const uint8_t buff[DL_BAND_CFG_SIZE], llabs_dl_band_cfg_t *dl_cfg) {
    size_t i = 0;
    dl_cfg->band_edge_lower = read_uint32(buff, &i);
    dl_cfg->band_edge_upper = read_uint32(buff, &i);
    dl_cfg->band_edge_guard = read_uint32(buff, &i);
    dl_cfg->chan_step_size = read_uint8(buff, &i);
    dl_cfg->chan_step_offset = read_uint8(buff, &i);
}

uint16_t ll_dl_band_cfg_serialize(const llabs_dl_band_cfg_t *dl_cfg, uint8_t buff[DL_BAND_CFG_SIZE]) {
    uint8_t * buff_cpy = buff;
    write_uint32(dl_cfg->band_edge_lower, &buff_cpy);
    write_uint32(dl_cfg->band_edge_upper, &buff_cpy);
    write_uint32(dl_cfg->band_edge_guard, &buff_cpy);
    write_uint8(dl_cfg->chan_step_size, &buff_cpy);
    write_uint8(dl_cfg->chan_step_offset, &buff_cpy);
    return buff_cpy - buff;
}

void ll_stats_deserialize(const uint8_t buff[STATS_SIZE], llabs_stats_t *stats) {
    size_t i = 0;
    stats->num_send_calls = read_uint32(buff, &i);
    stats->num_pkts_transmitted = read_uint32(buff, &i);
    stats->num_gateway_scans = read_uint32(buff, &i);
    stats->num_collisions = read_uint32(buff, &i);
    stats->num_ack_successes = read_uint32(buff, &i);
    stats->num_ack_failures = read_uint32(buff, &i);
    stats->num_sync_failures = read_uint32(buff, &i);
    stats->num_canceled_pkts_ack = read_uint32(buff, &i);
    stats->num_canceled_pkts_csma = read_uint32(buff, &i);
    stats->num_rx_errors = read_uint32(buff, &i);
}

uint16_t ll_stats_serialize(const llabs_stats_t *stats, uint8_t buff[STATS_SIZE]) {
    uint8_t * buff_cpy = buff;
    write_uint32(stats->num_send_calls, &buff_cpy);
    write_uint32(stats->num_pkts_transmitted, &buff_cpy);
    write_uint32(stats->num_gateway_scans, &buff_cpy);
    write_uint32(stats->num_collisions, &buff_cpy);
    write_uint32(stats->num_ack_successes, &buff_cpy);
    write_uint32(stats->num_ack_failures, &buff_cpy);
    write_uint32(stats->num_sync_failures, &buff_cpy);
    write_uint32(stats->num_canceled_pkts_ack, &buff_cpy);
    write_uint32(stats->num_canceled_pkts_csma, &buff_cpy);
    write_uint32(stats->num_rx_errors, &buff_cpy);
    return buff_cpy - buff;
}

int32_t transport_write(uint8_t *buff, uint16_t len) {
    int32_t i32_ret;
    i32_ret = Serial1.write(buff, len);
    if (i32_ret < 0) {
        return -1;
    }

    return 0;
}


/*
 * Return:
 *  0 if len bytes were read before the timeout time,
 *  -1 otherwise
 */
int32_t transport_read(uint8_t *buff, uint16_t len) {
    int32_t i32_ret_inner;

    uint8_t rx_byte;
    uint16_t bytes_received = 0;

    uint32_t start_tick = millis();
    uint32_t timeout_val = (500 * 1);

    while((millis() - start_tick) < timeout_val) {
        rx_byte = Serial1.read();

        if (rx_byte > 0) {
            // We received a byte
            buff[bytes_received++] = rx_byte;

            if (bytes_received == len) {
                // We received all of the requested bytes
                return(0);
            }
        }
    }

    return(-1);
}

/**
 * @brief
 *   recv_packet
 *
 * @param[in] op
 *   opcode of the command that we're trying to receive
 *
 * @param[in] message_num
 *   message number of the command that we're trying to receive
 *
 * @param[in] buf
 *   byte array for storing data returned from the module
 *
 * @param[in] len
 *   size of the output buffer in bytes
 *
 * @return
 *   positive number of bytes returned,
 *   negative if an error
 *   Error Codes:
 *       -1 NACK received - Command not supported
 *       -2 NACK received - Incorrect Checksum
 *       -3 NACK received - Payload length out of range
 *       -4 NACK received - Payload out of range
 *       -5 NACK received - Not allowed, bootup in progress
 *       -6 NACK received - Busy try again
 *       -7 NACK received - Application token not registered
 *       -8 NACK received - Payload length greater than maximum supported length
 *      -99 NACK received - Other
 *     -103 Message Number in response doesn't match expected
 *     -104 Checksum mismatch
 *     -105 Command mismatch (responding to a different command)
 *     -106 Timed out waiting for Rx bytes from interface
 *     -107 Response larger than provided output buffer
 *     -108 transport_read failed getting FRAME_START
 *     -109 transport_read failed getting header
 */
uint16_t compute_checksum(uint8_t *hdr, uint16_t hdr_len, uint8_t *payload, uint16_t payload_len) {
    uint16_t crc = 0x0;
    uint16_t i;

    for (i = 0; i < hdr_len; i++) {
        crc  = (crc >> 8) | (crc << 8);
        crc ^= hdr[i];
        crc ^= (crc & 0xff) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0xff) << 5;
    }

    for (i = 0; i < payload_len; i++) {
        crc  = (crc >> 8) | (crc << 8);
        crc ^= payload[i];
        crc ^= (crc & 0xff) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0xff) << 5;
    }

    return crc;
}

int32_t recv_packet(opcode_t op, uint8_t message_num, uint8_t *buf, uint16_t len) {
    uint8_t  header_buf[RESP_HEADER_LEN];
    uint16_t header_idx;

    uint8_t curr_byte = 0;
    uint8_t checksum_buff[2];
    uint16_t computed_checksum;
    int32_t ret_value = 0;
    int32_t ret;

    memset(header_buf, 0, sizeof(header_buf));
    //TODO: have conditionally compiled cases for various platforms to ensure accurate timeout
    uint32_t max_clock = (1.5 * 1000.0);
    uint32_t t = millis();

    do {
        /* Timeout of infinite Rx loop if responses never show up*/
        ret = transport_read(&curr_byte, 1);
        if((millis()- t) > max_clock) {
            len = 0;
            return (-106);
        }
    } while(curr_byte != FRAME_START);

    if (ret < 0) { 
        /* transport_read failed - return an error */
        return (-108);
    }

    header_idx = 0;
    header_buf[header_idx++] = FRAME_START;

    ret = transport_read(header_buf + 1, RESP_HEADER_LEN - 1);
    if (ret < 0) {
        /* transport_read failed - return an error */
        return (-109);
    }

    uint16_t len_from_header = (uint16_t)header_buf[5] + ((uint16_t)header_buf[4] << 8);

    if (header_buf[1] != op) {
        // Command Byte should match what was sent
        ret_value = -105;
    }

    if (header_buf[2] != message_num) {
        // Message Number should match
        ret_value = -103;
    }

    if (header_buf[3] != 0x00) {
        // NACK Received
        // Map NACK code to error code
        ret_value = 0 - header_buf[3];
    }

    if (len_from_header > len) {
        // response is larger than the caller expects.
        // Pull the bytes out of the Rx fifo
        int32_t ret;
        do {
            uint8_t temp_byte;
            ret = transport_read(&temp_byte, 1);
        } while (ret == 0);
        return -107;
    }
    else if (len_from_header < len) {
        // response is shorter than caller expects.
        len = len_from_header;
    }

    if (ret_value == 0) {
        // If we got here, then we:
        // 1) Received the FRAME_START in the response
        // 2) The message number matched
        // 3) The ACK byte was ACK (not NACK)
        // 4) The received payload length is less than or equal to the size of the buffer
        //      allocated for the payload

        // Grab the payload if there is supposed to be one
        if ((buf != NULL) && (len > 0)) {
            transport_read(buf, len);
        }
    }

    // Finally, make sure the checksum matches
    transport_read(checksum_buff, 2);

    computed_checksum = compute_checksum(header_buf, RESP_HEADER_LEN, buf, len);
    if (((uint16_t)checksum_buff[0] << 8) + checksum_buff[1] != computed_checksum) {
        return(-104);
    }

    if (ret_value == 0) {
        // Success! Return the number of bytes in the payload (0 or positive number)
        return len;
    }
    else {
        // Failure! Return an error, such as NACK response from the firmware
        return ret_value;
    }
}

/**
 * @brief
 *   compute_checksum
 *
 * @param[in] hdr
 *   header array to compute checksum on
 *
 * @param[in] hdr_len
 *   size of the header array in bytes
 *
 * @param[in] payload
 *   payload array to compute checksum on
 *
 * @param[in] payload_len
 *   size of the payload array in bytes
 *
 * @return
 *   The 8-bit checksum
 */
 
void send_packet(opcode_t op, uint8_t message_num, uint8_t *buf, uint16_t len) {
    #define SP_NUM_ZEROS (4)
    #define SP_HEADER_SIZE (CMD_HEADER_LEN + SP_NUM_ZEROS)
    uint8_t header_buf[SP_HEADER_SIZE];
    uint8_t checksum_buff[2];
    uint16_t computed_checksum;
    uint16_t header_idx = 0;
    uint16_t i;

    // Send a couple wakeup bytes, just-in-case
    for (i = 0; i < SP_NUM_ZEROS; i++) {
        header_buf[header_idx ++] = 0xff;
    }

    header_buf[header_idx++] = FRAME_START;
    header_buf[header_idx++] = op;
    header_buf[header_idx++] = message_num;
    header_buf[header_idx++] = (uint8_t)(0xFF & (len >> 8));
    header_buf[header_idx++] = (uint8_t)(0xFF & (len >> 0));

    computed_checksum = compute_checksum(header_buf + SP_NUM_ZEROS, CMD_HEADER_LEN, buf, len);

    transport_write(header_buf, SP_HEADER_SIZE);

    if (buf != NULL) {
        transport_write(buf, len);
    }

    checksum_buff[0] = (computed_checksum >> 8);
    checksum_buff[1] = (computed_checksum >> 0);
    transport_write(checksum_buff, 2);
}

int32_t hal_read_write(opcode_t op, uint8_t buf_in[], uint16_t in_len, uint8_t buf_out[], uint16_t out_len) {
    //  int i;
    //  int curr_byte;
    //  int num_bytes;

    int32_t ret;
    static int32_t message_num = 0;

    // Error checking:
    // Only valid combinations of bufher & length pairs are:
    // buf == NULL, len = 0
    // buf != NULL, len > 0
    if (((buf_in  != NULL) && ( in_len == 0)) || (( buf_in == NULL) && ( in_len > 0))) {
        return(LL_IFC_ERROR_INCORRECT_PARAMETER);
    }

    if (((buf_out != NULL) && (out_len == 0)) || ((buf_out == NULL) && (out_len > 0))) {
        return(LL_IFC_ERROR_INCORRECT_PARAMETER);
    }

    // OK, inputs have been sanitized. Carry on...
    send_packet(op, message_num, buf_in, in_len);

    ret = recv_packet(op, message_num, buf_out, out_len);

    message_num++;

    return(ret);
}

int32_t ll_packet_recv(uint16_t num_timeout_symbols, uint8_t buf[], uint16_t len, uint8_t *bytes_received) {
    uint8_t buff[2];
    int32_t rw_response;

    if (buf == NULL || len <= 0 || bytes_received == NULL) {
        return 0;
    }

    // Make the uint16_t value big-endian
    buff[0] = (num_timeout_symbols >> 8) & 0xFF;
    buff[1] = (num_timeout_symbols >> 0) & 0xFF;

    rw_response = hal_read_write(OP_PKT_RECV, buff, sizeof(num_timeout_symbols), buf, len);

    if (rw_response < 0) {
        *bytes_received = 0;
        return(-1);
    }
    else {
        *bytes_received = (uint8_t) (rw_response & 0xFF);
        return(0);
    }
}

int32_t ll_packet_recv_with_rssi(uint16_t num_timeout_symbols, uint8_t buf[], uint16_t len, uint8_t *bytes_received) {
    uint8_t buff[2];
    int32_t rw_response;

    if (buf == NULL || len <= 0 || bytes_received == NULL) {
        return 0;
    }

    // Make the uint16_t value big-endian
    buff[0] = (num_timeout_symbols >> 8) & 0xFF;
    buff[1] = (num_timeout_symbols >> 0) & 0xFF;

    rw_response = hal_read_write(OP_PKT_RECV_RSSI, buff, sizeof(num_timeout_symbols), buf, len);

    if (rw_response < 0) {
        *bytes_received = 0;
        return(-1);
    }
    else {
        *bytes_received = (uint8_t) (rw_response & 0xFF);
        return(0);
    }
}

int32_t ll_retrieve_message(uint8_t *buf, uint8_t *size, int16_t *rssi, uint8_t *snr) {
    uint8_t rx_size;
    int32_t ret = ll_packet_recv_with_rssi(0, buf, MAX_RX_MSG_LEN + 3, &rx_size);
    if (LL_IFC_ACK > ret) {
        return ret;
    }

    // Size is required
    *size = rx_size - 3;

    // Optional RSSI
    if(NULL != rssi) {
        *rssi = 0;
        *rssi = buf[0] + ((uint16_t)buf[1] << 8);
    }

    // Optional RSSI
    if(NULL != snr) {
        *snr = buf[2];
    }

    //get rid of snr and rssi in buffer
    memmove(buf, buf + 3, *size);

    return ret;
}

int32_t ll_packet_send_queue(uint8_t buf[], uint16_t len) {
    if (buf == NULL || len <= 0) {
        return 0;
    }

    uint8_t cmd_response;

    int32_t rw_response = hal_read_write(OP_PKT_SEND_QUEUE, buf, len, &cmd_response, 1);

    if (rw_response < 0) {
        return((int8_t)rw_response);
    }
    else {
        return(cmd_response);
    }
}

int32_t ll_packet_send(uint8_t buf[], uint16_t len) {
    return ll_packet_send_queue(buf, len);
}

int32_t ll_firmware_type_get(ll_firmware_type_t *t) {
    uint8_t buf[FIRMWARE_TYPE_LEN];
    int32_t ret;
    if(t == NULL) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    ret = hal_read_write(OP_FIRMWARE_TYPE, NULL, 0, buf, FIRMWARE_TYPE_LEN);
    if (ret == FIRMWARE_TYPE_LEN) {
        t->cpu_code = buf[0] << 8 | buf[1];
        t->functionality_code = buf[2] << 8 | buf[3];
    }

    return ret;
}

int32_t ll_hardware_type_get(ll_hardware_type_t *t) {
    uint8_t type;
    int32_t ret;
    if(t == NULL) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    ret = hal_read_write(OP_HARDWARE_TYPE, NULL, 0, &type, sizeof(type));
    if (ret == sizeof(type)) {
        *t = (ll_hardware_type_t) type;
    }

    return ret;
}

int32_t ll_interface_version_get(ll_version_t *version) {
    uint8_t buf[VERSION_LEN];
    int32_t ret;
    if(version == NULL) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    ret = hal_read_write(OP_IFC_VERSION, NULL, 0, buf, VERSION_LEN);
    if (ret == VERSION_LEN) {
        version->major = buf[0];
        version->minor = buf[1];
        version->tag = buf[2] << 8 | buf[3];
    }

    return ret;
}

int32_t ll_version_get(ll_version_t *version) {
    uint8_t buf[VERSION_LEN];
    int32_t ret;
    if(version == NULL) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    ret = hal_read_write(OP_VERSION, NULL, 0, buf, VERSION_LEN);
    if (ret == VERSION_LEN) {
        version->major = buf[0];
        version->minor = buf[1];
        version->tag = buf[2] << 8 | buf[3];
    }

    return ret;
}

int32_t ll_sleep_block(void) {
  return hal_read_write(OP_SLEEP_BLOCK, (uint8_t*) "1", 1, NULL, 0);
}

int32_t ll_sleep_unblock(void) {
  return hal_read_write(OP_SLEEP_BLOCK, (uint8_t*) "0", 1, NULL, 0);
}

int32_t ll_receive_mode_set(uint8_t rx_mode) {
    // TODO: Should these use the enum?
    if (rx_mode >= NUM_DOWNLINK_MODES) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    return hal_read_write(OP_RX_MODE_SET, &rx_mode, 1, NULL, 0);
}

int32_t ll_receive_mode_get(uint8_t *rx_mode) {
    if (rx_mode == NULL) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    return hal_read_write(OP_RX_MODE_GET, NULL, 0, rx_mode, sizeof(*rx_mode));
}

int32_t ll_mailbox_request(void) {
    return hal_read_write(OP_MAILBOX_REQUEST, NULL, 0, NULL, 0);
}

int32_t ll_qos_request(uint8_t qos) {
    if (qos > 15) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    return hal_read_write(OP_QOS_REQUEST, &qos, 1, NULL, 0);
}

int32_t ll_qos_get(uint8_t *qos) {
    if (qos == NULL) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    return hal_read_write(OP_QOS_GET, NULL, 0, qos, sizeof(*qos));
}

int32_t ll_app_token_set(const uint8_t *app_token, uint8_t len) {
    if (app_token == NULL) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    if (10 != len) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    return hal_read_write(OP_APP_TOKEN_SET, (uint8_t*) app_token, 10, NULL, 0);
}

int32_t ll_app_token_get(uint8_t *app_token) {
    if (app_token == NULL) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    return hal_read_write(OP_APP_TOKEN_GET, NULL, 0, app_token, 10);
}

int32_t ll_app_reg_get(uint8_t *is_registered) {
    if (is_registered == NULL) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    return hal_read_write(OP_APP_TOKEN_REG_GET, NULL, 0, is_registered, 1);
}

int32_t ll_encryption_key_exchange_request(void) {
    return hal_read_write(OP_CRYPTO_KEY_XCHG_REQ, NULL, 0, NULL, 0);
}

int32_t ll_get_state(enum ll_state *state, enum ll_tx_state *tx_state, enum ll_rx_state *rx_state) {
    int32_t ret = LL_IFC_ACK;

    if (NULL != state) {
        uint8_t u8_state;
        ret = hal_read_write(OP_STATE, NULL, 0, &u8_state, 1);
        if (LL_IFC_ACK > ret) {
            return ret;
        }

        *state = (enum ll_state)(int8_t)u8_state;
    }

    if (NULL != tx_state) {
        uint8_t u8_tx_state;
        ret = hal_read_write(OP_TX_STATE, NULL, 0, &u8_tx_state, 1);
        if (LL_IFC_ACK > ret) {
            return ret;
        }

        *tx_state = (enum ll_tx_state)(int8_t)u8_tx_state;
    }

    if (NULL != rx_state) {
        uint8_t u8_rx_state;
        ret = hal_read_write(OP_RX_STATE, NULL, 0, &u8_rx_state, 1);
        if (LL_IFC_ACK > ret) {
            return ret;
        }

        *rx_state = (enum ll_rx_state)(int8_t)u8_rx_state;
    }

    return ret;
}

int32_t ll_mac_mode_set(ll_mac_type_t mac_mode) {
    if (mac_mode >= NUM_MACS) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    uint8_t u8_mac_mode = (uint8_t)mac_mode;
    return hal_read_write(OP_MAC_MODE_SET, &u8_mac_mode, 1, NULL, 0);
}

int32_t ll_mac_mode_get(ll_mac_type_t *mac_mode) {
    int32_t ret;
    if (NULL == mac_mode) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    uint8_t u8_mac_mode;
    ret = hal_read_write(OP_MAC_MODE_GET, NULL, 0, &u8_mac_mode, sizeof(uint8_t));
    *mac_mode = (ll_mac_type_t)u8_mac_mode;
    return ret;
}

int32_t ll_antenna_set(uint8_t ant) {
    if((ant == 1) || (ant == 2)) {
        return hal_read_write(OP_ANTENNA_SET, &ant, 1, NULL, 0);
    }
    else {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
}

int32_t ll_antenna_get(uint8_t *ant) {
    if (ant == NULL) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    return hal_read_write(OP_ANTENNA_GET, NULL, 0, ant, 1);
}

int32_t ll_net_token_get(uint32_t *p_net_token) {
    uint8_t buff[4];
    if (p_net_token == NULL) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    int32_t ret = hal_read_write(OP_NET_TOKEN_GET, NULL, 0, buff, 4);
    *p_net_token = 0;
    *p_net_token |= (uint32_t)buff[0] << 24;
    *p_net_token |= (uint32_t)buff[1] << 16;
    *p_net_token |= (uint32_t)buff[2] << 8;
    *p_net_token |= (uint32_t)buff[3];
    
    return ret;
}

int32_t ll_net_token_set(uint32_t net_token) {
    if(net_token != 0xFFFFFFFF) {
        uint8_t buff[4];
        buff[0] = (net_token >> 24) & 0xFF;
        buff[1] = (net_token >> 16) & 0xFF;
        buff[2] = (net_token >> 8) & 0xFF;
        buff[3] = (net_token) & 0xFF;
    
        return hal_read_write(OP_NET_TOKEN_SET, buff, 4, NULL, 0);
    }
    else {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
}

int32_t ll_config_get(uint32_t *net_token, uint8_t app_token[APP_TOKEN_LEN],
                      enum ll_downlink_mode *dl_mode, uint8_t *qos) {
    int32_t ret;

    ret = ll_net_token_get(net_token);
    if (LL_IFC_ACK > ret) {
        return ret;
    }

    ret = ll_app_token_get(app_token);
    if (LL_IFC_ACK > ret) {
        return ret;
    }

    ret = ll_receive_mode_get((uint8_t *)dl_mode);
    if (LL_IFC_ACK > ret) {
        return ret;
    }

    ret = ll_qos_get(qos);
    if (LL_IFC_ACK > ret) {
        return ret;
    }

    return ret;
}

int32_t ll_net_info_get(llabs_network_info_t *p_net_info) {
    uint8_t buff[NET_INFO_BUFF_SIZE];
    if (p_net_info == NULL) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    int32_t ret = hal_read_write(OP_NET_INFO_GET, NULL, 0, buff, NET_INFO_BUFF_SIZE);
    ll_net_info_deserialize(buff, p_net_info);
    
    return ret;
}

int32_t ll_stats_get(llabs_stats_t *s) {
    uint8_t buff[STATS_SIZE];
    if (s == NULL) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    int32_t ret = hal_read_write(OP_STATS_GET, NULL, 0, buff, STATS_SIZE);
    ll_stats_deserialize(buff, s);
    
    return ret;
}

int32_t ll_dl_band_cfg_get(llabs_dl_band_cfg_t *p) {
    uint8_t buff[DL_BAND_CFG_SIZE];
    if (p == NULL) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    int32_t ret = hal_read_write(OP_DL_BAND_CFG_GET, NULL, 0, buff, DL_BAND_CFG_SIZE);
    ll_dl_band_cfg_deserialize(buff, p);
    
    return ret;
}

int32_t ll_dl_band_cfg_set(const llabs_dl_band_cfg_t *p) {
    uint8_t buff[DL_BAND_CFG_SIZE];
    if (p == NULL) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    ll_dl_band_cfg_serialize(p, buff);
    
    return hal_read_write(OP_DL_BAND_CFG_SET, buff, DL_BAND_CFG_SIZE, NULL, 0);
}

int32_t ll_config_set(uint32_t net_token, const uint8_t app_token[APP_TOKEN_LEN], 
                      enum ll_downlink_mode dl_mode, uint8_t qos) {
    int32_t ret;

    ret = ll_net_token_set(net_token);
    if (LL_IFC_ACK > ret) {
        return ret;
    }

    ret = ll_app_token_set(app_token, APP_TOKEN_LEN);
    if (LL_IFC_ACK > ret) {
        return ret;
    }

    ret = ll_receive_mode_set(dl_mode);
    if (LL_IFC_ACK > ret) {
        return ret;
    }

    ret = ll_qos_request(qos);
    if (LL_IFC_ACK > ret) {
        return ret;
    }

    return ret;
}

int32_t ll_rssi_scan_set(uint32_t u1, uint32_t u2, uint32_t u3, uint32_t u4) {
    uint8_t buf[16];

    memset(buf, 0, sizeof(buf));

    // Big Endian
    buf[ 0] = (u1 >> 24) & 0xFF;
    buf[ 1] = (u1 >> 16) & 0xFF;
    buf[ 2] = (u1 >>  8) & 0xFF;
    buf[ 3] = (u1      ) & 0xFF;

    buf[ 4] = (u2 >> 24) & 0xFF;
    buf[ 5] = (u2 >> 16) & 0xFF;
    buf[ 6] = (u2 >>  8) & 0xFF;
    buf[ 7] = (u2      ) & 0xFF;

    buf[ 8] = (u3 >> 24) & 0xFF;
    buf[ 9] = (u3 >> 16) & 0xFF;
    buf[10] = (u3 >>  8) & 0xFF;
    buf[11] = (u3      ) & 0xFF;

    buf[12] = (u4 >> 24) & 0xFF;
    buf[13] = (u4 >> 16) & 0xFF;
    buf[14] = (u4 >>  8) & 0xFF;
    buf[15] = (u4      ) & 0xFF;

    return hal_read_write(OP_RSSI_SET, buf, 16, NULL, 0);
}

int32_t ll_rssi_scan_get(uint8_t buf[], uint16_t len, uint8_t *bytes_received) {
    int32_t rw_response;

    if (buf == NULL || len <= 0 || bytes_received == NULL) {
        return 0;
    }

    rw_response = hal_read_write(OP_RSSI_GET, NULL, 0, buf, len);

    if (rw_response < 0) {
        *bytes_received = 0;
        return(-1);
    }
    else {
        *bytes_received = (uint8_t) (rw_response & 0xFF);
        return(0);
    }
}

int32_t ll_unique_id_get(uint64_t *unique_id) {
    uint8_t buff[8];
    int32_t ret;
    int i;
    if (unique_id == NULL) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    ret = hal_read_write(OP_MODULE_ID, NULL, 0, buff, 8);
    *unique_id = 0;
    for (i = 0; i < 8; i++) {
        *unique_id |= ((uint64_t) buff[i]) << (8 * (7 - i));
    }

    return ret;
}

int32_t ll_radio_params_get(uint8_t *sf, uint8_t *cr, uint8_t *bw, uint32_t *freq,
                            uint16_t *preamble_syms, uint8_t *header_enabled, uint8_t *crc_enabled,
                            uint8_t *iq_inverted) {
    int32_t ret;
    uint8_t buff[8];
    ret = hal_read_write(OP_GET_RADIO_PARAMS, NULL, 0, buff, 8);

    *sf = (buff[0] >> 4) + 6;
    *cr = ((buff[0] >> 2) & 0x03) + 1;
    *bw = buff[0] & 0x03;

    *header_enabled = buff[1] & (1u << 0u);
    *crc_enabled = buff[1] & (1u << 1u);
    *iq_inverted = buff[1] & (1u << 2u);

    *preamble_syms = ((uint16_t)buff[2] << 8) | (uint16_t)buff[3];

    *freq  = ((uint32_t)buff[4] << 24);
    *freq |= ((uint32_t)buff[5] << 16);
    *freq |= ((uint32_t)buff[6] <<  8);
    *freq |= ((uint32_t)buff[7]      );

    return ret;
}

int32_t ll_radio_params_set(uint8_t flags, uint8_t sf, uint8_t cr, uint8_t bw, uint32_t freq,
                            uint16_t preamble_syms, uint8_t enable_header, uint8_t enable_crc,
                            uint8_t enable_iq_inversion) {
    uint8_t buf[9];

    memset(buf, 0, sizeof(buf));

    buf[0] = flags;

    if (flags & RADIO_PARAM_FLAGS_SF) {
        if (sf < 6 || sf > 12) {
           return LL_IFC_ERROR_INCORRECT_PARAMETER;
        }

        sf = sf - 6;
        buf[1] |= ((sf&0x07) << 4);
    }

    if (flags & RADIO_PARAM_FLAGS_CR) {
        if (cr < 1 || cr > 4) {
            return LL_IFC_ERROR_INCORRECT_PARAMETER;
        }

        cr = cr - 1;
        buf[1] |= ((cr&0x03) << 2);
    }

    if (flags & RADIO_PARAM_FLAGS_BW) {
        if (bw > 3) {
            return LL_IFC_ERROR_INCORRECT_PARAMETER;
        }

        buf[1] |= ((bw&0x03)     );
    }

    if ((flags & RADIO_PARAM_FLAGS_HEADER) && enable_header) {
        buf[2] |= (1u << 0u);
    }

    if ((flags & RADIO_PARAM_FLAGS_CRC) && enable_crc) {
        buf[2] |= (1u << 1u);
    }

    if ((flags & RADIO_PARAM_FLAGS_IQ) && enable_iq_inversion) {
        buf[2] |= (1u << 2u);
    }

    if (flags & RADIO_PARAM_FLAGS_PREAMBLE) {
        buf[3] = (preamble_syms >> 8) & 0xFF;
        buf[4] = (preamble_syms >> 0) & 0xFF;
    }

    if(flags & RADIO_PARAM_FLAGS_FREQ) {
        buf[5] = (freq >> 24) & 0xFF;
        buf[6] = (freq >> 16) & 0xFF;
        buf[7] = (freq >>  8) & 0xFF;
        buf[8] = (freq      ) & 0xFF;
    }

    return hal_read_write(OP_SET_RADIO_PARAMS, buf, 9, NULL, 0);
}

int32_t ll_bandwidth_set(uint8_t bandwidth) {
    return ll_radio_params_set(RADIO_PARAM_FLAGS_BW, 0, 0, bandwidth, 0, 0, 0, 0, 0);
}

int32_t ll_spreading_factor_set(uint8_t sf) {
    return ll_radio_params_set(RADIO_PARAM_FLAGS_SF, sf, 0, 0, 0, 0, 0, 0, 0);
}

int32_t ll_coding_rate_set(uint8_t coding_rate) {
    return ll_radio_params_set(RADIO_PARAM_FLAGS_CR, 0, coding_rate, 0, 0, 0, 0, 0, 0);
}

int32_t ll_tx_power_set(int8_t pwr) {
    if (pwr < -4 || pwr > 26) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    return hal_read_write(OP_TX_POWER_SET, (uint8_t *)&pwr, 1, NULL, 0);
}

int32_t ll_tx_power_get(int8_t *pwr) {
    if (NULL == pwr) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    return hal_read_write(OP_TX_POWER_GET, NULL, 0, (uint8_t *)pwr, 1);
}

int32_t ll_frequency_set(uint32_t freq) {
    return ll_radio_params_set(RADIO_PARAM_FLAGS_FREQ, 0, 0, 0, freq, 0, 0, 0, 0);
}

int32_t ll_preamble_syms_set(uint16_t num_syms) {
    return ll_radio_params_set(RADIO_PARAM_FLAGS_PREAMBLE, 0, 0, 0, 0, num_syms, 0, 0, 0);
}

int32_t ll_header_enabled_set(uint8_t enabled) {
    return ll_radio_params_set(RADIO_PARAM_FLAGS_HEADER, 0, 0, 0, 0, 0, enabled, 0, 0);
}

int32_t ll_crc_enabled_set(uint8_t enabled) {
    return ll_radio_params_set(RADIO_PARAM_FLAGS_CRC, 0, 0, 0, 0, 0, 0, enabled, 0);
}

int32_t ll_iq_inversion_set(uint8_t inverted) {
    return ll_radio_params_set(RADIO_PARAM_FLAGS_IQ, 0, 0, 0, 0, 0, 0, 0, inverted);
}

int32_t ll_sync_word_set(uint8_t sync_word) {
    return hal_read_write(OP_SYNC_WORD_SET, (uint8_t *)&sync_word, 1, NULL, 0);
}

int32_t ll_sync_word_get(uint8_t *sync_word) {
    if (NULL == sync_word) {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    return hal_read_write(OP_SYNC_WORD_GET, NULL, 0, sync_word, 1);
}

int32_t ll_settings_store(void) {
    return hal_read_write(OP_STORE_SETTINGS, NULL, 0, NULL, 0);
}

int32_t ll_settings_delete(void) {
    return hal_read_write(OP_DELETE_SETTINGS, NULL, 0, NULL, 0);
}

int32_t ll_restore_defaults(void) {
    return hal_read_write(OP_RESET_SETTINGS, NULL, 0, NULL, 0);
}

int32_t ll_sleep(void) {
    return hal_read_write(OP_SLEEP, NULL, 0, NULL, 0);
}

int32_t ll_reset_mcu(void) {
    return hal_read_write(OP_RESET_MCU, NULL, 0, NULL, 0);
}

int32_t ll_bootloader_mode(void) {
    return hal_read_write(OP_TRIGGER_BOOTLOADER, NULL, 0, NULL, 0);
}

int32_t ll_echo_mode() {
    return hal_read_write(OP_PKT_ECHO, NULL, 0, NULL, 0);
}

int32_t ll_packet_recv_cont(uint8_t buf[], uint16_t len, uint8_t *bytes_received) {
    int32_t rw_response;

    if (buf == NULL || len == 0 || bytes_received == NULL) {
        return 0;
    }

    rw_response = hal_read_write(OP_PKT_RECV_CONT, NULL, 0, buf, len);

    if (rw_response < 0) {
        *bytes_received = 0;
        return(-1);
    }
    else {
        *bytes_received = (uint8_t) (rw_response & 0xFF);
        return(0);
    }
}

/**
 * @return
 *   0 - success
 *   negative = error, as defined by hal_read_write
 */
int32_t ll_irq_flags(uint32_t flags_to_clear, uint32_t *flags) {
    // Assuming big endian convention over the interface

    uint8_t in_buf[4];
    uint8_t out_buf[4] = {0,0,0,0};

    in_buf[0] = (uint8_t)((flags_to_clear >> 24) & 0xFF);
    in_buf[1] = (uint8_t)((flags_to_clear >> 16) & 0xFF);
    in_buf[2] = (uint8_t)((flags_to_clear >>  8) & 0xFF);
    in_buf[3] = (uint8_t)((flags_to_clear      ) & 0xFF);

    int32_t rw_response = hal_read_write(OP_IRQ_FLAGS, in_buf, 4, out_buf, 4);

    if(rw_response > 0) {
        uint32_t flags_temp = 0;
        flags_temp |= (((uint32_t)out_buf[0]) << 24);
        flags_temp |= (((uint32_t)out_buf[1]) << 16);
        flags_temp |= (((uint32_t)out_buf[2]) << 8);
        flags_temp |= (((uint32_t)out_buf[3]));
        *flags = flags_temp;
    }

    return(rw_response);
}

int32_t ll_packet_send_ack(uint8_t buf[], uint16_t len) {
    if (buf == NULL || len <= 0) {
        return 0;
    }

    return hal_read_write(OP_PKT_SEND_ACK, buf, len, NULL, 0);
}

int32_t ll_packet_send_unack(uint8_t buf[], uint16_t len) {
    if (buf == NULL || len <= 0) {
        return 0;
    }

    return hal_read_write(OP_PKT_SEND_UNACK, buf, len, NULL, 0);
}

int32_t ll_transmit_cw(void) {
    return hal_read_write(OP_TX_CW, NULL, 0, NULL, 0);
}
