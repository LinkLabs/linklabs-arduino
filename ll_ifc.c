#include "ll_ifc.h"
#include "ll_ifc_consts.h"
#include "ifc_struct_defs.h"
#include "ll_ifc_symphony.h"
#include "ll_ifc_no_mac.h"
#include "time.h"
#include <stdio.h>
#include <string.h>

#ifndef NULL    // <time.h> defines NULL on *some* platforms
#define NULL                (0)
#endif
#define CMD_HEADER_LEN      (5)
#define RESP_HEADER_LEN     (6)
static uint16_t compute_checksum(uint8_t *hdr, uint16_t hdr_len, uint8_t *payload, uint16_t payload_len);
static void send_packet(opcode_t op, uint8_t message_num, uint8_t *buf, uint16_t len);
static int32_t recv_packet(opcode_t op, uint8_t message_num, uint8_t *buf, uint16_t len);

const uint32_t OPEN_NET_TOKEN = 0x4f50454e;

static int32_t message_num = 0;

int32_t hal_read_write(opcode_t op, uint8_t buf_in[], uint16_t in_len, uint8_t buf_out[], uint16_t out_len)
{
    //  int i;
    //  int curr_byte;
    //  int num_bytes;

    int32_t ret;

    // Error checking:
    // Only valid combinations of bufher & length pairs are:
    // buf == NULL, len = 0
    // buf != NULL, len > 0
    if (((buf_in  != NULL) && ( in_len == 0)) || (( buf_in == NULL) && ( in_len > 0)))
    {
        return(LL_IFC_ERROR_INCORRECT_PARAMETER);
    }
    if (((buf_out != NULL) && (out_len == 0)) || ((buf_out == NULL) && (out_len > 0)))
    {
        return(LL_IFC_ERROR_INCORRECT_PARAMETER);
    }

    // OK, inputs have been sanitized. Carry on...
    send_packet(op, message_num, buf_in, in_len);

    ret = recv_packet(op, message_num, buf_out, out_len);

    message_num++;

    return(ret);
}

int32_t hal_read_write_exact(opcode_t op, uint8_t buf_in[], uint16_t in_len, uint8_t buf_out[], uint16_t out_len)
{
    int32_t ret = hal_read_write(op, buf_in, in_len, buf_out, out_len);
    if (ret >= 0)
    {
        if (ret != out_len)
        {
            return LL_IFC_ERROR_INCORRECT_RESPONSE_LENGTH;
        }
        ret = 0;
    }
    return ret;
}

char const * ll_return_code_name(int32_t return_code)
{
    switch (return_code)
    {
        case -LL_IFC_ACK:                            return "ACK";
        case -LL_IFC_NACK_CMD_NOT_SUPPORTED:         return "CMD_NOT_SUPPORTED";
        case -LL_IFC_NACK_INCORRECT_CHKSUM:          return "INCORRECT_CHKSUM ";
        case -LL_IFC_NACK_PAYLOAD_LEN_OOR:           return "PAYLOAD_LEN_OOR";
        case -LL_IFC_NACK_PAYLOAD_OOR:               return "PAYLOAD_OOR";
        case -LL_IFC_NACK_BOOTUP_IN_PROGRESS:        return "BOOTUP_IN_PROGRESS";
        case -LL_IFC_NACK_BUSY_TRY_AGAIN:            return "BUSY_TRY_AGAIN";
        case -LL_IFC_NACK_APP_TOKEN_REG:             return "APP_TOKEN_REG";
        case -LL_IFC_NACK_PAYLOAD_LEN_EXCEEDED:      return "PAYLOAD_LEN_EXCEEDED";
        case -LL_IFC_NACK_NOT_IN_MAILBOX_MODE:       return "NOT IN MAILBOX MODE";
        case -LL_IFC_NACK_PAYLOAD_BAD_PROPERTY:      return "BAD PROPERTY ID";
        case -LL_IFC_NACK_NODATA:                    return "NO DATA AVAIL";
        case -LL_IFC_NACK_QUEUE_FULL:                return "QUEUE FULL";
        case -LL_IFC_NACK_OTHER:                     return "OTHER";

        case LL_IFC_ERROR_INCORRECT_PARAMETER:       return "INCORRECT_PARAMETER";
        case LL_IFC_ERROR_INCORRECT_RESPONSE_LENGTH: return "INCORRECT_RESPONSE_LENGTH";
        case LL_IFC_ERROR_MESSAGE_NUMBER_MISMATCH:   return "MESSAGE_NUMBER_MISMATCH";
        case LL_IFC_ERROR_CHECKSUM_MISMATCH:         return "CHECKSUM_MISMATCH";
        case LL_IFC_ERROR_COMMAND_MISMATCH:          return "COMMAND_MISMATCH";
        case LL_IFC_ERROR_HOST_INTERFACE_TIMEOUT:    return "HOST_INTERFACE_TIMEOUT";
        case LL_IFC_ERROR_BUFFER_TOO_SMALL:          return "BUFFER_TOO_SMALL";
        case LL_IFC_ERROR_START_OF_FRAME:            return "START_OF_FRAME";
        case LL_IFC_ERROR_HEADER:                    return "HEADER";
        case LL_IFC_ERROR_TIMEOUT:                   return "TIMEOUT";
        case LL_IFC_ERROR_INCORRECT_MESSAGE_SIZE:    return "INCORRECT_MESSAGE_SIZE";
        case LL_IFC_ERROR_NO_NETWORK:                return "NO_NETWORK";

        default:                                     return "UNKNOWN";
    }
}

char const * ll_return_code_description(int32_t return_code)
{
    switch (return_code)
    {
        case -LL_IFC_ACK:                            return "success";
        case -LL_IFC_NACK_CMD_NOT_SUPPORTED:         return "Command not supported";
        case -LL_IFC_NACK_INCORRECT_CHKSUM:          return "Incorrect Checksum";
        case -LL_IFC_NACK_PAYLOAD_LEN_OOR:           return "Length of payload sent in command was out of range";
        case -LL_IFC_NACK_PAYLOAD_OOR:               return "Payload sent in command was out of range.";
        case -LL_IFC_NACK_BOOTUP_IN_PROGRESS:        return "Not allowed since firmware bootup still in progress. Wait.";
        case -LL_IFC_NACK_BUSY_TRY_AGAIN:            return "Operation prevented by temporary event. Retry later.";
        case -LL_IFC_NACK_APP_TOKEN_REG:             return "Application token is not registered for this node.";
        case -LL_IFC_NACK_PAYLOAD_LEN_EXCEEDED:      return "Payload length is greater than the max supported length";
        case -LL_IFC_NACK_NOT_IN_MAILBOX_MODE:       return "Command invalid, not in mailbox mode";
        case -LL_IFC_NACK_PAYLOAD_BAD_PROPERTY:      return "Bad property ID specified";
        case -LL_IFC_NACK_NODATA:                    return "No msg data available to return";
        case -LL_IFC_NACK_QUEUE_FULL:                return "Data cannot be enqueued for transmission, queue is full";
        case -LL_IFC_NACK_OTHER:                     return "Unspecified error";

        case LL_IFC_ERROR_INCORRECT_PARAMETER:       return "The parameter value was invalid";
        case LL_IFC_ERROR_INCORRECT_RESPONSE_LENGTH: return "Module response was not the expected size";
        case LL_IFC_ERROR_MESSAGE_NUMBER_MISMATCH:   return "Message number in response doesn't match expected";
        case LL_IFC_ERROR_CHECKSUM_MISMATCH:         return "Checksum mismatch";
        case LL_IFC_ERROR_COMMAND_MISMATCH:          return "Command mismatch (responding to a different command)";
        case LL_IFC_ERROR_HOST_INTERFACE_TIMEOUT:    return "Timed out waiting for Rx bytes from interface";
        case LL_IFC_ERROR_BUFFER_TOO_SMALL:          return "Response larger than provided output buffer";
        case LL_IFC_ERROR_START_OF_FRAME:            return "transport_read failed getting FRAME_START";
        case LL_IFC_ERROR_HEADER:                    return "transport_read failed getting header";
        case LL_IFC_ERROR_TIMEOUT:                   return "The operation timed out";
        case LL_IFC_ERROR_INCORRECT_MESSAGE_SIZE:    return "The message size from the device was incorrect";
        case LL_IFC_ERROR_NO_NETWORK:                return "No network was available";

        default:                                     return "unknown error";
    }
}

int32_t ll_firmware_type_get(ll_firmware_type_t *t)
{
    uint8_t buf[FIRMWARE_TYPE_LEN];
    int32_t ret;
    if(t == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    ret = hal_read_write(OP_FIRMWARE_TYPE, NULL, 0, buf, FIRMWARE_TYPE_LEN);
    if (ret == FIRMWARE_TYPE_LEN)
    {
        t->cpu_code = buf[0] << 8 | buf[1];
        t->functionality_code = buf[2] << 8 | buf[3];
    }
    return ret;
}

int32_t ll_hardware_type_get(ll_hardware_type_t *t)
{
    uint8_t type;
    int32_t ret;
    if(t == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    ret = hal_read_write(OP_HARDWARE_TYPE, NULL, 0, &type, sizeof(type));
    if (ret == sizeof(type))
    {
        *t = (ll_hardware_type_t) type;
    }
    return ret;
}

const char * ll_hardware_type_string(ll_hardware_type_t t)
{
    switch(t)
    {
        case UNAVAILABLE: return "unavailable";
        case LLRLP20_V2:  return "LLRLP20 v2";
        case LLRXR26_V2:  return "LLRXR26 v2";
        case LLRLP20_V3:  return "LLRLP20 v3";
        case LLRXR26_V3:  return "LLRXR26 v3";
        default:          return "unknown";
    }
}

int32_t ll_interface_version_get(ll_version_t *version)
{
    uint8_t buf[VERSION_LEN];
    int32_t ret;
    if(version == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    ret = hal_read_write(OP_IFC_VERSION, NULL, 0, buf, VERSION_LEN);
    if (ret == VERSION_LEN)
    {
        version->major = buf[0];
        version->minor = buf[1];
        version->tag = buf[2] << 8 | buf[3];
    }
    return ret;
}

int32_t ll_version_get(ll_version_t *version)
{
    uint8_t buf[VERSION_LEN];
    int32_t ret;
    if(version == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    ret = hal_read_write(OP_VERSION, NULL, 0, buf, VERSION_LEN);
    if (ret == VERSION_LEN)
    {
        version->major = buf[0];
        version->minor = buf[1];
        version->tag = buf[2] << 8 | buf[3];
    }
    return ret;
}

int32_t ll_sleep_block(void)
{
    return hal_read_write(OP_SLEEP_BLOCK, (uint8_t*) "1", 1, NULL, 0);
}

int32_t ll_sleep_unblock(void)
{
    return hal_read_write(OP_SLEEP_BLOCK, (uint8_t*) "0", 1, NULL, 0);
}

int32_t ll_mac_mode_set(ll_mac_type_t mac_mode)
{
    if (mac_mode >= NUM_MACS)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    uint8_t u8_mac_mode = (uint8_t)mac_mode;
    return hal_read_write(OP_MAC_MODE_SET, &u8_mac_mode, 1, NULL, 0);
}

int32_t ll_mac_mode_get(ll_mac_type_t *mac_mode)
{
    int32_t ret;
    if (NULL == mac_mode)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    uint8_t u8_mac_mode;
    ret = hal_read_write(OP_MAC_MODE_GET, NULL, 0, &u8_mac_mode, sizeof(uint8_t));
    *mac_mode = (ll_mac_type_t)u8_mac_mode;
    return ret;
}

int32_t ll_antenna_set(uint8_t ant)
{
    if((ant == 1) || (ant == 2))
    {
        return hal_read_write(OP_ANTENNA_SET, &ant, 1, NULL, 0);
    }
    else
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
}

int32_t ll_antenna_get(uint8_t *ant)
{
    if (ant == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    return hal_read_write(OP_ANTENNA_GET, NULL, 0, ant, 1);
}

int32_t ll_unique_id_get(uint64_t *unique_id)
{
    uint8_t buff[8];
    int32_t ret;
    int i;
    if (unique_id == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    ret = hal_read_write(OP_MODULE_ID, NULL, 0, buff, 8);
    *unique_id = 0;
    for (i = 0; i < 8; i++)
    {
        *unique_id |= ((uint64_t) buff[i]) << (8 * (7 - i));
    }

    return ret;
}

int32_t ll_settings_store(void)
{
    return hal_read_write(OP_STORE_SETTINGS, NULL, 0, NULL, 0);
}

int32_t ll_settings_delete(void)
{
    return hal_read_write(OP_DELETE_SETTINGS, NULL, 0, NULL, 0);
}

int32_t ll_restore_defaults(void)
{
    return hal_read_write(OP_RESET_SETTINGS, NULL, 0, NULL, 0);
}

int32_t ll_sleep(void)
{
    return hal_read_write(OP_SLEEP, NULL, 0, NULL, 0);
}

int32_t ll_reset_mcu(void)
{
    return hal_read_write(OP_RESET_MCU, NULL, 0, NULL, 0);
}

int32_t ll_bootloader_mode(void)
{
    return hal_read_write(OP_TRIGGER_BOOTLOADER, NULL, 0, NULL, 0);
}

/**
 * @return
 *   0 - success
 *   negative = error, as defined by hal_read_write
 */
int32_t ll_irq_flags(uint32_t flags_to_clear, uint32_t *flags)
{
    // Assuming big endian convention over the interface

    uint8_t in_buf[4];
    uint8_t out_buf[4] = {0,0,0,0};

    in_buf[0] = (uint8_t)((flags_to_clear >> 24) & 0xFF);
    in_buf[1] = (uint8_t)((flags_to_clear >> 16) & 0xFF);
    in_buf[2] = (uint8_t)((flags_to_clear >>  8) & 0xFF);
    in_buf[3] = (uint8_t)((flags_to_clear      ) & 0xFF);

    int32_t rw_response = hal_read_write(OP_IRQ_FLAGS, in_buf, 4, out_buf, 4);

    if(rw_response > 0)
    {
        uint32_t flags_temp = 0;
        flags_temp |= (((uint32_t)out_buf[0]) << 24);
        flags_temp |= (((uint32_t)out_buf[1]) << 16);
        flags_temp |= (((uint32_t)out_buf[2]) << 8);
        flags_temp |= (((uint32_t)out_buf[3]));
        *flags = flags_temp;
    }

    return(rw_response);
}


int32_t ll_reset_state( void )
{
    message_num = 0;
    return 0;
}


/*
 * Command/response payload formats
 *
 * OP_LORAWAN_ACTIVATE
 *
 * command:
 *  - subtype (1 byte): ll_lorawan_activation_e
 *  - remaining payload...
 *
 * For LL_LORAWAN_ACTIVATION_OVER_THE_AIR:
 *  - network_type (1 byte)
 *  - device_class (1 byte)
 *  - devEui (8 bytes)
 *  - appEui (8 bytes)
 *  - appKey (16 bytes)
 *
 * For LL_LORAWAN_ACTIVATION_PERSONALIZATION:
 *  - network_type (1 byte)
 *  - device_class (1 byte)
 *  - netID (4 bytes)
 *  - devAddr (4 bytes)
 *  - netSKey (16 bytes)
 *  - appSKey (16 bytes)
 *
 * For LL_LORAWAN_ACTIVATION_QUERY, no additional payload
 *
 * response:
 *  - status (1 byte): ll_lorawan_activation_status_e
 *
 * OP_LORAWAN_PARAM
 *
 * command:
 *  - data_type (1 byte): 0x03 = i32
 *  - parameter_id (1 byte)
 *  - data_size (1 byte): in bytes, 0 for get operation
 *  - data (data_size bytes): big endian
 *
 * response:
 *  - data_type (1 byte)
 *  - parameter_id (1 byte)
 *  - data_size (1 byte)
 *  - data (data_size bytes): big endian
 *
 *
 * OP_LORAWAN_MSG_SEND
 *
 * command:
 *  - flags (1 byte)
 *  - fPort (1 byte)
 *  - retries (1 byte)
 *  - data_size (1 byte)
 *  - data (data_size bytes)
 *
 * response: empty (just ACK/status bit used)
 *
 *
 * OP_LORAWAN_MSG_RECEIVE (poll for an already received message)
 *
 * command has no payload
 *
 * response:
 *  - flags (1 byte):
 *    - rx: a receive message with payload was received
 *    - rx_multicast: the received message was multicast
 *    - ack: an acknowledgment was received
 *    - link_check: link check data was received
 *  - TxNbRetries (1 byte): only valid if tx
 *  - DemodMargin (1 byte): only valid if link_check is set
 *  - NbGateways (1 byte): only valid if link_check is set
 *  - RxRssi (1 byte): 0xff = +20 dB
 *  - RxSnr (1 byte)
 *  - RxPort (1 byte), only valid if rx
 *  - bytes_received (1 byte), only valid if rx
 *  - data (bytes_received bytes), only valid if rx
 *
 */

/**
 * @brief
 *  send_packet
 *
 * @param[in] op
 *   opcode of the command being sent to the module
 *
 * @param[in] message_num
 *   message_num
 *
 * @param[in] buf
 *   byte array containing the data payload to be sent to the module
 *
 * @param[in] len
 *   size of the output buffer in bytes
 *
 * @return
 *   none
 */
static void send_packet(opcode_t op, uint8_t message_num, uint8_t *buf, uint16_t len)
{
    #define SP_NUM_ZEROS (4)
    #define SP_HEADER_SIZE (CMD_HEADER_LEN + SP_NUM_ZEROS)
    uint8_t header_buf[SP_HEADER_SIZE];
    uint8_t checksum_buff[2];
    uint16_t computed_checksum;
    uint16_t header_idx = 0;
    uint16_t i;

    // Send a couple wakeup bytes, just-in-case
    for (i = 0; i < SP_NUM_ZEROS; i++)
    {
        header_buf[header_idx ++] = 0xff;
    }

    header_buf[header_idx++] = FRAME_START;
    header_buf[header_idx++] = op;
    header_buf[header_idx++] = message_num;
    header_buf[header_idx++] = (uint8_t)(0xFF & (len >> 8));
    header_buf[header_idx++] = (uint8_t)(0xFF & (len >> 0));

    computed_checksum = compute_checksum(header_buf + SP_NUM_ZEROS, CMD_HEADER_LEN, buf, len);

    transport_write(header_buf, SP_HEADER_SIZE);

    if (buf != NULL)
    {
        transport_write(buf, len);
    }

    checksum_buff[0] = (computed_checksum >> 8);
    checksum_buff[1] = (computed_checksum >> 0);
    transport_write(checksum_buff, 2);

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
static int32_t recv_packet(opcode_t op, uint8_t message_num, uint8_t *buf, uint16_t len)
{
    uint8_t  header_buf[RESP_HEADER_LEN];
    uint16_t header_idx;

    uint8_t curr_byte = 0;
    uint8_t checksum_buff[2];
    uint16_t computed_checksum;
    int32_t ret_value = 0;
    int32_t ret;

    memset(header_buf, 0, sizeof(header_buf));
    //TODO: have conditionally compiled cases for various platforms to ensure accurate timeout
   // clock_t max_clock = (clock_t) (1.5 * (float)CLOCKS_PER_SEC);
 //   clock_t t = clock();

    do
    {
        /* Timeout of infinite Rx loop if responses never show up*/
        ret = transport_read(&curr_byte, 1);
        if(ret<0)
        {
            len = 0;
            return LL_IFC_ERROR_HOST_INTERFACE_TIMEOUT;
        }
    } while(curr_byte != FRAME_START);

    if (ret < 0)
    {
        /* transport_read failed - return an error */
        return LL_IFC_ERROR_START_OF_FRAME;
    }

    header_idx = 0;
    header_buf[header_idx++] = FRAME_START;

    ret = transport_read(header_buf + 1, RESP_HEADER_LEN - 1);
    if (ret < 0)
    {
        /* transport_read failed - return an error */
        return LL_IFC_ERROR_HEADER;
    }

    uint16_t len_from_header = (uint16_t)header_buf[5] + ((uint16_t)header_buf[4] << 8);

    if (header_buf[1] != op)
    {
        // Command Byte should match what was sent
        ret_value = LL_IFC_ERROR_COMMAND_MISMATCH;
    }
    if (header_buf[2] != message_num)
    {
        // Message Number should match
        ret_value = LL_IFC_ERROR_MESSAGE_NUMBER_MISMATCH;
    }
    if (header_buf[3] != 0x00)
    {
        // NACK Received
        // Map NACK code to error code
        ret_value = 0 - header_buf[3];
    }
    if (len_from_header > len)
    {
        // response is larger than the caller expects.
        // Pull the bytes out of the Rx fifo
        int32_t ret;
        do
        {
            uint8_t temp_byte;
            ret = transport_read(&temp_byte, 1);
        }
        while (ret == 0);
        return LL_IFC_ERROR_BUFFER_TOO_SMALL;
    }
    else if (len_from_header < len)
    {
        // response is shorter than caller expects.
        len = len_from_header;
    }

    if (ret_value == 0)
    {

        // If we got here, then we:
        // 1) Received the FRAME_START in the response
        // 2) The message number matched
        // 3) The ACK byte was ACK (not NACK)
        // 4) The received payload length is less than or equal to the size of the buffer
        //      allocated for the payload

        // Grab the payload if there is supposed to be one
        if ((buf != NULL) && (len > 0))
        {
            transport_read(buf, len);
        }
    }

    // Finally, make sure the checksum matches
    transport_read(checksum_buff, 2);

    computed_checksum = compute_checksum(header_buf, RESP_HEADER_LEN, buf, len);
    if (((uint16_t)checksum_buff[0] << 8) + checksum_buff[1] != computed_checksum)
    {
        return LL_IFC_ERROR_CHECKSUM_MISMATCH;
    }

    if (ret_value == 0)
    {
        // Success! Return the number of bytes in the payload (0 or positive number)
        return len;
    }
    else
    {
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
static uint16_t compute_checksum(uint8_t *hdr, uint16_t hdr_len, uint8_t *payload, uint16_t payload_len)
{
    uint16_t crc = 0x0;
    uint16_t i;

    for (i = 0; i < hdr_len; i++)
    {
        crc  = (crc >> 8) | (crc << 8);
        crc ^= hdr[i];
        crc ^= (crc & 0xff) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0xff) << 5;
    }

    for (i = 0; i < payload_len; i++)
    {
        crc  = (crc >> 8) | (crc << 8);
        crc ^= payload[i];
        crc ^= (crc & 0xff) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0xff) << 5;
    }

    return crc;
}
