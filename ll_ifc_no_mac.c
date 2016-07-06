#include "ll_ifc_no_mac.h"
#include "ll_ifc.h"
#include "ll_ifc_private.h"
#include <string.h>  // memset

int32_t ll_rssi_scan_set(uint32_t u1, uint32_t u2, uint32_t u3, uint32_t u4)
{
    uint8_t buf[16];

    memset(buf, 0, sizeof(buf));

    // Little Endian
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

int32_t ll_rssi_scan_get(uint8_t buf[], uint16_t len, uint8_t *bytes_received)
{
    int32_t rw_response;

    if (buf == NULL || len <= 0 || bytes_received == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    rw_response = hal_read_write(OP_RSSI_GET, NULL, 0, buf, len);

    if (rw_response < 0)
    {
        *bytes_received = 0;
        return(-1);
    }
    else
    {
        *bytes_received = (uint8_t) (rw_response & 0xFF);
        return(0);
    }
}

int32_t ll_radio_params_get(uint8_t *sf, uint8_t *cr, uint8_t *bw, uint32_t *freq,
                            uint16_t *preamble_syms, uint8_t *header_enabled, uint8_t *crc_enabled,
                            uint8_t *iq_inverted)
{
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

    *freq  = (uint32_t)(buff[4] << 24);
    *freq |= (uint32_t)(buff[5] << 16);
    *freq |= (uint32_t)(buff[6] <<  8);
    *freq |= (uint32_t)(buff[7]      );

    return ret;
}

int32_t ll_radio_params_set(uint8_t flags, uint8_t sf, uint8_t cr, uint8_t bw, uint32_t freq,
                            uint16_t preamble_syms, uint8_t enable_header, uint8_t enable_crc,
                            uint8_t enable_iq_inversion)
{
    uint8_t buf[9];

    memset(buf, 0, sizeof(buf));

    buf[0] = flags;

    if(flags & RADIO_PARAM_FLAGS_SF)
    {
        if (sf < 6 || sf > 12)
        {
           return LL_IFC_ERROR_INCORRECT_PARAMETER;
        }
        sf = sf - 6;
        buf[1] |= ((sf&0x07) << 4);
    }
    if(flags & RADIO_PARAM_FLAGS_CR)
    {
        if (cr < 1 || cr > 4)
        {
            return LL_IFC_ERROR_INCORRECT_PARAMETER;
        }
        cr = cr - 1;
        buf[1] |= ((cr&0x03) << 2);
    }
    if(flags & RADIO_PARAM_FLAGS_BW)
    {
        if (bw > 3)
        {
            return LL_IFC_ERROR_INCORRECT_PARAMETER;
        }
        buf[1] |= ((bw&0x03)     );
    }

    if ((flags & RADIO_PARAM_FLAGS_HEADER) && enable_header)
    {
        buf[2] |= (1u << 0u);
    }
    if ((flags & RADIO_PARAM_FLAGS_CRC) && enable_crc)
    {
        buf[2] |= (1u << 1u);
    }
    if ((flags & RADIO_PARAM_FLAGS_IQ) && enable_iq_inversion)
    {
        buf[2] |= (1u << 2u);
    }

    if (flags & RADIO_PARAM_FLAGS_PREAMBLE)
    {
        buf[3] = (preamble_syms >> 8) & 0xFF;
        buf[4] = (preamble_syms >> 0) & 0xFF;
    }

    if(flags & RADIO_PARAM_FLAGS_FREQ)
    {
        buf[5] = (freq >> 24) & 0xFF;
        buf[6] = (freq >> 16) & 0xFF;
        buf[7] = (freq >>  8) & 0xFF;
        buf[8] = (freq      ) & 0xFF;
    }

    return hal_read_write(OP_SET_RADIO_PARAMS, buf, 9, NULL, 0);
}

int32_t ll_bandwidth_set(uint8_t bandwidth)
{
    return ll_radio_params_set(RADIO_PARAM_FLAGS_BW, 0, 0, bandwidth, 0, 0, 0, 0, 0);
}

int32_t ll_spreading_factor_set(uint8_t sf)
{
    return ll_radio_params_set(RADIO_PARAM_FLAGS_SF, sf, 0, 0, 0, 0, 0, 0, 0);
}

int32_t ll_coding_rate_set(uint8_t coding_rate)
{
    return ll_radio_params_set(RADIO_PARAM_FLAGS_CR, 0, coding_rate, 0, 0, 0, 0, 0, 0);
}

int32_t ll_tx_power_set(int8_t pwr)
{
    if (pwr < -4 || pwr > 26)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    return hal_read_write(OP_TX_POWER_SET, (uint8_t *)&pwr, 1, NULL, 0);
}

int32_t ll_tx_power_get(int8_t *pwr)
{
    if (NULL == pwr)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    return hal_read_write(OP_TX_POWER_GET, NULL, 0, (uint8_t *)pwr, 1);
}

int32_t ll_frequency_set(uint32_t freq)
{
    return ll_radio_params_set(RADIO_PARAM_FLAGS_FREQ, 0, 0, 0, freq, 0, 0, 0, 0);
}

int32_t ll_preamble_syms_set(uint16_t num_syms)
{
    return ll_radio_params_set(RADIO_PARAM_FLAGS_PREAMBLE, 0, 0, 0, 0, num_syms, 0, 0, 0);
}

int32_t ll_header_enabled_set(uint8_t enabled)
{
    return ll_radio_params_set(RADIO_PARAM_FLAGS_HEADER, 0, 0, 0, 0, 0, enabled, 0, 0);
}

int32_t ll_crc_enabled_set(uint8_t enabled)
{
    return ll_radio_params_set(RADIO_PARAM_FLAGS_CRC, 0, 0, 0, 0, 0, 0, enabled, 0);
}

int32_t ll_iq_inversion_set(uint8_t inverted)
{
    return ll_radio_params_set(RADIO_PARAM_FLAGS_IQ, 0, 0, 0, 0, 0, 0, 0, inverted);
}

int32_t ll_sync_word_set(uint8_t sync_word)
{
    return hal_read_write(OP_SYNC_WORD_SET, (uint8_t *)&sync_word, 1, NULL, 0);
}

int32_t ll_sync_word_get(uint8_t *sync_word)
{
    if (NULL == sync_word)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    return hal_read_write(OP_SYNC_WORD_GET, NULL, 0, sync_word, 1);
}

int32_t ll_echo_mode(void)
{
    return hal_read_write(OP_PKT_ECHO, NULL, 0, NULL, 0);
}

int32_t ll_packet_send(uint8_t buf[], uint16_t len)
{
    return ll_packet_send_queue(buf, len);
}

int32_t ll_packet_send_queue(uint8_t buf[], uint16_t len)
{
    if (buf == NULL || len <= 0)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    uint8_t cmd_response;

    int32_t rw_response = hal_read_write(OP_PKT_SEND_QUEUE, buf, len, &cmd_response, 1);

    if (rw_response < 0)
    {
        return((int8_t)rw_response);
    }
    else
    {
        return(cmd_response);
    }
}


int32_t ll_transmit_cw(void)
{
    return hal_read_write(OP_TX_CW, NULL, 0, NULL, 0);
}

int32_t ll_packet_recv_cont(uint8_t buf[], uint16_t len, uint8_t *bytes_received)
{
    int32_t rw_response;

    if (buf == NULL || len == 0 || bytes_received == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    rw_response = hal_read_write(OP_PKT_RECV_CONT, NULL, 0, buf, len);

    if (rw_response < 0)
    {
        *bytes_received = 0;
        return(-1);
    }
    else
    {
        *bytes_received = (uint8_t) (rw_response & 0xFF);
        return(0);
    }
}

int32_t ll_packet_recv(uint16_t num_timeout_symbols, uint8_t buf[], uint16_t len, uint8_t *bytes_received)
{
    uint8_t buff[2];
    int32_t rw_response;

    if (buf == NULL || len <= 0 || bytes_received == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    // Make the uint16_t value big-endian
    buff[0] = (num_timeout_symbols >> 8) & 0xFF;
    buff[1] = (num_timeout_symbols >> 0) & 0xFF;

    rw_response = hal_read_write(OP_PKT_RECV, buff, sizeof(num_timeout_symbols), buf, len);

    if (rw_response < 0)
    {
        *bytes_received = 0;
        return(-1);
    }
    else
    {
        *bytes_received = (uint8_t) (rw_response & 0xFF);
        return(0);
    }
}

int32_t ll_packet_recv_with_rssi(uint16_t num_timeout_symbols, uint8_t buf[], uint16_t len, uint8_t *bytes_received)
{
    uint8_t buff[2];
    int32_t rw_response;

    if (buf == NULL || len <= 0 || bytes_received == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    // Make the uint16_t value big-endian
    buff[0] = (num_timeout_symbols >> 8) & 0xFF;
    buff[1] = (num_timeout_symbols >> 0) & 0xFF;

    rw_response = hal_read_write(OP_MSG_RECV_RSSI, buff, sizeof(num_timeout_symbols), buf, len);

    if (rw_response < 0)
    {
        *bytes_received = 0;
        return(-1);
    }
    else
    {
        *bytes_received = (uint8_t) (rw_response & 0xFF);
        return(0);
    }
}
