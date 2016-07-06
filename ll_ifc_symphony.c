#include "ll_ifc_symphony.h"
#include "ll_ifc_private.h"
#include <string.h>  // memmove



static int32_t ll_net_token_get(uint32_t *p_net_token)
{
    uint8_t buff[4];
    if (p_net_token == NULL)
    {
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

static int32_t ll_net_token_set(uint32_t net_token)
{
    if(net_token != 0xFFFFFFFF)
    {
        uint8_t buff[4];
        buff[0] = (net_token >> 24) & 0xFF;
        buff[1] = (net_token >> 16) & 0xFF;
        buff[2] = (net_token >> 8) & 0xFF;
        buff[3] = (net_token) & 0xFF;
        return hal_read_write(OP_NET_TOKEN_SET, buff, 4, NULL, 0);
    }
    else
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
}

static int32_t ll_app_token_set(const uint8_t *app_token, uint8_t len)
{
    if (app_token == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    if(10 != len)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    return hal_read_write(OP_APP_TOKEN_SET, (uint8_t*) app_token, 10, NULL, 0);
}

static int32_t ll_app_token_get(uint8_t *app_token)
{
    if (app_token == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    return hal_read_write(OP_APP_TOKEN_GET, NULL, 0, app_token, 10);
}

static int32_t ll_receive_mode_set(uint8_t rx_mode)
{
    // TODO: Should these use the enum?
    if (rx_mode >= NUM_DOWNLINK_MODES)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    return hal_read_write(OP_RX_MODE_SET, &rx_mode, 1, NULL, 0);
}

static int32_t ll_receive_mode_get(uint8_t *rx_mode)
{
    if (rx_mode == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    return hal_read_write(OP_RX_MODE_GET, NULL, 0, rx_mode, sizeof(*rx_mode));
}

static int32_t ll_qos_request(uint8_t qos)
{
    if (qos > 15)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    return hal_read_write(OP_QOS_REQUEST, &qos, 1, NULL, 0);
}

static int32_t ll_qos_get(uint8_t *qos)
{
    if (qos == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    return hal_read_write(OP_QOS_GET, NULL, 0, qos, sizeof(*qos));
}


int32_t ll_config_set(uint32_t net_token, const uint8_t app_token[APP_TOKEN_LEN],
                      enum ll_downlink_mode dl_mode, uint8_t qos)
{
    int32_t ret;

    ret = ll_net_token_set(net_token);
    if (LL_IFC_ACK > ret)
    {
        return ret;
    }

    ret = ll_app_token_set(app_token, APP_TOKEN_LEN);
    if (LL_IFC_ACK > ret)
    {
        return ret;
    }

    ret = ll_receive_mode_set(dl_mode);
    if (LL_IFC_ACK > ret)
    {
        return ret;
    }

    ret = ll_qos_request(qos);
    if (LL_IFC_ACK > ret)
    {
        return ret;
    }

    return ret;
}

int32_t ll_config_get(uint32_t *net_token, uint8_t app_token[APP_TOKEN_LEN],
                      enum ll_downlink_mode *dl_mode, uint8_t *qos)
{
    int32_t ret;

    ret = ll_net_token_get(net_token);
    if (LL_IFC_ACK > ret)
    {
        return ret;
    }

    ret = ll_app_token_get(app_token);
    if (LL_IFC_ACK > ret)
    {
        return ret;
    }

    ret = ll_receive_mode_get((uint8_t *)dl_mode);
    if (LL_IFC_ACK > ret)
    {
        return ret;
    }

    ret = ll_qos_get(qos);
    if (LL_IFC_ACK > ret)
    {
        return ret;
    }

    return ret;
}

int32_t ll_get_state(enum ll_state *state, enum ll_tx_state *tx_state, enum ll_rx_state *rx_state)
{
    int32_t ret = LL_IFC_ACK;

    if (NULL != state)
    {
        uint8_t u8_state;
        ret = hal_read_write(OP_STATE, NULL, 0, &u8_state, 1);
        if (LL_IFC_ACK > ret)
        {
            return ret;
        }
        *state = (enum ll_state)(int8_t)u8_state;
    }

    if (NULL != tx_state)
    {
        uint8_t u8_tx_state;
        ret = hal_read_write(OP_TX_STATE, NULL, 0, &u8_tx_state, 1);
        if (LL_IFC_ACK > ret)
        {
            return ret;
        }
        *tx_state = (enum ll_tx_state)(int8_t)u8_tx_state;
    }

    if (NULL != rx_state)
    {
        uint8_t u8_rx_state;
        ret = hal_read_write(OP_RX_STATE, NULL, 0, &u8_rx_state, 1);
        if (LL_IFC_ACK > ret)
        {
            return ret;
        }
        *rx_state = (enum ll_rx_state)(int8_t)u8_rx_state;
    }

    return ret;
}

int32_t ll_mailbox_request(void)
{
    return hal_read_write(OP_MAILBOX_REQUEST, NULL, 0, NULL, 0);
}

int32_t ll_app_reg_get(uint8_t *is_registered)
{
    if (is_registered == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    return hal_read_write(OP_APP_TOKEN_REG_GET, NULL, 0, is_registered, 1);
}

int32_t ll_encryption_key_exchange_request(void)
{
    return hal_read_write(OP_CRYPTO_KEY_XCHG_REQ, NULL, 0, NULL, 0);
}

int32_t ll_net_info_get(llabs_network_info_t *p_net_info)
{
    uint8_t buff[NET_INFO_BUFF_SIZE];
    if (p_net_info == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    int32_t ret = hal_read_write(OP_NET_INFO_GET, NULL, 0, buff, NET_INFO_BUFF_SIZE);
    ll_net_info_deserialize(buff, p_net_info);
    return ret;
}

int32_t ll_stats_get(llabs_stats_t *s)
{
    uint8_t buff[STATS_SIZE];
    if (s == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    int32_t ret = hal_read_write(OP_STATS_GET, NULL, 0, buff, STATS_SIZE);
    ll_stats_deserialize(buff, s);
    return ret;
}

int32_t ll_retrieve_message(uint8_t *buf, uint8_t *size, int16_t *rssi, uint8_t *snr)
{
    uint8_t buff[2] = {0, 0};  // num_timeout_symbols not used for Symphony
    int32_t rw_response;

    if (buf == NULL || size == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    rw_response = hal_read_write(OP_MSG_RECV_RSSI, buff, sizeof(buff), buf, MAX_RX_MSG_LEN + 3);

    if (rw_response < LL_IFC_ACK)
    {
        *size = 0;
        return rw_response;
    }

    // Size is required
    *size = (uint8_t)(rw_response & 0xFF) - 3;

    // Optional RSSI
    if(NULL != rssi)
    {
        *rssi = 0;
        *rssi = buf[0] + ((uint16_t)buf[1] << 8);
    }

    // Optional RSSI
    if(NULL != snr)
    {
        *snr = buf[2];
    }

    //get rid of snr and rssi in buffer
    memmove(buf, buf + 3, *size);

    return 0;
}

int32_t ll_dl_band_cfg_get(llabs_dl_band_cfg_t *p)
{
    uint8_t buff[DL_BAND_CFG_SIZE];
    if (p == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    int32_t ret = hal_read_write(OP_DL_BAND_CFG_GET, NULL, 0, buff, DL_BAND_CFG_SIZE);
    ll_dl_band_cfg_deserialize(buff, p);
    return ret;
}

int32_t ll_dl_band_cfg_set(const llabs_dl_band_cfg_t *p)
{
    uint8_t buff[DL_BAND_CFG_SIZE];
    if (p == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    ll_dl_band_cfg_serialize(p, buff);
    return hal_read_write(OP_DL_BAND_CFG_SET, buff, DL_BAND_CFG_SIZE, NULL, 0);
}

int32_t ll_system_time_get(llabs_time_info_t *time_info)
{
    uint8_t buff[TIME_INFO_SIZE];
    if (time_info == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    int32_t ret = hal_read_write(OP_SYSTEM_TIME_GET, NULL, 0, buff, STATS_SIZE);
    ll_time_deserialize(buff, time_info);
    return ret;
}

int32_t ll_system_time_sync(uint8_t sync_mode)
{
    if (sync_mode > 1)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }
    return hal_read_write(OP_SYSTEM_TIME_SYNC, &sync_mode, sizeof(sync_mode), NULL, 0);
}

int32_t ll_message_send_ack(uint8_t buf[], uint16_t len)
{
    if (buf == NULL || len <= 0)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    return hal_read_write(OP_MSG_SEND_ACK, buf, len, NULL, 0);
}


int32_t ll_message_send_unack(uint8_t buf[], uint16_t len)
{
    if (buf == NULL || len <= 0)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    return hal_read_write(OP_MSG_SEND_UNACK, buf, len, NULL, 0);
}

