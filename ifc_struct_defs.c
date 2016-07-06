#include <stddef.h>
#include "ifc_struct_defs.h"

uint8_t read_uint8(const uint8_t ** buffer) {
    return *((*buffer)++);
}

uint16_t read_uint16(const uint8_t ** buffer) {
    return (((uint16_t) read_uint8(buffer)) << 8) |
            ((uint16_t) read_uint8(buffer));
}

uint32_t read_uint32(const uint8_t ** buffer) {
    return (((uint32_t) read_uint16(buffer)) << 16) |
            ((uint32_t) read_uint16(buffer));
}

uint64_t read_uint64(const uint8_t ** buffer) {
    return (((uint64_t) read_uint32(buffer)) << 32) |
            ((uint64_t) read_uint32(buffer));
}

void write_uint8(uint8_t x, uint8_t ** buffer) {
    *((*buffer)++) = x;
}

void write_uint16(uint16_t x, uint8_t ** buffer) {
    write_uint8(x >> 8, buffer);
    write_uint8(x, buffer);
}

void write_uint32(uint32_t x, uint8_t ** buffer) {
    write_uint16(x >> 16, buffer);
    write_uint16(x, buffer);
}

void write_uint64(uint64_t x, uint8_t ** buffer) {
    write_uint32(x >> 32, buffer);
    write_uint32(x, buffer);
}

// Parse a serialized llabs_network_info_t struct.
void ll_net_info_deserialize(const uint8_t buff[NET_INFO_BUFF_SIZE], llabs_network_info_t *net_info)
{
    uint8_t const * b = buff;
    net_info->network_id_node = read_uint32(&b);
    net_info->network_id_gw = read_uint32(&b);
    net_info->gateway_channel = read_uint8(&b);
    net_info->gateway_frequency = read_uint32(&b);
    net_info->last_rx_tick = read_uint32(&b);
    net_info->rssi = read_uint16(&b);
    net_info->snr = read_uint8(&b);
    net_info->connection_status = read_uint8(&b);
    net_info->is_scanning_gateways = read_uint8(&b);
    net_info->gateway_id = read_uint64(&b);
}

// Serializes an llabs_network_info_t struct into a buffer to be sent over the host interface.
// Returns the size of the serialized struct in the buffer.
uint16_t ll_net_info_serialize(const llabs_network_info_t *net_info, uint8_t buff[NET_INFO_BUFF_SIZE])
{
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

void ll_dl_band_cfg_deserialize(const uint8_t buff[DL_BAND_CFG_SIZE], llabs_dl_band_cfg_t *dl_cfg)
{
    uint8_t const * b = buff;
    dl_cfg->band_edge_lower = read_uint32(&b);
    dl_cfg->band_edge_upper = read_uint32(&b);
    dl_cfg->band_edge_guard = read_uint32(&b);
    dl_cfg->chan_step_size = read_uint8(&b);
    dl_cfg->chan_step_offset = read_uint8(&b);
}

uint16_t ll_dl_band_cfg_serialize(const llabs_dl_band_cfg_t *dl_cfg, uint8_t buff[DL_BAND_CFG_SIZE])
{
    uint8_t * b = buff;
    write_uint32(dl_cfg->band_edge_lower, &b);
    write_uint32(dl_cfg->band_edge_upper, &b);
    write_uint32(dl_cfg->band_edge_guard, &b);
    write_uint8(dl_cfg->chan_step_size, &b);
    write_uint8(dl_cfg->chan_step_offset, &b);
    return b - buff;
}

void ll_stats_deserialize(const uint8_t buff[STATS_SIZE], llabs_stats_t *stats)
{
    uint8_t const * b = buff;
    stats->num_send_calls = read_uint32(&b);
    stats->num_pkts_transmitted = read_uint32(&b);
    stats->num_gateway_scans = read_uint32(&b);
    stats->num_collisions = read_uint32(&b);
    stats->num_ack_successes = read_uint32(&b);
    stats->num_ack_failures = read_uint32(&b);
    stats->num_sync_failures = read_uint32(&b);
    stats->num_canceled_pkts_ack = read_uint32(&b);
    stats->num_canceled_pkts_csma = read_uint32(&b);
    stats->num_rx_errors = read_uint32(&b);
}

uint16_t ll_stats_serialize(const llabs_stats_t *stats, uint8_t buff[STATS_SIZE])
{
    uint8_t * b = buff;
    write_uint32(stats->num_send_calls, &b);
    write_uint32(stats->num_pkts_transmitted, &b);
    write_uint32(stats->num_gateway_scans, &b);
    write_uint32(stats->num_collisions, &b);
    write_uint32(stats->num_ack_successes, &b);
    write_uint32(stats->num_ack_failures, &b);
    write_uint32(stats->num_sync_failures, &b);
    write_uint32(stats->num_canceled_pkts_ack, &b);
    write_uint32(stats->num_canceled_pkts_csma, &b);
    write_uint32(stats->num_rx_errors, &b);
    return b - buff;
}

void ll_time_deserialize(const uint8_t buff[TIME_INFO_SIZE], llabs_time_info_t *time_info)
{
    uint8_t const * b = buff;
    time_info->sync_mode = read_uint8(&b);
    time_info->curr.seconds = read_uint32(&b);
    time_info->curr.millis = read_uint16(&b);
    time_info->last_sync.seconds = read_uint32(&b);
    time_info->last_sync.millis = read_uint16(&b);
}

uint16_t ll_time_serialize(const llabs_time_info_t *time_info, uint8_t buff[TIME_INFO_SIZE])
{
    uint8_t * b = buff;
    write_uint8(time_info->sync_mode, &b);
    write_uint32(time_info->curr.seconds, &b);
    write_uint16(time_info->curr.millis, &b);
    write_uint32(time_info->last_sync.seconds, &b);
    write_uint16(time_info->last_sync.millis, &b);
    return b - buff;
}
