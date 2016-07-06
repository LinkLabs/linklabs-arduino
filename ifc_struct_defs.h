#ifndef __IFC_STRUCT_DEFS_H_
#define __IFC_STRUCT_DEFS_H_

#include <stdint.h>

#define NET_INFO_BUFF_SIZE (30)
#define DL_BAND_CFG_SIZE (3 * 4 + 2)
#define STATS_SIZE (10 * 4)
#define TIME_INFO_SIZE (6 * 2 + 1)

#ifndef PACKED
#ifdef __GNU_C__
    #define PACKED __attribute ((__packed__))
#else
    #ifndef PACKED
        #define PACKED
    #endif
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup Link_Labs_Interface_Library
 * @{
 */

/**
 * @addtogroup Module_Interface
 * @brief
 * @{
 */


typedef enum
{
    LLABS_CONNECT_INITIAL = 0,
    LLABS_CONNECT_DISCONNECTED,
    LLABS_CONNECT_CONNECTED,
    LLABS_NUM_CONNECT_STATUSES
} llabs_connect_status_t;

typedef struct PACKED llabs_network_info_t
{
    uint32_t network_id_node;
    uint32_t network_id_gw;
    int8_t gateway_channel;
    uint32_t gateway_frequency;
    uint32_t last_rx_tick;
    int16_t rssi;
    int8_t snr;
    llabs_connect_status_t connection_status;
    uint8_t is_scanning_gateways;
    uint64_t gateway_id;
} llabs_network_info_t;

// Defines the band-specific frequency parameters (FCC 902-928, etc...)
typedef struct PACKED llabs_dl_band_cfg
{
    uint32_t band_edge_lower;
    uint32_t band_edge_upper;
    uint32_t band_edge_guard;
    uint8_t chan_step_size;
    uint8_t chan_step_offset;
} llabs_dl_band_cfg_t;

typedef struct llabs_stats
{
    uint32_t num_send_calls;            // Number of times SendMessage has been called successfully
    uint32_t num_pkts_transmitted;      // Number of packet transmissions (includes retries)
    uint32_t num_gateway_scans;         // Number of gateway scans
    uint32_t num_collisions;            // Number of CSMA collisions detected
    uint32_t num_ack_successes;         // Number of successful acknowledgments
    uint32_t num_ack_failures;          // Number of failed acknowledgments
    uint32_t num_sync_failures;         // Number of Sync failures
    uint32_t num_canceled_pkts_ack;     // Number of times packet was canceled due to LLABS_ACK_FAIL_RETRIES
    uint32_t num_canceled_pkts_csma;    // Number of times packet was canceled due to LLABS_MAX_CSMA_COLLISIONS
    uint32_t num_rx_errors;             // Number of times we received a Rx error from the back end
} llabs_stats_t;

typedef struct PACKED llabs_time
{
    uint32_t seconds;                   // Seconds since UNIX epoch 00:00:00 UTC on 1 January 1970
    uint16_t millis;                    // number of milliseconds since time seconds since the epoch
} llabs_time_t;

typedef struct PACKED llabs_time_info
{
    uint8_t sync_mode;                  // 0: Time sync only when requested, 1: Time sync opportunistically
    llabs_time_t curr;
    llabs_time_t last_sync;
} llabs_time_info_t;

void ll_net_info_deserialize(const uint8_t buff[NET_INFO_BUFF_SIZE], llabs_network_info_t * net_info);
uint16_t ll_net_info_serialize(const llabs_network_info_t * net_info, uint8_t buff[NET_INFO_BUFF_SIZE]);
void ll_dl_band_cfg_deserialize(const uint8_t buff[DL_BAND_CFG_SIZE], llabs_dl_band_cfg_t * dl_cfg);
uint16_t ll_dl_band_cfg_serialize(const llabs_dl_band_cfg_t * dl_cfg, uint8_t buff[DL_BAND_CFG_SIZE]);
void ll_stats_deserialize(const uint8_t buff[STATS_SIZE], llabs_stats_t * stats);
uint16_t ll_stats_serialize(const llabs_stats_t * stats, uint8_t buff[STATS_SIZE]);
void ll_time_deserialize(const uint8_t buff[TIME_INFO_SIZE], llabs_time_info_t *time_info);
uint16_t ll_time_serialize(const llabs_time_info_t *time_info, uint8_t buff[TIME_INFO_SIZE]);


/** @} (end addtogroup Module_Interface) */

/*
 * @defgroup ifc_serialize Serialization utilities
 * @private
 * @brief Format common data types for the host IFC message
 *      protocol.
 *
 * The utility functions contained in this group serialize (marshal) and
 * deserialize (unmarshal) the common integer datatypes exchanged within
 * messages over the host interface.  The write_* functions serialize an
 * integer into a byte stream , and the read_* functions deserialize an
 * integer from a byte stream.
 * 
 * See <a href="https://en.wikipedia.org/wiki/Serialization">
 * serialization</a> on Wikipedia for more details about why these functions
 * exist.
 */
 /**
 * @{
 */

/**
 * @brief
 *   Read a u8 from the buffer [not memory safe].
 *
 * @private
 * @param[inout] buffer
 *   The pointer to the big-endian buffer.  The pointer is
 *   incremented to the next location past the value read.
 *
 * @return
 *   The u8 read from the buffer.
 */
uint8_t read_uint8(const uint8_t ** buffer);

/**
 * @brief
 *   Read a u16 from the buffer [not memory safe].
 *
 * @private
 * @param[inout] buffer
 *   The pointer to the big-endian buffer.  The pointer is
 *   incremented to the next location past the value read.
 *
 * @return
 *   The u16 read from the buffer.
 */
uint16_t read_uint16(const uint8_t ** buffer);

/**
 * @brief
 *   Read a u32 from the buffer [not memory safe].
 *
 * @private
 * @param[inout] buffer
 *   The pointer to the big-endian buffer.  The pointer is
 *   incremented to the next location past the value read.
 *
 * @return
 *   The u32 read from the buffer.
 */
uint32_t read_uint32(const uint8_t ** buffer);

/**
 * @brief
 *   Read a u64 from the buffer [not memory safe].
 *
 * @private
 * @param[inout] buffer
 *   The pointer to the big-endian buffer.  The pointer is
 *   incremented to the next location past the value read.
 *
 * @return
 *   The u64 read from the buffer.
 */
uint64_t read_uint64(const uint8_t ** buffer);

/**
 * @brief
 *   Write a u8 to the buffer [not memory safe].
 *
 * @private
 * @param[in] x
 *   The u8 to write.
 *
 * @param[inout] buffer
 *   The pointer to the big-endian buffer.  The pointer is
 *   incremented to the next location past the value written.
 */
void write_uint8(uint8_t x, uint8_t ** buffer);

/**
 * @brief
 *   Write a u16 to the buffer [not memory safe].
 *
 * @private
 * @param[in] x
 *   The u16 to write.
 *
 * @param[inout] buffer
 *   The pointer to the big-endian buffer.  The pointer is
 *   incremented to the next location past the value written.
 */
void write_uint16(uint16_t x, uint8_t ** buffer);

/**
 * @brief
 *   Write a u32 to the buffer [not memory safe].
 *
 * @private
 * @param[in] x
 *   The u32 to write.
 *
 * @param[inout] buffer
 *   The pointer to the big-endian buffer.  The pointer is
 *   incremented to the next location past the value written.
 */
void write_uint32(uint32_t x, uint8_t ** buffer);

/**
 * @brief
 *   Write a u64 to the buffer [not memory safe].
 *
 * @private
 * @param[in] x
 *   The u64 to write.
 *
 * @param[inout] buffer
 *   The pointer to the big-endian buffer.  The pointer is
 *   incremented to the next location past the value written.
 */
void write_uint64(uint64_t x, uint8_t ** buffer);

/** @} (end defgroup ifc_serialize) */

/** @} (end addtogroup Link_Labs_Interface_Library) */

#ifdef __cplusplus
}
#endif

#endif
