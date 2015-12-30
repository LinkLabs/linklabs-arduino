#ifndef SYMPHONY_H
#define SYMPHONY_H

#define IFC_VERSION_MAJOR (0)
#define IFC_VERSION_MINOR (3)
#define IFC_VERSION_TAG   (2)

#define APP_TOKEN_LEN     (10)
#define MAX_RX_MSG_LEN    (128)

#define VERSION_LEN                 (4)
#define FIRMWARE_TYPE_LEN           (4)

#define LL_IFC_ACK                          (0)   // All good.
#define LL_IFC_NACK_CMD_NOT_SUPPORTED       (1)   // Command not supported.
#define LL_IFC_NACK_INCORRECT_CHKSUM        (2)   // Incorrect Checksum.
#define LL_IFC_NACK_PAYLOAD_LEN_OOR         (3)   // Length of payload sent in command was out of range.
#define LL_IFC_NACK_PAYLOAD_OOR             (4)   // Payload sent in command was out of range.
#define LL_IFC_NACK_BOOTUP_IN_PROGRESS      (5)   // Not allowed since firmware bootup still in progress. Wait.
#define LL_IFC_NACK_BUSY_TRY_AGAIN          (6)   // Operation prevented by temporary event. Re-try should work.
#define LL_IFC_NACK_APP_TOKEN_REG           (7)   // Application token is not registered for this node.
#define LL_IFC_NACK_PAYLOAD_LEN_EXCEEDED    (8)   // Payload length is greater than the max supported length
#define LL_IFC_NACK_NOT_IN_MAILBOX_MODE     (9)   // Module must be in DOWNLINK_MAILBOX mode
#define LL_IFC_NACK_OTHER                   (99)

/** Error Codes */
/* Note: Error codes -1 to -99 map to NACK codes received from the radio */
#define LL_IFC_ERROR_INCORRECT_PARAMETER    (-101)
// TODO: Define more error codes

/** Bit Definitions for OP_SET_RADIO_PARAMS */
#define RADIO_PARAM_FLAGS_SF        (1u<<0u)
#define RADIO_PARAM_FLAGS_CR        (1u<<1u)
#define RADIO_PARAM_FLAGS_BW        (1u<<2u)
#define RADIO_PARAM_FLAGS_FREQ      (1u<<3u)
#define RADIO_PARAM_FLAGS_PREAMBLE  (1u<<4u)
#define RADIO_PARAM_FLAGS_HEADER    (1u<<5u)
#define RADIO_PARAM_FLAGS_CRC       (1u<<6u)
#define RADIO_PARAM_FLAGS_IQ        (1u<<7u)

/** Bit Definitions for OP_IRQ_FLAGS */
#define IRQ_FLAGS_WDOG_RESET                  (0x00000001UL)  // Set every time the module reboots after a Watchdog reboot
#define IRQ_FLAGS_RESET                       (0x00000002UL)  // Set every time the module reboots for any reason
#define IRQ_FLAGS_TX_DONE                     (0x00000010UL)  // Set every time a Tx Queue goes from non-empty to empty
#define IRQ_FLAGS_TX_ERROR                    (0x00000020UL)  // Set every time there is a Tx Error
#define IRQ_FLAGS_RX_DONE                     (0x00000100UL)  // Set every time a new packet is received
#define IRQ_FLAGS_CONNECTED                   (0x00001000UL)  // Set every time we transition from the disconnected -> connected state
#define IRQ_FLAGS_DISCONNECTED                (0x00002000UL)  // Set every time we transition from the connected -> disconnected state
#define IRQ_FLAGS_CRYPTO_ESTABLISHED          (0x00010000UL)  // Set every time we transition from the crypto not established -> crytpto established state
#define IRQ_FLAGS_APP_TOKEN_CONFIRMED         (0x00020000UL)  // Set every time an application token is confirmed by Conductor
#define IRQ_FLAGS_DOWNLINK_REQUEST_ACK        (0x00040000UL)  // (NOT IMPLEMENTED) Set every time a downlink registration request is acknowledged
#define IRQ_FLAGS_CRYPTO_ERROR                (0x00100000UL)  // Set when a crypto exchange attempt fails
#define IRQ_FLAGS_APP_TOKEN_ERROR             (0x00200000UL)  // Set when an application token registration fails
#define IRQ_FLAGS_ASSERT                      (0x80000000UL)  // Set every time we transition from the connected->disconnected state

#define NET_INFO_BUFF_SIZE      (30)
#define DL_BAND_CFG_SIZE        (3 * 4 + 2)
#define STATS_SIZE              (10 * 4)

#ifndef PACKED
        #define PACKED
 #endif

#ifndef NULL    // <time.h> defines NULL on *some* platforms
#define NULL            (0)
#endif
#define CMD_HEADER_LEN      (5)
#define RESP_HEADER_LEN     (6)

typedef enum {
    LLABS_CONNECT_INITIAL = 0,
    LLABS_CONNECT_DISCONNECTED,
    LLABS_CONNECT_CONNECTED,
    LLABS_NUM_CONNECT_STATUSES
} llabs_connect_status_t;

typedef struct PACKED llabs_network_info_t {
    uint32_t network_id_node;
    uint32_t network_id_gw;
    int8_t gateway_channel;
    uint32_t gateway_frequency;
    uint32_t last_rx_tick;
    int16_t rssi;
    uint8_t snr;
    llabs_connect_status_t connection_status;
    uint8_t is_scanning_gateways;
    uint64_t gateway_id;
} llabs_network_info_t;

// Defines the band-specific frequency parameters (FCC 902-928, etc...)
typedef struct PACKED llabs_dl_band_cfg {
    uint32_t band_edge_lower;
    uint32_t band_edge_upper;
    uint32_t band_edge_guard;
    uint8_t chan_step_size;
    uint8_t chan_step_offset;
} llabs_dl_band_cfg_t;

typedef struct llabs_stats {
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

typedef enum {
    OP_VERSION = 0,                 // 0x00
    OP_IFC_VERSION = 1,             // 0x01
    OP_STATE = 2,                   // 0x02
    OP_TX_STATE = 3,                // 0x03
    OP_RX_STATE = 4,                // 0x04
    OP_FREQUENCY = 6,               // 0x06
    OP_TX_POWER_SET = 7,            // 0x07
    OP_RESET_SETTINGS = 8,          // 0x08
    OP_GET_RADIO_PARAMS = 9,        // 0x09
    OP_SET_RADIO_PARAMS = 10,       // 0x0A
    OP_PKT_SEND_QUEUE = 11,         // 0x0B
    OP_TX_POWER_GET = 12,           // 0x0C
    OP_SYNC_WORD_SET = 13,          // 0x0D
    OP_SYNC_WORD_GET = 14,          // 0x0E
    OP_IRQ_FLAGS = 15,              // 0x0F
    OP_IRQ_FLAGS_MASK = 16,         // 0x10
    OP_SLEEP = 20,                  // 0x14
    OP_SLEEP_BLOCK = 21,            // 0x15
    OP_PKT_ECHO = 31,               // 0x1F
    OP_PKT_RECV = 40,               // 0x28
    OP_PKT_RECV_RSSI = 41,          // 0x29
    OP_PKT_RECV_CONT = 42,          // 0x2A
    OP_MODULE_ID = 50,              // 0x32
    OP_STORE_SETTINGS = 51,         // 0x33
    OP_DELETE_SETTINGS = 52,        // 0x34
    OP_RESET_MCU = 60,              // 0x3C
    OP_TRIGGER_BOOTLOADER = 61,     // 0x3D
    OP_MAC_MODE_SET = 70,           // 0x46
    OP_MAC_MODE_GET = 71,           // 0x47
    OP_PKT_SEND_ACK = 90,           // 0x5A
    OP_PKT_SEND_UNACK = 91,         // 0x5B
    OP_TX_CW = 98,                  // 0x61
    OP_RX_MODE_SET = 110,           // 0x6E
    OP_RX_MODE_GET = 111,           // 0x6F
    OP_QOS_REQUEST = 112,           // 0x70
    OP_QOS_GET     = 113,           // 0x71
    OP_ANTENNA_SET = 114,           // 0x72
    OP_ANTENNA_GET = 115,           // 0x73
    OP_NET_TOKEN_SET  = 116,        // 0x74
    OP_NET_TOKEN_GET  = 117,        // 0x75
    OP_NET_INFO_GET= 118,           // 0x76
    OP_STATS_GET   = 119,           // 0x77
    OP_RSSI_SET    = 120,           // 0x78
    OP_RSSI_GET    = 121,           // 0x79
    OP_DL_BAND_CFG_GET       = 122, // 0x7A
    OP_DL_BAND_CFG_SET       = 123, // 0x7B
    OP_APP_TOKEN_SET         = 124, // 0x7C
    OP_APP_TOKEN_GET         = 125, // 0x7D
    OP_APP_TOKEN_REG_GET     = 126, // 0x7E
    OP_CRYPTO_KEY_XCHG_REQ   = 128, // 0x80
    OP_MAILBOX_REQUEST       = 129, // 0x81
    OP_HARDWARE_TYPE = 254,         // 0xFE
    OP_FIRMWARE_TYPE = 255          // 0xFF
} opcode_t;

typedef enum {
    FRAME_START = 0xC4
} frame_t;

typedef enum {
    LORA_NO_MAC = 0,
    LORAWAN_EU,
    LORAWAN_FCC,
    SYMPHONY_LINK,
    NUM_MACS,
    MAC_INVALID = 255,
} ll_mac_type_t;

// Hardware Types
typedef enum {
    UNAVAILABLE    = 0,             // 0x00
    LLRLP20_V2     = 1,             // 0x01
    LLRXR26_V2     = 2,             // 0x02
    LLRLP20_V3     = 3,             // 0x03
    LLRXR26_V3     = 4,             // 0x04
} ll_hardware_type_t;

/**
 * Firmware identifiers
 */
typedef enum {
    CPU_EFM32TG210F32 = 0,          // 0x00
    CPU_EFM32G210F128 = 1,          // 0x01
    CPU_R5F51115ADNE  = 2,          // 0x02
    CPU_R5F51116ADNE  = 3,          // 0x03
} cpu_code_t;

typedef enum {
    GATEWAY_TX_ONLY = 0,            // 0x00
    MODULE_END_NODE = 1             // 0x01
    // TBD - How to define others ?
} functionality_code_t;

typedef struct {
    uint16_t cpu_code;
    uint16_t functionality_code;
} ll_firmware_type_t;

typedef struct {
    uint8_t major;
    uint8_t minor;
    uint16_t tag;
} ll_version_t;

typedef enum {
    DOWNLINK_MODE_OFF = 0,          // 0x00
    DOWNLINK_MODE_ALWAYS_ON = 1,    // 0x01
    DOWNLINK_MODE_MAILBOX = 2,      // 0x02
    DOWNLINK_MODE_PERIODIC = 3,     // 0x03
    NUM_DOWNLINK_MODES,
    DOWNLINK_MODE_FAILURE = 255,    // 0xFF
} downlink_mode_t;

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

uint8_t read_uint8(const uint8_t buff[], size_t * index);
uint16_t read_uint16(const uint8_t buff[], size_t * index);
uint32_t read_uint32(const uint8_t buff[], size_t * index);
uint64_t read_uint64(const uint8_t buff[], size_t *index);
void write_uint8(uint8_t x, uint8_t **buff);
void write_uint16(uint16_t x, uint8_t **buff);
void write_uint32(uint32_t x, uint8_t **buff);
void write_uint64(uint64_t x, uint8_t **buff);
void ll_net_info_deserialize(const uint8_t buff[NET_INFO_BUFF_SIZE], llabs_network_info_t *net_info);
uint16_t ll_net_info_serialize(const llabs_network_info_t *net_info, uint8_t buff[NET_INFO_BUFF_SIZE]);
void ll_dl_band_cfg_deserialize(const uint8_t buff[DL_BAND_CFG_SIZE], llabs_dl_band_cfg_t *dl_cfg);
uint16_t ll_dl_band_cfg_serialize(const llabs_dl_band_cfg_t *dl_cfg, uint8_t buff[DL_BAND_CFG_SIZE]);
void ll_stats_deserialize(const uint8_t buff[STATS_SIZE], llabs_stats_t *stats);
uint16_t ll_stats_serialize(const llabs_stats_t *stats, uint8_t buff[STATS_SIZE]);
int32_t transport_write(uint8_t *buff, uint16_t len);
int32_t transport_read(uint8_t *buff, uint16_t len);
uint16_t compute_checksum(uint8_t *hdr, uint16_t hdr_len, uint8_t *payload, uint16_t payload_len);
int32_t recv_packet(opcode_t op, uint8_t message_num, uint8_t *buf, uint16_t len);
void send_packet(opcode_t op, uint8_t message_num, uint8_t *buf, uint16_t len);
int32_t hal_read_write(opcode_t op, uint8_t buf_in[], uint16_t in_len, uint8_t buf_out[], uint16_t out_len);
int32_t ll_packet_recv(uint16_t num_timeout_symbols, uint8_t buf[], uint16_t len, uint8_t *bytes_received);
int32_t ll_packet_recv_with_rssi(uint16_t num_timeout_symbols, uint8_t buf[], uint16_t len, uint8_t *bytes_received);
int32_t ll_retrieve_message(uint8_t *buf, uint8_t *size, int16_t *rssi, uint8_t *snr);
int32_t ll_packet_send_queue(uint8_t buf[], uint16_t len);
int32_t ll_packet_send(uint8_t buf[], uint16_t len);
int32_t ll_firmware_type_get(ll_firmware_type_t *t);
int32_t ll_hardware_type_get(ll_hardware_type_t *t);
int32_t ll_interface_version_get(ll_version_t *version);
int32_t ll_version_get(ll_version_t *version);
int32_t ll_sleep_block(void);
int32_t ll_sleep_unblock(void);
int32_t ll_receive_mode_set(uint8_t rx_mode);
int32_t ll_receive_mode_get(uint8_t *rx_mode);
int32_t ll_mailbox_request(void);
int32_t ll_qos_request(uint8_t qos);
int32_t ll_qos_get(uint8_t *qos);
int32_t ll_app_token_set(const uint8_t *app_token, uint8_t len);
int32_t ll_app_token_get(uint8_t *app_token);
int32_t ll_app_reg_get(uint8_t *is_registered);
int32_t ll_encryption_key_exchange_request(void);
int32_t ll_get_state(enum ll_state *state, enum ll_tx_state *tx_state, enum ll_rx_state *rx_state);
int32_t ll_mac_mode_set(ll_mac_type_t mac_mode);
int32_t ll_mac_mode_get(ll_mac_type_t *mac_mode);
int32_t ll_antenna_set(uint8_t ant);
int32_t ll_antenna_get(uint8_t *ant);
int32_t ll_net_token_get(uint32_t *p_net_token);
int32_t ll_net_token_set(uint32_t net_token);
int32_t ll_config_get(uint32_t *net_token, uint8_t app_token[APP_TOKEN_LEN], enum ll_downlink_mode *dl_mode, uint8_t *qos);
int32_t ll_net_info_get(llabs_network_info_t *p_net_info);
int32_t ll_stats_get(llabs_stats_t *s);
int32_t ll_dl_band_cfg_get(llabs_dl_band_cfg_t *p);
int32_t ll_dl_band_cfg_set(const llabs_dl_band_cfg_t *p);
int32_t ll_config_set(uint32_t net_token, const uint8_t app_token[APP_TOKEN_LEN], enum ll_downlink_mode dl_mode, uint8_t qos);
int32_t ll_rssi_scan_set(uint32_t u1, uint32_t u2, uint32_t u3, uint32_t u4);
int32_t ll_rssi_scan_get(uint8_t buf[], uint16_t len, uint8_t *bytes_received);
int32_t ll_unique_id_get(uint64_t *unique_id);

int32_t ll_radio_params_get(uint8_t *sf, uint8_t *cr, uint8_t *bw, uint32_t *freq, uint16_t *preamble_syms, uint8_t *header_enabled, uint8_t *crc_enabled, uint8_t *iq_inverted);
int32_t ll_radio_params_set(uint8_t flags, uint8_t sf, uint8_t cr, uint8_t bw, uint32_t freq,
                            uint16_t preamble_syms, uint8_t enable_header, uint8_t enable_crc,
                            uint8_t enable_iq_inversion);
int32_t ll_bandwidth_set(uint8_t bandwidth);
int32_t ll_spreading_factor_set(uint8_t sf);
int32_t ll_coding_rate_set(uint8_t coding_rate);
int32_t ll_tx_power_set(int8_t pwr);
int32_t ll_tx_power_get(int8_t *pwr);
int32_t ll_frequency_set(uint32_t freq);
int32_t ll_preamble_syms_set(uint16_t num_syms);
int32_t ll_header_enabled_set(uint8_t enabled);

int32_t ll_crc_enabled_set(uint8_t enabled);
int32_t ll_iq_inversion_set(uint8_t inverted);
int32_t ll_sync_word_set(uint8_t sync_word);
int32_t ll_sync_word_get(uint8_t *sync_word);
int32_t ll_settings_store(void);
int32_t ll_settings_delete(void);
int32_t ll_restore_defaults(void);
int32_t ll_sleep(void);
int32_t ll_reset_mcu(void);
int32_t ll_bootloader_mode(void);
int32_t ll_echo_mode();
int32_t ll_packet_recv_cont(uint8_t buf[], uint16_t len, uint8_t *bytes_received);
int32_t ll_irq_flags(uint32_t flags_to_clear, uint32_t *flags);
int32_t ll_packet_send_ack(uint8_t buf[], uint16_t len);
int32_t ll_packet_send_unack(uint8_t buf[], uint16_t len);
int32_t ll_transmit_cw(void);

#endif