#ifndef __LL_IFC_CONSTS_H
#define __LL_IFC_CONSTS_H

#include <stdint.h>

/**
 * @addtogroup Link_Labs_Interface_Library
 * @{
 */

/**
 * @addtogroup Module_Interface
 * @brief
 * @{
 */

#define IFC_VERSION_MAJOR (0)
#define IFC_VERSION_MINOR (4)
#define IFC_VERSION_TAG (1)

#define APP_TOKEN_LEN (10)
#define MAX_RX_MSG_LEN (128)

extern const uint32_t OPEN_NET_TOKEN;

typedef enum
{
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
    OP_MSG_RECV_RSSI = 41,          // 0x29
    OP_PKT_RECV_CONT = 42,          // 0x2A
    OP_MODULE_ID = 50,              // 0x32
    OP_STORE_SETTINGS = 51,         // 0x33
    OP_DELETE_SETTINGS = 52,        // 0x34
    OP_RESET_MCU = 60,              // 0x3C
    OP_TRIGGER_BOOTLOADER = 61,     // 0x3D
    OP_MAC_MODE_SET = 70,           // 0x46
    OP_MAC_MODE_GET = 71,           // 0x47
    OP_MSG_SEND_ACK = 90,           // 0x5A
    OP_MSG_SEND_UNACK = 91,         // 0x5B
    OP_TX_CW = 98,                  // 0x61
    OP_SYSTEM_TIME_GET = 108,       // 0x6C
    OP_SYSTEM_TIME_SYNC = 109,      // 0x6D
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

    OP_LORAWAN_ACTIVATE         = 133, // 0x85
    OP_LORAWAN_PARAM            = 134, // 0x86
    OP_LORAWAN_MSG_SEND         = 135, // 0x87
    OP_LORAWAN_MSG_RECEIVE      = 136, // 0x88

    OP_UMODE_PROP_GET_REQ    = 140,     // 0x8C		// Micromode property get 
    OP_UMODE_GET_MSG_CNT_REQ = 141,     // 0x8D		// Micromode get msg count
    OP_UMODE_GET_NEXT_MSG_REQ = 142,    // 0x8E		// Micromode get next msg
    OP_UMODE_SET_TIME_REQ    = 143,     // 0x8F		// Micromode set time (UTC)
    OP_UMODE_GET_TIME_REQ    = 144,     // 0x90		// Micromode get time (UTC)
    OP_SEND_MSG_TO_GW        = 145,     // 0x91     // Send/enqueue msg for endpoint to send to GW
    OP_DEBUG_DUMP            = 146,     // 0x92     // send (back) debug dump buffer from moduule to Host - TO BE DEPRECATED
    OP_UMODE_PROP_SET_REQ    = 147,     // 0x93		// Micromode property set
    OP_UMODE_LOST_MSG_REQ    = 148,     // 0x94		// Micromode get/set lost message count
    OP_SEND_MAIL_TO_EP       = 149,     // 0x95     // Send/enqueue msg for GW to send mail to EP
    OP_GET_MAIL_FROM_GW      = 150,     // 0x96     // Get mail back (if rx'ed) from GW

    OP_HARDWARE_TYPE = 254,         // 0xFE
    OP_FIRMWARE_TYPE = 255          // 0xFF
} opcode_t;

typedef enum
{
    FRAME_START = 0xC4
} frame_t;

typedef enum
{
    LORA_NO_MAC = 0,
    LORAWAN,
    LORAWAN_HYBRID,
    SYMPHONY_LINK,
    MICROMODE_MAC,                  // small footprint Star topology non-persistent CSMA MAC
    NUM_MACS,
    MAC_INVALID = 255,
} ll_mac_type_t;

/**
 * version struct
 */
typedef struct {
    uint8_t major;
    uint8_t minor;
    uint16_t tag;
} ll_version_t;

#define VERSION_LEN                 (4)

// Hardware Types
typedef enum
{
    UNAVAILABLE    = 0,             // 0x00
    LLRLP20_V2     = 1,             // 0x01
    LLRXR26_V2     = 2,             // 0x02
    LLRLP20_V3     = 3,             // 0x03
    LLRXR26_V3     = 4,             // 0x04
} ll_hardware_type_t;

/**
 * Firmware identifiers
 */
typedef enum
{
    CPU_EFM32TG210F32 = 0,          // 0x00
    CPU_EFM32G210F128 = 1,          // 0x01
    CPU_R5F51115ADNE  = 2,          // 0x02
    CPU_R5F51116ADNE  = 3,          // 0x03
} cpu_code_t;

typedef enum
{
    GATEWAY_TX_ONLY = 0,            // 0x00
    MODULE_END_NODE = 1             // 0x01
    // TBD - How to define others ?
} functionality_code_t;

/**
 * Link Labs LORA enumeration identifiers for Bandwidth
 */
typedef enum
{
    PROPERTY_LORA_BW_MIN    = 0,    // range limit value (lower)
    PROPERTY_LORA_BW_62_5   = 0,    // 62.5KHz BW
    PROPERTY_LORA_BW_125    = 1,    // 125KHz BW
    PROPERTY_LORA_BW_250    = 2,    // 250KHz BW
    PROPERTY_LORA_BW_500    = 3,    // 500KHz BW
    PROPERTY_LORA_BW_MAX,           // range limit value (upper)
} property_bw_t;

/**
 * Link Labs LORA enumeration identifiers for Spreading Factor
 */
typedef enum
{
    PROPERTY_LORA_SF_MIN    = 6,    // range limit value (lower)
    PROPERTY_LORA_SF_6      = 6,    // SF 6
    PROPERTY_LORA_SF_7      = 7,    // SF 7
    PROPERTY_LORA_SF_8      = 8,    // SF 8
    PROPERTY_LORA_SF_9      = 9,    // SF 9
    PROPERTY_LORA_SF_10     = 10,   // SF 10
    PROPERTY_LORA_SF_11     = 11,   // SF 11
    PROPERTY_LORA_SF_12     = 12,   // SF 12
    PROPERTY_LORA_SF_MAX,           // range limit value (upper)
} property_sf_t;

/**
 * Link Labs LORA enumeration identifiers for Coding rate
 */
typedef enum
{
    PROPERTY_LORA_CR_MIN    = 1,    // range limit value (lower)
    PROPERTY_LORA_CR_4_5    = 1,    // 
    PROPERTY_LORA_CR_4_6    = 2,    // 
    PROPERTY_LORA_CR_4_7    = 3,    // 
    PROPERTY_LORA_CR_4_8    = 4,    // 
    PROPERTY_LORA_CR_MAX,           // range limit value (upper)
} property_cr_t;

typedef struct {
    uint16_t cpu_code;
    uint16_t functionality_code;
} ll_firmware_type_t;

#define FIRMWARE_TYPE_LEN           (4)

/** Possible downlink modes for OP_DOWNLINK_MODE */
typedef enum
{
    DOWNLINK_MODE_OFF = 0,          // 0x00
    DOWNLINK_MODE_ALWAYS_ON = 1,    // 0x01
    DOWNLINK_MODE_MAILBOX = 2,      // 0x02
    DOWNLINK_MODE_PERIODIC = 3,     // 0x03
    NUM_DOWNLINK_MODES,
    DOWNLINK_MODE_FAILURE = 255,    // 0xFF
} downlink_mode_t;

/** ACK/NACK Codes */
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
#define LL_IFC_NACK_PAYLOAD_BAD_PROPERTY    (10)  // Invalid property specified in command
#define LL_IFC_NACK_NODATA                  (11)  // No data is available to be returned
#define LL_IFC_NACK_QUEUE_FULL              (12)  // Data could not be enqueued for transmission (queue full)
#define LL_IFC_NACK_OTHER                   (99)
/* When adding a new value, update ll_return_code_name() and ll_return_code_description() */

/** Error Codes */
/* Note: Error codes -1 to -99 map to NACK codes received from the radio */
typedef enum ll_ifc_error_codes_e {
    LL_IFC_ERROR_INCORRECT_PARAMETER        = -101, //< The parameter value was invalid.
    LL_IFC_ERROR_INCORRECT_RESPONSE_LENGTH  = -102, //< Module response was not the expected size.
    LL_IFC_ERROR_MESSAGE_NUMBER_MISMATCH    = -103, //< Message number in response doesn't match expected
    LL_IFC_ERROR_CHECKSUM_MISMATCH          = -104, //< Checksum mismatch
    LL_IFC_ERROR_COMMAND_MISMATCH           = -105, //< Command mismatch (responding to a different command)
    LL_IFC_ERROR_HOST_INTERFACE_TIMEOUT     = -106, //< Timed out waiting for Rx bytes from interface
    LL_IFC_ERROR_BUFFER_TOO_SMALL           = -107, //< Response larger than provided output buffer
    LL_IFC_ERROR_START_OF_FRAME             = -108, //< transport_read failed getting FRAME_START
    LL_IFC_ERROR_HEADER                     = -109, //< transport_read failed getting header
    LL_IFC_ERROR_TIMEOUT                    = -110, //< The operation timed out.
    LL_IFC_ERROR_INCORRECT_MESSAGE_SIZE     = -111, //< The message size from the device was incorrect.
    LL_IFC_ERROR_NO_NETWORK                 = -112, //< No network was available.
    /* When adding a new value, update ll_return_code_name() and ll_return_code_description() */
} ll_ifc_error_codes_t;


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
#define IRQ_FLAGS_TX_ERROR                    (0x00000020UL)  // Set every time there is a Tx Error (for Ensemble this is when a message cannot be deliverd to a Host)
#define IRQ_FLAGS_ACK_RECEIVED                (0x00000040UL)  // Set when a transmitted has been ACKnowledged to have been recieved by Host (Ensemble feature only)
#define IRQ_FLAGS_RX_DONE                     (0x00000100UL)  // Set every time a new packet is received
#define IRQ_FLAGS_CONNECTED                   (0x00001000UL)  // Set every time we transition from the disconnected -> connected state
#define IRQ_FLAGS_DISCONNECTED                (0x00002000UL)  // Set every time we transition from the connected -> disconnected state
#define IRQ_FLAGS_CRYPTO_ESTABLISHED          (0x00010000UL)  // Set every time we transition from the crypto not established -> crytpto established state
#define IRQ_FLAGS_APP_TOKEN_CONFIRMED         (0x00020000UL)  // Set every time an application token is confirmed by Conductor
#define IRQ_FLAGS_DOWNLINK_REQUEST_ACK        (0x00040000UL)  // Set every time a downlink registration request is acknowledged
#define IRQ_FLAGS_INITIALIZATION_COMPLETE     (0x00080000UL)  // Set every time the MAC has completed initialization
#define IRQ_FLAGS_CRYPTO_ERROR                (0x00100000UL)  // Set when a crypto exchange attempt fails
#define IRQ_FLAGS_APP_TOKEN_ERROR             (0x00200000UL)  // Set when an application token registration fails
#define IRQ_FLAGS_ASSERT                      (0x80000000UL)  // Set every time we transition from the connected->disconnected state

/**
 * @brief
 *   The operations for ll_timestamp_set().
 */
typedef enum ll_timestamp_operation_e {
    /**
     * @brief
     *   No set operation.
     *
     * @details
     *   Just get the current timestamp.
     */
    LL_TIMESTAMP_NO_OPERATION,

    /**
     * @brief
     *   Directly set the timestamp from the provided value.
     *
     * @details
     *   The value is not applied until the command is processed by the
     *   module, and it does not account for transmission delay.
     */
    LL_TIMESTAMP_SET_IMMEDIATE,

    /**
     * @brief
     *   Synchronize the timestamp using the provided value which
     *   corresponds to the most recent event.
     *
     * @details
     *   Use this mechanism when the host directly controls the trigger
     *   event received by both the reference time source and the module.
     *   This mechanism guarantees the module that the reference value
     *   aligns to the module's timestamp value when the most recent
     *   trigger event occurred.
     */
    LL_TIMESTAMP_SYNC_IMMEDIATE,

    /**
     * @brief
     *   Intelligently synchronize the timestamp using the provided value
     *   accounting for possible mismatched trigger events.
     *
     * @details
     *   Use this mechanism when the host does not control the trigger
     *   event.  When the trigger event is free-running such as with a
     *   GPS pulse-per-seconds (PPS) output, the reference value may
     *   be one event behind the module's value.  This mechanism allows
     *   the module to detect and account for this variability.  To
     *   correctly update the module's timestamp using this mechanism:
     *
     *   1. Read the reference timestamp.
     *   2. Call ll_timestamp_set()
     *   3. Read the reference timestamp.
     *   4. Call ll_timestamp_set()
     */
    LL_TIMESTAMP_SYNC
} ll_timestamp_operation_t;

/** @} (end addtogroup Module_Interface) */


/* ------------------ LoRaWAN ------------------------------------------ */

/**
 * @addtogroup LoRaWAN_Interface
 * @{
 */

/**
 * @brief
 *   The LoRaWAN network type designation.
 */
typedef enum ll_lorawan_network_type_e {

    /** The LoRaWAN network is public (default). */
    LL_LORAWAN_PUBLIC = 0,

    /** The LoRaWAN network is private. */
    LL_LORAWAN_PRIVATE = 1
} ll_lorawan_network_type_t;


/**
 * @brief
 *   The LoRaWAN network activation subcommand.
 */
typedef enum ll_lorawan_activation_e {
    /**
     * @brief
     *   Perform over-the-air activation.
     */
    LL_LORAWAN_ACTIVATION_OVER_THE_AIR = 0,

    /**
     * @brief
     *   Perform activation using predefined network information.
     */
    LL_LORAWAN_ACTIVATION_PERSONALIZATION = 1,

    /**
     * @brief
     *   Query the current activation status.
     */
    LL_LORAWAN_ACTIVATION_QUERY = 2
} ll_lorawan_activation_t;

/**
 * @brief
 *   The LoRaWAN device class.
 *
 * @details
 *   The device class determines how the device communicates with the
 *   network.  The device class selection can drastically affect
 *   power consumption and communication latencies.  See the LoRaWAN
 *   specification for details.
 */
typedef enum ll_lorawan_device_class_e {
    /**
     * @brief
     *   Class A
     *
     * @details
     *   Class A devices consume the lowest power but have the highest
     *   amount of latency.  The gateway can only communicate with the
     *   device after the device transmits a message.  The gateway cannot
     *   initiate communications to the device.
     */
    LL_LORAWAN_CLASS_A = 0,

    /**
     * @brief
     *   Class B
     *
     * @details
     *   Class B devices consume slightly higher power but allow for the
     *   gateway to initiate communication at fixed intervals.  The
     *   gateway implements a beacon and can provide predictable service
     *   intervals, unlike class A.  The drawback is slightly higher power
     *   consumption compared to class A.
     */
    LL_LORAWAN_CLASS_B = 1,

    /**
     * @brief
     *   Class C
     *
     * @details
     *   Class C devices are always listening and can receive messages
     *   from the gateway at any time.  The drawback is significantly
     *   higher power consumption that is usually not compatible for
     *   non-rechargable battery-powered devices.
     */
    LL_LORAWAN_CLASS_C = 2,
} ll_lorawan_device_class_t;

/**
 * @brief
 *   The activation status.
 */
typedef enum ll_lorawan_activation_status_e
{
    LL_LORAWAN_ACTIVATION_STATUS_COMPLETED = 0,
    LL_LORAWAN_ACTIVATION_STATUS_PENDING = 1,
    LL_LORAWAN_ACTIVATION_STATUS_FAILED = 2,
} ll_lorawan_activation_status_t;

/**
 * @brief
 *   The configurable LoRaWAN parameters.
 */
enum ll_lorawan_param_e {
    /**
     * @brief
     *   When zero, use only the default rate for a channel.  When
     *   non-zero, enable adaptive rate.
     */
    LL_LORAWAN_PARAM_ADAPTIVE_DATA_RATE_ENABLED,

    /**
     * @brief
     *   The current ll_lorawan_activation_status_t (read only).
     *
     * @details
     *   Equivalent to OP_LORAWAN_ACTIVATE with LL_LORAWAN_ACTIVATION_QUERY.
     */
    LL_LORAWAN_PARAM_NETWORK_ACTIVATION_STATUS,

    /**
     * @brief
     *   The network type as set during join (read only).
     */
    LL_LORAWAN_PARAM_NETWORK_TYPE,

    /**
     * @brief
     *   The device class as set during join (read only).
     */
    LL_LORAWAN_PARAM_DEVICE_CLASS,

    /**
     * @brief
     *   The device's uplink counter (read only).
     */
    LL_LORAWAN_PARAM_UPLINK_COUNTER,

    /**
     * @brief
     *   The device's downlink counter (read only).
     */
    LL_LORAWAN_PARAM_DOWNLINK_COUNTER,
};

/**
 * @brief
 *   The option flags for sending messages.
 */
typedef enum ll_lorawan_send_flags_e {
    /**
     * @brief
     *   Check the link quality to the gateway.
     */
    LL_LORAWAN_SEND_LINK_CHECK = 1,

    /**
     * @brief
     *   Flag set internally to indicate confirmed transmission.
     */
    LL_LORAWAN_SEND_CONFIRMED = 0x80,
} ll_lorawan_send_flags_t;

/**
 * @brief
 *   The option flags for receiving messages.
 */
typedef enum ll_lorawan_receive_flags_e {

    /**
     * @brief
     *   The receive metadata contains link check information.
     */
    LL_LORAWAN_RECEIVE_LINK_CHECK = (1 << 0),

    /**
     * @brief
     *   The receive returned a message.
     */
    LL_LORAWAN_RECEIVE_MESSAGE = (1 << 1),

    /**
     * @brief
     *   The receive returned a message that was multicast.
     *
     * @details
     *   LL_LORAWAN_RECEIVE_MESSAGE will also be set.
     */
    LL_LORAWAN_RECEIVE_MESSAGE_MULTICAST = (1 << 2),

    /**
     * @brief
     *   The receive indicated that the gateway acknowledge a sent message.
     */
    LL_LORAWAN_RECEIVE_ACK = (1 << 3),
} ll_lorawan_receive_flags_t;

/**
 * @brief
 *   Convert signed integer RSSI to the message value
 *
 * @param[in] x
 *   The input RSSI.
 *
 * @return
 *   The uint8_t representation of x.
 */
#define LL_LORAWAN_RSSI_TO_PKT(x) ((uint8_t) ((x) + 255 - 20))

/**
 * @brief
 *   Convert the uint8_t RSSI value to the signed integer value.
 *
 * @param[in] x
 *   The RSSI format representation.
 *
 * @return
 *   The actual int16_t RSSI.
 */
#define LL_LORAWAN_RSSI_FROM_PKT(x) ((int16_t) (x) + 20 - 255)

/** @} (end defgroup Module_Interface) */

/** @} (end addtogroup Link_Labs_Interface_Library) */

#endif /* __LL_IFC_CONSTS_H */
