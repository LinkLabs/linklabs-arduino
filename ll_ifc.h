#ifndef __LL_IFC_H
#define __LL_IFC_H

#include <stdint.h>
#include <time.h>
#include "ll_ifc_consts.h"
#include "ifc_struct_defs.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup Link_Labs_Interface_Library ll_ifc
 *
 * @brief External host library which simplifies communication to an RF
 *      network using a Link Labs module.
 *
 * This library runs on an <b>external host</b> which can be nearly any
 * microcontroller or PC with a UART interface.  The library
 * is written in standard ANSI C (C89) which is supported by nearly every
 * C compiler.  In addition to the standard C library, the external host
 * must implement a handful of functions.  See @ref HAL_Interface
 * for the list of required functions.
 *
 * Link Labs modules can support different modes of operation,
 * and the external host can dynamically select the operating mode using
 * ll_mac_mode_set().  The available operating modes are:
 * - @ref Symphony_Interface (915 MHz)
 * - @ref LoRaWAN_Interface (868 MHz and 915 MHz)
 * - @ref NoMac_Interface (868 MHz and 915 MHz)
 * - @ref Ensemble_Interface (868 MHz and 915 MHz)
 *
 * In addition to the functions unique to each operating mode, the API
 * includes a number of common management functions.  The
 * \ref Module_Interface API provides these common functions.
 *
 * @{
 */

    /**
     * @defgroup HAL_Interface HAL
     * 
     * @brief The hardware abstraction layer (HAL) for ll_ifc.
     *
     * All functions in the HAL are used by the Link Lab's Interface library
     * (ll_ifc).  These functions must be defined by the program using the
     * Link Lab's Interface Library.
     *
     * @{
     */

    /**
     * @brief Write data to the Link Lab's module.
     *
     * @param[in] buff
     *   The buffer containing the data to write to the module.  The size of
     *   buff must be at least len bytes.
     *
     * @param[in] len
     *   The number of bytes to write.
     *
     * @return
     *   0 - success, negative otherwise
     *
     * This function is usually a simple UART wrapper.
     */
    int32_t transport_write(uint8_t *buff, uint16_t len);

    /**
     * @brief Read data from the Link Lab's module.
     *
     * @param[inout] buff
     *   The buffer that will be modified with the data read from the module.
     *   The size of buff must be at least len bytes.
     *
     * @param[in] len
     *   The number of bytes to read.
     *
     * @return
     *   0 - success, negative otherwise
     *
     * This function is usually a simple UART wrapper.
     */
    int32_t transport_read(uint8_t *buff, uint16_t len);

    /**
     * @brief The structure used to store time.
     */
    struct time
    {
        long tv_sec;
        long tv_nsec;
    };

    /**
     * @brief
     *   Get the current time.
     *
     * @param[out] tp
     *   Current value of the clock which
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t gettime(struct time *tp);

    /**
     * @brief
     *   Sleep for a number of milliseconds
     *
     * @param[in] millis
     *   number of milliseconds
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t sleep_ms(int32_t millis);

    /** @} (end defgroup HAL_Interface) */


    /**
     * @addtogroup Module_Interface Core
     * @brief Core functions and data structures used to communicate with the
     *      module across different MAC modes.
     *
     * @{
     */

    /**
     * @brief
     *   Convert an return code into a short name string.
     *
     * @param[in] return_code
     *   The return code.
     *
     * @return
     *   The short name string.
     */
    char const * ll_return_code_name(int32_t return_code);

    /**
     * @brief
     *   Convert an return code into a description.
     *
     * @param[in] return_code
     *   The return code.
     *
     * @return
     *   The user-meaningful return code description.
     */
    char const * ll_return_code_description(int32_t return_code);

    /**
     * @brief
     *   Get the module firmware type
     *
     * @param[out] t
     *   pointer to a firmware type struct
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_firmware_type_get(ll_firmware_type_t *t);

    /**
     * @brief
     *   Get the module hardware type
     *
     * @param[out] t
     *   pointer to a hardware type enum
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_hardware_type_get(ll_hardware_type_t *t);

    /**
     * @brief
     *   Get the string for the module hardware type
     *
     * @param[in] t
     *   The hardware type enum, usually from ll_hardware_type_get().
     *
     * @return
     *   The string describing the hardware type.
     */
    const char * ll_hardware_type_string(ll_hardware_type_t t);

    /**
     * @brief
     *   Get the host interface version number
     *
     * @param[out] version
     *   pointer to a version struct
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_interface_version_get(ll_version_t *version);

    /**
     * @brief
     *   Get the module firmware version number
     *
     * @param[out] version
     *   pointer to a version struct
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_version_get(ll_version_t *version);

    /**
	 * @brief
	 *   Block any sleep mode.
	 *
     * @details
     *   This function blocks any sleep mode.  ll_sleep_unblock() allows the
     *   module to enter into the sleep mode again.
     *
	 * @return
	 *   0 - success, negative otherwise
	 */
	int32_t ll_sleep_block(void);

    /**
	 * @brief
	 *   Unblock sleep modes
	 *
     * @details
     *   This function unblocks sleep mode regardless of how many times
     *   ll_sleep_block() has been called.
     *
	 * @return
	 *   0 - success, negative otherwise
	 */
	int32_t ll_sleep_unblock(void);

	/**
     * @brief
     *   Set the MAC Mode
     *
     * @param[in] mac_mode
     *   0 = No MAC (Pass through mode)
     *   1 = LoRaMAC (EU version)
     *   2 = LoRaMAC (US version)
     *   3 = Symphony Link
     *   4 = Link Labs Ensemble
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_mac_mode_set(ll_mac_type_t mac_mode);

    /**
     * @brief
     *   Get the MAC Mode
     *
     * @param[out] mac_mode The current MAC operating mode.
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_mac_mode_get(ll_mac_type_t *mac_mode);

    /**
     * @brief
     *   Get the module unique identifier
     *
     * @param[out] unique_id
     *   pointer to a unsigned 64-bit integer
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_unique_id_get(uint64_t *unique_id);

    /**
     * @brief
     *   Get the antenna configuration
     *
     * @param[out] ant
     *   Antenna configuration.
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_antenna_get(uint8_t *ant);

    /**
     * @brief
     *   Set the antenna configuration
     *
     * @param[in] ant
     *   Antenna configuration. (1=>U.FL, 2=>trace)
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_antenna_set(uint8_t ant);

     /**
     * @brief
     *   Store the current radio parameters to flash:
     *     - mode
     *     - frequency
     *     - bandwidth
     *     - spreading factor
     *     - coding rate
     *     - low_rate_opt
     *     - tx power
     *
     * @details
     *   This function stores settings to flash.
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_settings_store(void);

    /**
     * @brief
     *   Delete all settings from flash storage.
     *
     * @details
     *   This function stores settings to flash.
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_settings_delete(void);

    /**
     * @brief
     *   Restore the default radio parameters:
     *     - frequency
     *     - bandwidth
     *     - spreading factor
     *     - coding rate
     *     - low_rate_opt
     *     - tx power
     *
     * @details
     *   This function stores settings to flash.
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_restore_defaults(void);

    /**
     * @brief
     *   Force the module to enter sleep.
     *
     * @details
     *   This function puts the module into the sleep mode.  The wakeup
     *   signal is issued with a logic high pulse of at least 60 us duration.
     *   Alternatively, the module will enter the idle state in response to a received
     *   byte on the host UART.
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_sleep(void);

    /**
     * @brief
     *   Force the module to reset (takes a few seconds).
     *
     * @details
     *   This function forces the module to reset.
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_reset_mcu(void);

    /**
     * @brief
     *   Force the module to reset and enter bootloader mode (takes a few seconds).
     *
     * @details
     *   This function forces the module to reset and enter bootloader mode.
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_bootloader_mode(void);

     /**
      * @brief
      *   Read (and optionally clear) IRQ flags in module
      *
      * @details
      *   This function allows the external host to check whether an event has occurred in the
      *   module that has latched a bit in the "IRQ Flags" vector.
      *
      * @param[in] flags_to_clear
      *   A uint32_t bit vector containing flags that should be cleared if they are set. This can be
      *   0 if the host interface just wants to read without clearing. If a bit is set, this function
      *   performs a clear-on-read of the irq_flags bits passed in.
      *
      * @param[out] flags
      *   A uint32_t bit vector - the value of the irq_flags in the module. Note that if the flags_to_clear
      *   argument is non-zero, this argument is the value of the flags before the clear operation.
      *
      * @return
      *   0 - success, negative otherwise
      */
     int32_t ll_irq_flags(uint32_t flags_to_clear, uint32_t *flags);

#if 0
     /**
      * @brief
      *   Set IRQ flags in module
      *
      * @details
      *
      * @param[in] none
      *
      * @param[out] flags mask
      *
      * @return
      *   0 - success, negative otherwise
      */
     int8_t ll_irq_flags_mask_get(uint32_t flags_to_clear, uint32_t *flags);

     /**
      * @brief
      *   Set IRQ flags in module
      *
      * @details
      *
      * @param[in] none
      *
      * @param[out] flags mask
      *
      * @return
      *   0 - success, negative otherwise
      */
     int8_t ll_irq_flags_mask_set(uint32_t flags_to_clear, uint32_t *flags);
#endif

    /**
     * @brief
     *   Reset the host-side state maintained by the interface.
     *
     * @details
     *   The host implementation maintains a very small amount of state
     *   including the current message identifier.  This function resets
     *   this internal state and is intended to allow for controlled
     *   testing.  This function is not normally used in production code.
     *
     * @return
     *   0 - success, negative otherwise.
     */
    int32_t ll_reset_state( void );

    /** @} (end defgroup Module_Interface) */


    /** @} (end defgroup Link_Labs_Interface_Library) */

#ifdef __cplusplus
}
#endif

#endif /* __LL_IFC_H */
