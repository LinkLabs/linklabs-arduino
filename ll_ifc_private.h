#ifndef __LL_IFC_PRIVATE_H
#define __LL_IFC_PRIVATE_H

#include "ll_ifc_consts.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief
 *   Read/Write from the module.  If the opcode does not have a payload
 *   associated with it, then buf_in and/or buf_out should be set to NULL
 *
 * @param[in] op
 *   opcode of the command being sent to the module
 *
 * @param[in] buf_in
 *   byte array containing the data payload to be sent to the module
 *
 * @param[in] in_len
 *   size of the output buffer in bytes
 *
 * @param[in] buf_out
 *   byte array for storing data returned from the module
 *
 * @param[in] out_len
 *   size of the output buffer in bytes
 *
 * @return
 *   positive number of bytes returned,
 *   negative if an error
 *   Error Values:
 *    LL_IFC_ERROR_INCORRECT_PARAMETER = Invalid values in one or more arguments
 *    other negative values defined by recv_packet()
 *
 */
int32_t hal_read_write(opcode_t op, uint8_t buf_in[], uint16_t in_len, uint8_t buf_out[], uint16_t out_len);

int32_t hal_read_write_exact(opcode_t op, uint8_t buf_in[], uint16_t in_len, uint8_t buf_out[], uint16_t out_len);


#ifdef __cplusplus
}
#endif


#endif /* __LL_IFC_PRIVATE_H */
