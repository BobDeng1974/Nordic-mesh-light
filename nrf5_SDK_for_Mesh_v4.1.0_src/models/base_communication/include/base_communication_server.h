
#ifndef BASE_COMMUNICATION_SERVER_H__
#define BASE_COMMUNICATION_SERVER_H__

#include <stdint.h>
#include <stdbool.h>
#include "access.h"
#include "base_communication_common.h"

/**
 * @defgroup BASE_COMMUNICATION_SERVER communication commands Server
 * @ingroup BASE_COMMUNICATION_MODEL
 * This module implements a vendor specific communication commands Server.
 * @{
 */

/** communication commands Server model ID. */
#define BASE_COMMUNICATION_SERVER_MODEL_ID (0x0009)

/** Forward declaration. */
typedef struct __base_communication_server base_communication_server_t;

/**
 * command callback type.
 * @param[in] p_self Pointer to the communication commands Server context structure.
 * @param[in] command
 */
typedef uint8_t (*base_communication_command_cb_t)(const base_communication_server_t * p_self, const base_communication_msg_t *command, base_communication_msg_response_t * p_response);

/** communication commands Server state structure. */
struct __base_communication_server
{
    /** Model handle assigned to the server. */
    access_model_handle_t model_handle;
    /** command callback. */
    base_communication_command_cb_t command_cb;
};

/**
 * Initializes the communication commands server.
 *
 * @note This function should only be called _once_.
 * @note The server handles the model allocation and adding.
 *
 * @param[in] p_server      communication commands Server structure pointer.
 * @param[in] element_index Element index to add the server model.
 *
 * @retval NRF_SUCCESS         Successfully added server.
 * @retval NRF_ERROR_NULL      NULL pointer supplied to function.
 * @retval NRF_ERROR_NO_MEM    No more memory available to allocate model.
 * @retval NRF_ERROR_FORBIDDEN Multiple model instances per element is not allowed.
 * @retval NRF_ERROR_NOT_FOUND Invalid element index.
 */
uint32_t base_communication_server_init(base_communication_server_t * p_server, uint16_t element_index);

/**
 * Publishes unsolicited status message.
 *
 * This API can be used to send unsolicited status messages to report updated state value as a result
 * of local action.
 *
 * @param[in]  p_server         communication commands Server structure pointer
 * @param[in]  command          communication command
 *
 * @retval NRF_SUCCESS              Successfully queued packet for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer supplied to function.
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_ADDR   The element index is greater than the number of local unicast
 *                                  addresses stored by the @ref DEVICE_STATE_MANAGER.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_INVALID_LENGTH Attempted to send message larger than @ref ACCESS_MESSAGE_LENGTH_MAX.
 *
 */
uint32_t base_communication_server_status_publish(base_communication_server_t * p_server, base_communication_msg_t command);

/** @} end of BASE_COMMUNICATION_SERVER */

#endif /* BASE_COMMUNICATION_SERVER_H__ */
