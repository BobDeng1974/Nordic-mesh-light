
#ifndef BASE_COMMUNICATION_CLIENT_H__
#define BASE_COMMUNICATION_CLIENT_H__

#include <stdint.h>
#include "access.h"
#include "base_communication_common.h"

/**
 * @defgroup BASE_COMMUNICATION_CLIENT commands Client
 * @ingroup BASE_COMMUNICATION_MODEL
 * This module implements a vendor specific communication commands Client.
 *
 * @{
 */

/** communication commands Client model ID. */
#define BASE_COMMUNICATION_CLIENT_MODEL_ID (0x0008)



/** Forward declaration. */
typedef struct __base_communication_client base_communication_client_t;

/**
 * communication commands status callback type.
 *
 * @param[in] p_self Pointer to the communication commands client structure that received the status.
 * @param[in] status The received status of the remote server.
 * @param[in] src    Element address of the remote server.
 */
typedef void (*base_communication_status_cb_t)(const base_communication_client_t * p_self, base_communication_status_t status, uint16_t src);
/**
 * communication commands timeout callback type.
 *
 * @param[in] handle Model handle
 * @param[in] p_self Pointer to the communication commands client structure that received the status.
 */
typedef void (*base_communication_timeout_cb_t)(access_model_handle_t handle, void * p_self);

/** communication commands Client state structure. */
struct __base_communication_client
{
    /** Model handle assigned to the client. */
    access_model_handle_t model_handle;
    /** Status callback called after status received from server. */
    base_communication_status_cb_t status_cb;
    /** Timeout callback called after acknowledged message sending times out */
    base_communication_timeout_cb_t timeout_cb;
    /** Internal client state. */
    struct
    {
        bool reliable_transfer_active; /**< Variable used to determine if a transfer is currently active. */
        base_communication_msg_t data;  /**< Variable reflecting the data stored in the server. */
    } state;
};

/**
 * Initializes the communication commands client.
 *
 * @note This function should only be called _once_.
 * @note The client handles the model allocation and adding.
 *
 * @param[in,out] p_client      communication commands Client structure pointer.
 * @param[in]     element_index Element index to add the server model.
 *
 * @retval NRF_SUCCESS         Successfully added client.
 * @retval NRF_ERROR_NULL      NULL pointer supplied to function.
 * @retval NRF_ERROR_NO_MEM    No more memory available to allocate model.
 * @retval NRF_ERROR_FORBIDDEN Multiple model instances per element is not allowed.
 * @retval NRF_ERROR_NOT_FOUND Invalid element index.
 */
uint32_t base_communication_client_init(base_communication_client_t * p_client, uint16_t element_index);

uint32_t base_communication_command_send_unreliable(base_communication_client_t *p_client, uint8_t command_id, uint8_t *data, uint16_t data_lenght, uint8_t repeats);

/**
 * Sets the state of the communication commands server.
 *
 * @param[in,out] p_client communication commands Client structure pointer.
 * @param[in]     command  base communication command.
 *
 * @retval NRF_SUCCESS              Successfully sent message.
 * @retval NRF_ERROR_NULL           NULL pointer in function arguments
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_ADDR   The element index is greater than the number of local unicast
 *                                  addresses stored by the @ref DEVICE_STATE_MANAGER.
 * @retval NRF_ERROR_INVALID_STATE  Message already scheduled for a reliable transfer.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 */
uint32_t base_communication_client_command(base_communication_client_t * p_client, base_communication_msg_t command);

/**
 * Cancel any ongoing reliable message transfer.
 *
 * @param[in,out] p_client Pointer to the client instance structure.
 */
void base_communication_client_pending_msg_cancel(base_communication_client_t * p_client);


/** @} end of BASE_COMMUNICATION_CLIENT */

#endif /* BASE_COMMUNICATION_CLIENT_H__ */
