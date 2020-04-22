#include "base_communication_client.h"
#include "base_communication_common.h"

#include "stdlib.h"

#include <stdint.h>
#include <stddef.h>

#include "access.h"
#include "access_config.h"
#include "access_reliable.h"
#include "device_state_manager.h"
#include "nrf_mesh.h"
#include "nrf_mesh_assert.h"
#include "log.h"

/*****************************************************************************
 * Static variables
 *****************************************************************************/

/** Keeps a single global TID for all transfers. */
static uint8_t m_tid;

/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void reliable_status_cb(access_model_handle_t model_handle,
                               void * p_args,
                               access_reliable_status_t status)
{
    base_communication_client_t * p_client = p_args;
    NRF_MESH_ASSERT(p_client->status_cb != NULL);

    p_client->state.reliable_transfer_active = false;
    switch (status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            /* Ignore */
            break;
        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            p_client->status_cb(p_client, BASE_COMMUNICATION_STATUS_ERROR_NO_REPLY, NRF_MESH_ADDR_UNASSIGNED);
            break;
        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            p_client->status_cb(p_client, BASE_COMMUNICATION_STATUS_CANCELLED, NRF_MESH_ADDR_UNASSIGNED);
            break;
        default:
            /* Should not be possible. */
            NRF_MESH_ASSERT(false);
            break;
    }
}

/** Returns @c true if the message received was from the address corresponding to the clients
 * publish address. */
static bool is_valid_source(const base_communication_client_t * p_client,
                            const access_message_rx_t * p_message)
{
    /* Check the originator of the status. */
    dsm_handle_t publish_handle;
    nrf_mesh_address_t publish_address;
    if (access_model_publish_address_get(p_client->model_handle, &publish_handle) != NRF_SUCCESS ||
        publish_handle == DSM_HANDLE_INVALID ||
        dsm_address_get(publish_handle, &publish_address) != NRF_SUCCESS ||
        publish_address.value != p_message->meta_data.src.value)
    {
        return false;
    }
    else
    {
        return true;
    }
}

static uint32_t send_reliable_message(const base_communication_client_t * p_client,
                                      base_communication_opcode_t opcode,
                                      const uint8_t * p_data,
                                      uint16_t length)
{
    access_reliable_t reliable;
    reliable.model_handle = p_client->model_handle;
    reliable.message.p_buffer = p_data;
    reliable.message.length = length;
    reliable.message.opcode.opcode = opcode;
    reliable.message.opcode.company_id = BASE_COMMUNICATION_COMPANY_ID;
    reliable.message.force_segmented = false;
    reliable.message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    reliable.message.access_token = nrf_mesh_unique_token_get();
    reliable.reply_opcode.opcode = BASE_COMMUNICATION_OPCODE_STATUS;
    reliable.reply_opcode.company_id = BASE_COMMUNICATION_COMPANY_ID;
    reliable.timeout = ACCESS_RELIABLE_TIMEOUT_MIN;
    reliable.status_cb = reliable_status_cb;

    return access_model_reliable_publish(&reliable);
}

/*****************************************************************************
 * Opcode handler callback(s)
 *****************************************************************************/

static void handle_status_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    base_communication_client_t * p_client = p_args;
    NRF_MESH_ASSERT(p_client->status_cb != NULL);

    if (!is_valid_source(p_client, p_message))
    {
        return;
    }

    base_communication_msg_status_t * p_status =
        (base_communication_msg_status_t *) p_message->p_data;
    base_communication_status_t status = p_status->status;
    p_client->status_cb(p_client, status, p_message->meta_data.src.value);
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {{BASE_COMMUNICATION_OPCODE_STATUS, BASE_COMMUNICATION_COMPANY_ID}, handle_status_cb}
};

static void handle_publish_timeout(access_model_handle_t handle, void * p_args)
{
    base_communication_client_t * p_client = p_args;

    if (p_client->timeout_cb != NULL)
    {
        p_client->timeout_cb(handle, p_args);
    }
}
/*****************************************************************************
 * Public API
 *****************************************************************************/

uint32_t base_communication_client_init(base_communication_client_t * p_client, uint16_t element_index)
{
    if (p_client == NULL ||
        p_client->status_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "element_index %d\n", element_index);

    access_model_add_params_t init_params;
    init_params.model_id.model_id = BASE_COMMUNICATION_CLIENT_MODEL_ID;
    init_params.model_id.company_id = BASE_COMMUNICATION_COMPANY_ID;
    init_params.element_index = element_index;
    init_params.p_opcode_handlers = &m_opcode_handlers[0];
    init_params.opcode_count = sizeof(m_opcode_handlers) / sizeof(m_opcode_handlers[0]);
    init_params.p_args = p_client;
    init_params.publish_timeout_cb = handle_publish_timeout;
    return access_model_add(&init_params, &p_client->model_handle);
}

uint32_t base_communication_client_command(base_communication_client_t * p_client, base_communication_msg_t command) {
    return NRF_SUCCESS;
}


uint32_t base_communication_command_send_unreliable(base_communication_client_t *p_client, uint8_t command_id, uint8_t *data, uint16_t data_lenght, uint8_t repeats)
{

    uint8_t *command = malloc(data_lenght+2 * sizeof(uint8_t));
    command[0] = command_id;
    command[1] = m_tid++;
    memcpy(&command[2], data, data_lenght);

    access_message_tx_t message;
    message.opcode.opcode = BASE_COMMUNICATION_OPCODE_COMMAND;
    message.opcode.company_id = BASE_COMMUNICATION_COMPANY_ID;
    message.p_buffer = command;
    message.length = data_lenght+2;
    message.force_segmented = false;
    message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;

    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "base communication msg", message.p_buffer, message.length);

    uint32_t status = NRF_SUCCESS;
    for (uint8_t i = 0; i < repeats; ++i)
    {
        message.access_token = nrf_mesh_unique_token_get();
        status = access_model_publish(p_client->model_handle, &message);
        if (status != NRF_SUCCESS)
        {
            break;
        }
    }

    free(command);

    return status;
}

/**
 * Cancel any ongoing reliable message transfer.
 *
 * @param[in] p_client Pointer to the client instance structure.
 */
void base_communication_client_pending_msg_cancel(base_communication_client_t * p_client)
{
    (void)access_model_reliable_cancel(p_client->model_handle);
}
