
#include "base_communication_server.h"
#include "base_communication_common.h"

#include <stdint.h>
#include <stddef.h>

#include "access.h"
#include "nrf_mesh_assert.h"
#include "log.h"

/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void reply_status(const base_communication_server_t * p_server,
                         const access_message_rx_t * p_message,
                         uint8_t response)
{
    base_communication_msg_status_t status;
    status.status = response;
    access_message_tx_t reply;
    reply.opcode.opcode = BASE_COMMUNICATION_OPCODE_STATUS;
    reply.opcode.company_id = BASE_COMMUNICATION_COMPANY_ID;
    reply.p_buffer = (const uint8_t *) &status;
    reply.length = sizeof(status);
    reply.force_segmented = false;
    reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    reply.access_token = nrf_mesh_unique_token_get();

    (void) access_model_reply(p_server->model_handle, p_message, &reply);
}

static void reply_message(const base_communication_server_t * p_server,
                         const access_message_rx_t * p_message,
                         const base_communication_msg_response_t * response)
{
    access_message_tx_t reply;
    reply.opcode.opcode = BASE_COMMUNICATION_OPCODE_RESPONSE;
    reply.opcode.company_id = BASE_COMMUNICATION_COMPANY_ID;
    reply.p_buffer = response->response;
    reply.length = response->length;
    reply.force_segmented = false;
    reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    reply.access_token = nrf_mesh_unique_token_get();

    (void) access_model_reply(p_server->model_handle, p_message, &reply);
}

/*****************************************************************************
 * Opcode handler callbacks
 *****************************************************************************/

static void handle_base_communication_command_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    base_communication_server_t * p_server = p_args;

    NRF_MESH_ASSERT(p_server->command_cb != NULL);

    base_communication_msg_t command = {
        .opcode = p_message->p_data[0],
        .tid = p_message->p_data[1],
        .command = &p_message->p_data[2],
        .length = p_message->length-2,
    };

    base_communication_msg_response_t response;
    response.length = UINT8_MAX;

    uint8_t value = p_server->command_cb(p_server, &command, &response);
    if (value != BASE_COMMUNICATION_STATUS_NO_REPLY) {
        if (value == BASE_COMMUNICATION_STATUS_MESSAGE) {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "len %d\n", response.length);
            __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "response", response.response, response.length);
            NRF_MESH_ASSERT(response.length != UINT8_MAX);
            reply_message(p_server, p_message, &response);
        } else {
            reply_status(p_server, p_message, value);
        }
    }
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_VENDOR(BASE_COMMUNICATION_OPCODE_COMMAND,	BASE_COMMUNICATION_COMPANY_ID), handle_base_communication_command_cb}
};

/*****************************************************************************
 * Public API
 *****************************************************************************/

uint32_t base_communication_server_init(base_communication_server_t * p_server, uint16_t element_index)
{
    if (p_server == NULL ||
        p_server->command_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    access_model_add_params_t init_params;
    init_params.element_index =  element_index;
    init_params.model_id.model_id = BASE_COMMUNICATION_SERVER_MODEL_ID;
    init_params.model_id.company_id = BASE_COMMUNICATION_COMPANY_ID;
    init_params.p_opcode_handlers = &m_opcode_handlers[0];
    init_params.opcode_count = sizeof(m_opcode_handlers) / sizeof(m_opcode_handlers[0]);
    init_params.p_args = p_server;
    init_params.publish_timeout_cb = NULL;
    return access_model_add(&init_params, &p_server->model_handle);
}

