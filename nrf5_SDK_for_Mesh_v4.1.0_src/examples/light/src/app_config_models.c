#include "app_config_models.h"

// #include <stdint.h>
// #include <string.h>

#include "mesh_stack.h"
#include "access_config.h"

#include "log.h"

#define GATEWAY_PUBLISH_ARRD 0x7ff0

static uint16_t get_element_index(uint16_t element_address, dsm_local_unicast_address_t node_address) {

    if (element_address < node_address.address_start)
    {
        return ACCESS_ELEMENT_INDEX_INVALID;
    }

    uint16_t retval = element_address - node_address.address_start;
    if (retval >= (uint16_t) ACCESS_ELEMENT_COUNT)
    {
        return ACCESS_ELEMENT_INDEX_INVALID;
    }
    else
    {
        return retval;
    }
}

static uint32_t config_model_publication_set(dsm_local_unicast_address_t node_address, access_model_id_t model_id, bool sig_model, 
                                        uint16_t publish_address, 
                                        uint8_t pubstate_publish_ttl,
                                        uint8_t pubstate_publish_period,
                                        uint8_t pubstate_retransmit_count,
                                        uint8_t pubstate_retransmit_interval) {

    uint16_t element_address = node_address.address_start; 

    uint16_t pubstate_appkey_index = 0x0000;

    uint16_t element_index = get_element_index(element_address, node_address);

    if (element_index == ACCESS_ELEMENT_INDEX_INVALID) {
        return ACCESS_STATUS_INVALID_ADDRESS;
    }

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE)) {
        return ACCESS_STATUS_INVALID_MODEL;
    }

    /* Get the application key handle for the application key to publish on: */
    dsm_handle_t publish_appkey_handle = dsm_appkey_index_to_appkey_handle(pubstate_appkey_index);
    if (publish_appkey_handle == DSM_HANDLE_INVALID) {
        return ACCESS_STATUS_INVALID_APPKEY;
    }

    /* Validate and add the publish address to the DSM: */
    dsm_handle_t publish_address_handle = DSM_HANDLE_INVALID;
    nrf_mesh_address_t publish_address_stored;
    nrf_mesh_address_type_t publish_addr_type = NRF_MESH_ADDRESS_TYPE_GROUP;

            /* Check if given publish address is different than the currently assigned address */
            if (access_model_publish_address_get(model_handle, &publish_address_handle) != NRF_SUCCESS)
            {
                status = dsm_address_publish_add(publish_address, &publish_address_handle);
            }
            else
            {
                if (dsm_address_get(publish_address_handle, &publish_address_stored) == NRF_SUCCESS)
                {

                    if ((publish_address_stored.type == NRF_MESH_ADDRESS_TYPE_VIRTUAL) ||
                        (publish_address_stored.type != NRF_MESH_ADDRESS_TYPE_VIRTUAL  &&
                         publish_address_stored.value != publish_address))
                    {
                        /* This should never assert */
                        NRF_MESH_ASSERT(dsm_address_publish_remove(publish_address_handle) == NRF_SUCCESS);
                        status = dsm_address_publish_add(publish_address, &publish_address_handle);
                    }
                    else
                    {
                        /* Use the retrieved publish_address_handle */
                    }
                }
                else
                {
                    status = dsm_address_publish_add(publish_address, &publish_address_handle);
                }
            }


    switch (status) {
        case NRF_ERROR_NO_MEM:
            return ACCESS_STATUS_INSUFFICIENT_RESOURCES;
        case NRF_SUCCESS:
            break;
        default:
            return ACCESS_STATUS_UNSPECIFIED_ERROR;
    }

    /* If publish address is unassigned for non virtual set, ignore all incoming parameters */
    if (publish_address != NRF_MESH_ADDR_UNASSIGNED)
    {
        access_publish_period_t publish_period;
        access_publish_retransmit_t publish_retransmit;
        publish_period.step_res = pubstate_publish_period >> ACCESS_PUBLISH_STEP_NUM_BITS;
        publish_period.step_num = pubstate_publish_period & ~(0xff << ACCESS_PUBLISH_STEP_NUM_BITS);
        publish_retransmit.count = pubstate_retransmit_count;
        publish_retransmit.interval_steps = pubstate_retransmit_interval;


        if (publish_period.step_num != 0)
        {
            /* Disable publishing for the model while updating the publication parameters: */
            status = access_model_publish_period_set(model_handle, ACCESS_PUBLISH_RESOLUTION_100MS, 0);
            switch (status) {
                case NRF_SUCCESS:
                    break;
                case NRF_ERROR_NOT_SUPPORTED:
                    return ACCESS_STATUS_FEATURE_NOT_SUPPORTED;

                default:
                    /* No other error should be possible. */
                    NRF_MESH_ASSERT(false);
                    return NRF_ERROR_NOT_SUPPORTED;
            }

            /* Set publishing parameters for the model: */
            NRF_MESH_ASSERT(access_model_publish_period_set(model_handle, (access_publish_resolution_t) publish_period.step_res,
                            publish_period.step_num) == NRF_SUCCESS);
        }

        NRF_MESH_ASSERT(access_model_publish_retransmit_set(model_handle, publish_retransmit) == NRF_SUCCESS);
        NRF_MESH_ASSERT(access_model_publish_address_set(model_handle, publish_address_handle) == NRF_SUCCESS);
        NRF_MESH_ASSERT(access_model_publish_application_set(model_handle, publish_appkey_handle) == NRF_SUCCESS);
        NRF_MESH_ASSERT(access_model_publish_ttl_set(model_handle, pubstate_publish_ttl) == NRF_SUCCESS);
    }
    else
    {
        NRF_MESH_ASSERT(access_model_publication_stop(model_handle) == NRF_SUCCESS);
    }

    //TODO access_load_config_apply or initialization_data_store
   // access_flash_config_store();
    return NRF_SUCCESS;
}

uint32_t app_config_health_model_publication(dsm_local_unicast_address_t node_address) {

    access_model_id_t model_id;
    model_id.model_id = HEALTH_SERVER_MODEL_ID;
    model_id.company_id = ACCESS_COMPANY_ID_NONE;
    bool sig_model = true;

    uint16_t publish_address = GATEWAY_PUBLISH_ARRD;

    uint8_t pubstate_publish_ttl = 0xff;
    uint8_t pubstate_publish_period = 0x81;
    uint8_t pubstate_retransmit_count = 0x00; 
    uint8_t pubstate_retransmit_interval = 0x04;

    uint32_t status = config_model_publication_set(node_address, model_id, sig_model, 
                                        publish_address, 
                                        pubstate_publish_ttl,
                                        pubstate_publish_period,
                                        pubstate_retransmit_count,
                                        pubstate_retransmit_interval);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "config_health_model_publication: %x\n",status);	

    return status;
}
