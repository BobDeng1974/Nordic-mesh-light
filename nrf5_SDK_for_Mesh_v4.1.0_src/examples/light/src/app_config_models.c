#include "app_config_models.h"

// #include <stdint.h>
// #include <string.h>

#include "mesh_stack.h"
#include "access_config.h"
#include "generic_onoff_server.h"
#include "generic_level_server.h"
#include "base_communication_common.h"
#include "base_communication_client.h"
#include "base_communication_server.h"
#include "log.h"

#define GATEWAY_PUBLISH_ARRD 0x7ff0
#define COMMUNICATION_COMMON_GROUP_ARRD 0xFF80

uint16_t get_element_index(uint16_t element_address, dsm_local_unicast_address_t node_address) {

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

uint32_t config_model_publication_set(dsm_local_unicast_address_t node_address, access_model_id_t model_id, bool sig_model, 
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

uint32_t app_config_led_status_model_publication(dsm_local_unicast_address_t node_address) {

    access_model_id_t model_id = ACCESS_MODEL_SIG(GENERIC_ONOFF_SERVER_MODEL_ID);
    bool sig_model = true;

    uint16_t publish_address = GATEWAY_PUBLISH_ARRD;

    uint8_t pubstate_publish_ttl = 0xff;
    uint8_t pubstate_publish_period = 0x00;
    uint8_t pubstate_retransmit_count = 0x00;
    uint8_t pubstate_retransmit_interval = 0x00;

    return config_model_publication_set(node_address, model_id, sig_model, 
                                        publish_address, 
                                        pubstate_publish_ttl,
                                        pubstate_publish_period,
                                        pubstate_retransmit_count,
                                        pubstate_retransmit_interval);

}

uint32_t app_config_led_level_model_publication(dsm_local_unicast_address_t node_address) {

    access_model_id_t model_id = ACCESS_MODEL_SIG(GENERIC_LEVEL_SERVER_MODEL_ID);
    bool sig_model = true;

    uint16_t publish_address = GATEWAY_PUBLISH_ARRD;

    uint8_t pubstate_publish_ttl = 0xff;
    uint8_t pubstate_publish_period = 0x00;
    uint8_t pubstate_retransmit_count = 0x00;
    uint8_t pubstate_retransmit_interval = 0x00;

    return config_model_publication_set(node_address, model_id, sig_model, 
                                        publish_address, 
                                        pubstate_publish_ttl,
                                        pubstate_publish_period,
                                        pubstate_retransmit_count,
                                        pubstate_retransmit_interval);

}


uint32_t app_config_communication_client_model_publication(dsm_local_unicast_address_t node_address) {

    access_model_id_t model_id;
    model_id.model_id = BASE_COMMUNICATION_CLIENT_MODEL_ID;
    model_id.company_id = BASE_COMMUNICATION_COMPANY_ID;
    bool sig_model = false;

    uint16_t publish_address = COMMUNICATION_COMMON_GROUP_ARRD;

    uint8_t pubstate_publish_ttl = 0xff;
    uint8_t pubstate_publish_period = 0x00;
    uint8_t pubstate_retransmit_count = 0x00; // ??? 0x01
    uint8_t pubstate_retransmit_interval = 0x04; // ??? 0x00

    return config_model_publication_set(node_address, model_id, sig_model, 
                                        publish_address, 
                                        pubstate_publish_ttl,
                                        pubstate_publish_period,
                                        pubstate_retransmit_count,
                                        pubstate_retransmit_interval);

}

uint32_t app_config_subscription_to_communication_group(dsm_local_unicast_address_t node_address) {

    access_model_id_t model_id;
    model_id.model_id = BASE_COMMUNICATION_SERVER_MODEL_ID;
    model_id.company_id = BASE_COMMUNICATION_COMPANY_ID;
    bool sig_model = false;

    uint16_t element_index = get_element_index(node_address.address_start, node_address);
    if (ACCESS_ELEMENT_INDEX_INVALID == element_index) {
        return ACCESS_STATUS_INVALID_ADDRESS;
    }

    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE)) {
        return ACCESS_STATUS_INVALID_MODEL;
    }

    dsm_handle_t subscription_address_handle;
    status = dsm_address_subscription_add(COMMUNICATION_COMMON_GROUP_ARRD, &subscription_address_handle);
    if (status != NRF_SUCCESS) {
        switch (status) {
            case NRF_ERROR_INVALID_ADDR:
                return ACCESS_STATUS_INVALID_ADDRESS;
                break;
            case NRF_ERROR_NO_MEM:
                return ACCESS_STATUS_INSUFFICIENT_RESOURCES;
                break;
            default:
                return ACCESS_STATUS_UNSPECIFIED_ERROR;
                break;
        }
    }

    status = access_model_subscription_add(model_handle, subscription_address_handle);
    if (status != NRF_SUCCESS) {
        NRF_MESH_ASSERT(dsm_address_subscription_remove(subscription_address_handle) == NRF_SUCCESS);

        if (status == NRF_ERROR_NOT_SUPPORTED) {
            return ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL;
        } else {
            return ACCESS_STATUS_UNSPECIFIED_ERROR;
        }
    } else {
        //access_flash_config_store();
        return NRF_SUCCESS;
    }
}


uint32_t app_config_bind_app_key(dsm_local_unicast_address_t node_address, access_model_id_t model_id, bool sig_model) {

    uint16_t element_index = get_element_index(node_address.address_start, node_address);
    if (ACCESS_ELEMENT_INDEX_INVALID == element_index) {
        return ACCESS_STATUS_INVALID_ADDRESS;
    }
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != NRF_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE)) {
        return ACCESS_STATUS_INVALID_MODEL;
    }
    uint16_t appkey_index = 0x0000 & CONFIG_MSG_KEY_INDEX_12_MASK;
    dsm_handle_t appkey_handle = dsm_appkey_index_to_appkey_handle(appkey_index);

    status = access_model_application_bind(model_handle, appkey_handle);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "access_model_application_bind %x : %x\n",model_id.model_id, status);

    switch (status) {
        case NRF_SUCCESS:
            return ACCESS_STATUS_SUCCESS;
            //TODO
            //access_flash_config_store();
            break;
        case NRF_ERROR_NOT_FOUND:
            return ACCESS_STATUS_INVALID_MODEL;
            break;
        case NRF_ERROR_INVALID_PARAM:
            return ACCESS_STATUS_INVALID_APPKEY;
            break;
        default:
            return ACCESS_STATUS_UNSPECIFIED_ERROR;
            break;
    }
}

uint32_t app_config_bind_app_keys(dsm_local_unicast_address_t node_address) {

    access_model_id_t model_id;

    //model_id = ACCESS_MODEL_SIG(HEALTH_SERVER_MODEL_ID);
    model_id.model_id = HEALTH_SERVER_MODEL_ID;
    model_id.company_id = ACCESS_COMPANY_ID_NONE;
    uint32_t status = app_config_bind_app_key(node_address, model_id, true);
    if (status != NRF_SUCCESS) {
        return status;
    }
    
    //model_id = ACCESS_MODEL_SIG(GENERIC_ONOFF_SERVER_MODEL_ID);
    model_id.model_id = GENERIC_ONOFF_SERVER_MODEL_ID;
    model_id.company_id = ACCESS_COMPANY_ID_NONE;
    status = app_config_bind_app_key(node_address, model_id, true);
    if (status != NRF_SUCCESS) {
        return status;
    }

    //model_id = ACCESS_MODEL_SIG(GENERIC_LEVEL_SERVER_MODEL_ID);
    model_id.model_id = GENERIC_LEVEL_SERVER_MODEL_ID;
    model_id.company_id = ACCESS_COMPANY_ID_NONE;
    status = app_config_bind_app_key(node_address, model_id, true);
    if (status != NRF_SUCCESS) {
        return status;
    }

    
    model_id.model_id = BASE_COMMUNICATION_SERVER_MODEL_ID;
    model_id.company_id = BASE_COMMUNICATION_COMPANY_ID;
    status = app_config_bind_app_key(node_address, model_id, false);
    if (status != NRF_SUCCESS) {
        return status;
    }

    model_id.model_id = BASE_COMMUNICATION_CLIENT_MODEL_ID;
    model_id.company_id = BASE_COMMUNICATION_COMPANY_ID;
    status = app_config_bind_app_key(node_address, model_id, false);
    if (status != NRF_SUCCESS) {
        return status;
    }
    

    return NRF_SUCCESS;
}