
#ifndef APP_CONFIG_MODELS_H
#define APP_CONFIG_MODELS_H

#include "device_state_manager.h"

uint32_t app_config_health_model_publication(dsm_local_unicast_address_t node_address);                                       
uint32_t app_config_led_status_model_publication(dsm_local_unicast_address_t node_address);
uint32_t app_config_led_level_model_publication(dsm_local_unicast_address_t node_address);
uint32_t app_config_subscription_to_communication_group(dsm_local_unicast_address_t node_address);
uint32_t app_config_communication_client_model_publication(dsm_local_unicast_address_t node_address);
uint32_t app_config_bind_app_keys(dsm_local_unicast_address_t node_address);                                        

#endif //APP_CONFIG_MODELS_H

