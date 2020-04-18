/* Copyright (c) 2010 - 2020, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "app_timer.h"

#include "simple_hal.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "proxy.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "generic_onoff_server.h"
#include "generic_level_server.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "nrf_mesh_config_app.h"
#include "nrf.h"
#include "example_common.h"
#include "pwm_utils.h"
#include "nrf_mesh_config_examples.h"
#include "app_onoff.h"
#include "app_level.h"
#include "ble_softdevice_support.h"
#include "config_server.h"
/*****************************************************************************
 * Definitions
 *****************************************************************************/
#define ONOFF_SERVER_0_LED          (BSP_LED_0)
#define APP_ONOFF_ELEMENT_INDEX     (0)
#define APP_LEVEL_ELEMENT_INDEX     (0)

#define APP_LEVEL_STEP_SIZE     (1024L)

/* Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_FORCE_SEGMENTATION  (false)
/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE            (NRF_MESH_TRANSMIC_SIZE_SMALL)


/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static void app_onoff_server_set_cb(const app_onoff_server_t * p_server, bool onoff);
static void app_onoff_server_get_cb(const app_onoff_server_t * p_server, bool * p_present_onoff);
static void app_onoff_server_transition_cb(const app_onoff_server_t * p_server,
                                                uint32_t transition_time_ms, bool target_onoff);


static void app_level_server_set_cb(const app_level_server_t * p_server, uint16_t present_level);
static void app_level_server_get_cb(const app_level_server_t * p_server, uint16_t * p_present_level);
static void app_level_server_transition_cb(const app_level_server_t * p_server,
                                                uint32_t transition_time_ms, uint16_t target_level,
                                                app_transition_type_t transition_type);


/*****************************************************************************
 * Static variables
 *****************************************************************************/
static bool m_device_provisioned;

dsm_local_unicast_address_t node_address;

/* PWM hardware instance and associated variables */
/* Note: PWM cycle period determines the the max value that can be used to represent 100%
 * duty cycles, therefore present_level value scaling is required to get pwm tick value
 * between 0 and pwm_utils_contex_t:pwm_ticks_max.
 */
static APP_PWM_INSTANCE(PWM0, 1);
static app_pwm_config_t m_pwm0_config = APP_PWM_DEFAULT_CONFIG_1CH(200, BSP_LED_0);
static pwm_utils_contex_t m_pwm = {
                                    .p_pwm = &PWM0,
                                    .p_pwm_config = &m_pwm0_config,
                                    .channel = 0
                                  };


/************************** LED Control *************************************/

static uint16_t led1_level;
static bool led1_state_on;

void led1_level_set(uint16_t level) {

    led1_level = level;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SET: Level: %d\n", led1_level);

    if (led1_state_on) {
        pwm_utils_level_set(&m_pwm, led1_level);

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Get: Level: %d\n", pwm_utils_level_get(&m_pwm));
    }
}

void led1_state_on_set(bool is_on) {

    led1_state_on = is_on;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SET: State: %d\n", led1_state_on);

    if (led1_state_on) {
        pwm_utils_enable(&m_pwm);
        led1_level_set(led1_level);
    } else {
        pwm_utils_disable(&m_pwm);
    }

}

/* Generic OnOff server structure definition and initialization */
APP_ONOFF_SERVER_DEF(m_onoff_server_0,
                     APP_FORCE_SEGMENTATION,
                     APP_MIC_SIZE,
                     app_onoff_server_set_cb,
                     app_onoff_server_get_cb,
                     app_onoff_server_transition_cb)

/* Callback for updating the hardware state */
static void app_onoff_server_set_cb(const app_onoff_server_t * p_server, bool onoff)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */

   // hal_led_pin_set(ONOFF_SERVER_0_LED, onoff);
    led1_state_on_set(onoff);
}

/* Callback for reading the hardware state */
static void app_onoff_server_get_cb(const app_onoff_server_t * p_server, bool * p_present_onoff)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */

    //*p_present_onoff = hal_led_pin_get(ONOFF_SERVER_0_LED);

    *p_present_onoff = led1_state_on;
}

/* Callback for updating the hardware state */
static void app_onoff_server_transition_cb(const app_onoff_server_t * p_server,
                                                uint32_t transition_time_ms, bool target_onoff)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transition time: %d, Target OnOff: %d\n",
                                       transition_time_ms, target_onoff);
}

/* Application level generic level server structure definition and initialization */
APP_LEVEL_SERVER_DEF(m_level_server_0,
                     APP_FORCE_SEGMENTATION,
                     APP_MIC_SIZE,
                     NULL,
                     app_level_server_set_cb,
                     app_level_server_get_cb,
                     app_level_server_transition_cb);


/* Application variable for holding instantaneous level value */
//static int32_t m_pwm0_present_level;

/* Callback for updating the hardware state */
static void app_level_server_set_cb(const app_level_server_t * p_server, uint16_t present_level)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */

//    uint16_t level = present_level;
//    if (present_level < 0) {
//        level += UINT16_MAX;
//    }

    led1_level_set(present_level);
//    m_pwm0_present_level = present_level;
//    pwm_utils_level_set(&m_pwm, m_pwm0_present_level);
}

/* Callback for reading the hardware state */
static void app_level_server_get_cb(const app_level_server_t * p_server, uint16_t * p_present_level)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */
//    int16_t level = led1_level;
//    if (led1_level < 0) {
//        level -= UINT16_MAX;
//    }

    *p_present_level = led1_level;
}

/* Callback for updateing according to transition time. */
static void app_level_server_transition_cb(const app_level_server_t * p_server,
                                                uint32_t transition_time_ms, uint16_t target_level,
                                                app_transition_type_t transition_type)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transition time: %d, Target level: %d, Transition type: %d\n",
                                       transition_time_ms, target_level, transition_type);
}

static void app_model_init(void)
{
    /* Instantiate onoff server on element index APP_ONOFF_ELEMENT_INDEX */
    ERROR_CHECK(app_onoff_init(&m_onoff_server_0, APP_ONOFF_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App OnOff Model Handle: %d\n", m_onoff_server_0.server.model_handle);

    /* Instantiate level server on element index 0 */
    ERROR_CHECK(app_level_init(&m_level_server_0, APP_LEVEL_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App Level Model Handle: %d\n", m_level_server_0.server.model_handle);
}

/*************************************************************************************************/

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    /* This function may return if there are ongoing flash operations. */
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

#if NRF_MESH_LOG_ENABLE
static const char m_usage_string[] =
    "\n"
    "\t\t-------------------------------------------------------------\n"
    "\t\t RTT 1) The brightness of the LED 1 decresses in large steps.\n"
    "\t\t RTT 2) The brightness of the LED 1 incresses in large steps.\n"
    "\t\t RTT 4) Clear all the states to reset the node.\n"
    "\t\t-------------------------------------------------------------\n";
#endif

static void button_event_handler(uint32_t button_number)
{
    /* Increase button number because the buttons on the board is marked with 1 to 4 */
    button_number++;
    uint16_t new_level = 0;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);
    switch (button_number)
    {
        /* Sending value `1` or `2` via RTT will result in LED state to change and trigger the
        STATUS message to inform client about the state change. This is a demonstration of state
        change publication due to local event. */
        case 1:
        {
//            if (led1_level < APP_LEVEL_STEP_SIZE) {
//                new_level = 0;
//            } else {
//                new_level = led1_level - APP_LEVEL_STEP_SIZE;
//            }
            new_level = (led1_level < APP_LEVEL_STEP_SIZE) ? 0 : led1_level - APP_LEVEL_STEP_SIZE;
//            new_level = (led1_level - APP_LEVEL_STEP_SIZE) <= INT16_MIN ? INT16_MIN : m_pwm0_present_level - APP_LEVEL_STEP_SIZE;
            break;
        }

        case 2:
        {

            new_level = ((UINT16_MAX - led1_level) < APP_LEVEL_STEP_SIZE) ? UINT16_MAX : led1_level + APP_LEVEL_STEP_SIZE;
            //new_level = (led1_level + APP_LEVEL_STEP_SIZE) >= INT16_MAX ? INT16_MAX : m_pwm0_present_level + APP_LEVEL_STEP_SIZE;
            break;
        }

        /* Pressing SW1 on the Development Kit will result in LED state to toggle and trigger
        the STATUS message to inform client about the state change. This is a demonstration of
        state change publication due to local event. */
        case 3:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "User action \n");
            //hal_led_pin_set(ONOFF_SERVER_0_LED, !hal_led_pin_get(ONOFF_SERVER_0_LED));
            //app_onoff_status_publish(&m_onoff_server_0);

            led1_state_on_set(!led1_state_on);

            break;
        }

        /* Initiate node reset */
        case 4:
        {
            /* Clear all the states to reset the node. */
            if (mesh_stack_is_device_provisioned())
            {
#if MESH_FEATURE_GATT_PROXY_ENABLED
                (void) proxy_stop();
#endif
                mesh_stack_config_clear();
                node_reset();
            }
            else
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "The device is unprovisioned. Resetting has no effect.\n");
            }
            break;
        }

        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
            break;
    }

    if (button_number == 1 || button_number == 2)
    {
    //    pwm_utils_level_set(&m_pwm, new_level);
        
        led1_level_set(new_level);

//        uint32_t status = app_level_current_value_publish(&m_level_server_0);
//        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "level: %d\n", m_pwm0_present_level);
//        if ( status != NRF_SUCCESS)
//        {
//            __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Unable to publish status message, status: %d\n", status);
//        }
    }
}

static void app_rtt_input_handler(int key)
{
    if (key >= '1' && key <= '4')
    {
        uint32_t button_number = key - '1';
        button_event_handler(button_number);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
    }
}

static void device_identification_start_cb(uint8_t attention_duration_s)
{

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "device_identification_start_cb\n");

    hal_led_mask_set(LEDS_MASK, false);
    hal_led_blink_ms(BSP_LED_2_MASK  | BSP_LED_3_MASK,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_s));
}

static void device_identification_stop_cb()
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "device_identification_stop_cb\n");
}

static void provisioning_aborted_cb(void)
{

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "provisioning_aborted_cb\n");

    hal_led_blink_stop();
}

static void unicast_address_print(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

#if MESH_FEATURE_GATT_ENABLED
    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();
#endif

    dsm_local_unicast_addresses_get(&node_address);

    unicast_address_print();

    hal_led_blink_stop();
    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    app_model_init();
}

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };

    uint32_t status = mesh_stack_init(&init_params, &m_device_provisioned);
    switch (status)
    {
        case NRF_ERROR_INVALID_DATA:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Data in the persistent memory was corrupted. Device starts as unprovisioned.\n");
			__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Reset device before start provisioning.\n");
            break;
        case NRF_SUCCESS:
            break;
        default:
            ERROR_CHECK(status);
    }
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light -----\n");

    pwm_utils_init(&m_pwm);

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

    ble_stack_init();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();
}

static uint16_t get_element_index(uint16_t element_address)
{
//    dsm_local_unicast_address_t node_address;
//    dsm_local_unicast_addresses_get(&node_address);

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

uint32_t config_model_publication_set(access_model_id_t model_id, bool sig_model, 
                                        uint16_t publish_address, 
                                        uint8_t pubstate_publish_ttl,
                                        uint8_t pubstate_publish_period,
                                        uint8_t pubstate_retransmit_count,
                                        uint8_t pubstate_retransmit_interval) {

    uint16_t element_address = node_address.address_start; 

    uint16_t pubstate_appkey_index = 0x0000;

    uint16_t element_index = get_element_index(element_address);

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
            NRF_MESH_ASSERT(access_model_publish_retransmit_set(model_handle, publish_retransmit) == NRF_SUCCESS);
        }

        NRF_MESH_ASSERT(access_model_publish_address_set(model_handle, publish_address_handle) == NRF_SUCCESS);
        NRF_MESH_ASSERT(access_model_publish_application_set(model_handle, publish_appkey_handle) == NRF_SUCCESS);
        NRF_MESH_ASSERT(access_model_publish_ttl_set(model_handle, pubstate_publish_ttl) == NRF_SUCCESS);
    }
    else
    {
        NRF_MESH_ASSERT(access_model_publication_stop(model_handle) == NRF_SUCCESS);
    }

    //TODO
   // access_flash_config_store();
    return NRF_SUCCESS;
}

static void start(void)
{
    rtt_input_enable(app_rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_sd_ble_opt_set_cb = NULL,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = device_identification_stop_cb,
            .prov_abort_cb = provisioning_aborted_cb,
            .p_device_uri = EX_URI_DM_SERVER
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {

        dsm_local_unicast_addresses_get(&node_address);

        unicast_address_print();

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);

        access_model_id_t model_id;
        model_id.model_id = HEALTH_SERVER_MODEL_ID;
        model_id.company_id = ACCESS_COMPANY_ID_NONE;
        bool sig_model = true;

        uint16_t publish_address = 0x7ff0;

        uint8_t pubstate_publish_ttl = 0xff;
        uint8_t pubstate_publish_period = 0x81;
        uint8_t pubstate_retransmit_count = 0x00; 
        uint8_t pubstate_retransmit_interval = 0x04;

        uint32_t status = config_model_publication_set(model_id, sig_model, 
                                        publish_address, 
                                        pubstate_publish_ttl,
                                        pubstate_publish_period,
                                        pubstate_retransmit_count,
                                        pubstate_retransmit_interval);

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "config_health_model_publication: %x\n",status);
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

int main(void)
{
    initialize();
    start();

    led1_level = APP_LEVEL_STEP_SIZE*10;

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
