
#ifndef APP_MAIN_CONFIG_H__
#define APP_MAIN_CONFIG_H__

#include <stdbool.h>
#include <stdint.h>
#include <string.h>


/*****************************************************************************
 * Definitions
 *****************************************************************************/

#define APP_ONOFF_ELEMENT_INDEX     (0)
#define APP_LEVEL_ELEMENT_INDEX     (0)

#define APP_LEVEL_STEP_SIZE     (1024L)

/* Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_FORCE_SEGMENTATION  (false)
/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE            (NRF_MESH_TRANSMIC_SIZE_SMALL)

#define LED_LEVEL_DEFAULT INT16_MAX
#define LED_MIN_LEVEL_DEFAULT 0
#define LED_MAX_LEVEL_DEFAULT UINT16_MAX



/******** Storage definitions *********/
#define LED_CONFIG_RESERVE_LEN MESH_CONFIG_ENTRY_MAX_SIZE-16



/******** End storage definitions *********/



/**************************** Structs ****************************************/
typedef struct {
    bool        on;
    uint16_t    level;
    uint16_t    min_level;
    uint16_t    max_level;
} led_config_t;
/*****************************************************************************/

#endif /* APP_MAIN_CONFIG_H__ */
