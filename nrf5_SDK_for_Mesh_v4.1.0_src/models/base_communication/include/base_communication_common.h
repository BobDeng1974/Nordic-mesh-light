
#ifndef BASE_COMMUNICATION_COMMON_H__
#define BASE_COMMUNICATION_COMMON_H__

#include <stdint.h>
#include "access.h"

/*lint -align_max(push) -align_max(1) */

/** Vendor specific company ID for communication commands model */
#define BASE_COMMUNICATION_COMPANY_ID    (ACCESS_COMPANY_M_TECHNOLOGIES)

/** communication commands opcodes. */
typedef enum {
    BASE_COMMUNICATION_OPCODE_COMMAND   = 0xF0,
    BASE_COMMUNICATION_OPCODE_RESPONSE  = 0xF1,
    BASE_COMMUNICATION_OPCODE_STATUS    = 0xF2
} base_communication_opcode_t;

/** communication sub commands opcodes. */
typedef enum {
    BASE_COMMUNICATION_SUB_COMMAND_COMISSIONING_FINISHED = 0x01,
    BASE_COMMUNICATION_SUB_COMMAND_IP_ADDR_GET = 0x02,
    BASE_COMMUNICATION_SUB_COMMAND_FW_VERSION_GET = 0x03,

    BASE_COMMUNICATION_SUB_COMMAND_MAX_LEVEL_1_SET = 0x10,
    BASE_COMMUNICATION_SUB_COMMAND_MAX_LEVEL_1_GET = 0x11,
    BASE_COMMUNICATION_SUB_COMMAND_MIN_LEVEL_1_SET = 0x12,
    BASE_COMMUNICATION_SUB_COMMAND_MIN_LEVEL_1_GET = 0x13
    
} base_communication_sub_command_t;

/** communication commands status codes. */
typedef enum
{
    BASE_COMMUNICATION_STATUS_SUCCESS,
    BASE_COMMUNICATION_STATUS_MESSAGE,
    BASE_COMMUNICATION_STATUS_CANCELLED,
    BASE_COMMUNICATION_STATUS_FAIL,
    BASE_COMMUNICATION_STATUS_ERROR,
    BASE_COMMUNICATION_STATUS_ERROR_NO_HANDLER,
    BASE_COMMUNICATION_STATUS_ERROR_NO_REPLY,
    BASE_COMMUNICATION_STATUS_NO_REPLY = 0xFF
} base_communication_status_t;
    

/** Message format for the communication commands message. */
typedef struct __attribute((packed))
{
    uint8_t             opcode;       		/**< command code */
    uint8_t             tid;              	/**< Transaction number */
    const uint8_t * 	command;   			/**< command body */
    uint16_t            length;           	/** Length of command_buffer */
} base_communication_msg_t;


typedef struct __attribute((packed))
{
    uint8_t             opcode;       		/**< command code */
    uint8_t             tid;              	/**< Transaction number */
    const uint8_t * 	command;   			/**< command body */
} base_communication_command_msg_t;

/** Message format for the communication command Status message. */
typedef struct __attribute((packed))
{
    uint8_t status; 	/**< Current status. */
} base_communication_msg_status_t;

/** Message format for the communication command Status message. */
typedef struct __attribute((packed))
{
    const uint8_t * 	response;
    uint8_t            	length;
} base_communication_msg_response_t;


/*lint -align_max(pop) */

/** @} end of BASE_COMMUNICATION_COMMON */
/** @} end of BASE_COMMUNICATION_MODEL */
#endif /* BASE_COMMUNICATION_COMMON_H__ */
