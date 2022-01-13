#define PARAM_STR_SIZE 65
#define PARAM_MAX_QTY 5

typedef enum {
	COMM_OK,
	COMM_ERR_SENDING_COMMAND,
	COMM_ERR_RECEVING_RESPONSE,
	COMM_ERR_INVALID_MAC,
	COMM_ERR_PARSING_COMMAND,
	COMM_ERR_WRONG_RNDN,
	COMM_ERR_WRONG_COUNTER
} comm_status_t;

typedef enum {
	OP_PROTOCOL_START,
	OP_SAMPLING_START,
	OP_SAMPLING_PAUSE,
	OP_CONFIG_WRITE,
	OP_CONFIG_READ,
	OP_RESTART,
	OP_SET_RTC,
	OP_FW_UPDATE,
	OP_QUERY_STATUS,
	OP_GET_DATA,
	OP_DELETE_DATA,
	OP_GET_WAVEFORM,
	OP_DISCONNECT,
	OPCODE_NUM
} protocol_opcodes_t;

typedef struct opcode_metadata_s {
	char opcode_text[3];
	int parameter_qty;
} opcode_metadata_t;

typedef enum {
	R_SUCESS,
	R_ERR_UNSPECIFIED,
	R_ERR_SAMPLING_RUNNING,
	R_ERR_INVALID_PARAMETER
} protocol_errors_t;

void communication_task(void *pvParameters);
