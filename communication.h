typedef enum {
	OP_PROTOCOL_START,
	OP_SAMPLING_START,
	OP_SAMPLING_PAUSE,
	OP_CONFIG_WRITE,
	OP_CONFIG_READ,
	OP_RESTART,
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

void network_task(void *pvParameters);
