#include "protocol.h"
#include "master_transport.h"
#include "termal_shock.h"
#include "esp_log.h"

static const char *TAG = "MASTER_CMD";

void master_handle_command(const payload_command_t *cmd)
{
    switch (cmd->cmd_id) {

	case CMD_START_TEST:
	    ESP_LOGI(TAG, "CMD_START_TEST");
	
	    /* Start MASTER logic */
	    //thermal_shock_init(3);
	    
	    
	    //ts_data.max_cycles  = 500;
	    
	    
	    ts_data.max_cycles  = 250;
	    

	    thermal_shock_start();
	

	    break;


	case CMD_STOP_TEST:
	    ESP_LOGI(TAG, "CMD_STOP_TEST");
	
	    thermal_shock_reset();
	
	    actuator_send_command(
	        CMD_FORCE_RELAY,
	        0x00,   // all relays off
	        0
	    );
	    break;


    case CMD_REQUEST_STATE:
        ESP_LOGI(TAG, "CMD_REQUEST_STATE");
        /* optional: send status packet */
        break;

    default:
        ESP_LOGW(TAG, "Unknown CMD: %u", cmd->cmd_id);
        break;
    }
}
