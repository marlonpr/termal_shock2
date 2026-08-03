#include "protocol.h"
#include "master_transport.h"
#include "termal_shock.h"
#include "esp_log.h"

#include "driver/uart.h"
#include "board_pins.h"

static const char *TAG = "MASTER_CMD";


/* Default mode after startup */
static test_mode_t current_test_mode = TEST_MODE_1;

static uint16_t get_mode_max_cycles(test_mode_t mode)
{
    switch (mode) {

    case TEST_MODE_1:
        return MODE_1_MAX_CYCLES;

    case TEST_MODE_2:
    default:
        return MODE_2_MAX_CYCLES;
    }
}

static void send_test_mode_state(void)
{
    char tx_buf[64];

    /*
     * Make sure ts_data and current_test_mode agree.
     */
    ts_data.max_cycles =
        get_mode_max_cycles(current_test_mode);

    int n = snprintf(
        tx_buf,
        sizeof(tx_buf),
        "TEST_MODE=%u,MAX_CYCLES=%lu\n",
        (unsigned)current_test_mode,
        (unsigned long)ts_data.max_cycles
    );

    if (n > 0) {
        uart_write_bytes(UART_UI, tx_buf, n);
    }

    ESP_LOGI(
        TAG,
        "Sent UI state: mode=%u max_cycles=%lu",
        (unsigned)current_test_mode,
        (unsigned long)ts_data.max_cycles
    );
}

static void switch_test_mode(void)
{
    if (current_test_mode == TEST_MODE_1) {
        current_test_mode = TEST_MODE_2;
    } else {
        current_test_mode = TEST_MODE_1;
    }

    ts_data.max_cycles = get_mode_max_cycles(current_test_mode);

    ESP_LOGI(TAG,
             "Test mode changed: MODE_%u, max_cycles=%u",
             (unsigned)current_test_mode,
             (unsigned)ts_data.max_cycles);
}



void master_handle_command(const payload_command_t *cmd)
{
    switch (cmd->cmd_id) {

    case CMD_START_TEST:
        ESP_LOGI(TAG, "CMD_START_TEST");

        /*
         * Use the selected mode instead of a hardcoded value.
         */
        ts_data.max_cycles = get_mode_max_cycles(current_test_mode);

        ESP_LOGI(TAG,
                 "Starting test: MODE_%u, max_cycles=%u",
                 (unsigned)current_test_mode,
                 (unsigned)ts_data.max_cycles);

        thermal_shock_start();
        break;

		case CMD_SWITCH_MODE:
		    ESP_LOGI(TAG, "CMD_SWITCH_MODE");

		    if (ts_data.state == TS_RUNNING) {
		        ESP_LOGW(
		            TAG,
		            "Cannot change mode while test is running"
		        );

		        /*
		         * Return current mode so the UI remains synchronized.
		         */
		        send_test_mode_state();
		        break;
		    }

		    switch_test_mode();

		    /*
		     * Send resulting mode to the UI.
		     */
		    send_test_mode_state();
		    break;

    case CMD_STOP_TEST:
        ESP_LOGI(TAG, "CMD_STOP_TEST");

        thermal_shock_reset();

        actuator_send_command(
            CMD_FORCE_RELAY,
            0x00,   // All relays off
            0
        );
        break;

		case CMD_REQUEST_STATE:
		    ESP_LOGI(TAG, "CMD_REQUEST_STATE");

		    send_test_mode_state();
		    break;

    default:
        ESP_LOGW(TAG, "Unknown CMD: %u", cmd->cmd_id);
        break;
    }
}





