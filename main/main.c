#include "esp_log.h"
#include "app_master.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "System boot");

    app_master_init();
    app_master_start();
}
