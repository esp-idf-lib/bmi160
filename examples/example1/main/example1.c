#include <esp_log.h>
#include <template-component.h>

#define TAG "example1"

void app_main(void)
{
    uint8_t foo = 1;
    uint8_t bar = 2;
    esp_err_t err = func1(foo, bar);

    ESP_LOGI(TAG, "result: %d", err);
}
