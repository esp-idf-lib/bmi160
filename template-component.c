#include "template-component.h"

esp_err_t func1(uint8_t arg1, uint8_t arg2)
{
    esp_err_t err = ESP_FAIL;
    if (arg1 != arg2)
    {
        goto fail;
    }
    err = ESP_OK;
fail:
    return err;
}
