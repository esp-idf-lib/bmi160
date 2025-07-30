/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Tomoyuki Sakurai
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __TEMPLATE_COMPONENT_H__
#define __TEMPLATE_COMPONENT_H__

#include <stdint.h> // system headers first
#include <esp_err.h> // then, esp-idf headers
// #include "local.h" // local header files at the end.

#ifdef __cplusplus
extern "C" {
#endif

#define EXAMPLE_CHIP_ID  0x58 /*!< The chip ID, 0x58 */

/** @cond */
#define HIDDEN_ID 0xFF /* This will not be shown in HTML */
/** @endcond */

/**
* @brief Example port number
*/
typedef enum {
    EXAMPLE_NUM_0 = 0, /*!< EXAMPLE port 0 */
    EXAMPLE_NUM_1, /*!< EXAMPLE port 1 */
    EXAMPLE_NUM_MAX, /*!< EXAMPLE port max */
} example_port_t;

/**
* @brief A brief explanation for this function.
*        A longer explanation should be here.
*
* @note Please note that this is an example component
*
* @code{c}
*
* #include <esp_log.h>
*
* // include the header file.
* #include <template-component.h>
*
* #define TAG "example1"
*
* void app_main(void)
* {
*     uint8_t foo = 1;
*     uint8_t bar = 2;
*     esp_err_t err = func1(foo, bar);
*
*     ESP_LOGI(TAG, "result: %d", err);
* }
* @endcode
*
* @warning This is warning.
*
* @param[in] arg1 The first argument
* @param[in] arg2 The second argument
*
* @return
*   - ESP_OK: Successful
*   - ESP_FAIL: Failure
*
*/
esp_err_t func1(uint8_t arg1, uint8_t arg2);

#ifdef __cplusplus
}
#endif

#endif // __TEMPLATE_COMPONENT_H__
