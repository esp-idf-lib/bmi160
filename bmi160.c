/*
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) 2025 Lukasz Bielinski <lbielinski01@gmail.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include <esp_err.h>
#include <string.h>

#include "bmi160.h"

static const char *TAG = "bmi160";

#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp-idf, but device supports up to 2.44Mhz

#define I2C_PORT 0
#define WARNING_CHANNEL 1
#define WARNING_CURRENT (40.0)


#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define CHECK_LOGE(x, msg, ...) do { \
        esp_err_t __; \
        if ((__ = x) != ESP_OK) { \
            ESP_LOGE(TAG, msg, ## __VA_ARGS__); \
            return __; \
        } \
    } while (0)

static esp_err_t is_acc_odr_fits_mode(bmi160_acc_odr_t odr, bmi160_pmu_acc_mode_t mode, bmi160_acc_lp_avg_t avg);
static esp_err_t is_gyr_mode_valid(bmi160_pmu_gyr_mode_t mode);
static esp_err_t is_acc_mode_valid(bmi160_pmu_acc_mode_t mode);
static esp_err_t is_acc_us_valid(bmi160_acc_us_t acc_us);
static esp_err_t is_gyr_odr_valid(bmi160_gyr_odr_t odr);
static esp_err_t is_conf_valid(const bmi160_conf_t* const conf);
static esp_err_t is_int_out_conf_valid(const bmi160_int_out_conf_t* const intOutConf);
static esp_err_t is_tap_conf_valid(const bmi160_tap_conf_t* const tapConf);
static esp_err_t bmi160_read_reg_internal(bmi160_t *dev, uint8_t reg, uint8_t *val);
static esp_err_t bmi160_read_reg_array_internal(bmi160_t *dev, uint8_t reg, uint8_t *val, uint8_t num);
static esp_err_t bmi160_write_reg_internal(bmi160_t *dev, uint8_t reg, uint8_t val);
static esp_err_t bmi160_write_reg_array_internal(bmi160_t *dev, uint8_t reg, const uint8_t* val, uint8_t num);
static esp_err_t bmi160_read_data_internal(bmi160_t *dev, bmi160_result_t *result);
static esp_err_t bmi160_set_acc_range_internal(bmi160_t *dev, bmi160_acc_range_t range);
static esp_err_t bmi160_set_gyr_range_internal(bmi160_t *dev, bmi160_gyr_range_t range);
static esp_err_t bmi160_set_acc_conf_internal(bmi160_t *dev, bmi160_acc_odr_t odr, bmi160_acc_lp_avg_t avg, bmi160_acc_us_t acc_us);
static esp_err_t bmi160_set_gyr_odr_internal(bmi160_t *dev, bmi160_gyr_odr_t odr);
static esp_err_t bmi160_switch_accMode(bmi160_t *dev, bmi160_pmu_acc_mode_t accMode);
static esp_err_t bmi160_switch_gyrMode(bmi160_t *dev, bmi160_pmu_gyr_mode_t gyrMode);
static esp_err_t bmi160_startAcc(bmi160_t *dev, const bmi160_conf_t* const conf);
static esp_err_t bmi160_startGyr(bmi160_t *dev, const bmi160_conf_t* const conf);
static esp_err_t bmi160_reset(bmi160_t *dev);
static esp_err_t bmi160_int_config(bmi160_t *dev, const bmi160_int_out_conf_t* const intOutConf);
static esp_err_t bmi160_int_en(bmi160_t *dev, uint8_t reg, uint8_t mask);

typedef struct
{
    uint8_t add;
    uint8_t val;
} bmi160_reg_t;

/* Public functions */

esp_err_t bmi160_read_reg(bmi160_t *dev, uint8_t reg, uint8_t *val)
{
    CHECK_ARG(dev && val);
    esp_err_t ret = ESP_OK;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    ret = bmi160_read_reg_internal(dev, reg, val);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ret;
}

esp_err_t bmi160_read_reg_array(bmi160_t *dev, uint8_t reg, uint8_t *val, uint8_t num)
{
    CHECK_ARG(dev && val);
    esp_err_t ret = ESP_OK;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    ret = bmi160_read_reg_array_internal(dev, reg, val, num);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ret;
}

esp_err_t bmi160_write_reg(bmi160_t *dev, uint8_t reg, uint8_t val)
{
    CHECK_ARG(dev);
    esp_err_t ret = ESP_OK;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    ret = bmi160_write_reg_internal(dev, reg, val);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ret;
}

esp_err_t bmi160_write_reg_array(bmi160_t *dev, uint8_t reg, uint8_t* val, uint8_t num)
{
    CHECK_ARG(dev && val);
    esp_err_t ret = ESP_OK;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    ret = bmi160_write_reg_array_internal(dev, reg, val, num);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ret;
}

esp_err_t bmi160_init(bmi160_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);
    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    memset(&dev->aBias, 0, sizeof(dev->aBias));
    memset(&dev->gBias, 0, sizeof(dev->gBias));

    if (i2c_dev_create_mutex(&dev->i2c_dev) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t bmi160_free(bmi160_t *dev)
{
    CHECK_ARG(dev);
    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t bmi160_set_acc_range(bmi160_t *dev, bmi160_acc_range_t range)
{
    CHECK_ARG(dev);
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_set_acc_range_internal(dev, range), "Acc range set failed");
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}


esp_err_t bmi160_set_gyr_range(bmi160_t *dev, bmi160_gyr_range_t range)
{
    CHECK_ARG(dev);
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_set_gyr_range_internal(dev, range), "Gyr range set failed");
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}


esp_err_t bmi160_set_acc_conf(bmi160_t *dev, bmi160_acc_odr_t odr, bmi160_acc_lp_avg_t avg, bmi160_acc_us_t acc_us)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_set_acc_conf_internal(dev, odr, avg, acc_us), "Acc conf set failed");
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;

}


esp_err_t bmi160_set_gyr_odr(bmi160_t *dev, bmi160_gyr_odr_t odr)
{
    CHECK_ARG(dev);
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_set_gyr_odr_internal(dev, odr), "Gyr odr set failed");
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;

}


esp_err_t bmi160_read_data(bmi160_t *dev, bmi160_result_t *result)
{
    CHECK_ARG(dev && result);
    esp_err_t ret = ESP_OK;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    ret = bmi160_read_data_internal(dev, result);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ret;
}


esp_err_t bmi160_start(bmi160_t *dev, const bmi160_conf_t* const conf)
{
    CHECK_ARG(dev && conf);

    //validate parameters before starting process
    CHECK_LOGE(is_conf_valid(conf), "Invalid configuration");

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    //read device id
    uint8_t device_id;
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_read_reg_internal(dev, BMI160_CHIP_ID, &device_id), "Chip ID read failed");

    ESP_LOGD(TAG, "Device ID: 0x%02x", device_id);
    if (device_id != BMI160_CHIP_ID_DEFAULT_VALUE)
    {
        ESP_LOGE(TAG, "Wrong device ID: 0x%02x", device_id);
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }

    //read error status
    uint8_t err;
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_read_reg_internal(dev, BMI160_ERR_REG, &err), "Error reg read failed");
    ESP_LOGD(TAG, "Error: 0x%02x", err);
    if (err != 0)
    {
        ESP_LOGE(TAG, "Error: 0x%02x", err);
    }

    //reset device
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_reset(dev), " Reset fail");

    if (conf->accMode != BMI160_PMU_ACC_SUSPEND)
    {
        //start up accelerometer
        I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_startAcc(dev, conf), "bmi160_startAcc failed");
    }

    if (conf->gyrMode != BMI160_PMU_GYR_SUSPEND)
    {
        //start up gyroscope
        I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_startGyr(dev, conf), "bmi160_startGyr failed");
    }

    //pmu status
    uint8_t pmu_status = 0;
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_read_reg_internal(dev, BMI160_PMU_STATUS, &pmu_status), "Pmu status read failed");
    ESP_LOGD(TAG, "ACC PMU Status: 0x%02x", (pmu_status & 0x30) >> 4);
    ESP_LOGD(TAG, "GYR PMU Status: 0x%02x", (pmu_status & 0x0C) >> 2);
    ESP_LOGD(TAG, "MAG PMU Status: 0x%02x", (pmu_status & 0x03));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t bmi160_calibrate(bmi160_t *dev)
{
    CHECK_ARG(dev);
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    //calibrate accelerometer and gyroscope to calculate bias from 64 readings in 20 ms period

    bmi160_result_t result;
    float accX = 0, accY = 0, accZ = 0, gyroX = 0, gyroY = 0, gyroZ = 0;
    for (int i = 0; i < 64; i++)
    {
        I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_read_data_internal(dev, &result), "Calibrate read_data failed");
        accX += result.accX;
        accY += result.accY;
        accZ += result.accZ;
        gyroX += result.gyroX;
        gyroY += result.gyroY;
        gyroZ += result.gyroZ;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    //calculate average
    accX /= 64.0f;
    accY /= 64.0f;
    accZ /= 64.0f;
    gyroX /= 64.0f;
    gyroY /= 64.0f;
    gyroZ /= 64.0f;

    //store bias values
    dev->aBias[0] = accX;
    dev->aBias[1] = accY;
    dev->aBias[2] = accZ;
    dev->gBias[0] = gyroX;
    dev->gBias[1] = gyroY;
    dev->gBias[2] = gyroZ;

    //print bias values
    ESP_LOGD(TAG, "Accel Bias: %+.3f %+.3f %+.3f Gyro Bias: %+.3f %+.3f %+.3f", dev->aBias[0], dev->aBias[1], dev->aBias[2], dev->gBias[0], dev->gBias[1], dev->gBias[2]);

    return ESP_OK;
}

/**
 * @brief self test for the BMI160
 *
 * @note
 *
 * @param dev
 * @return esp_err_t
 */
esp_err_t bmi160_self_test(bmi160_t *dev)
{
    CHECK_ARG(dev);
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    //read device id
    uint8_t device_id;
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_read_reg_internal(dev, BMI160_CHIP_ID, &device_id), "Chip ID read failed");
    ESP_LOGD(TAG, "Device ID: 0x%02x", device_id);
    if (device_id != BMI160_CHIP_ID_DEFAULT_VALUE)
    {
        ESP_LOGE(TAG, "Wrong device ID: 0x%02x", device_id);
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }

    //reset device
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_reset(dev), " Reset fail");

    /* 1. acceletrometer */
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_set_acc_range_internal(dev, BMI160_ACC_RANGE_8G), "bmi160_set_acc_range_internal failed");
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_ACC_CONF, 0x2C), "Set Acc Conf failed");  // Set Accel ODR to 1600hz, BWP mode to Oversample 2, acc_us = 0

    // test negative direction
    ESP_LOGD(TAG, "Accel self test sign 0");
    uint8_t reg = (0x01 << 0) | (0x00 << 2) | (0x01 << 3); // acc_self_test_en = 1, acc_self_test_sign = 0, acc_self_test_amp = 1
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_SELF_TEST, reg), "Start self test failed");

    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t acc_self_test_result;
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_read_reg_internal(dev, BMI160_STATUS, &acc_self_test_result), "Read status failed");

    ESP_LOGD(TAG, "Accel self test result: %02x", acc_self_test_result);

    uint8_t rawData[6];
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_read_reg_array_internal(dev, BMI160_ACC_X_L, rawData, 6), "Read ACC_X_L failed");
    float accX0 = (float)((int16_t)(rawData[1] << 8) | rawData[0]) * dev->aRes;
    float accY0 = (float)((int16_t)(rawData[3] << 8) | rawData[2]) * dev->aRes;
    float accZ0 = (float)((int16_t)(rawData[5] << 8) | rawData[4]) * dev->aRes;

    ESP_LOGD(TAG, "Accel self test: %.3f %.3f %.3f", accX0, accY0, accZ0);

    // test positive direction
    ESP_LOGD(TAG, "Accel self test sign 1");
    reg = (0x01 << 0) | (0x01 << 2) | (0x01 << 3); // acc_self_test_en = 1, acc_self_test_sign = 1, acc_self_test_amp = 1
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_SELF_TEST, reg), "Start self test failed");

    vTaskDelay(pdMS_TO_TICKS(100));
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_read_reg_internal(dev, BMI160_STATUS, &acc_self_test_result), "Read status failed");

    ESP_LOGD(TAG, "Accel self test result: %02x", acc_self_test_result);


    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_read_reg_array_internal(dev, BMI160_ACC_X_L, rawData, 6), "Read ACC_X_L failed");
    float accX1 = (float)((int16_t)(rawData[1] << 8) | rawData[0]) * dev->aRes;
    float accY1 = (float)((int16_t)(rawData[3] << 8) | rawData[2]) * dev->aRes;
    float accZ1 = (float)((int16_t)(rawData[5] << 8) | rawData[4]) * dev->aRes;

    ESP_LOGD(TAG, "Accel self test: %.3f %.3f %.3f", accX1, accY1, accZ1);

    ESP_LOGD(TAG, "Accel self test diff: %.3f %.3f %.3f", accX1 - accX0, accY1 - accY0, accZ1 - accZ0);


    /* 2. gyroscope */

    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_set_gyr_range_internal(dev, BMI160_GYR_RANGE_1000DPS), "Set gyr range failed");
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_GYR_CONF, 0x2C), "Write gyr config failed");  // Set Gyro ODR to 1600hz, BWP mode to Oversample 2, gyr_us = 0

    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_SELF_TEST, (uint8_t)(0x1 << 4)), "Write self test failed"); // gyr_self_test_en = 1

    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t gyr_self_test_result;
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_read_reg_internal(dev, BMI160_STATUS, &gyr_self_test_result), "Read status failed");
    ESP_LOGD(TAG, "Gyro self test result: %02x", gyr_self_test_result);
    if (gyr_self_test_result & (0x1 << 1))
    {
        ESP_LOGD(TAG, "Gyro self test failed");
    }
    else
    {
        ESP_LOGD(TAG, "Gyro self test passed");
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t bmi160_enable_int_new_data(bmi160_t *dev, const bmi160_int_out_conf_t* const intOutConf)
{
    CHECK_ARG(dev && intOutConf);

    CHECK_LOGE(is_int_out_conf_valid(intOutConf), "Invalid intOutConf");

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t data = 0;

    //configure interrupt output
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_int_config(dev, intOutConf), "Config in_out_ctrl failed");

    //map interrupt data ready
    if (intOutConf->intPin == BMI160_PIN_INT1)
    {
        data = (1u << 7); //set bit for INT1
    }
    else
    {
        data = (1u << 3); //set bit for INT2
    }
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_INT_MAP_1, data), "Map data ready interrupt failed"); //map data ready interrupt to INT1

    //enable interrupt
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_int_en(dev, BMI160_INT_EN_1, (1u << 4)), "Enable interrupt flag failed"); // enable new data interrupt

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t bmi160_enable_step_counter(bmi160_t *dev, bmi160_step_counter_mode_t mode)
{
    CHECK_ARG(dev);
    //enable step counter

    uint8_t config[2] = {0, 0};
    switch (mode)
    {
        case BMI160_STEP_COUNTER_NORMAL:
            config[0] = 0x15;
            config[1] = 0x03;
            break;

        case BMI160_STEP_COUNTER_SENSITIVE:
            config[0] = 0x2D;
            config[1] = 0x00;
            break;

        case BMI160_STEP_COUNTER_ROBUST:
            config[0] = 0x1D;
            config[1] = 0x07;
            break;

        default:
            ESP_LOGE(TAG, "Invalid step counter mode");
            return ESP_FAIL;
    }

    config[1] |= (1u << 3); //enable step counter

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    //configure step counter
    // I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_array_internal(dev, BMI160_STEP_CONF_0, config, 2), "Write step conf failed");
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_STEP_CONF_0, config[0]), "Write step conf failed");
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_STEP_CONF_1, config[1]), "Write step conf failed");
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t bmi160_read_step_counter(bmi160_t *dev, uint16_t *stepCounter)
{
    CHECK_ARG(dev && stepCounter);
    uint8_t data[2];
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_read_reg_array_internal(dev, BMI160_STEP_CNT_0, data, 2), "Read step cnt failed");
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *stepCounter = (uint16_t)((data[1] << 8) | data[0]);
    return ESP_OK;
}

esp_err_t bmi160_reset_step_counter(bmi160_t *dev)
{
    CHECK_ARG(dev);
    return bmi160_write_reg(dev, BMI160_CMD, BMI160_CMD_STEP_RESET); // reset step counter
}


esp_err_t bmi160_enable_int_step(bmi160_t *dev, const bmi160_int_out_conf_t* const intOutConf)
{
    CHECK_ARG(dev && intOutConf);

    CHECK_LOGE(is_int_out_conf_valid(intOutConf), "Invalid intOutConf");

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t data = 0;

    //configure interrupt output
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_int_config(dev, intOutConf), "Config in_out_ctrl failed");

    //map interrupt step detection
    data = (1u << 0); //set bit for step detection
    if (intOutConf->intPin == BMI160_PIN_INT1)
    {
        I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_INT_MAP_0, data), "Write int map 0 failed"); //map step detection interrupt to INT1
    }
    else
    {
        I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_INT_MAP_2, data), "Write int map 1 failed"); //map step detection interrupt to INT2
    }

    //enable interrupt
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_int_en(dev, BMI160_INT_EN_2, (1u << 3)), "Enable interrupt flag failed"); // enable step detection interrupt

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}


esp_err_t bmi160_switch_mode(bmi160_t *dev, bmi160_pmu_acc_mode_t accMode, bmi160_pmu_gyr_mode_t gyrMode)
{
    CHECK_ARG(dev);
    //validate parameters
    CHECK_LOGE(is_acc_mode_valid(accMode), "Invalid Accelerometer Mode");
    CHECK_LOGE(is_gyr_mode_valid(gyrMode), "Invalid Gyroscope Mode");

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    //start up accelerometer
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_switch_accMode(dev, accMode), "Switch accMode failed");

    //start up gyroscope
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_switch_gyrMode(dev, gyrMode), "Switch gyrMode failed");

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

esp_err_t bmi160_enable_tap_detection(bmi160_t *dev, const bmi160_tap_conf_t* const tapConf)
{
    CHECK_ARG(dev && tapConf);
    //validate parameters
    CHECK_LOGE(is_tap_conf_valid(tapConf), "Invalid tapConf");

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t data = 0;
    data |= (uint8_t)(tapConf->tapQuiet << 7); //set quiet bit
    data |= (uint8_t)(tapConf->tapShock << 6); //set shock bit
    data |= (uint8_t)(tapConf->tapDur); //set duration bits
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_INT_TAP_0, data), "Write int map 0 failed");

    data = (uint8_t)(tapConf->tapTh); //set threshold bits
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_INT_TAP_1, data), "Write int map 1 failed");
    dev->tapMode = tapConf->tapMode;
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

esp_err_t bmi160_enable_int_tap(bmi160_t *dev, const bmi160_int_out_conf_t* const intOutConf)
{
    CHECK_ARG(dev && intOutConf);

    CHECK_LOGE(is_int_out_conf_valid(intOutConf), "Invalid intOutConf");

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t data = 0;

    //configure interrupt output
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_int_config(dev, intOutConf), "Config in_out_ctrl failed");

    //map interrupt step detection
    if (dev->tapMode == BMI160_TAP_MODE_SINGLE)
    {
        data = (1u << 5); //set bit for single tap detection
    }
    else
    {
        data = (1u << 4); //set bit for double tap detection
    }
    if (intOutConf->intPin == BMI160_PIN_INT1)
    {
        I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_INT_MAP_0, data), "Write int map 0 failed"); //map step detection interrupt to INT1
    }
    else
    {
        I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_INT_MAP_1, data), "Write int map 0 failed"); //map step detection interrupt to INT2
    }

    //enable interrupt
    uint8_t mask = 0;
    if (dev->tapMode == BMI160_TAP_MODE_SINGLE)
    {
        mask = (1u << 5); //set bit for single tap detection
    }
    else
    {
        mask = (1u << 4); //set bit for double tap detection
    }
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_int_en(dev, BMI160_INT_EN_0, mask), "Enable interrupt flag failed"); // enable tap interrupt

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t bmi160_read_tap_orient(bmi160_t *dev, uint8_t *orient)
{
    CHECK_ARG(dev && orient);
    //read int_status_2
    uint8_t data;
    CHECK_LOGE(bmi160_read_reg(dev, BMI160_INT_STATUS_2, &data), "Read STATUS_2 failed");

    *orient = data;
    return ESP_OK;
}

/* Local functions */

/**
 * @brief Internal function to read a register from the BMI160
 * @note This function does not take the mutex, the caller should ensure thread safety
 *
 * @param dev Pointer to the BMI160 device structure
 * @param reg Register address to read from
 * @param val Pointer to a variable to store the read value
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 *
 */
static esp_err_t bmi160_read_reg_internal(bmi160_t *dev, uint8_t reg, uint8_t *val)
{
    CHECK_LOGE(i2c_dev_read_reg(&dev->i2c_dev, reg, val, 1), "i2c_dev_read_reg fail: reg=0x%02x, size=%d", reg, 1);
    ESP_LOGD(TAG, "i2c read [0x%02x] = 0x%02x", reg, *val);
    return ESP_OK;
}

/**
 * @brief Internal function to read multiple registers from the BMI160
 * @note This function does not take the mutex, the caller should ensure thread safety
 *
 * @param dev Pointer to the BMI160 device structure
 * @param reg Starting register address to read from
 * @param val Pointer to a buffer to store the read values
 * @param num Number of bytes to read
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t bmi160_read_reg_array_internal(bmi160_t *dev, uint8_t reg, uint8_t *val, uint8_t num)
{
    CHECK_LOGE(i2c_dev_read_reg(&dev->i2c_dev, reg, val, num), "i2c_dev_read_reg fail: reg=0x%02x, size=%d", reg, num);
    return ESP_OK;
}

/**
 * @brief Internal function to write a register to the BMI160
 * @note This function does not take the mutex, the caller should ensure thread safety
 *
 * @param dev Pointer to the BMI160 device structure
 * @param reg Register address to write to
 * @param val Value to write to the register
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t bmi160_write_reg_internal(bmi160_t *dev, uint8_t reg, uint8_t val)
{
    ESP_LOGD(TAG, "i2c write [0x%02x] = 0x%02x", reg, val);
    CHECK_LOGE(i2c_dev_write_reg(&dev->i2c_dev, reg, &val, 1), "i2c_dev_write_reg fail: reg=0x%02x, size=%d", reg, 1);
    /**
     * BMI160 has timing requirements. Datasheet 3.2.4 states that "The minimum time between write any next
     * operation must be >2us in Normal mode and >450 us in Suspended mode."
     * The times are preliminary and may need to be adjusted based on testing. For now, we just add a delay
     * after each write operation to ensure timing requirements are met.
     */
    vTaskDelay(pdMS_TO_TICKS(1)); //delay 1 ms after each write operation

    return ESP_OK;
}

/**
 * @brief Internal function to write multiple registers to the BMI160
 * @note This function does not take the mutex, the caller should ensure thread safety
 *
 * @param dev Pointer to the BMI160 device structure
 * @param reg Starting register address to write to
 * @param val Pointer to a buffer containing the values to write
 * @param num Number of bytes to write
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t bmi160_write_reg_array_internal(bmi160_t *dev, uint8_t reg, const uint8_t* val, uint8_t num)
{
    CHECK_LOGE(i2c_dev_write_reg(&dev->i2c_dev, reg, val, num),  "i2c_dev_write_reg fail: reg=0x%02x, size=%d", reg, num);
    /**
     * BMI160 has timing requirements. Datasheet 3.2.4 states that "The minimum time between write any next
     * operation must be >2us in Normal mode and >450 us in Suspended mode."
     * The times are preliminary and may need to be adjusted based on testing. For now, we just add a delay
     * after each write operation to ensure timing requirements are met.
     */
    vTaskDelay(pdMS_TO_TICKS(1)); //delay 1 ms after each write operation
    return ESP_OK;
}

/**
 * @brief Internal function to set the accelerometer range of the BMI160
 * @note This function does not take the mutex, the caller should ensure thread safety
 *
 * @param dev Pointer to the BMI160 device structure
 * @param range Accelerometer range to set
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t bmi160_set_acc_range_internal(bmi160_t *dev, bmi160_acc_range_t range)
{

    float aRes = 0.0f;
    switch (range)
    {
        case BMI160_ACC_RANGE_2G:
            aRes = 2.0f / 32768.0f;
            break;
        case BMI160_ACC_RANGE_4G:
            aRes = 4.0f / 32768.0f;
            break;
        case BMI160_ACC_RANGE_8G:
            aRes = 8.0f / 32768.0f;
            break;
        case BMI160_ACC_RANGE_16G:
            aRes = 16.0f / 32768.0f;
            break;
        default:
            ESP_LOGE(TAG, "Invalid Accelerometer Range");
            return ESP_FAIL;
    }

    esp_err_t ret = bmi160_write_reg_internal(dev, BMI160_ACC_RANGE, range);  // Set up scale Accel range.
    if (ESP_OK == ret)
    {
        dev->accRange = range;
        dev->aRes = aRes;
    }
    return ret;
}

/**
 * @brief Internal function to set the gyroscope range of the BMI160
 * @note This function does not take the mutex, the caller should ensure thread safety
 *
 * @param dev Pointer to the BMI160 device structure
 * @param range Gyroscope range to set
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t bmi160_set_gyr_range_internal(bmi160_t *dev, bmi160_gyr_range_t range)
{
    float gRes = 0.0f;
    switch (range)
    {
        case BMI160_GYR_RANGE_2000DPS:
            gRes = 2000.0f / 32768.0f;
            break;
        case BMI160_GYR_RANGE_1000DPS:
            gRes = 1000.0f / 32768.0f;
            break;
        case BMI160_GYR_RANGE_500DPS:
            gRes = 500.0f / 32768.0f;
            break;
        case BMI160_GYR_RANGE_250DPS:
            gRes = 250.0f / 32768.0f;
            break;
        case BMI160_GYR_RANGE_125DPS:
            gRes = 125.0f / 32768.0f;
            break;
        default:
            ESP_LOGE(TAG, "Invalid Gyroscope Range");
            return ESP_FAIL;
    }
    esp_err_t ret = bmi160_write_reg_internal(dev, BMI160_GYR_RANGE, range);  // Set up scale Gyro range.
    if (ESP_OK == ret)
    {
        dev->gyrRange = range;
        dev->gRes = gRes;
    }

    return ret;
}

/**
 * @brief Internal function to set the accelerometer configuration of the BMI160
 * @note This function does not take the mutex, the caller should ensure thread safety
 *
 * @param dev Pointer to the BMI160 device structure
 * @param odr Accelerometer output data rate to set
 * @param avg Accelerometer low power average sample to set
 * @param acc_us Accelerometer undersampling to set
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t bmi160_set_acc_conf_internal(bmi160_t *dev, bmi160_acc_odr_t odr, bmi160_acc_lp_avg_t avg, bmi160_acc_us_t acc_us)
{
    CHECK_LOGE(is_acc_odr_fits_mode(odr, dev->accMode, avg), "Invalid odr (%d) or avg(%d) for accMode (%d)", odr, avg, dev->accMode);
    CHECK_LOGE(is_acc_us_valid(acc_us), "Invalid acc_us (%d) ", acc_us);

    uint8_t accConf = (odr & 0x0Fu) | ((avg & 0x07u) << 4) | ((acc_us & 0x1u) << 7);
    esp_err_t ret = bmi160_write_reg_internal(dev, BMI160_ACC_CONF,  accConf);  // Set Accel CONF
    if (ESP_OK == ret)
    {
        dev->accConf = accConf;
        dev->accOdr = odr;
    }

    return ret;
}

/**
 * @brief Internal function to set the gyroscope output data rate of the BMI160
 * @note This function does not take the mutex, the caller should ensure thread safety
 *
 * @param dev Pointer to the BMI160 device structure
 * @param odr Gyroscope output data rate to set
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t bmi160_set_gyr_odr_internal(bmi160_t *dev, bmi160_gyr_odr_t odr)
{
    CHECK_LOGE(is_gyr_odr_valid(odr), "Invalid gyroscope ODR");
    esp_err_t ret = bmi160_write_reg_internal(dev, BMI160_GYR_CONF, odr);  // Set Gyro ODR
    if (ESP_OK == ret)
    {
        dev->gyrOdr = odr;
    }

    return ret;
}

/**
 * @brief Internal function to read accelerometer and gyroscope data from the BMI160
 * @note This function does not take the mutex, the caller should ensure thread safety
 *
 * @param dev Pointer to the BMI160 device structure
 * @param result Pointer to the result structure to store the read data
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t bmi160_read_data_internal(bmi160_t *dev, bmi160_result_t *result)
{
    uint8_t rawData[12];
    CHECK_LOGE(bmi160_read_reg_array_internal(dev, BMI160_GYR_X_L, rawData, 12), "Read data failed");

    int16_t data[6];
    //loop to convert 2 8bit values to 16 bit value
    for (int i = 0; i < 6; i++)
    {
        data[i] = ((int16_t)(rawData[i * 2 + 1]) << 8) | rawData[i * 2];
    }

    result->accX = ((float)data[3] * dev->aRes) - dev->aBias[0]; //acceleration x
    result->accY = ((float)data[4] * dev->aRes) - dev->aBias[1]; //acceleration y
    result->accZ = ((float)data[5] * dev->aRes) - dev->aBias[2]; //acceleration z
    result->gyroX = ((float)data[0] * dev->gRes) - dev->gBias[0]; //gyro x
    result->gyroY = ((float)data[1] * dev->gRes) - dev->gBias[1]; //gyro y
    result->gyroZ = ((float)data[2] * dev->gRes) - dev->gBias[2]; //gyro z

    return ESP_OK;
}

/**
 * @brief Internal function to validate the BMI160 configuration
 *
 * @param conf Pointer to the BMI160 configuration structure to validate
 *
 * @return esp_err_t ESP_OK if the configuration is valid, or an error code if it is invalid
 */
static esp_err_t is_conf_valid(const bmi160_conf_t* const conf)
{
    CHECK(is_acc_mode_valid(conf->accMode));
    CHECK(is_gyr_mode_valid(conf->gyrMode));
    CHECK(is_acc_odr_fits_mode(conf->accOdr, conf->accMode, conf->accAvg));
    CHECK(is_acc_us_valid(conf->accUs));
    CHECK(is_gyr_odr_valid(conf->gyrOdr));
    return ESP_OK;
}

/**
 * @brief Internal function to validate the accelerometer mode
 *
 * @param mode Accelerometer mode to validate
 *
 * @return esp_err_t ESP_OK if the mode is valid, or an error code if it is invalid
 */
static esp_err_t is_acc_mode_valid(bmi160_pmu_acc_mode_t mode)
{
    switch (mode)
    {
        case BMI160_PMU_ACC_SUSPEND:
        case BMI160_PMU_ACC_NORMAL:
        case BMI160_PMU_ACC_LOW_POWER:
            return ESP_OK;
        default:
            ESP_LOGD(TAG, "Invalid accelerometer mode");
            return ESP_FAIL;
    }
}

/**
 * @brief Internal function to validate the gyroscope mode
 *
 * @param mode Gyroscope mode to validate
 *
 * @return esp_err_t ESP_OK if the mode is valid, or an error code if it is invalid
 */
static esp_err_t is_gyr_mode_valid(bmi160_pmu_gyr_mode_t mode)
{
    switch (mode)
    {
        case BMI160_PMU_GYR_SUSPEND:
        case BMI160_PMU_GYR_NORMAL:
        case BMI160_PMU_GYR_FAST_STARTUP:
            return ESP_OK;
        default:
            ESP_LOGD(TAG, "Invalid gyroscope mode");
            return ESP_FAIL;
    }
}

/**
 * @brief Internal function to validate the accelerometer undersampling configuration
 *
 * @param acc_us Accelerometer undersampling configuration to validate
 *
 * @return esp_err_t ESP_OK if the configuration is valid, or an error code if it is invalid
 */
static esp_err_t is_acc_us_valid(bmi160_acc_us_t acc_us)
{
    switch (acc_us)
    {
        case BMI160_ACC_US_OFF:
        case BMI160_ACC_US_ON:
            return ESP_OK;
        default:
            ESP_LOGD(TAG, "Invalid acc_us");
            return ESP_FAIL;
    }
}

/**
 * @brief Internal function to validate the gyroscope output data rate configuration
 *
 * @param odr Gyroscope output data rate configuration to validate
 *
 * @return esp_err_t ESP_OK if the configuration is valid, or an error code if it is invalid
 */
static esp_err_t is_gyr_odr_valid(bmi160_gyr_odr_t odr)
{
    esp_err_t ret = ESP_FAIL;
    if (odr >= BMI160_GYR_ODR_25HZ && odr <= BMI160_GYR_ODR_1600HZ)
    {
        ret = ESP_OK;
    }
    else
    {
        ESP_LOGD(TAG, "Invalid gyroscope ODR");
    }
    return ret;
}

/**
 * @brief Internal function to validate if the accelerometer output data rate and low power average configuration fits the accelerometer mode
 *
 * @param odr Accelerometer output data rate configuration to validate
 * @param mode Accelerometer mode configuration to validate
 * @param avg Accelerometer low power average configuration to validate
 *
 * @return esp_err_t ESP_OK if the configuration is valid, or an error code if it is invalid
 */
static esp_err_t is_acc_odr_fits_mode(bmi160_acc_odr_t odr, bmi160_pmu_acc_mode_t mode, bmi160_acc_lp_avg_t avg)
{
    esp_err_t result = ESP_FAIL;
    // check ranges of odr and avg
    if ((odr < BMI160_ACC_ODR_0_78HZ) || (odr > BMI160_ACC_ODR_1600HZ))
    {
        ESP_LOGD(TAG, "Invalid acc ODR");
        return ESP_FAIL;
    }
    if ((avg < BMI160_ACC_LP_AVG_1) || (avg > BMI160_ACC_LP_AVG_128))
    {
        ESP_LOGD(TAG, "Invalid acc lp average");
        return ESP_FAIL;
    }

    // check mode fit
    switch (mode)
    {
        case BMI160_PMU_ACC_SUSPEND:
            result = ESP_OK;
            break;
        case BMI160_PMU_ACC_NORMAL:
            if (odr >= BMI160_ACC_ODR_12_5HZ)
            {
                result = ESP_OK;
            }
            else
            {
                ESP_LOGD(TAG, "Invalid acc ODR for normal mode");
            }
            break;
        case BMI160_PMU_ACC_LOW_POWER:
            if ((odr == BMI160_ACC_ODR_400HZ) && (avg <= BMI160_ACC_LP_AVG_2))
            {
                result = ESP_OK;
            }
            else if ((odr == BMI160_ACC_ODR_200HZ) && (avg <= BMI160_ACC_LP_AVG_4))
            {
                result = ESP_OK;
            }
            else if ((odr == BMI160_ACC_ODR_100HZ) && (avg <= BMI160_ACC_LP_AVG_8))
            {
                result = ESP_OK;
            }
            else if ((odr == BMI160_ACC_ODR_50HZ) && (avg <= BMI160_ACC_LP_AVG_16))
            {
                result = ESP_OK;
            }
            else if ((odr == BMI160_ACC_ODR_25HZ) && (avg <= BMI160_ACC_LP_AVG_32))
            {
                result = ESP_OK;
            }
            else if ((odr == BMI160_ACC_ODR_12_5HZ) && (avg <= BMI160_ACC_LP_AVG_64))
            {
                result = ESP_OK;
            }
            else if ((odr <= BMI160_ACC_ODR_6_25HZ) && (avg <= BMI160_ACC_LP_AVG_128))
            {
                result = ESP_OK;
            }
            else
            {
                ESP_LOGD(TAG, "Invalid acc ODR and LP_AVG for low power mode");
                result = ESP_FAIL;
            }
            break;
        default:
            ESP_LOGD(TAG, "Invalid acc mode");
            result = ESP_FAIL;
            break;
    }

    return result;
}

/**
 * @brief Internal function to validate the interrupt output configuration
 *
 * @param intOutConf Pointer to the interrupt output configuration structure to validate
 *
 * @return esp_err_t ESP_OK if the configuration is valid, or an error code if it is invalid
 */
static esp_err_t is_int_out_conf_valid(const bmi160_int_out_conf_t* const intOutConf)
{
    /* intEnable */
    if (!((intOutConf->intEnable >= BMI160_INT_DISABLE) && (intOutConf->intEnable <= BMI160_INT_ENABLE)))
    {
        ESP_LOGD(TAG, "Invalid intEnable");
        return ESP_FAIL;
    }

    /* intLevel */
    if (!((intOutConf->intLevel >= BMI160_INT_ACTIVE_LOW) && (intOutConf->intLevel <= BMI160_INT_ACTIVE_HIGH)))
    {
        ESP_LOGD(TAG, "Invalid intLevel");
        return ESP_FAIL;
    }

    /* intOd */
    if (!((intOutConf->intOd >= BMI160_INT_PUSH_PULL) && (intOutConf->intOd <= BMI160_INT_OPEN_DRAIN)))
    {
        ESP_LOGD(TAG, "Invalid intOd");
        return ESP_FAIL;
    }

    /* intPin */
    if (!((intOutConf->intPin >= BMI160_PIN_INT1) && (intOutConf->intPin <= BMI160_PIN_INT2)))
    {
        ESP_LOGD(TAG, "Invalid intPin");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Internal function to validate the tap detection configuration
 *
 * @param tapConf Pointer to the tap detection configuration structure to validate
 *
 * @return esp_err_t ESP_OK if the configuration is valid, or an error code if it is invalid
 */
static esp_err_t is_tap_conf_valid(const bmi160_tap_conf_t* const tapConf)
{
    if (!((tapConf->tapDur >= BMI160_TAP_DUR_50MS) && (tapConf->tapDur <= BMI160_TAP_DUR_700MS)))
    {
        ESP_LOGD(TAG, "Invalid tap duration");
        return ESP_FAIL;
    }
    if (!((tapConf->tapTh >= BMI160_TAP_TH_0_032G) && (tapConf->tapTh <= BMI160_TAP_TH_4G)))
    {
        ESP_LOGD(TAG, "Invalid tap threshold");
        return ESP_FAIL;
    }
    if (!((tapConf->tapQuiet >= BMI160_TAP_QUIET_30MS) && (tapConf->tapQuiet <= BMI160_TAP_QUIET_20MS)))
    {
        ESP_LOGD(TAG, "Invalid tap quiet time");
        return ESP_FAIL;
    }
    if (!((tapConf->tapShock >= BMI160_TAP_SHOCK_50MS) && (tapConf->tapShock <= BMI160_TAP_SHOCK_75MS)))
    {
        ESP_LOGD(TAG, "Invalid tap shock time");
        return ESP_FAIL;
    }
    if (!((tapConf->tapMode >= BMI160_TAP_MODE_SINGLE) && (tapConf->tapMode <= BMI160_TAP_MODE_DOUBLE)))
    {
        ESP_LOGD(TAG, "Invalid tap mode");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief Internal function to read the accelerometer mode from the BMI160
 * @note This function does not take the mutex, the caller should ensure thread safety
 *
 * @param dev Pointer to the BMI160 device structure
 * @param accMode Pointer to a variable to store the read accelerometer mode
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t bmi160_read_accMode(bmi160_t *dev, bmi160_pmu_acc_mode_t* accMode)
{
    uint8_t pmu_status = 0;
    CHECK_LOGE(bmi160_read_reg_internal(dev, BMI160_PMU_STATUS, &pmu_status), "Pmu status read failed");

    *accMode = ((pmu_status & 0x30) >> 4); // parse pmu status to get accelerator status
    ESP_LOGD(TAG, "AccMode = 0x%02x ", *accMode);
    *accMode += BMI160_PMU_ACC_SUSPEND; // map to bmi160_pmu_acc_mode_t
    dev->accMode = *accMode;
    return ESP_OK;
}

/**
 * @brief Internal function to switch the accelerometer mode of the BMI160
 * @note This function does not take the mutex, the caller should ensure thread safety
 *
 * @param dev Pointer to the BMI160 device structure
 * @param accMode Accelerometer mode to switch to
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t bmi160_switch_accMode(bmi160_t *dev, bmi160_pmu_acc_mode_t accMode)
{
    bmi160_pmu_acc_mode_t accModeNow = BMI160_PMU_ACC_SUSPEND;

    CHECK_LOGE(bmi160_read_accMode(dev, &accModeNow), "Mode read failed");

    if (accMode == accModeNow)
    {
        // mode already set
        return ESP_OK;
    }

    ESP_LOGD(TAG, "Setting accMode 0x%02x", accMode);
    CHECK_LOGE(bmi160_write_reg_internal(dev, BMI160_CMD, accMode), "Mode %d not set", accMode);

    vTaskDelay(pdMS_TO_TICKS(100));

    CHECK_LOGE(bmi160_read_accMode(dev, &accModeNow), "Mode read failed");
    if (accModeNow != accMode)
    {
        ESP_LOGE(TAG, "Wrong accelerometer mode: 0x%02x instead 0x%02x", accModeNow, accMode);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Internal function to read the gyroscope mode from the BMI160
 * @note This function does not take the mutex, the caller should ensure thread safety
 *
 * @param dev Pointer to the BMI160 device structure
 * @param gyrMode Pointer to a variable to store the read gyroscope mode
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t bmi160_read_gyrMode(bmi160_t *dev, bmi160_pmu_gyr_mode_t* gyrMode)
{
    uint8_t pmu_status = 0;
    CHECK_LOGE(bmi160_read_reg_internal(dev, BMI160_PMU_STATUS, &pmu_status), "Pmu status read failed");

    *gyrMode = ((pmu_status & 0x0C) >> 2); // parse pmu status to get gyroscope mode
    ESP_LOGD(TAG, "GyrMode = 0x%02x ", *gyrMode);
    *gyrMode += BMI160_PMU_GYR_SUSPEND; //map to bmi160_pmu_gyr_mode_t
    dev->gyrMode = *gyrMode;
    return ESP_OK;
}

/**
 * @brief Internal function to switch the gyroscope mode of the BMI160
 * @note This function does not take the mutex, the caller should ensure thread safety
 *
 * @param dev Pointer to the BMI160 device structure
 * @param gyrMode Gyroscope mode to switch to
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t bmi160_switch_gyrMode(bmi160_t *dev, bmi160_pmu_gyr_mode_t gyrMode)
{
    bmi160_pmu_gyr_mode_t gyrModeNow = BMI160_PMU_GYR_SUSPEND;

    CHECK_LOGE(bmi160_read_gyrMode(dev, &gyrModeNow), "Mode read failed");

    if (gyrMode == gyrModeNow)
    {
        // mode already set
        return ESP_OK;
    }

    ESP_LOGD(TAG, "Setting gyrMode 0x%02x", gyrMode);
    CHECK_LOGE(bmi160_write_reg_internal(dev, BMI160_CMD, gyrMode), "Cmd gyrMode failed");

    vTaskDelay(pdMS_TO_TICKS(100));

    CHECK_LOGE(bmi160_read_gyrMode(dev, &gyrModeNow), "Mode read failed");
    if (gyrMode != gyrModeNow)
    {
        ESP_LOGE(TAG, "Wrong gyroscope mode: 0x%02x instead of 0x%02x", gyrModeNow, gyrMode);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Internal function to start the accelerometer with the specified configuration
 * @note This function does not take the mutex, the caller should ensure thread safety
 *
 * @param dev Pointer to the BMI160 device structure
 * @param conf Pointer to the BMI160 configuration structure containing the desired accelerometer configuration
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t bmi160_startAcc(bmi160_t *dev, const bmi160_conf_t* const conf)
{
    CHECK_LOGE(bmi160_switch_accMode(dev, conf->accMode), "Switch mode for acc failed");

    CHECK_LOGE(bmi160_set_acc_range_internal(dev, conf->accRange), "Set acc range failed");

    CHECK_LOGE(bmi160_set_acc_conf_internal(dev, conf->accOdr, conf->accAvg, conf->accUs), "Set acc configuration failed");

    return ESP_OK;
}

/**
 * @brief Internal function to start the gyroscope with the specified configuration
 * @note This function does not take the mutex, the caller should ensure thread safety
 *
 * @param dev Pointer to the BMI160 device structure
 * @param conf Pointer to the BMI160 configuration structure containing the desired gyroscope configuration
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t bmi160_startGyr(bmi160_t *dev, const bmi160_conf_t* const conf)
{
    CHECK_LOGE(bmi160_switch_gyrMode(dev, conf->gyrMode), "Switch mode for gyr failed");

    CHECK_LOGE(bmi160_set_gyr_range_internal(dev, conf->gyrRange), "Set gyr range failed");

    CHECK_LOGE(bmi160_set_gyr_odr_internal(dev, conf->gyrOdr), "Set gyr odr failed");

    return ESP_OK;
}

/**
 * @brief Internal function to perform a software reset of the BMI160
 * @note This function does not take the mutex, the caller should ensure thread safety
 *
 * @param dev Pointer to the BMI160 device structure
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t bmi160_reset(bmi160_t *dev)
{
    CHECK_LOGE(bmi160_write_reg_internal(dev, BMI160_CMD, BMI160_CMD_SOFT_RESET), "Cmd reset failed"); // toggle software reset
    dev->accMode = BMI160_PMU_ACC_SUSPEND;
    dev->gyrMode = BMI160_PMU_GYR_SUSPEND;

    //delay 100ms
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}

/**
 * @brief Internal function to configure the interrupt output of the BMI160
 * @note This function does not take the mutex, the caller should ensure thread safety
 *
 * @param dev Pointer to the BMI160 device structure
 * @param intOutConf Pointer to the interrupt output configuration structure containing the desired interrupt output configuration
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t bmi160_int_config(bmi160_t *dev, const bmi160_int_out_conf_t* const intOutConf)
{
    //configure interrupt output
    uint8_t data = 0;
    CHECK_LOGE(bmi160_read_reg_internal(dev, BMI160_INT_OUT_CTRL, &data), "Read in_out_ctrl failed");
    data &= ~(0xfu << (intOutConf->intPin * 4u)); //clear bits
    data |= (uint8_t)(intOutConf->intEnable << ((intOutConf->intPin * 4u) + 3u)); //set enable bit
    data |= (uint8_t)(intOutConf->intOd << ((intOutConf->intPin * 4u) + 2u)); //set open-drain bit
    data |= (uint8_t)(intOutConf->intLevel << ((intOutConf->intPin * 4u) + 1u)); //set active high bit
    CHECK_LOGE(bmi160_write_reg_internal(dev, BMI160_INT_OUT_CTRL, data), "Write in_out_ctrl failed");

    return ESP_OK;
}

/**
 * @brief Internal function to enable an interrupt in the BMI160
 * @note This function does not take the mutex, the caller should ensure thread safety
 *
 * @param dev Pointer to the BMI160 device structure
 * @param reg Register address of the interrupt enable register to write to
 * @param mask Mask indicating which interrupt bits to enable
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
static esp_err_t bmi160_int_en(bmi160_t *dev, uint8_t reg, uint8_t mask)
{
    uint8_t data = 0;
    //enable interrupt
    CHECK_LOGE(bmi160_read_reg_internal(dev, reg, &data), "Read Enable interrupt flag failed");
    data |= mask; // enable interrupt
    CHECK_LOGE(bmi160_write_reg_internal(dev, reg, data), "Write Enable interrupt flag failed");

    return ESP_OK;
}
