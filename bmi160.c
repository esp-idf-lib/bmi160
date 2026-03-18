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


static bool is_acc_odr_fits_mode(bmi160_acc_odr_t odr, bmi160_pmu_acc_mode_t mode, bmi160_acc_lp_avg_t avg);
static bool is_gyr_mode_valid(bmi160_pmu_gyr_mode_t mode);
static bool is_acc_mode_valid(bmi160_pmu_acc_mode_t mode);
static bool is_acc_us_valid(bmi160_acc_us_t acc_us);
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


esp_err_t bmi160_read_reg(bmi160_t *dev, uint8_t reg, uint8_t *val)
{
    if (!dev || !val) return ESP_FAIL;
    esp_err_t ret = ESP_OK;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    ret = bmi160_read_reg_internal(dev, reg, val);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ret;
}

esp_err_t bmi160_read_reg_array(bmi160_t *dev, uint8_t reg, uint8_t *val, uint8_t num)
{
    if (!dev || !val) return ESP_FAIL;
    esp_err_t ret = ESP_OK;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    ret = bmi160_read_reg_array_internal(dev, reg, val, num);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ret;
}

esp_err_t bmi160_write_reg(bmi160_t *dev, uint8_t reg, uint8_t val)
{
    if (!dev) return ESP_FAIL;
    esp_err_t ret = ESP_OK;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    ret = bmi160_write_reg_internal(dev, reg, val);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ret;
}

esp_err_t bmi160_write_reg_array(bmi160_t *dev, uint8_t reg, uint8_t* val, uint8_t num)
{
    if (!dev) return ESP_FAIL;
    esp_err_t ret = ESP_OK;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    ret = bmi160_write_reg_array_internal(dev, reg, val, num);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ret;
}

static esp_err_t bmi160_read_reg_internal(bmi160_t *dev, uint8_t reg, uint8_t *val)
{
    esp_err_t ret = i2c_dev_read_reg(&dev->i2c_dev, reg, &val, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_dev_read_reg fail: reg=0x%02x, size=%d", reg, 1);
    }
    return ret;
}

static esp_err_t bmi160_read_reg_array_internal(bmi160_t *dev, uint8_t reg, uint8_t *val, uint8_t num)
{
    esp_err_t ret = i2c_dev_read_reg(&dev->i2c_dev, reg, val, num);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_dev_read_reg fail: reg=0x%02x, size=%d", reg, num);
    }
    return ret;
}

static esp_err_t bmi160_write_reg_internal(bmi160_t *dev, uint8_t reg, uint8_t val)
{
    esp_err_t ret = i2c_dev_write_reg(&dev->i2c_dev, reg, &val, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_dev_write_reg fail: reg=0x%02x, size=%d", reg, 1);
    }
    return ret;
}

static esp_err_t bmi160_write_reg_array_internal(bmi160_t *dev, uint8_t reg, const uint8_t* val, uint8_t num)
{
    esp_err_t ret = i2c_dev_write_reg(&dev->i2c_dev, reg, val, num);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_dev_write_reg fail: reg=0x%02x, size=%d", reg, num);
    }
    return ret;
}

esp_err_t bmi160_init(bmi160_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    if (!dev) return ESP_FAIL;
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
    if (!dev) return ESP_FAIL;
    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t bmi160_set_acc_range(bmi160_t *dev, bmi160_acc_range_t range)
{

    if (!dev) return ESP_FAIL;
    esp_err_t ret = ESP_OK;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    ret = bmi160_set_acc_range_internal(dev, range);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ret;

}

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

esp_err_t bmi160_set_gyr_range(bmi160_t *dev, bmi160_gyr_range_t range)
{
    if (!dev) return ESP_FAIL;
    esp_err_t ret = ESP_OK;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    ret = bmi160_set_gyr_range_internal(dev, range);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ret;
}

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

esp_err_t bmi160_set_acc_conf(bmi160_t *dev, bmi160_acc_odr_t odr, bmi160_acc_lp_avg_t avg, bmi160_acc_us_t acc_us)
{
    if (!dev) return ESP_FAIL;
    esp_err_t ret = ESP_OK;
    if (!is_acc_odr_fits_mode(odr, dev->accMode, avg))
    {
        ESP_LOGE(TAG, "Invalid odr (%d) or avg(%d) for accMode (%d)", odr, avg, dev->accMode);
        return ESP_FAIL;
    }
    if (!is_acc_us_valid(acc_us))
    {
        ESP_LOGE(TAG, "Invalid acc_us (%d) ", acc_us);
        return ESP_FAIL;
    }

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_set_acc_conf_internal(dev, odr, avg, acc_us), "Acc conf set failed");
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ret;

}

static esp_err_t bmi160_set_acc_conf_internal(bmi160_t *dev, bmi160_acc_odr_t odr, bmi160_acc_lp_avg_t avg, bmi160_acc_us_t acc_us)
{
    uint8_t accConf = (odr & 0x0Fu) | ((avg & 0x07u) << 4) | ((acc_us & 0x1u) << 7);
    esp_err_t ret = bmi160_write_reg_internal(dev, BMI160_ACC_CONF,  accConf);  // Set Accel CONF
    if (ESP_OK == ret)
    {
        dev->accConf = accConf;
        dev->accOdr = odr;
    }

    return ret;
}

esp_err_t bmi160_set_gyr_odr(bmi160_t *dev, bmi160_gyr_odr_t odr)
{
    if (!dev) return ESP_FAIL;
    esp_err_t ret = ESP_OK;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    ret = bmi160_set_gyr_odr_internal(dev, odr);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ret;

}

static esp_err_t bmi160_set_gyr_odr_internal(bmi160_t *dev, bmi160_gyr_odr_t odr)
{
    esp_err_t ret = bmi160_write_reg_internal(dev, BMI160_GYR_CONF, odr);  // Set Gyro ODR
    if (ESP_OK == ret)
    {
        dev->gyrOdr = odr;
    }

    return ret;
}

esp_err_t bmi160_read_data(bmi160_t *dev, bmi160_result_t *result)
{
    if (!dev || !result) return ESP_FAIL;
    esp_err_t ret = ESP_OK;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    ret = bmi160_read_data_internal(dev, result);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ret;
}

static esp_err_t bmi160_read_data_internal(bmi160_t *dev, bmi160_result_t *result)
{
    uint8_t rawData[12];
    esp_err_t ret = bmi160_read_reg_array_internal(dev, BMI160_GYR_X_L, rawData, 12);
    if (ESP_OK != ret)
    {
        return ret;
    }

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

static bool is_acc_mode_valid(bmi160_pmu_acc_mode_t mode)
{
    switch (mode)
    {
        case BMI160_PMU_ACC_SUSPEND:
        case BMI160_PMU_ACC_NORMAL:
        case BMI160_PMU_ACC_LOW_POWER:
            return true;
        default:
            return false;
    }
}

static bool is_gyr_mode_valid(bmi160_pmu_gyr_mode_t mode)
{
    switch (mode)
    {
        case BMI160_PMU_GYR_SUSPEND:
        case BMI160_PMU_GYR_NORMAL:
        case BMI160_PMU_GYR_FAST_STARTUP:
            return true;
        default:
            return false;
    }
}

static bool is_acc_us_valid(bmi160_acc_us_t acc_us)
{
    switch (acc_us)
    {
        case BMI160_ACC_US_OFF:
        case BMI160_ACC_US_ON:
            return true;
        default:
            return false;
    }
}

static bool is_acc_odr_fits_mode(bmi160_acc_odr_t odr, bmi160_pmu_acc_mode_t mode, bmi160_acc_lp_avg_t avg)
{
    bool result = false;
    switch (mode)
    {
        case BMI160_PMU_ACC_SUSPEND:
            result = true;
            break;
        case BMI160_PMU_ACC_NORMAL:
            result = (odr >= BMI160_ACC_ODR_12_5HZ);
            break;
        case BMI160_PMU_ACC_LOW_POWER:
            if ((odr == BMI160_ACC_ODR_400HZ) && (avg <= BMI160_ACC_LP_AVG_2))
            {
                result = true;
            }
            else if ((odr == BMI160_ACC_ODR_200HZ) && (avg <= BMI160_ACC_LP_AVG_4))
            {
                result = true;
            }
            else if ((odr == BMI160_ACC_ODR_100HZ) && (avg <= BMI160_ACC_LP_AVG_8))
            {
                result = true;
            }
            else if ((odr == BMI160_ACC_ODR_50HZ) && (avg <= BMI160_ACC_LP_AVG_16))
            {
                result = true;
            }
            else if ((odr == BMI160_ACC_ODR_25HZ) && (avg <= BMI160_ACC_LP_AVG_32))
            {
                result = true;
            }
            else if ((odr == BMI160_ACC_ODR_12_5HZ) && (avg <= BMI160_ACC_LP_AVG_64))
            {
                result = true;
            }
            else if ((odr <= BMI160_ACC_ODR_6_25HZ) && (avg <= BMI160_ACC_LP_AVG_128))
            {
                result = true;
            }
            else
            {
                result = false;
            }
            break;
        default:
            result = false;
            break;
    }

    return result;
}

static esp_err_t bmi160_switch_accMode(bmi160_t *dev, bmi160_pmu_acc_mode_t accMode)
{
    if (ESP_OK != bmi160_write_reg_internal(dev, BMI160_CMD, accMode))
    {
        ESP_LOGE(TAG, "Mode %d not set", accMode);
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t pmu_status = 0;
    if (ESP_OK != bmi160_read_reg_internal(dev, BMI160_PMU_STATUS, &pmu_status))
    {
        ESP_LOGE(TAG, "Pmu status read failed");
        return ESP_FAIL;
    }
    if (((pmu_status & 0x30) >> 4) != (accMode & 0x3))
    {
        ESP_LOGE(TAG, "Accelerometer PMU status: 0x%02x", (pmu_status & 0x30) >> 4);
        return ESP_FAIL;
    }

    dev->accMode = accMode;
    return ESP_OK;
}

static esp_err_t bmi160_switch_gyrMode(bmi160_t *dev, bmi160_pmu_gyr_mode_t gyrMode)
{
    if (ESP_OK != bmi160_write_reg_internal(dev, BMI160_CMD, gyrMode))
    {
        ESP_LOGE(TAG, "Cmd gyrMode failed");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t pmu_status = 0;
    if (ESP_OK != bmi160_read_reg_internal(dev, BMI160_PMU_STATUS, &pmu_status))
    {
        ESP_LOGE(TAG, "Pmu status read failed");
        return ESP_FAIL;
    }
    if (((pmu_status & 0x0C) >> 2) != (gyrMode & 0x3))
    {
        ESP_LOGE(TAG, "Gyroscope PMU status: 0x%02x", (pmu_status & 0x0C) >> 2);
        return ESP_FAIL;
    }
    dev->gyrMode = gyrMode;

    return ESP_OK;
}

static esp_err_t bmi160_startAcc(bmi160_t *dev, const bmi160_conf_t* const conf)
{
    if (ESP_OK != bmi160_switch_accMode(dev, conf->accMode))
    {
        ESP_LOGE(TAG, "Switch mode for acc failed");
        return ESP_FAIL;
    }

    if (ESP_OK != bmi160_set_acc_range_internal(dev, conf->accRange))
    {
        return ESP_FAIL;
    }
    if (ESP_FAIL == bmi160_set_acc_conf_internal(dev, conf->accOdr, conf->accAvg, conf->accUs))
    {
        ESP_LOGE(TAG, "Invalid Accelerometer configuration");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t bmi160_startGyr(bmi160_t *dev, const bmi160_conf_t* const conf)
{
    if (ESP_OK != bmi160_switch_gyrMode(dev, conf->gyrMode))
    {
        ESP_LOGE(TAG, "Switch mode for gyr failed");
        return ESP_FAIL;
    }
    if (ESP_FAIL == bmi160_set_gyr_range_internal(dev, conf->gyrRange))
    {
        return ESP_FAIL;
    }
    if (ESP_FAIL == bmi160_set_gyr_odr_internal(dev, conf->gyrOdr))
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t bmi160_start(bmi160_t *dev, const bmi160_conf_t* const conf)
{
    if (!dev || !conf) return ESP_FAIL;
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


    //validate parameters
    if (!is_acc_mode_valid(conf->accMode))
    {
        ESP_LOGE(TAG, "Invalid Accelerometer Mode");
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    if (!is_gyr_mode_valid(conf->gyrMode))
    {
        ESP_LOGE(TAG, "Invalid Gyroscope Mode");
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    if (!is_acc_odr_fits_mode(conf->accOdr, conf->accMode, conf->accAvg))
    {
        ESP_LOGE(TAG, "Invalid Accelerometer ODR for the mode");
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    if (!is_acc_us_valid(conf->accUs))
    {
        ESP_LOGE(TAG, "Invalid accelerometer undersampling configuration");
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }

    //reset device
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_CMD, BMI160_CMD_SOFT_RESET), "Cmd reset failed"); // toggle software reset

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    //delay 100ms
    vTaskDelay(pdMS_TO_TICKS(100));

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

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
    if (!dev) return ESP_FAIL;
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

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

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
    if (!dev) return ESP_FAIL;
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
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_CMD, BMI160_CMD_SOFT_RESET), "Cmd reset failed"); // toggle software reset

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    //delay 100ms
    vTaskDelay(pdMS_TO_TICKS(100));

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    /* 1. acceletrometer */
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_set_acc_range_internal(dev, BMI160_ACC_RANGE_8G), "bmi160_set_acc_range_internal failed");
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_ACC_CONF, 0x2C), "Set Acc Conf failed");  // Set Accel ODR to 1600hz, BWP mode to Oversample 2, acc_us = 0

    // test negative direction
    ESP_LOGD(TAG, "Accel self test sign 0");
    uint8_t reg = (0x01 << 0) | (0x00 << 2) | (0x01 << 3); // acc_self_test_en = 1, acc_self_test_sign = 0, acc_self_test_amp = 1
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_SELF_TEST, reg), "Start self test failed");

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    vTaskDelay(pdMS_TO_TICKS(100));
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
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

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    vTaskDelay(pdMS_TO_TICKS(100));
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
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

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    vTaskDelay(pdMS_TO_TICKS(100));
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

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
    if (!dev || !intOutConf) return ESP_FAIL;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t data = 0;

    //configure interrupt output
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_read_reg_internal(dev, BMI160_INT_OUT_CTRL, &data), "Read in_out_ctrl failed");
    data &= ~(0xfu << (intOutConf->intPin * 4u)); //clear bits
    data |= (uint8_t)(intOutConf->intEnable << ((intOutConf->intPin * 4u) + 3u)); //set enable bit
    data |= (uint8_t)(intOutConf->intOd << ((intOutConf->intPin * 4u) + 2u)); //set open-drain bit
    data |= (uint8_t)(intOutConf->intLevel << ((intOutConf->intPin * 4u) + 1u)); //set active high bit
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_INT_OUT_CTRL, data), "Write in_out_ctrl failed");

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
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_read_reg_internal(dev, BMI160_INT_EN_1, &data), "Read Enable interrupt flag failed");
    data |= (1u << 4); // enable data ready interrupt
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_INT_EN_1, data), "Write Enable interrupt flag failed");

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t bmi160_enable_step_counter(bmi160_t *dev, bmi160_step_counter_mode_t mode)
{
    if (!dev) return ESP_FAIL;
    esp_err_t ret = ESP_OK;
    //enable step counter
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_PMU_TRIGGER, 0x01), "Write PMU trigger failed");
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    vTaskDelay(pdMS_TO_TICKS(100));

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
            ESP_LOGE(TAG, "Step counter mode out of range");
            return ESP_FAIL;
    }

    config[1] |= (1u << 3); //enable step counter

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    //configure step counter
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_array_internal(dev, BMI160_STEP_CONF_0, config, 2), "Write step conf failed");
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ret;
}

esp_err_t bmi160_read_step_counter(bmi160_t *dev, uint16_t *stepCounter)
{
    if (!dev || !stepCounter) return ESP_FAIL;
    uint8_t data[2];
    esp_err_t ret = ESP_OK;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_read_reg_array_internal(dev, BMI160_STEP_CNT_0, data, 2), "Read step cnt failed");

    *stepCounter = (uint16_t)((data[1] << 8) | data[0]);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ret;
}

esp_err_t bmi160_reset_step_counter(bmi160_t *dev)
{
    if (!dev) return ESP_FAIL;
    return bmi160_write_reg(dev, BMI160_CMD, BMI160_CMD_STEP_RESET); // reset step counter
}

esp_err_t bmi160_enable_int_step(bmi160_t *dev, const bmi160_int_out_conf_t* const intOutConf)
{
    if (!dev || !intOutConf) return ESP_FAIL;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t data = 0;

    //configure interrupt output
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_read_reg_internal(dev, BMI160_INT_OUT_CTRL, &data), "Read in_out_ctrl failed");
    data &= ~(0xfu << (intOutConf->intPin * 4u)); //clear bits
    data |= (uint8_t)(intOutConf->intEnable << ((intOutConf->intPin * 4u) + 3u)); //set enable bit
    data |= (uint8_t)(intOutConf->intOd << ((intOutConf->intPin * 4u) + 2u)); //set open-drain bit
    data |= (uint8_t)(intOutConf->intLevel << ((intOutConf->intPin * 4u) + 1u)); //set active high bit
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_INT_OUT_CTRL, data), "Write in_out_ctrl failed");

    //map interrupt step detection
    data = (1u << 0); //set bit for step detection
    if (intOutConf->intPin == BMI160_PIN_INT1)
    {
        I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_INT_MAP_0, data), "Write int map 0 failed"); //map step detection interrupt to INT1
    }
    else
    {
        I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_INT_MAP_1, data), "Write int map 1 failed"); //map step detection interrupt to INT2
    }

    //enable interrupt
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_read_reg_internal(dev, BMI160_INT_EN_2, &data), "Read Enable interrupt flag failed");
    data |= (1u << 3); // enable step detection interrupt
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_INT_EN_2, data), "Write Enable interrupt flag failed");

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}


esp_err_t bmi160_switch_mode(bmi160_t *dev, bmi160_pmu_acc_mode_t accMode, bmi160_pmu_gyr_mode_t gyrMode)
{
    if (!dev) return ESP_FAIL;
    //validate parameters
    if (!is_acc_mode_valid(accMode))
    {
        ESP_LOGE(TAG, "Invalid Accelerometer Mode");
        return ESP_FAIL;
    }
    if (!is_gyr_mode_valid(gyrMode))
    {
        ESP_LOGE(TAG, "Invalid Gyroscope Mode");
        return ESP_FAIL;
    }

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
    if (!dev || !tapConf) return ESP_FAIL;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    dev->tapMode = tapConf->tapMode;
    uint8_t data = 0;
    data |= (uint8_t)(tapConf->tapQuiet << 7); //set quiet bit
    data |= (uint8_t)(tapConf->tapShock << 6); //set shock bit
    data |= (uint8_t)(tapConf->tapDur << 2); //set duration bits
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_INT_TAP_0, data), "Write int map 0 failed");

    data = (uint8_t)(tapConf->tapTh); //set threshold bits
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_INT_TAP_1, data), "Write int map 1 failed");
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

esp_err_t bmi160_enable_int_tap(bmi160_t *dev, const bmi160_int_out_conf_t* const intOutConf)
{
    if (!dev || !intOutConf) return ESP_FAIL;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t data = 0;

    //configure interrupt output
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_read_reg_internal(dev, BMI160_INT_OUT_CTRL, &data), "Read in_out_ctrl failed");
    data &= ~(0xfu << (intOutConf->intPin * 4u)); //clear bits
    data |= (uint8_t)(intOutConf->intEnable << ((intOutConf->intPin * 4u) + 3u)); //set enable bit
    data |= (uint8_t)(intOutConf->intOd << ((intOutConf->intPin * 4u) + 2u)); //set open-drain bit
    data |= (uint8_t)(intOutConf->intLevel << ((intOutConf->intPin * 4u) + 1u)); //set active high bit
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_INT_OUT_CTRL, data), "Write in_out_ctrl failed");

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
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_read_reg_internal(dev, BMI160_INT_EN_0, &data), "Read Enable interrupt flag failed");
    if (dev->tapMode == BMI160_TAP_MODE_SINGLE)
    {
        data |= (1u << 5); //set bit for single tap detection
    }
    else
    {
        data |= (1u << 4); //set bit for double tap detection
    }
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, bmi160_write_reg_internal(dev, BMI160_INT_EN_0, data), "Write Enable interrupt flag failed");
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t bmi160_read_tap_orient(bmi160_t *dev, uint8_t *orient)
{
    if (!dev || !orient) return ESP_FAIL;
    //read int_status_2
    uint8_t data;
    if (ESP_OK != bmi160_read_reg(dev, BMI160_INT_STATUS_2, &data))
    {
        return ESP_FAIL;
    }
    *orient = data;
    return ESP_OK;
}
