// ina219.h - Updated to use new I2C driver API
/*
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __INA219_H__
#define __INA219_H__

#include <esp_err.h>
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define INA219_ADDR_GND_GND 0x40 //!< I2C address, A1 pin - GND, A0 pin - GND
#define INA219_ADDR_GND_VS  0x41 //!< I2C address, A1 pin - GND, A0 pin - VS+
#define INA219_ADDR_GND_SDA 0x42 //!< I2C address, A1 pin - GND, A0 pin - SDA
#define INA219_ADDR_GND_SCL 0x43 //!< I2C address, A1 pin - GND, A0 pin - SCL
#define INA219_ADDR_VS_GND  0x44 //!< I2C address, A1 pin - VS+, A0 pin - GND
#define INA219_ADDR_VS_VS   0x45 //!< I2C address, A1 pin - VS+, A0 pin - VS+
#define INA219_ADDR_VS_SDA  0x46 //!< I2C address, A1 pin - VS+, A0 pin - SDA
#define INA219_ADDR_VS_SCL  0x47 //!< I2C address, A1 pin - VS+, A0 pin - SCL
#define INA219_ADDR_SDA_GND 0x48 //!< I2C address, A1 pin - SDA, A0 pin - GND
#define INA219_ADDR_SDA_VS  0x49 //!< I2C address, A1 pin - SDA, A0 pin - VS+
#define INA219_ADDR_SDA_SDA 0x4a //!< I2C address, A1 pin - SDA, A0 pin - SDA
#define INA219_ADDR_SDA_SCL 0x4b //!< I2C address, A1 pin - SDA, A0 pin - SCL
#define INA219_ADDR_SCL_GND 0x4c //!< I2C address, A1 pin - SCL, A0 pin - GND
#define INA219_ADDR_SCL_VS  0x4d //!< I2C address, A1 pin - SCL, A0 pin - VS+
#define INA219_ADDR_SCL_SDA 0x4e //!< I2C address, A1 pin - SCL, A0 pin - SDA
#define INA219_ADDR_SCL_SCL 0x4f //!< I2C address, A1 pin - SCL, A0 pin - SCL

/**
 * Bus voltage range
 */
typedef enum
{
    INA219_BUS_RANGE_16V = 0, //!< 16V FSR
    INA219_BUS_RANGE_32V      //!< 32V FSR (default)
} ina219_bus_voltage_range_t;

/**
 * PGA gain for shunt voltage
 */
typedef enum
{
    INA219_GAIN_1 = 0,     //!< Gain: 1, Range: +-40 mV
    INA219_GAIN_0_5,       //!< Gain: 1/2, Range: +-80 mV
    INA219_GAIN_0_25,      //!< Gain: 1/4, Range: +-160 mV
    INA219_GAIN_0_125,     //!< Gain: 1/8, Range: +-320 mV (default)
    INA219_GAIN_0_320MV = 3 //!< Alias for INA219_GAIN_0_125
} ina219_gain_t;

/**
 * ADC resolution/averaging
 */
typedef enum
{
    INA219_RES_9BIT_1S    = 0,  //!< 9 bit, 1 sample, conversion time 84 us
    INA219_RES_10BIT_1S   = 1,  //!< 10 bit, 1 sample, conversion time 148 us
    INA219_RES_11BIT_1S   = 2,  //!< 11 bit, 1 sample, conversion time 276 us
    INA219_RES_12BIT_1S   = 3,  //!< 12 bit, 1 sample, conversion time 532 us (default)
    INA219_RES_12BIT_2S   = 9,  //!< 12 bit, 2 samples, conversion time 1.06 ms
    INA219_RES_12BIT_4S   = 10, //!< 12 bit, 4 samples, conversion time 2.13 ms
    INA219_RES_12BIT_8S   = 11, //!< 12 bit, 8 samples, conversion time 4.26 ms
    INA219_RES_12BIT_16S  = 12, //!< 12 bit, 16 samples, conversion time 8.51 ms
    INA219_RES_12BIT_32S  = 13, //!< 12 bit, 32 samples, conversion time 17.02 ms
    INA219_RES_12BIT_64S  = 14, //!< 12 bit, 64 samples, conversion time 34.05 ms
    INA219_RES_12BIT_128S = 15, //!< 12 bit, 128 samples, conversion time 68.1 ms
} ina219_resolution_t;

/**
 * Operating mode
 */
typedef enum
{
    INA219_MODE_POWER_DOWN = 0, //!< Power-down
    INA219_MODE_TRIG_SHUNT,     //!< Shunt voltage, triggered
    INA219_MODE_TRIG_BUS,       //!< Bus voltage, triggered
    INA219_MODE_TRIG_SHUNT_BUS, //!< Shunt and bus, triggered
    INA219_MODE_DISABLED,       //!< ADC off (disabled)
    INA219_MODE_CONT_SHUNT,     //!< Shunt voltage, continuous
    INA219_MODE_CONT_BUS,       //!< Bus voltage, continuous
    INA219_MODE_CONT_SHUNT_BUS  //!< Shunt and bus, continuous (default)
} ina219_mode_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_master_dev_handle_t i2c_dev;  // New I2C device handle
    uint8_t i2c_addr;                 // Device address
    uint16_t config;
    float i_lsb, p_lsb;
    float shunt;                      // Shunt resistance in mOhm
} ina219_t;

/**
 * @brief Add INA219 device to I2C bus
 *
 * @param bus_handle I2C bus handle from i2c_new_master_bus()
 * @param dev Device descriptor to initialize
 * @param addr Device I2C address
 * @return `ESP_OK` on success
 */
esp_err_t ina219_init_desc(i2c_master_bus_handle_t bus_handle, ina219_t *dev, uint8_t addr);

/**
 * @brief Remove device from I2C bus
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ina219_free_desc(ina219_t *dev);

/**
 * @brief Init device
 *
 * Read current device configuration into `dev->config`
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ina219_init(ina219_t *dev);

/**
 * @brief Reset device
 *
 * Same as power-on reset. Resets all registers to default values.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ina219_reset(ina219_t *dev);

/**
 * @brief Set device configuration
 *
 * @param dev Device descriptor
 * @param u_range Bus voltage range
 * @param gain Shunt voltage gain
 * @param u_res Bus voltage resolution and averaging
 * @param i_res Shunt voltage resolution and averaging
 * @param mode Device operational mode
 * @return `ESP_OK` on success
 */
esp_err_t ina219_configure(ina219_t *dev, ina219_bus_voltage_range_t u_range,
                           ina219_gain_t gain, ina219_resolution_t u_res,
                           ina219_resolution_t i_res, ina219_mode_t mode);

/**
 * @brief Get bus voltage range
 *
 * @param dev Device descriptor
 * @param[out] range Bus voltage range
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_bus_voltage_range(ina219_t *dev, ina219_bus_voltage_range_t *range);

/**
 * @brief Get shunt voltage gain
 *
 * @param dev Device descriptor
 * @param[out] gain Shunt voltage gain
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_gain(ina219_t *dev, ina219_gain_t *gain);

/**
 * @brief Get bus voltage resolution and averaging
 *
 * @param dev Device descriptor
 * @param[out] res Bus voltage resolution and averaging
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_bus_voltage_resolution(ina219_t *dev, ina219_resolution_t *res);

/**
 * @brief Get shunt voltage resolution and averaging
 *
 * @param dev Device descriptor
 * @param[out] res Shunt voltage resolution and averaging
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_shunt_voltage_resolution(ina219_t *dev, ina219_resolution_t *res);

/**
 * @brief Get operating mode
 *
 * @param dev Device descriptor
 * @param[out] mode Operating mode
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_mode(ina219_t *dev, ina219_mode_t *mode);

/**
 * @brief Calibrate the device
 *
 * @param dev Device descriptor
 * @param max_current_ma Maximum expected current in milliamps
 * @param shunt_mohm Shunt resistor value in milliohms
 * @return `ESP_OK` on success
 */
esp_err_t ina219_calibrate(ina219_t *dev, float max_current_ma, float shunt_mohm);

/**
 * @brief Trigger single conversion
 *
 * Function will return an error if current operating
 * mode is not `INA219_MODE_TRIG_SHUNT`/`INA219_MODE_TRIG_BUS`/`INA219_MODE_TRIG_SHUNT_BUS`
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ina219_trigger(ina219_t *dev);

/**
 * @brief Read bus voltage
 *
 * @param dev Device descriptor
 * @param[out] voltage Bus voltage, V
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_bus_voltage(ina219_t *dev, float *voltage);

/**
 * @brief Read shunt voltage
 *
 * @param dev Device descriptor
 * @param[out] voltage Shunt voltage, mV
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_shunt_voltage(ina219_t *dev, float *voltage);

/**
 * @brief Read current
 *
 * This function works properly only after calibration.
 *
 * @param dev Device descriptor
 * @param[out] current Current, mA
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_current(ina219_t *dev, float *current);

/**
 * @brief Read power
 *
 * This function works properly only after calibration.
 *
 * @param dev Device descriptor
 * @param[out] power Power, mW
 * @return `ESP_OK` on success
 */
esp_err_t ina219_get_power(ina219_t *dev, float *power);

#ifdef __cplusplus
}
#endif

#endif /* __INA219_H__ */

// ===============================================
// ina219.c - Updated to use new I2C driver API
// ===============================================

#include <esp_log.h>
#include <math.h>
#include <string.h>
#include "ina219.h"

#define I2C_TIMEOUT_MS 1000

static const char *TAG = "ina219";

// INA219 Registers
#define REG_CONFIG      0x00
#define REG_SHUNT_U     0x01
#define REG_BUS_U       0x02
#define REG_POWER       0x03
#define REG_CURRENT     0x04
#define REG_CALIBRATION 0x05

// Configuration register bits
#define BIT_RST   15
#define BIT_BRNG  13
#define BIT_PG0   11
#define BIT_BADC0 7
#define BIT_SADC0 3
#define BIT_MODE  0

// Configuration masks
#define MASK_PG   (3 << BIT_PG0)
#define MASK_BADC (0xf << BIT_BADC0)
#define MASK_SADC (0xf << BIT_SADC0)
#define MASK_MODE (7 << BIT_MODE)
#define MASK_BRNG (1 << BIT_BRNG)

#define DEF_CONFIG 0x399f

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static const float u_shunt_max[] =
{
    [INA219_GAIN_1]     = 0.04,
    [INA219_GAIN_0_5]   = 0.08,
    [INA219_GAIN_0_25]  = 0.16,
    [INA219_GAIN_0_125] = 0.32,
};

static esp_err_t read_reg_16(ina219_t *dev, uint8_t reg, uint16_t *val)
{
    CHECK_ARG(dev && val);
    
    uint8_t data[2];
    esp_err_t ret;
    
    // Write register address
    ret = i2c_master_transmit(dev->i2c_dev, &reg, 1, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register address: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Read register data
    ret = i2c_master_receive(dev->i2c_dev, data, 2, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register data: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Convert from big-endian
    *val = (data[0] << 8) | data[1];
    
    return ESP_OK;
}

static esp_err_t write_reg_16(ina219_t *dev, uint8_t reg, uint16_t val)
{
    CHECK_ARG(dev);
    
    uint8_t data[3];
    data[0] = reg;
    data[1] = (val >> 8) & 0xFF;  // MSB
    data[2] = val & 0xFF;          // LSB
    
    esp_err_t ret = i2c_master_transmit(dev->i2c_dev, data, 3, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02x: %s", reg, esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

static esp_err_t read_conf_bits(ina219_t *dev, uint16_t mask, uint8_t bit, uint16_t *res)
{
    uint16_t raw;
    CHECK(read_reg_16(dev, REG_CONFIG, &raw));
    *res = (raw & mask) >> bit;
    return ESP_OK;
}

esp_err_t ina219_init_desc(i2c_master_bus_handle_t bus_handle, ina219_t *dev, uint8_t addr)
{
    CHECK_ARG(bus_handle && dev);
    
    if (addr < INA219_ADDR_GND_GND || addr > INA219_ADDR_SCL_SCL) {
        ESP_LOGE(TAG, "Invalid I2C address: 0x%02x", addr);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Configure device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,  // INA219 supports up to 400kHz
    };
    
    // Add device to bus
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    dev->i2c_addr = addr;
    dev->shunt = 100;  // Default 100 mOhm
    
    ESP_LOGI(TAG, "Device descriptor initialized for address 0x%02x", addr);
    return ESP_OK;
}

esp_err_t ina219_free_desc(ina219_t *dev)
{
    CHECK_ARG(dev);
    
    if (dev->i2c_dev) {
        esp_err_t ret = i2c_master_bus_rm_device(dev->i2c_dev);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to remove device from bus: %s", esp_err_to_name(ret));
            return ret;
        }
        dev->i2c_dev = NULL;
    }
    
    return ESP_OK;
}

esp_err_t ina219_init(ina219_t *dev)
{
    CHECK_ARG(dev);
    
    CHECK(read_reg_16(dev, REG_CONFIG, &dev->config));
    ESP_LOGD(TAG, "Initialize, config: 0x%04x", dev->config);
    
    return ESP_OK;
}

esp_err_t ina219_reset(ina219_t *dev)
{
    CHECK_ARG(dev);
    
    CHECK(write_reg_16(dev, REG_CONFIG, 1 << BIT_RST));
    dev->config = DEF_CONFIG;
    
    // Wait for reset to complete
    vTaskDelay(pdMS_TO_TICKS(1));
    
    ESP_LOGD(TAG, "Device reset");
    return ESP_OK;
}

esp_err_t ina219_configure(ina219_t *dev, ina219_bus_voltage_range_t u_range,
                           ina219_gain_t gain, ina219_resolution_t u_res,
                           ina219_resolution_t i_res, ina219_mode_t mode)
{
    CHECK_ARG(dev);
    CHECK_ARG(u_range <= INA219_BUS_RANGE_32V);
    CHECK_ARG(gain <= INA219_GAIN_0_125);
    CHECK_ARG(u_res <= INA219_RES_12BIT_128S);
    CHECK_ARG(i_res <= INA219_RES_12BIT_128S);
    CHECK_ARG(mode <= INA219_MODE_CONT_SHUNT_BUS);
    
    dev->config = (u_range << BIT_BRNG) |
                  (gain << BIT_PG0) |
                  (u_res << BIT_BADC0) |
                  (i_res << BIT_SADC0) |
                  (mode << BIT_MODE);
    
    ESP_LOGD(TAG, "Config: 0x%04x", dev->config);
    
    return write_reg_16(dev, REG_CONFIG, dev->config);
}

esp_err_t ina219_get_bus_voltage_range(ina219_t *dev, ina219_bus_voltage_range_t *range)
{
    CHECK_ARG(dev && range);
    *range = 0;
    return read_conf_bits(dev, MASK_BRNG, BIT_BRNG, (uint16_t *)range);
}

esp_err_t ina219_get_gain(ina219_t *dev, ina219_gain_t *gain)
{
    CHECK_ARG(dev && gain);
    *gain = 0;
    return read_conf_bits(dev, MASK_PG, BIT_PG0, (uint16_t *)gain);
}

esp_err_t ina219_get_bus_voltage_resolution(ina219_t *dev, ina219_resolution_t *res)
{
    CHECK_ARG(dev && res);
    *res = 0;
    return read_conf_bits(dev, MASK_BADC, BIT_BADC0, (uint16_t *)res);
}

esp_err_t ina219_get_shunt_voltage_resolution(ina219_t *dev, ina219_resolution_t *res)
{
    CHECK_ARG(dev && res);
    *res = 0;
    return read_conf_bits(dev, MASK_SADC, BIT_SADC0, (uint16_t *)res);
}

esp_err_t ina219_get_mode(ina219_t *dev, ina219_mode_t *mode)
{
    CHECK_ARG(dev && mode);
    *mode = 0;
    return read_conf_bits(dev, MASK_MODE, BIT_MODE, (uint16_t *)mode);
}

esp_err_t ina219_calibrate(ina219_t *dev, float max_current_ma, float shunt_mohm)
{
    CHECK_ARG(dev);
    CHECK_ARG(max_current_ma > 0);
    CHECK_ARG(shunt_mohm > 0);
    
    dev->shunt = shunt_mohm;
    
    // Get current gain setting
    ina219_gain_t gain;
    CHECK(ina219_get_gain(dev, &gain));
    
    // Calculate Current LSB
    float current_lsb = max_current_ma / 32768.0;
    
    // Round to nearest 0.0001
    current_lsb = ceilf(current_lsb * 10000) / 10000;
    dev->i_lsb = current_lsb;
    
    // Calculate Power LSB (20 * Current LSB)
    dev->p_lsb = 20 * current_lsb;

    // Calculate calibration value
    // Cal = 0.04096 / (Current_LSB * R_shunt)
    // Note: 0.04096 is internal fixed value, Current_LSB in A, R_shunt in Ohm
    uint16_t cal = (uint16_t)(40960.0 / (current_lsb * (shunt_mohm / 1000.0)));

    ESP_LOGD(TAG, "Calibration: shunt=%.3f mOhm, max_current=%.1f mA, cal=0x%04x, current_lsb=%.6f mA, power_lsb=%.6f mW", shunt_mohm, max_current_ma, cal, current_lsb, dev->p_lsb);

    return write_reg_16(dev, REG_CALIBRATION, cal);
}

esp_err_t ina219_trigger(ina219_t *dev)
{
    CHECK_ARG(dev);
    
    uint16_t mode = (dev->config & MASK_MODE) >> BIT_MODE;
    if (mode < INA219_MODE_TRIG_SHUNT || mode > INA219_MODE_TRIG_SHUNT_BUS) {
        ESP_LOGE(TAG, "Could not trigger conversion in this mode: %d", mode);
        return ESP_ERR_INVALID_STATE;
    }
    
    return write_reg_16(dev, REG_CONFIG, dev->config);
}

esp_err_t ina219_get_bus_voltage(ina219_t *dev, float *voltage)
{
    CHECK_ARG(dev && voltage);
    
    uint16_t raw;
    CHECK(read_reg_16(dev, REG_BUS_U, &raw));
    
    // Overflow flag is bit 0, voltage data is bits 15-3
    if (raw & 0x01) {
        ESP_LOGW(TAG, "Bus voltage overflow");
    }
    
    // Bus voltage is in bits 15-3, LSB = 4mV
    *voltage = ((raw >> 3) * 4) / 1000.0;  // Convert to V
    
    return ESP_OK;
}

esp_err_t ina219_get_shunt_voltage(ina219_t *dev, float *voltage)
{
    CHECK_ARG(dev && voltage);
    
    int16_t raw;
    CHECK(read_reg_16(dev, REG_SHUNT_U, (uint16_t *)&raw));
    
    // Shunt voltage register is signed, LSB = 10uV
    *voltage = raw * 0.01;  // Convert to mV
    
    return ESP_OK;
}

esp_err_t ina219_get_current(ina219_t *dev, float *current)
{
    CHECK_ARG(dev && current);
    
    int16_t raw;
    CHECK(read_reg_16(dev, REG_CURRENT, (uint16_t *)&raw));
    
    // Current register is signed, value in Current LSB units
    *current = raw * dev->i_lsb;  // Result in mA

    return ESP_OK;
}

esp_err_t ina219_get_power(ina219_t *dev, float *power)
{
    CHECK_ARG(dev && power);

    int16_t raw;
    CHECK(read_reg_16(dev, REG_POWER, (uint16_t *)&raw));
    
    // Power register value in Power LSB units
    *power = raw * dev->p_lsb; // Result in mW

    return ESP_OK;
}