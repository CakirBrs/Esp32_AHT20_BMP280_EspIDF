#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// I2C Configuration
#define I2C_MASTER_SCL_IO           22    
#define I2C_MASTER_SDA_IO           21    
#define I2C_MASTER_NUM              0     
#define I2C_MASTER_FREQ_HZ          400000  // Reduced to 100kHz for better stability

// AHT20 Configuration
#define AHT20_SENSOR_ADDR           0x38
#define AHT20_INIT_CMD             0xBE
#define AHT20_TRIGGER_MEASUREMENT  0xAC
#define AHT20_STATUS_CMD          0x71

// BMP280 Configuration
#define BMP280_SENSOR_ADDR         0x77  // Try 0x76 if 0x77 doesn't work
#define BMP280_REG_ID             0xD0   // ID register
#define BMP280_CHIP_ID            0x58   // Expected chip ID
#define BMP280_REG_CONTROL         0xF4
#define BMP280_REG_CONFIG          0xF5
#define BMP280_REG_CALIB           0x88
#define BMP280_REG_TEMP           0xFA
#define BMP280_REG_PRESSURE       0xF7

static const char *TAG = "Sensors";

// BMP280 calibration data structure
struct bmp280_calib_data {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    int32_t  t_fine;
};

static struct bmp280_calib_data bmp280_cal;

// Initialize I2C
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Scan I2C bus
static void i2c_scan(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t i = 0; i < 128; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at address 0x%02x", i);
        }
    }
}

// Initialize AHT20
static esp_err_t aht20_init(void)
{
    // Soft reset
    vTaskDelay(pdMS_TO_TICKS(40));  // Power-on delay

    uint8_t cmd[] = {AHT20_INIT_CMD, 0x08, 0x00};
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (AHT20_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd_handle, cmd, sizeof(cmd), true);
    i2c_master_stop(cmd_handle);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AHT20 initialization failed: %d", ret);
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(40));
    return ESP_OK;
}

// Check BMP280 ID
static esp_err_t bmp280_check_id(void)
{
    uint8_t id;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_ID, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &id, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        if (id == BMP280_CHIP_ID) {
            ESP_LOGI(TAG, "BMP280 chip ID verified: 0x%02x", id);
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "Wrong chip ID: expected 0x%02x, got 0x%02x", BMP280_CHIP_ID, id);
            return ESP_FAIL;
        }
    }
    return ret;
}

// Read temperature and humidity
static esp_err_t aht20_read_data(float *temperature, float *humidity)
{
    uint8_t cmd[] = {AHT20_TRIGGER_MEASUREMENT, 0x33, 0x00};
    uint8_t data[7];
    
    // Trigger measurement
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (AHT20_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd_handle, cmd, sizeof(cmd), true);
    i2c_master_stop(cmd_handle);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(80)); // Wait for measurement

    // Read data
    cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (AHT20_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd_handle, data, sizeof(data), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd_handle);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
    
    if (ret == ESP_OK) {
        // Convert raw data to humidity and temperature
        uint32_t raw_humidity = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
        uint32_t raw_temperature = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
        
        *humidity = (float)raw_humidity * 100 / 0x100000;
        *temperature = (float)raw_temperature * 200 / 0x100000 - 50;
    }
    
    return ret;
}

// Initialize BMP280
static esp_err_t bmp280_init(void)
{
    // Read calibration data
    uint8_t calib_data[24];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_CALIB, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, calib_data, 24, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    // Parse calibration data
    bmp280_cal.dig_T1 = (calib_data[1] << 8) | calib_data[0];
    bmp280_cal.dig_T2 = (calib_data[3] << 8) | calib_data[2];
    bmp280_cal.dig_T3 = (calib_data[5] << 8) | calib_data[4];
    bmp280_cal.dig_P1 = (calib_data[7] << 8) | calib_data[6];
    bmp280_cal.dig_P2 = (calib_data[9] << 8) | calib_data[8];
    bmp280_cal.dig_P3 = (calib_data[11] << 8) | calib_data[10];
    bmp280_cal.dig_P4 = (calib_data[13] << 8) | calib_data[12];
    bmp280_cal.dig_P5 = (calib_data[15] << 8) | calib_data[14];
    bmp280_cal.dig_P6 = (calib_data[17] << 8) | calib_data[16];
    bmp280_cal.dig_P7 = (calib_data[19] << 8) | calib_data[18];
    bmp280_cal.dig_P8 = (calib_data[21] << 8) | calib_data[20];
    bmp280_cal.dig_P9 = (calib_data[23] << 8) | calib_data[22];

    // Configure sensor: Normal mode, oversampling x16 for temp and pressure
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_CONTROL, true);
    i2c_master_write_byte(cmd, 0xB7, true);  // Normal mode, temp x16, pressure x16
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

// Read BMP280 temperature and pressure
static esp_err_t bmp280_read_data(float *temperature, float *pressure)
{
    uint8_t data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_PRESSURE, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        int32_t adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);
        int32_t adc_T = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] >> 4);

        // Temperature calculation
        int32_t var1 = ((((adc_T >> 3) - ((int32_t)bmp280_cal.dig_T1 << 1))) * ((int32_t)bmp280_cal.dig_T2)) >> 11;
        int32_t var2 = (((((adc_T >> 4) - ((int32_t)bmp280_cal.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_cal.dig_T1))) >> 12) * ((int32_t)bmp280_cal.dig_T3)) >> 14;
        bmp280_cal.t_fine = var1 + var2;
        *temperature = (float)((bmp280_cal.t_fine * 5 + 128) >> 8) / 100.0;

        // Pressure calculation
        var1 = (((int32_t)bmp280_cal.t_fine) >> 1) - 64000;
        var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)bmp280_cal.dig_P6);
        var2 = var2 + ((var1 * ((int32_t)bmp280_cal.dig_P5)) << 1);
        var2 = (var2 >> 2) + (((int32_t)bmp280_cal.dig_P4) << 16);
        var1 = (((bmp280_cal.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)bmp280_cal.dig_P2) * var1) >> 1)) >> 18;
        var1 = ((((32768 + var1)) * ((int32_t)bmp280_cal.dig_P1)) >> 15);
        if (var1 == 0) return ESP_FAIL;

        uint32_t p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
        if (p < 0x80000000) {
            p = (p << 1) / ((uint32_t)var1);
        } else {
            p = (p / (uint32_t)var1) * 2;
        }

        var1 = (((int32_t)bmp280_cal.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
        var2 = (((int32_t)(p >> 2)) * ((int32_t)bmp280_cal.dig_P8)) >> 13;
        p = (uint32_t)((int32_t)p + ((var1 + var2 + bmp280_cal.dig_P7) >> 4));
        *pressure = (float)p / 100.0; // Convert to hPa
    }
    return ret;
}

void app_main(void)
{
    // Initialize I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Scan I2C bus
    i2c_scan();
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize AHT20
    esp_err_t ret = aht20_init();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "AHT20 initialized successfully");
    } else {
        ESP_LOGE(TAG, "Failed to initialize AHT20");
    }

    // Check and initialize BMP280
    ret = bmp280_check_id();
    if (ret == ESP_OK) {
        ret = bmp280_init();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "BMP280 initialized successfully");
        } else {
            ESP_LOGE(TAG, "Failed to initialize BMP280");
        }
    } else {
        ESP_LOGE(TAG, "BMP280 not found or wrong chip ID");
    }

    float temperature_aht, humidity;
    float temperature_bmp, pressure;

    while(1) {
        // Read AHT20
        if (aht20_read_data(&temperature_aht, &humidity) == ESP_OK) {
            ESP_LOGI(TAG, "AHT20 - Temperature: %.2f°C, Humidity: %.2f%%", 
                     temperature_aht, humidity);
        } else {
            ESP_LOGE(TAG, "Error reading AHT20");
        }

        // Read BMP280
        if (bmp280_read_data(&temperature_bmp, &pressure) == ESP_OK) {
            ESP_LOGI(TAG, "BMP280 - Temperature: %.2f°C, Pressure: %.2f hPa", 
                     temperature_bmp, pressure);
        } else {
            ESP_LOGE(TAG, "Error reading BMP280");
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
