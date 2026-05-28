#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

// MAX30102 I2C pinout for ESP32 DevKit1
// SDA -> GPIO21
// SCL -> GPIO22
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

#define MAX30102_I2C_ADDR           0x57
#define MAX30102_PART_ID_REG        0xFF
#define MAX30102_MODE_CONFIG_REG    0x09
#define MAX30102_SPO2_CONFIG_REG    0x0A
#define MAX30102_LED1_PA_REG        0x0C
#define MAX30102_LED2_PA_REG        0x0D
#define MAX30102_FIFO_CONFIG_REG    0x08
#define MAX30102_FIFO_DATA_REG      0x07
#define SAMPLE_WINDOW               25
#define SAMPLE_PERIOD_MS            120

static const char *TAG = "MAX30102";

static void compute_sensor_metrics(const uint32_t *red, const uint32_t *ir, size_t count, uint8_t *bpm, uint8_t *spo2)
{
    uint64_t sum_red = 0, sum_ir = 0;
    uint32_t max_red = 0, min_red = UINT32_MAX;
    uint32_t max_ir = 0, min_ir = UINT32_MAX;

    for (size_t i = 0; i < count; i++) {
        sum_red += red[i];
        sum_ir += ir[i];
        if (red[i] > max_red) max_red = red[i];
        if (red[i] < min_red) min_red = red[i];
        if (ir[i] > max_ir) max_ir = ir[i];
        if (ir[i] < min_ir) min_ir = ir[i];
    }

    uint32_t avg_red = sum_red / count;
    uint32_t avg_ir = sum_ir / count;
    uint32_t threshold_ir = (max_ir + min_ir) / 2;

    int peak_count = 0;
    for (size_t i = 1; i + 1 < count; i++) {
        if (ir[i] > ir[i - 1] && ir[i] > ir[i + 1] && ir[i] > threshold_ir) {
            peak_count++;
        }
    }

    if (peak_count > 0) {
        uint32_t window_ms = count * SAMPLE_PERIOD_MS;
        *bpm = (uint8_t)((peak_count * 60000U + window_ms / 2) / window_ms);
        if (*bpm < 30) *bpm = 30;
        if (*bpm > 200) *bpm = 200;
    } else {
        *bpm = 0;
    }

    uint64_t ac_red = 0, ac_ir = 0;
    for (size_t i = 0; i < count; i++) {
        uint32_t dr = red[i] > avg_red ? red[i] - avg_red : avg_red - red[i];
        uint32_t di = ir[i] > avg_ir ? ir[i] - avg_ir : avg_ir - ir[i];
        ac_red += dr;
        ac_ir += di;
    }

    if (ac_ir == 0 || ac_red == 0) {
        *spo2 = 0;
    } else {
        double ratio = (double)ac_red / (double)ac_ir;
        int spo2_val = (int)(110.0 - 18.0 * ratio);
        if (spo2_val < 70) spo2_val = 70;
        if (spo2_val > 100) spo2_val = 100;
        *spo2 = (uint8_t)spo2_val;
    }
}

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t max30102_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_write_to_device(I2C_MASTER_NUM,
                                      MAX30102_I2C_ADDR,
                                      write_buf,
                                      sizeof(write_buf),
                                      pdMS_TO_TICKS(1000));
}

static esp_err_t max30102_read_reg(uint8_t reg, uint8_t *value)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM,
                                        MAX30102_I2C_ADDR,
                                        &reg,
                                        1,
                                        value,
                                        1,
                                        pdMS_TO_TICKS(1000));
}

static esp_err_t max30102_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM,
                                        MAX30102_I2C_ADDR,
                                        &reg,
                                        1,
                                        data,
                                        len,
                                        pdMS_TO_TICKS(1000));
}

static esp_err_t max30102_init(void)
{
    uint8_t part_id = 0;
    esp_err_t err = max30102_read_reg(MAX30102_PART_ID_REG, &part_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PART ID (%s)", esp_err_to_name(err));
        return err;
    }

    if (part_id != 0x15) {
        ESP_LOGW(TAG, "Unexpected PART ID: 0x%02X", part_id);
    } else {
        ESP_LOGI(TAG, "Detected MAX30102 PART ID: 0x%02X", part_id);
    }

    err = max30102_write_reg(MAX30102_MODE_CONFIG_REG, 0x03);
    if (err != ESP_OK) return err;

    err = max30102_write_reg(MAX30102_SPO2_CONFIG_REG, 0x27);
    if (err != ESP_OK) return err;

    err = max30102_write_reg(MAX30102_LED1_PA_REG, 0x24);
    if (err != ESP_OK) return err;

    err = max30102_write_reg(MAX30102_LED2_PA_REG, 0x24);
    if (err != ESP_OK) return err;

    err = max30102_write_reg(MAX30102_FIFO_CONFIG_REG, 0x00);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

void app_main(void)
{
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        return;
    }

    err = max30102_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 init failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "MAX30102 initialization complete");

    uint32_t red_samples[SAMPLE_WINDOW] = {0};
    uint32_t ir_samples[SAMPLE_WINDOW] = {0};
    size_t sample_index = 0;

    while (true) {
        uint8_t fifo_data[6] = {0};
        err = max30102_read_bytes(MAX30102_FIFO_DATA_REG, fifo_data, sizeof(fifo_data));
        if (err == ESP_OK) {
            uint32_t red = ((uint32_t)fifo_data[0] << 16) |
                           ((uint32_t)fifo_data[1] << 8) |
                           fifo_data[2];
            uint32_t ir = ((uint32_t)fifo_data[3] << 16) |
                          ((uint32_t)fifo_data[4] << 8) |
                          fifo_data[5];
            red &= 0x3FFFF;
            ir &= 0x3FFFF;

            red_samples[sample_index] = red;
            ir_samples[sample_index] = ir;
            sample_index++;

            if (sample_index >= SAMPLE_WINDOW) {
                uint8_t bpm = 0;
                uint8_t spo2 = 0;
                compute_sensor_metrics(red_samples, ir_samples, SAMPLE_WINDOW, &bpm, &spo2);
                ESP_LOGI(TAG, "BPM=%u SpO2=%u%%", bpm, spo2);
                sample_index = 0;
            }
        } else {
            ESP_LOGW(TAG, "Failed to read FIFO (%s)", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
