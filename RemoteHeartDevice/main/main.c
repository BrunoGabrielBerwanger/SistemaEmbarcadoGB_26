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
#define SAMPLE_WINDOW               300
#define SAMPLE_PERIOD_MS            20
#define HISTORY_SIZE                10

static const char *TAG = "MAX30102";

// Circular buffer for raw sensor samples
static uint32_t red_buffer[SAMPLE_WINDOW] = {0};
static uint32_t ir_buffer[SAMPLE_WINDOW] = {0};
static int buffer_index = 0;
static bool buffer_full = false;

// History buffer for rolling average
static uint8_t bpm_history[HISTORY_SIZE] = {0};
static uint8_t spo2_history[HISTORY_SIZE] = {0};
static int history_index = 0;
static int history_count = 0;

static void add_to_history(uint8_t bpm, uint8_t spo2, uint8_t *avg_bpm, uint8_t *avg_spo2)
{
    if (bpm > 0 && spo2 > 0) {
        bpm_history[history_index] = bpm;
        spo2_history[history_index] = spo2;
        history_index = (history_index + 1) % HISTORY_SIZE;
        if (history_count < HISTORY_SIZE) {
            history_count++;
        }
    } else {
        // If finger is removed or readings are invalid, reset the rolling average history
        history_count = 0;
        history_index = 0;
        *avg_bpm = 0;
        *avg_spo2 = 0;
        return;
    }

    uint32_t sum_bpm = 0;
    uint32_t sum_spo2 = 0;
    for (int i = 0; i < history_count; i++) {
        sum_bpm += bpm_history[i];
        sum_spo2 += spo2_history[i];
    }
    *avg_bpm = (uint8_t)(sum_bpm / history_count);
    *avg_spo2 = (uint8_t)(sum_spo2 / history_count);
}

static void compute_sensor_metrics(const uint32_t *red, const uint32_t *ir, size_t count, uint8_t *bpm, uint8_t *spo2)
{
    // 1. Calculate Average DC of the window to check finger presence and remove DC
    uint64_t sum_red = 0;
    uint64_t sum_ir = 0;
    for (size_t i = 0; i < count; i++) {
        sum_red += red[i];
        sum_ir += ir[i];
    }
    uint32_t avg_red = sum_red / count;
    uint32_t avg_ir = sum_ir / count;

    // Threshold for finger presence: typically IR > 30000 when finger is on the sensor.
    if (avg_ir < 30000) {
        *bpm = 0;
        *spo2 = 0;
        return;
    }

    // 2. Remove DC Component (DC filter) to obtain the AC component.
    static int32_t ac_red[SAMPLE_WINDOW];
    static int32_t ac_ir[SAMPLE_WINDOW];
    
    for (size_t i = 0; i < count; i++) {
        ac_red[i] = (int32_t)red[i] - (int32_t)avg_red;
        ac_ir[i] = (int32_t)ir[i] - (int32_t)avg_ir;
    }

    // 3. Smooth the AC signals using a 5-point moving average filter to reduce high frequency noise
    static int32_t filtered_red[SAMPLE_WINDOW];
    static int32_t filtered_ir[SAMPLE_WINDOW];
    for (size_t i = 0; i < count; i++) {
        int32_t sum_filt_red = 0;
        int32_t sum_filt_ir = 0;
        int divisor = 0;
        for (int w = -2; w <= 2; w++) {
            int idx = (int)i + w;
            if (idx >= 0 && idx < (int)count) {
                sum_filt_red += ac_red[idx];
                sum_filt_ir += ac_ir[idx];
                divisor++;
            }
        }
        filtered_red[i] = sum_filt_red / divisor;
        filtered_ir[i] = sum_filt_ir / divisor;
    }

    // 4. Peak Detection on the filtered IR signal
    int32_t max_ac_ir = 0;
    for (size_t i = 0; i < count; i++) {
        int32_t val = filtered_ir[i] < 0 ? -filtered_ir[i] : filtered_ir[i];
        if (val > max_ac_ir) {
            max_ac_ir = val;
        }
    }
    
    // Adaptive noise threshold: 1/5th of max amplitude, at least 150 to avoid false positives
    int32_t noise_threshold = max_ac_ir / 5;
    if (noise_threshold < 150) noise_threshold = 150;

    int peak_indices[30];
    int peak_cnt = 0;

    // Detect peaks: local maximums that are above the noise threshold
    for (size_t i = 2; i < count - 2; i++) {
        if (filtered_ir[i] > 0 &&
            filtered_ir[i] > filtered_ir[i - 1] &&
            filtered_ir[i] > filtered_ir[i - 2] &&
            filtered_ir[i] > filtered_ir[i + 1] &&
            filtered_ir[i] > filtered_ir[i + 2] &&
            filtered_ir[i] > noise_threshold) {
            
            // Avoid peaks that are too close (min 300ms = 30 samples at 100Hz)
            if (peak_cnt == 0 || (int)i - peak_indices[peak_cnt - 1] > 30) {
                peak_indices[peak_cnt++] = i;
                if (peak_cnt >= 30) break;
            }
        }
    }

    // 5. Calculate BPM based on average interval between peaks
    if (peak_cnt >= 2) {
        uint32_t total_interval = 0;
        for (int i = 1; i < peak_cnt; i++) {
            total_interval += (peak_indices[i] - peak_indices[i - 1]);
        }
        double avg_interval = (double)total_interval / (peak_cnt - 1);
        
        // Sampling frequency is 100Hz, meaning interval in seconds is avg_interval / 100.
        // BPM = (100.0 / avg_interval) * 60 = 6000.0 / avg_interval
        int bpm_val = (int)(6000.0 / avg_interval);
        if (bpm_val >= 40 && bpm_val <= 200) {
            *bpm = (uint8_t)bpm_val;
        } else {
            *bpm = 0;
        }
    } else {
        *bpm = 0;
    }

    // 6. Calculate SpO2
    // Estimate peak-to-peak amplitude (max - min) of the filtered AC component
    int32_t max_red_ac = -999999, min_red_ac = 999999;
    int32_t max_ir_ac = -999999, min_ir_ac = 999999;
    for (size_t i = 0; i < count; i++) {
        if (filtered_red[i] > max_red_ac) max_red_ac = filtered_red[i];
        if (filtered_red[i] < min_red_ac) min_red_ac = filtered_red[i];
        if (filtered_ir[i] > max_ir_ac) max_ir_ac = filtered_ir[i];
        if (filtered_ir[i] < min_ir_ac) min_ir_ac = filtered_ir[i];
    }

    int32_t ac_p2p_red = max_red_ac - min_red_ac;
    int32_t ac_p2p_ir = max_ir_ac - min_ir_ac;

    if (ac_p2p_ir > 100 && ac_p2p_red > 100 && avg_ir > 0 && avg_red > 0) {
        double ratio = ((double)ac_p2p_red / avg_red) / ((double)ac_p2p_ir / avg_ir);
        // Empirical SpO2 linear calibration formula: SpO2 = 104 - 17 * R
        int spo2_val = (int)(104.0 - 17.0 * ratio);
        if (spo2_val < 70) spo2_val = 70;
        if (spo2_val > 100) spo2_val = 100;
        *spo2 = (uint8_t)spo2_val;
    } else {
        *spo2 = 0;
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

    err = max30102_write_reg(MAX30102_FIFO_CONFIG_REG, 0x10); // Enable FIFO Rollover, no averaging
    if (err != ESP_OK) return err;

    // Reset FIFO pointers
    err = max30102_write_reg(0x04, 0x00); // FIFO Write Pointer
    if (err != ESP_OK) return err;
    err = max30102_write_reg(0x05, 0x00); // FIFO Overflow Counter
    if (err != ESP_OK) return err;
    err = max30102_write_reg(0x06, 0x00); // FIFO Read Pointer
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

    int new_samples_count = 0;

    while (true) {
        uint8_t write_ptr = 0;
        uint8_t read_ptr = 0;
        int num_samples = 0;
        
        if (max30102_read_reg(0x04, &write_ptr) == ESP_OK &&
            max30102_read_reg(0x06, &read_ptr) == ESP_OK) {
            num_samples = (int)write_ptr - (int)read_ptr;
            if (num_samples < 0) {
                num_samples += 32;
            }
        }

        for (int i = 0; i < num_samples; i++) {
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

                // Push to circular buffer
                red_buffer[buffer_index] = red;
                ir_buffer[buffer_index] = ir;
                buffer_index = (buffer_index + 1) % SAMPLE_WINDOW;
                if (buffer_index == 0) {
                    buffer_full = true;
                }
                new_samples_count++;
            } else {
                ESP_LOGW(TAG, "Failed to read FIFO sample (%s)", esp_err_to_name(err));
            }
        }

        if (new_samples_count >= 50) {
            new_samples_count = 0;

            if (buffer_full) {
                // Unroll circular buffer sequentially
                static uint32_t red_ordered[SAMPLE_WINDOW];
                static uint32_t ir_ordered[SAMPLE_WINDOW];
                for (int i = 0; i < SAMPLE_WINDOW; i++) {
                    int idx = (buffer_index + i) % SAMPLE_WINDOW;
                    red_ordered[i] = red_buffer[idx];
                    ir_ordered[i] = ir_buffer[idx];
                }

                uint8_t bpm = 0;
                uint8_t spo2 = 0;
                compute_sensor_metrics(red_ordered, ir_ordered, SAMPLE_WINDOW, &bpm, &spo2);

                uint8_t avg_bpm = 0;
                uint8_t avg_spo2 = 0;
                add_to_history(bpm, spo2, &avg_bpm, &avg_spo2);

                if (avg_bpm > 0 && avg_spo2 > 0) {
                    ESP_LOGI(TAG, "METRICAS: BPM=%u SpO2=%u%% | Media Movel: BPM=%u SpO2=%u%%", 
                             bpm, spo2, avg_bpm, avg_spo2);
                } else {
                    ESP_LOGI(TAG, "METRICAS: Coloque o dedo no sensor...");
                }
            } else {
                // Buffer pre-filling progress
                ESP_LOGI(TAG, "Sincronizando sensor... [%d/%d]", buffer_index, SAMPLE_WINDOW);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
