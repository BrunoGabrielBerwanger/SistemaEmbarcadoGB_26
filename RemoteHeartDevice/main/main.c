#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "max30102.h"
#include "dsp.h"

#define SAMPLE_PERIOD_MS            20

static const char *TAG = "MAIN_APP";

void app_main(void)
{
    // Initialize the MAX30102 sensor and underlying I2C master driver
    esp_err_t err = max30102_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 initialization failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "MAX30102 initialization complete");

    int new_samples_count = 0;

    // Temporal static buffers to fetch samples from the sensor's FIFO
    static uint32_t temp_red[32];
    static uint32_t temp_ir[32];

    while (true) {
        // Read any available samples in the sensor's hardware FIFO
        int read_count = max30102_read_fifo(temp_red, temp_ir, 32);
        
        if (read_count > 0) {
            for (int i = 0; i < read_count; i++) {
                dsp_add_sample(temp_red[i], temp_ir[i]);
                new_samples_count++;
            }
        }

        // Periodically run DSP calculations (approx. every 0.5s of data = 50 samples)
        if (new_samples_count >= 50) {
            new_samples_count = 0;

            int samples_collected = 0;
            int total_needed = 0;
            bool is_full = dsp_get_buffer_status(&samples_collected, &total_needed);

            if (is_full) {
                uint8_t bpm = 0;
                uint8_t spo2 = 0;
                uint8_t avg_bpm = 0;
                uint8_t avg_spo2 = 0;

                if (dsp_get_metrics(&bpm, &spo2, &avg_bpm, &avg_spo2)) {
                    ESP_LOGI(TAG, "METRICAS: BPM=%u SpO2=%u%% | Media Movel: BPM=%u SpO2=%u%%", 
                             bpm, spo2, avg_bpm, avg_spo2);
                } else {
                    ESP_LOGI(TAG, "METRICAS: Coloque o dedo no sensor...");
                }
            } else {
                // Buffer pre-filling progress
                ESP_LOGI(TAG, "Sincronizando sensor... [%d/%d]", samples_collected, total_needed);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
