#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "max30102.h"
#include "dsp.h"

#define SAMPLE_PERIOD_MS            20

static const char *TAG = "MAIN_APP";

// Struct used as Critical Section
typedef struct {
    uint8_t bpm;
    uint8_t spo2;
    uint8_t avg_bpm;
    uint8_t avg_spo2;
    bool has_data;
    bool buffer_ready;
    int samples_collected;
    int total_needed;
} sensor_data_t;

static sensor_data_t g_sensor_data = {0};
static SemaphoreHandle_t g_sensor_mutex = NULL;

static uint32_t g_spp_handle = 0;
static bool g_spp_connected = false;

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Bluetooth Authentication Success: %s", param->auth_cmpl.device_name);
        } else {
            ESP_LOGE(TAG, "Bluetooth Authentication Failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_CFM_REQ_EVT. Numeric value comparison request: %lu", 
                 (unsigned long)param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey: %lu", 
                 (unsigned long)param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey");
        break;
    default:
        break;
    }
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG, "ESP_SPP_INIT_EVT: SPP initialized, starting SPP server...");
        esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, "SPP_SERVER");
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_SPP_OPEN_EVT: SPP client connection opened");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CLOSE_EVT: SPP client connection closed");
        g_spp_connected = false;
        g_spp_handle = 0;
        break;
    case ESP_SPP_START_EVT: {
        ESP_LOGI(TAG, "ESP_SPP_START_EVT: SPP server started");
        esp_bt_gap_set_device_name("CardioMonitor_GB");
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        
        // Set Class of Device (CoD) to Health (0x09) to help PCs identify the device type
        esp_bt_cod_t cod;
        cod.major = 0x09;      // Major class: Health (0x09)
        cod.minor = 0x00;      // Minor class: Generic
        cod.service = 0x00;    // Generic service
        esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_ALL);
        break;
    }
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(TAG, "ESP_SPP_DATA_IND_EVT len=%u handle=%lu", 
                 param->data_ind.len, (unsigned long)param->data_ind.handle);
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CONG_EVT cong=%u", param->cong.cong);
        break;
    case ESP_SPP_WRITE_EVT:
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_SPP_SRV_OPEN_EVT: Client connected successfully!");
        g_spp_handle = param->srv_open.handle;
        g_spp_connected = true;
        break;
    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(TAG, "ESP_SPP_SRV_STOP_EVT");
        break;
    default:
        break;
    }
}

/*
    Function to initialize the Bluetooth Serial Port Profile.
    Function calls required functions to set up the Bluetooth 
    controller, Bluedroid stack, and SPP profile.

    @Input: void
    @Output: esp_err_t - ESP_OK on success, error code on failure
*/
esp_err_t bluetooth_spp_init(void)
{
    esp_err_t ret;

    // Release memory of BLE controller since we only need Classic BT SPP
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.mode = ESP_BT_MODE_CLASSIC_BT;
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "%s gap register failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "%s spp register failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0,
    };
    if ((ret = esp_spp_enhanced_init(&spp_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "%s spp init failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

#ifdef CONFIG_BT_SSP_ENABLED
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use a fixed PIN Code "1234" to avoid issues with uninitialized variables
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code = {'1', '2', '3', '4'};
    esp_bt_gap_set_pin(pin_type, 4, pin_code);

    return ESP_OK;
}

/*
    Thread dedicate to MAX30102 sensor.
    This task is responsible to aquire the sensor data,
    calculate metrics and store the data.
*/
void TaskSensorAquisition(void *pvParameters)
{
    int new_samples_count = 0;
    static uint32_t temp_red[32];
    static uint32_t temp_ir[32];

    ESP_LOGI(TAG, "Task Aquisition started!");

    while (true) {
        // Read any available samples in the sensor's FIFO
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

            uint8_t bpm = 0;
            uint8_t spo2 = 0;
            uint8_t avg_bpm = 0;
            uint8_t avg_spo2 = 0;
            bool has_data = false;

            if (is_full) {
                has_data = dsp_get_metrics(&bpm, &spo2, &avg_bpm, &avg_spo2);
            }

            // Write to shared memory protected by Mutex
            if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                g_sensor_data.bpm = bpm;
                g_sensor_data.spo2 = spo2;
                g_sensor_data.avg_bpm = avg_bpm;
                g_sensor_data.avg_spo2 = avg_spo2;
                g_sensor_data.has_data = has_data;
                g_sensor_data.buffer_ready = is_full;
                g_sensor_data.samples_collected = samples_collected;
                g_sensor_data.total_needed = total_needed;
                
                xSemaphoreGive(g_sensor_mutex);
            } else {
                ESP_LOGW(TAG, "Could not acquire mutex in Task Aquisition");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

/*
    Thread dedicate to bluetooth communication.
    This task is responsible to send the pre-aquired
    data using bluetooth connection.
*/
void TaskBluetoothTx(void *pvParameters)
{
    ESP_LOGI(TAG, "Task Bluetooth Tx started!");
    char tx_buffer[256];

    while (true) {
        sensor_data_t local_data = {0};

        // Read from shared memory protected by Mutex
        if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            local_data = g_sensor_data;
            xSemaphoreGive(g_sensor_mutex);
        } else {
            ESP_LOGW(TAG, "Could not acquire mutex in Task Bluetooth Tx");
        }

        // If SPP client is connected, format and send the data via Bluetooth (JSON format)
        if (g_spp_connected && g_spp_handle != 0) {
            int len = 0;
            if (local_data.buffer_ready) {
                if (local_data.has_data) {
                    len = snprintf(tx_buffer, sizeof(tx_buffer),
                                   "{\"status\":\"synchronized\", \"bpm\":%u, \"spo2\":%u, \"avg_bpm\":%u, \"avg_spo2\":%u}\r\n",
                                   local_data.bpm, local_data.spo2, local_data.avg_bpm, local_data.avg_spo2);
                } else {
                    len = snprintf(tx_buffer, sizeof(tx_buffer),
                                   "{\"status\":\"no_finger\"}\r\n");
                }
            } else {
                len = snprintf(tx_buffer, sizeof(tx_buffer),
                               "{\"status\":\"syncing\", \"collected\":%d, \"total\":%d}\r\n",
                               local_data.samples_collected, local_data.total_needed);
            }

            if (len > 0) {
                esp_spp_write(g_spp_handle, len, (uint8_t *)tx_buffer);
                ESP_LOGI(TAG, "Bluetooth Sent: %s", tx_buffer);
            }
        } else {
            // Fallback: log to console if Bluetooth is not connected yet
            if (local_data.buffer_ready) {
                if (local_data.has_data) {
                    ESP_LOGI(TAG, "METRICAS (Aguardando BT): BPM=%u SpO2=%u%% | Media Movel: BPM=%u SpO2=%u%%", 
                             local_data.bpm, local_data.spo2, local_data.avg_bpm, local_data.avg_spo2);
                } else {
                    ESP_LOGI(TAG, "METRICAS (Aguardando BT): Coloque o dedo no sensor...");
                }
            } else {
                ESP_LOGI(TAG, "Sincronizando sensor (Aguardando BT)... [%d/%d]", 
                         local_data.samples_collected, local_data.total_needed);
            }
        }

        // Periodically transmit data (e.g., every 1 second)
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    esp_err_t err;

    // Initialize non volatile storage - NVS (required for Bluetooth bonding and keys)
    err = nvs_flash_init();
    // Verify if NVS initialization failed due to no free pages or new version
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Create Mutex for shared region protection
    g_sensor_mutex = xSemaphoreCreateMutex();
    if (g_sensor_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create shared memory Mutex");
        return;
    }

    // Initialize the MAX30102 sensor and I2C driver
    err = max30102_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 initialization failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "MAX30102 initialization complete");

    // Initialize Bluetooth Serial Port Profile (SPP) stack
    err = bluetooth_spp_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth SPP initialization failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Bluetooth SPP initialization complete");

    // Create tasks
    // Task A: Sensor Aquisition pinned to Core 0 (dedicated to fast I2C polling)
    xTaskCreate(TaskSensorAquisition, "TaskSensorAquisition", 4096, NULL, 5, NULL);

    // Task B: Bluetooth Transmission pinned to Core 1 (handles BT overhead and formatting)
    xTaskCreate(TaskBluetoothTx, "TaskBluetoothTx", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "System initialization complete. Concurrent tasks spawned.");
}

