#include "max30102.h"
#include "esp_log.h"

static const char *TAG = "MAX30102_DRV";

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

esp_err_t max30102_init(void)
{
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed (%s)", esp_err_to_name(err));
        return err;
    }

    uint8_t part_id = 0;
    err = max30102_read_reg(MAX30102_PART_ID_REG, &part_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PART ID (%s)", esp_err_to_name(err));
        return err;
    }

    if (part_id != 0x15) {
        ESP_LOGW(TAG, "Unexpected PART ID: 0x%02X", part_id);
    } else {
        ESP_LOGI(TAG, "Detected MAX30102 PART ID: 0x%02X", part_id);
    }

    // Configure Mode: SpO2 Mode (both Red and IR are active)
    err = max30102_write_reg(MAX30102_MODE_CONFIG_REG, 0x03);
    if (err != ESP_OK) return err;

    // Configure SpO2 sample rate and resolution:
    // 0x27 -> ADC full scale: 4096nA, Sample Rate: 100Hz, LED pulse width: 411us (18-bit resolution)
    err = max30102_write_reg(MAX30102_SPO2_CONFIG_REG, 0x27);
    if (err != ESP_OK) return err;

    // Configure LED Pulse Amplitudes (~7.2mA current)
    err = max30102_write_reg(MAX30102_LED1_PA_REG, 0x24);
    if (err != ESP_OK) return err;
    err = max30102_write_reg(MAX30102_LED2_PA_REG, 0x24);
    if (err != ESP_OK) return err;

    // Configure FIFO configuration:
    // 0x10 -> Enable FIFO Rollover, no sample averaging, interrupt at 0 empty spots
    err = max30102_write_reg(MAX30102_FIFO_CONFIG_REG, 0x10);
    if (err != ESP_OK) return err;

    // Reset internal FIFO pointers on startup
    err = max30102_write_reg(0x04, 0x00); // FIFO Write Pointer
    if (err != ESP_OK) return err;
    err = max30102_write_reg(0x05, 0x00); // FIFO Overflow Counter
    if (err != ESP_OK) return err;
    err = max30102_write_reg(0x06, 0x00); // FIFO Read Pointer
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Sensor registers configured successfully");
    return ESP_OK;
}

int max30102_read_fifo(uint32_t *red_buf, uint32_t *ir_buf, int max_buf_size)
{
    uint8_t write_ptr = 0;
    uint8_t read_ptr = 0;
    
    if (max30102_read_reg(0x04, &write_ptr) != ESP_OK ||
        max30102_read_reg(0x06, &read_ptr) != ESP_OK) {
        return -1; // I2C communication error
    }

    int num_samples = (int)write_ptr - (int)read_ptr;
    if (num_samples < 0) {
        num_samples += 32;
    }

    if (num_samples > max_buf_size) {
        num_samples = max_buf_size;
    }

    int successfully_read = 0;

    for (int i = 0; i < num_samples; i++) {
        uint8_t fifo_data[6] = {0};
        esp_err_t err = max30102_read_bytes(MAX30102_FIFO_DATA_REG, fifo_data, sizeof(fifo_data));
        if (err == ESP_OK) {
            uint32_t red = ((uint32_t)fifo_data[0] << 16) |
                           ((uint32_t)fifo_data[1] << 8) |
                           fifo_data[2];
            uint32_t ir = ((uint32_t)fifo_data[3] << 16) |
                          ((uint32_t)fifo_data[4] << 8) |
                          fifo_data[5];
            
            // Mask to 18-bits
            red_buf[successfully_read] = red & 0x3FFFF;
            ir_buf[successfully_read] = ir & 0x3FFFF;
            successfully_read++;
        } else {
            ESP_LOGW(TAG, "Failed to read single FIFO word (%s)", esp_err_to_name(err));
        }
    }

    return successfully_read;
}
