#ifndef MAX30102_H
#define MAX30102_H

#include "esp_err.h"
#include "driver/i2c.h"
#include <stdint.h>
#include <stddef.h>

// MAX30102 I2C pinout for ESP32 DevKit1
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

#define MAX30102_I2C_ADDR           0x57

// Registers
#define MAX30102_PART_ID_REG        0xFF
#define MAX30102_MODE_CONFIG_REG    0x09
#define MAX30102_SPO2_CONFIG_REG    0x0A
#define MAX30102_LED1_PA_REG        0x0C
#define MAX30102_LED2_PA_REG        0x0D
#define MAX30102_FIFO_CONFIG_REG    0x08
#define MAX30102_FIFO_DATA_REG      0x07

/**
 * @brief Initialize I2C master and configure the MAX30102 sensor registers.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30102_init(void);

/**
 * @brief Read available samples from MAX30102 hardware FIFO.
 * @param red_buf Output array to store Red values.
 * @param ir_buf Output array to store IR values.
 * @param max_buf_size Maximum number of samples the output buffers can hold (typically 32).
 * @return int Number of samples successfully read, or negative on I2C error.
 */
int max30102_read_fifo(uint32_t *red_buf, uint32_t *ir_buf, int max_buf_size);

#endif // MAX30102_H
