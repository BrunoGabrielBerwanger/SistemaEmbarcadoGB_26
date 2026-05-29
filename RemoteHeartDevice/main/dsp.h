#ifndef DSP_H
#define DSP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define SAMPLE_WINDOW               300
#define HISTORY_SIZE                10

/**
 * @brief Push a new raw sample (Red and IR) into the internal DSP buffers.
 * @param red Raw Red channel value.
 * @param ir Raw Infrared channel value.
 */
void dsp_add_sample(uint32_t red, uint32_t ir);

/**
 * @brief Process current window of samples and compute metrics.
 * @param bpm Output pointer for instantaneous BPM.
 * @param spo2 Output pointer for instantaneous SpO2.
 * @param avg_bpm Output pointer for rolling average BPM.
 * @param avg_spo2 Output pointer for rolling average SpO2.
 * @return true if a valid finger is detected and metrics computed.
 * @return false if no finger is detected or buffer is still filling.
 */
bool dsp_get_metrics(uint8_t *bpm, uint8_t *spo2, uint8_t *avg_bpm, uint8_t *avg_spo2);

/**
 * @brief Get the status of the circular buffer.
 * @param samples_collected Output pointer to number of samples collected so far.
 * @param total_needed Output pointer to the total window size required.
 * @return true if the buffer is completely full, false otherwise.
 */
bool dsp_get_buffer_status(int *samples_collected, int *total_needed);

/**
 * @brief Reset the internal DSP circular buffer and rolling histories.
 */
void dsp_reset(void);

#endif // DSP_H
