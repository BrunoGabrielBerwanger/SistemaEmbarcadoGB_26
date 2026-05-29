#include "dsp.h"
#include <string.h>

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

void dsp_add_sample(uint32_t red, uint32_t ir)
{
    red_buffer[buffer_index] = red;
    ir_buffer[buffer_index] = ir;
    buffer_index = (buffer_index + 1) % SAMPLE_WINDOW;
    if (buffer_index == 0) {
        buffer_full = true;
    }
}

bool dsp_get_buffer_status(int *samples_collected, int *total_needed)
{
    if (samples_collected) {
        *samples_collected = buffer_full ? SAMPLE_WINDOW : buffer_index;
    }
    if (total_needed) {
        *total_needed = SAMPLE_WINDOW;
    }
    return buffer_full;
}

void dsp_reset(void)
{
    memset(red_buffer, 0, sizeof(red_buffer));
    memset(ir_buffer, 0, sizeof(ir_buffer));
    buffer_index = 0;
    buffer_full = false;

    memset(bpm_history, 0, sizeof(bpm_history));
    memset(spo2_history, 0, sizeof(spo2_history));
    history_index = 0;
    history_count = 0;
}

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

bool dsp_get_metrics(uint8_t *bpm, uint8_t *spo2, uint8_t *avg_bpm, uint8_t *avg_spo2)
{
    if (!buffer_full) {
        *bpm = 0;
        *spo2 = 0;
        *avg_bpm = 0;
        *avg_spo2 = 0;
        return false;
    }

    // Unroll circular buffer sequentially
    static uint32_t red_ordered[SAMPLE_WINDOW];
    static uint32_t ir_ordered[SAMPLE_WINDOW];
    for (int i = 0; i < SAMPLE_WINDOW; i++) {
        int idx = (buffer_index + i) % SAMPLE_WINDOW;
        red_ordered[i] = red_buffer[idx];
        ir_ordered[i] = ir_buffer[idx];
    }

    uint8_t raw_bpm = 0;
    uint8_t raw_spo2 = 0;
    compute_sensor_metrics(red_ordered, ir_ordered, SAMPLE_WINDOW, &raw_bpm, &raw_spo2);

    add_to_history(raw_bpm, raw_spo2, avg_bpm, avg_spo2);
    *bpm = raw_bpm;
    *spo2 = raw_spo2;

    // Return true if finger presence was confirmed and rolling average is valid
    return (*avg_bpm > 0 && *avg_spo2 > 0);
}
