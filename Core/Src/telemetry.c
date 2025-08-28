#include "telemetry.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

// Global telemetry state
static TelemetryState telem_state = {0};

// External UART handle (assumes UART3 for telemetry)
extern UART_HandleTypeDef huart3;

void telemetry_init(void) {
    memset(&telem_state, 0, sizeof(telem_state));
    telem_state.enabled = false;
    telem_state.sample_counter = 0;
    telem_state.header_sent = false;
    telem_state.ring_head = 0;
    telem_state.ring_tail = 0;
}

void telemetry_enable(bool enable) {
    if (enable && !telem_state.enabled) {
        // Enabling - reset state and send header
        telem_state.sample_counter = 0;
        telem_state.header_sent = false;
        telem_state.ring_head = 0;
        telem_state.ring_tail = 0;
    }
    telem_state.enabled = enable;
}

bool telemetry_is_enabled(void) {
    return telem_state.enabled;
}

// Fast float to string conversion (avoids printf float overhead)
int ftoa_fast(float value, char* buffer, int decimals) {
    if (value < 0) {
        *buffer++ = '-';
        value = -value;
    }
    
    // Integer part
    int int_part = (int)value;
    int len = 0;
    
    if (int_part == 0) {
        *buffer++ = '0';
        len = 1;
    } else {
        // Convert integer part
        char temp[16];
        int temp_len = 0;
        while (int_part > 0) {
            temp[temp_len++] = '0' + (int_part % 10);
            int_part /= 10;
        }
        // Reverse
        for (int i = temp_len - 1; i >= 0; i--) {
            *buffer++ = temp[i];
            len++;
        }
    }
    
    if (decimals > 0) {
        *buffer++ = '.';
        len++;
        
        // Fractional part
        float frac = value - (int)value;
        for (int i = 0; i < decimals; i++) {
            frac *= 10;
            int digit = (int)frac;
            *buffer++ = '0' + digit;
            frac -= digit;
            len++;
        }
    }
    
    *buffer = '\0';
    return len;
}

// Add data to UART ring buffer
static bool ring_buffer_write(const char* data, uint32_t len) {
    uint32_t next_head = (telem_state.ring_head + len) % TELEMETRY_UART_RING_SIZE;
    
    // Check for overflow
    if (next_head == telem_state.ring_tail) {
        return false; // Ring buffer full, drop data
    }
    
    // Copy data to ring buffer
    for (uint32_t i = 0; i < len; i++) {
        telem_state.uart_ring[telem_state.ring_head] = data[i];
        telem_state.ring_head = (telem_state.ring_head + 1) % TELEMETRY_UART_RING_SIZE;
    }
    
    return true;
}

// Send CSV header
static void send_csv_header(void) {
    const char* header = 
        "timestamp_ms,vx_cmd,vy_cmd,wz_cmd,vx_actual,vy_actual,wz_actual,scale_factor,"
        "fr_ang_now,fr_ang_cmd,fr_v_cmd,fr_drive_sps,fr_pos_drive,fr_pos_steer,fr_inv,fr_flags,"
        "fl_ang_now,fl_ang_cmd,fl_v_cmd,fl_drive_sps,fl_pos_drive,fl_pos_steer,fl_inv,fl_flags,"
        "r_ang_now,r_ang_cmd,r_v_cmd,r_drive_sps,r_pos_drive,r_pos_steer,r_inv,r_flags\r\n";
    
    ring_buffer_write(header, strlen(header));
    telem_state.header_sent = true;
}

// Format and send CSV row
static void send_csv_row(const TelemetrySample* sample) {
    char buffer[TELEMETRY_BUFFER_SIZE];
    int pos = 0;
    
    // Timestamp
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%lu,", sample->timestamp_ms);
    
    // Chassis twist data
    char float_buf[16];
    ftoa_fast(sample->vx_cmd, float_buf, 3);
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%s,", float_buf);
    ftoa_fast(sample->vy_cmd, float_buf, 3);
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%s,", float_buf);
    ftoa_fast(sample->wz_cmd, float_buf, 3);
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%s,", float_buf);
    
    ftoa_fast(sample->vx_actual, float_buf, 3);
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%s,", float_buf);
    ftoa_fast(sample->vy_actual, float_buf, 3);
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%s,", float_buf);
    ftoa_fast(sample->wz_actual, float_buf, 3);
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%s,", float_buf);
    ftoa_fast(sample->scale_factor, float_buf, 3);
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%s,", float_buf);
    
    // Per-module data
    for (int i = 0; i < 3; i++) {
        const TelemetryModuleData* mod = &sample->modules[i];
        
        ftoa_fast(mod->ang_now_deg, float_buf, 3);
        pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%s,", float_buf);
        ftoa_fast(mod->ang_cmd_deg, float_buf, 3);
        pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%s,", float_buf);
        ftoa_fast(mod->v_cmd_mps, float_buf, 3);
        pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%s,", float_buf);
        
        pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%ld,%ld,%ld,%u,%u",
                       mod->drive_sps,
                       mod->pos_steps_drive, 
                       mod->pos_steps_steer,
                       mod->inv,
                       mod->flags);
        
        if (i < 2) {
            buffer[pos++] = ',';
        }
    }
    
    // End of row
    buffer[pos++] = '\r';
    buffer[pos++] = '\n';
    buffer[pos] = '\0';
    
    // Send to ring buffer
    ring_buffer_write(buffer, pos);
}

void telemetry_sample(const TelemetrySample* sample) {
    if (!telem_state.enabled) return;
    
    // Store latest sample
    telem_state.latest = *sample;
    
    // Check if it's time to emit (every 10th sample for 10Hz output)
    if (telem_state.sample_counter == 0) {
        // Send header if needed
        if (!telem_state.header_sent) {
            send_csv_header();
        }
        
        // Send CSV row
        send_csv_row(sample);
    }
    
    // Increment decimation counter
    telem_state.sample_counter = (telem_state.sample_counter + 1) % TELEMETRY_DECIMATION;
}

void telemetry_process_uart(void) {
    // UART output disabled to avoid conflict with CLI
    // Telemetry data is still captured and available via 'telem show'
    return;
}

void telemetry_emit_fault(const char* fault_msg) {
    if (!telem_state.enabled) return;
    
    char buffer[128];
    uint32_t timestamp = HAL_GetTick();
    int len = snprintf(buffer, sizeof(buffer), "FAULT,%lu,%s\r\n", timestamp, fault_msg);
    
    ring_buffer_write(buffer, len);
}

const TelemetrySample* telemetry_get_latest_sample(void) {
    return &telem_state.latest;
}

// UART DMA complete callback (should be called from HAL callback)
void telemetry_uart_tx_complete_callback(void) {
    // Continue processing if there's more data
    telemetry_process_uart();
}