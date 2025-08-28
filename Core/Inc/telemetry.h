#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Telemetry configuration
#define TELEMETRY_SAMPLE_RATE_HZ    100     // Sample at 100Hz
#define TELEMETRY_OUTPUT_RATE_HZ    10      // Output at 10Hz  
#define TELEMETRY_DECIMATION        (TELEMETRY_SAMPLE_RATE_HZ / TELEMETRY_OUTPUT_RATE_HZ)  // = 10

#define TELEMETRY_BUFFER_SIZE       800     // Stack buffer for CSV row
#define TELEMETRY_UART_RING_SIZE    4096    // UART DMA ring buffer

// Module flags (bitfield)
#define TELEM_FLAG_TICK_WHILE_IDLE  0x01
#define TELEM_FLAG_IMPOSSIBLE_DELTA 0x02
#define TELEM_FLAG_STEER_LIMIT      0x04
#define TELEM_FLAG_DRIVE_SATURATED  0x08

// Per-module telemetry data
typedef struct {
    float ang_now_deg;          // Current actual angle (degrees)
    float ang_cmd_deg;          // Last commanded angle (degrees) 
    float v_cmd_mps;            // Kinematics speed after scaling (m/s)
    int32_t drive_sps;          // Current drive motor target (steps/s)
    int32_t pos_steps_drive;    // Drive motor position (steps)
    int32_t pos_steps_steer;    // Steer motor position (steps)
    uint8_t inv;                // Drive inversion flag (0/1)
    uint8_t flags;              // Status flags (bitfield)
} TelemetryModuleData;

// Complete telemetry sample
typedef struct {
    uint32_t timestamp_ms;      // HAL_GetTick() timestamp
    
    // Chassis twist (commanded and actual)
    float vx_cmd, vy_cmd, wz_cmd;       // User commands
    float vx_actual, vy_actual, wz_actual;  // Slewed values
    float scale_factor;         // Speed scaling applied
    
    // Per-module data [FR, FL, R]
    TelemetryModuleData modules[3];
    
} TelemetrySample;

// Telemetry system state
typedef struct {
    bool enabled;               // Telemetry on/off
    uint32_t sample_counter;    // Decimation counter
    bool header_sent;           // CSV header sent flag
    
    // Latest sample data
    TelemetrySample latest;
    
    // UART ring buffer (if using DMA)
    uint8_t uart_ring[TELEMETRY_UART_RING_SIZE];
    volatile uint32_t ring_head;
    volatile uint32_t ring_tail;
    
} TelemetryState;

// Initialize telemetry system
void telemetry_init(void);

// Enable/disable telemetry
void telemetry_enable(bool enable);

// Check if telemetry is enabled
bool telemetry_is_enabled(void);

// Sample telemetry data (call at 100Hz after all updates)
void telemetry_sample(const TelemetrySample* sample);

// Process UART transmission (call periodically or in DMA callback)
void telemetry_process_uart(void);

// Emit immediate fault message
void telemetry_emit_fault(const char* fault_msg);

// Get the latest telemetry sample (for CLI display)
const TelemetrySample* telemetry_get_latest_sample(void);

// Fast float-to-string conversion (3 decimal places)
int ftoa_fast(float value, char* buffer, int decimals);

#ifdef __cplusplus
}
#endif

#endif // TELEMETRY_H