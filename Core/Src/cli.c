#include "cli.h"
#include "stepgen.h"
#include "motor.h"
#include "swerve_module.h"
#include "../Inc/swerve_kinematics.h"
#include "app_timers.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Constants for backward compatibility
#define WHEEL_RADIUS_M SWERVE_WHEEL_RADIUS_M

extern RCC_ClkInitTypeDef RCC_ClkInitStruct;

// External motor instances for testing
extern Motor m0, m1, m2, m3, m4, m5;  // All 6 motors
extern Motor motor_test;  // Legacy test motor

// Swerve module instances
SwerveModule swerve_fr, swerve_fl, swerve_r;

// Swerve kinematics instance with slew limiting
SKM_State swerve_kin;
static bool kinematics_initialized = false;

static void cli_send_string(const char* str);
static int cli_receive_line(char* buffer, int max_len);

static char cmd_buffer[CLI_MAX_COMMAND_LEN];
static char* args[CLI_MAX_ARGS];
static int arg_count = 0;

static void cmd_help(void);
static void cmd_status(void);
static void cmd_test(void);
static void cmd_version(void);
static void cmd_clk(void);
static void cmd_echo(void);
static void cmd_stepgen(void);
static void cmd_motor(void);
static void cmd_tick(void);
static void cmd_pc(void);
static void cmd_swerve(void);
static void cmd_twist(void);
static void cmd_mod(void);
static void cmd_kin(void);

typedef struct {
    const char* name;
    void (*function)(void);
    const char* description;
} command_t;

static const command_t commands[] = {
    {"help", cmd_help, "Show available commands"},
    {"status", cmd_status, "Show system status"},
    {"test", cmd_test, "Test communication"},
    {"version", cmd_version, "Show firmware version"},
    {"clk", cmd_clk, "Show system clock details"},
    {"echo", cmd_echo, "Echo back received arguments"},
    {"stepgen", cmd_stepgen, "Control test stepgen: stepgen <sps> or stepgen info"},
    {"motor", cmd_motor, "Control motors: motor <name> <cmd> [args]"},
    {"swerve", cmd_swerve, "Control swerve modules: swerve <module> <cmd> [args]"},
    {"twist", cmd_twist, "Control robot body twist: twist <vx> <vy> <wz> or twist stop"},
    {"mod", cmd_mod, "Module status: mod <i> stat"},
    {"kin", cmd_kin, "Kinematics config: kin setpos <module> <x> <y> or kin show"},
    {"tick", cmd_tick, "Show TIM6 5kHz ISR diagnostics and performance"},
    {"pc", cmd_pc, "Show pulse counts: pc [motor_id] or pc all"}
};

#define NUM_COMMANDS (sizeof(commands) / sizeof(commands[0]))

void cli_init(void)
{
    cli_send_string("\r\n=== Swerve Drive Controller ===\r\n");
    cli_send_string("Type 'help' for available commands\r\n");
    cli_print_prompt();
}

void cli_print_prompt(void)
{
    cli_send_string("swerve> ");
}

static int parse_command(char* cmd_line)
{
    arg_count = 0;
    char* token = strtok(cmd_line, " \t");
    
    while (token != NULL && arg_count < CLI_MAX_ARGS)
    {
        args[arg_count++] = token;
        token = strtok(NULL, " \t");
    }
    
    return arg_count;
}

static void execute_command(void)
{
    if (arg_count == 0) return;
    
    for (int i = 0; i < NUM_COMMANDS; i++)
    {
        if (strcmp(args[0], commands[i].name) == 0)
        {
            commands[i].function();
            return;
        }
    }
    
    cli_send_string("Unknown command: ");
    cli_send_string(args[0]);
    cli_send_string("\r\nType 'help' for available commands\r\n");
}

void cli_process(void)
{
    if (cli_receive_line(cmd_buffer, CLI_MAX_COMMAND_LEN))
    {
        cli_send_string("\r\n");
        
        if (parse_command(cmd_buffer) > 0)
        {
            execute_command();
        }
        
        cli_print_prompt();
    }
}

static void cmd_help(void)
{
    cli_send_string("Available commands:\r\n");
    for (int i = 0; i < NUM_COMMANDS; i++)
    {
        cli_send_string("  ");
        cli_send_string(commands[i].name);
        cli_send_string(" - ");
        cli_send_string(commands[i].description);
        cli_send_string("\r\n");
    }
}

static void cmd_status(void)
{
    cli_send_string("System Status:\r\n");
    cli_send_string("  MCU: STM32F767ZI\r\n");
    cli_send_string("  Status: Running\r\n");
    cli_send_string("  Communication: OK\r\n");
}

static void cmd_test(void)
{
    cli_send_string("Communication test: OK\r\n");
    cli_send_string("UART3 at 115200 baud working properly\r\n");
}

static void cmd_version(void)
{
    cli_send_string("Swerve Drive Firmware v1.0\r\n");
    cli_send_string("Build: " __DATE__ " " __TIME__ "\r\n");
}

static void cmd_clk(void)
{
    if (arg_count >= 2 && strcmp(args[1], "help") == 0)
    {
        cli_send_string("Clock Commands:\r\n");
        cli_send_string("  clk       - Show system clock configuration\r\n");
        cli_send_string("  clk help  - Show this help\r\n");
        return;
    }
    
    RCC_OscInitTypeDef osc_config;
    RCC_ClkInitTypeDef clk_config;
    uint32_t flash_latency;
    char buffer[64];
    
    HAL_RCC_GetOscConfig(&osc_config);
    HAL_RCC_GetClockConfig(&clk_config, &flash_latency);
    
    cli_send_string("System Clock Configuration:\r\n");
    
    sprintf(buffer, "  SYSCLK: %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
    cli_send_string(buffer);
    
    sprintf(buffer, "  HCLK (AHB): %lu Hz\r\n", HAL_RCC_GetHCLKFreq());
    cli_send_string(buffer);
    
    sprintf(buffer, "  PCLK1 (APB1): %lu Hz\r\n", HAL_RCC_GetPCLK1Freq());
    cli_send_string(buffer);
    
    sprintf(buffer, "  PCLK2 (APB2): %lu Hz\r\n", HAL_RCC_GetPCLK2Freq());
    cli_send_string(buffer);
    
    sprintf(buffer, "  Flash Latency: %lu\r\n", flash_latency);
    cli_send_string(buffer);
    
    cli_send_string("  PLL Source: HSE\r\n");
    sprintf(buffer, "  PLL M: %lu, N: %lu, P: %lu, Q: %lu\r\n", 
            osc_config.PLL.PLLM, osc_config.PLL.PLLN, 
            osc_config.PLL.PLLP, osc_config.PLL.PLLQ);
    cli_send_string(buffer);
}

static void cmd_echo(void)
{
    cli_send_string("Echo: ");
    for (int i = 0; i < arg_count; i++)
    {
        cli_send_string(args[i]);
        if (i < arg_count - 1)
            cli_send_string(" ");
    }
    cli_send_string("\r\n");
}

static void cmd_stepgen(void)
{
    cli_send_string("Stepgen command deprecated.\r\n");
    cli_send_string("Use 'motor' commands instead:\r\n");
    cli_send_string("  motor m0 info     - Show FR steer stepgen status\r\n");
    cli_send_string("  motor m1 vel 100 50 - Set FR drive to 100 SPS\r\n");
    cli_send_string("  motor all info    - Show all motor/stepgen status\r\n");
}

// Helper function to get motor by name
static Motor* get_motor_by_name(const char* name)
{
    if (strcmp(name, "m0") == 0) return &m0;  // FR steer
    if (strcmp(name, "m1") == 0) return &m1;  // FR drive
    if (strcmp(name, "m2") == 0) return &m2;  // FL steer
    if (strcmp(name, "m3") == 0) return &m3;  // FL drive
    if (strcmp(name, "m4") == 0) return &m4;  // R steer
    if (strcmp(name, "m5") == 0) return &m5;  // R drive
    return NULL;
}

// Helper function to get motor description
static const char* get_motor_desc(const char* name)
{
    if (strcmp(name, "m0") == 0) return "FR steer";
    if (strcmp(name, "m1") == 0) return "FR drive";
    if (strcmp(name, "m2") == 0) return "FL steer";
    if (strcmp(name, "m3") == 0) return "FL drive";
    if (strcmp(name, "m4") == 0) return "R steer";
    if (strcmp(name, "m5") == 0) return "R drive";
    return "unknown";
}

static void cmd_motor(void)
{
    char buffer[80];
    
    if (arg_count == 1 || (arg_count == 2 && strcmp(args[1], "help") == 0))
    {
        cli_send_string("Motor Commands:\r\n");
        cli_send_string("Single Motor:\r\n");
        cli_send_string("  motor <name> info               - Show motor status\r\n");
        cli_send_string("  motor <name> vel <sps> <accel>  - Set velocity mode\r\n");
        cli_send_string("  motor <name> pos <steps> <v> <a> - Move relative steps\r\n");
        cli_send_string("  motor <name> stop               - Smooth stop\r\n");
        cli_send_string("Group Control:\r\n");
        cli_send_string("  motor all info                  - Show all motor status\r\n");
        cli_send_string("  motor all vel <sps> <accel>     - Control ALL 6 motors\r\n");
        cli_send_string("  motor drives vel <sps> <accel>  - Control drive motors (m1,m3,m5)\r\n");
        cli_send_string("  motor steers vel <sps> <accel>  - Control steer motors (m0,m2,m4)\r\n");
        cli_send_string("  motor drives|steers|all pos <steps> <v> <a> - Move groups by steps\r\n");
        cli_send_string("  motor drives|steers|all stop    - Stop motor groups\r\n");
        cli_send_string("Motor Names:\r\n");
        cli_send_string("  m0=FR steer, m1=FR drive, m2=FL steer\r\n");
        cli_send_string("  m3=FL drive, m4=R steer, m5=R drive\r\n");
        cli_send_string("Examples:\r\n");
        cli_send_string("  motor m1 vel 1000 500      - FR drive: 1000 SPS\r\n");
        cli_send_string("  motor m0 pos 1000 800 400  - FR steer: move 1000 steps\r\n");
        cli_send_string("  motor drives pos 500 600 200 - All drives: move 500 steps\r\n");
        cli_send_string("  motor steers pos -200 400 300 - All steers: move -200 steps\r\n");
        cli_send_string("  motor all vel 100 300      - ALL 6 motors: 100 SPS\r\n");
        cli_send_string("  motor steers stop          - Stop all steering\r\n");
        cli_send_string("  motor help                 - Show this help\r\n");
        return;
    }
    
    if (arg_count >= 2)
    {
        // Handle group commands: all, drives, steers
        if (strcmp(args[1], "all") == 0 || strcmp(args[1], "drives") == 0 || strcmp(args[1], "steers") == 0)
        {
            const char* group_name = args[1];
            
            // Define motor groups
            Motor* all_motors[] = {&m0, &m1, &m2, &m3, &m4, &m5};
            const char* all_names[] = {"m0", "m1", "m2", "m3", "m4", "m5"};
            Motor* drive_motors[] = {&m1, &m3, &m5};  // FR, FL, R drives
            const char* drive_names[] = {"m1", "m3", "m5"};
            Motor* steer_motors[] = {&m0, &m2, &m4};  // FR, FL, R steers
            const char* steer_names[] = {"m0", "m2", "m4"};
            
            Motor** target_motors;
            const char** target_names;
            int count;
            
            if (strcmp(group_name, "all") == 0)
            {
                target_motors = all_motors;
                target_names = all_names;
                count = 6;
            }
            else if (strcmp(group_name, "drives") == 0)
            {
                target_motors = drive_motors;
                target_names = drive_names;
                count = 3;
            }
            else // steers
            {
                target_motors = steer_motors;
                target_names = steer_names;
                count = 3;
            }
            
            if (arg_count >= 3)
            {
                if (strcmp(args[2], "info") == 0)
                {
                    sprintf(buffer, "%s Motor Status:\r\n", 
                        strcmp(group_name, "all") == 0 ? "All" :
                        strcmp(group_name, "drives") == 0 ? "Drive" : "Steer");
                    cli_send_string(buffer);
                    
                    for (int i = 0; i < count; i++)
                    {
                        sprintf(buffer, "  %s (%s): %ld SPS, %ld pos, %s\r\n",
                            target_names[i], get_motor_desc(target_names[i]),
                            motor_get_velocity_sps(target_motors[i]),
                            motor_get_position_steps(target_motors[i]),
                            motor_is_busy(target_motors[i]) ? "BUSY" : "IDLE"
                        );
                        cli_send_string(buffer);
                    }
                }
                else if (strcmp(args[2], "vel") == 0 && arg_count >= 5)
                {
                    int32_t sps_target = atoi(args[3]);
                    int32_t accel = atoi(args[4]);
                    
                    for (int i = 0; i < count; i++)
                    {
                        motor_set_velocity(target_motors[i], sps_target, accel);
                    }
                    sprintf(buffer, "%s motors velocity: %ld SPS, %ld SPS/s accel\r\n",
                        strcmp(group_name, "all") == 0 ? "All" :
                        strcmp(group_name, "drives") == 0 ? "Drive" : "Steer",
                        sps_target, accel);
                    cli_send_string(buffer);
                }
                else if (strcmp(args[2], "pos") == 0 && arg_count >= 6)
                {
                    int32_t steps = atoi(args[3]);
                    int32_t v_max = atoi(args[4]);
                    int32_t accel = atoi(args[5]);
                    
                    for (int i = 0; i < count; i++)
                    {
                        motor_move_steps(target_motors[i], steps, v_max, accel);
                    }
                    sprintf(buffer, "%s motors move: %ld steps, %ld max SPS, %ld SPS/s accel\r\n",
                        strcmp(group_name, "all") == 0 ? "All" :
                        strcmp(group_name, "drives") == 0 ? "Drive" : "Steer",
                        steps, v_max, accel);
                    cli_send_string(buffer);
                }
                else if (strcmp(args[2], "stop") == 0)
                {
                    for (int i = 0; i < count; i++)
                    {
                        motor_stop_smooth(target_motors[i]);
                    }
                    sprintf(buffer, "%s motors smooth stop initiated\r\n",
                        strcmp(group_name, "all") == 0 ? "All" :
                        strcmp(group_name, "drives") == 0 ? "Drive" : "Steer");
                    cli_send_string(buffer);
                }
                else
                {
                    cli_send_string("Invalid group command. Use: info, vel <sps> <accel>, pos <steps> <v> <a>, stop\r\n");
                }
            }
            else
            {
                cli_send_string("Missing group command. Use: info, vel <sps> <accel>, pos <steps> <v> <a>, stop\r\n");
            }
            return;
        }
        
        // Get motor by name
        Motor* motor = get_motor_by_name(args[1]);
        if (!motor)
        {
            sprintf(buffer, "Unknown motor: %s\r\n", args[1]);
            cli_send_string(buffer);
            cli_send_string("Valid motors: m0, m1, m2, m3, m4, m5\r\n");
            return;
        }
        
        if (arg_count >= 3)
        {
            if (strcmp(args[2], "info") == 0)
            {
                sprintf(buffer, "Motor %s (%s) Status:\r\n", args[1], get_motor_desc(args[1]));
                cli_send_string(buffer);
                sprintf(buffer, "  Velocity: %ld SPS\r\n", motor_get_velocity_sps(motor));
                cli_send_string(buffer);
                sprintf(buffer, "  Position: %ld steps\r\n", motor_get_position_steps(motor));
                cli_send_string(buffer);
                sprintf(buffer, "  Busy: %s\r\n", motor_is_busy(motor) ? "Yes" : "No");
                cli_send_string(buffer);
                
                const char* state_names[] = {"IDLE", "VEL", "POS_ACCEL", "POS_CRUISE", "POS_DECEL"};
                if (motor->state < 5)
                {
                    sprintf(buffer, "  State: %s\r\n", state_names[motor->state]);
                    cli_send_string(buffer);
                }
                
                sprintf(buffer, "  Max SPS: %lu\r\n", motor->hw.sps_max);
                cli_send_string(buffer);
                
                // Show pulse counter info
                if (motor->hw.pulse_cnt_htim)
                {
                    uint32_t hw_count = __HAL_TIM_GET_COUNTER(motor->hw.pulse_cnt_htim);
                    sprintf(buffer, "  HW Counter: %lu (delta: %ld)\r\n", hw_count, (int32_t)(hw_count - motor->last_cnt));
                    cli_send_string(buffer);
                }
                else
                {
                    cli_send_string("  HW Counter: Not configured\r\n");
                }
                
                // Show position move info if active
                if (motor->state >= M_POS_ACCEL && motor->state <= M_POS_DECEL)
                {
                    sprintf(buffer, "  Remaining: %ld steps (HW), %ld planned\r\n", 
                        motor->remaining_steps, Q16_TO_SPS(motor->planned_remaining_q16));
                    cli_send_string(buffer);
                }
            }
            else if (strcmp(args[2], "vel") == 0 && arg_count >= 5)
            {
                int32_t sps_target = atoi(args[3]);
                int32_t accel = atoi(args[4]);
                
                motor_set_velocity(motor, sps_target, accel);
                sprintf(buffer, "Motor %s (%s) velocity: %ld SPS, %ld SPS/s accel\r\n", 
                    args[1], get_motor_desc(args[1]), sps_target, accel);
                cli_send_string(buffer);
            }
            else if (strcmp(args[2], "move") == 0 && arg_count >= 6)
            {
                int32_t steps = atoi(args[3]);
                int32_t v_max = atoi(args[4]);
                int32_t accel = atoi(args[5]);
                
                motor_move_steps(motor, steps, v_max, accel);
                sprintf(buffer, "Motor %s (%s) move: %ld steps, %ld max SPS, %ld SPS/s accel\r\n",
                    args[1], get_motor_desc(args[1]), steps, v_max, accel);
                cli_send_string(buffer);
            }
            else if (strcmp(args[2], "pos") == 0 && arg_count >= 6)
            {
                int32_t steps = atoi(args[3]);
                int32_t v_max = atoi(args[4]);
                int32_t accel = atoi(args[5]);
                
                motor_move_steps(motor, steps, v_max, accel);
                sprintf(buffer, "Motor %s (%s) move: %ld steps, %ld max SPS, %ld SPS/s accel\r\n",
                    args[1], get_motor_desc(args[1]), steps, v_max, accel);
                cli_send_string(buffer);
            }
            else if (strcmp(args[2], "stop") == 0)
            {
                motor_stop_smooth(motor);
                sprintf(buffer, "Motor %s (%s) smooth stop initiated\r\n", 
                    args[1], get_motor_desc(args[1]));
                cli_send_string(buffer);
            }
            else
            {
                cli_send_string("Invalid motor command. Type 'motor' for help.\r\n");
            }
        }
        else
        {
            cli_send_string("Missing command. Type 'motor' for help.\r\n");
        }
    }
}

static void cmd_tick(void)
{
    char buffer[80];
    const TimerDiagnostics* diag = app_timers_get_diagnostics();
    
    if (arg_count >= 2 && strcmp(args[1], "reset") == 0)
    {
        app_timers_reset_diagnostics();
        cli_send_string("Timer diagnostics reset\r\n");
        return;
    }
    
    cli_send_string("TIM6 5kHz ISR Diagnostics:\r\n");
    
    sprintf(buffer, "  Total Ticks: %lu\r\n", diag->tick_count);
    cli_send_string(buffer);
    
    if (diag->tick_count > 0)
    {
        uint32_t runtime_sec = diag->tick_count / 5000;
        sprintf(buffer, "  Runtime: %lu seconds\r\n", runtime_sec);
        cli_send_string(buffer);
        
        uint32_t actual_freq = (runtime_sec > 0) ? diag->tick_count / runtime_sec : 0;
        sprintf(buffer, "  Actual Frequency: ~%lu Hz\r\n", actual_freq);
        cli_send_string(buffer);
    }
    
    sprintf(buffer, "  Avg ISR Duration: %lu us\r\n", diag->avg_duration_us);
    cli_send_string(buffer);
    
    sprintf(buffer, "  Peak ISR Duration: %lu us\r\n", diag->max_duration_us);
    cli_send_string(buffer);
    
    sprintf(buffer, "  ISR Overruns: %lu\r\n", diag->overrun_count);
    cli_send_string(buffer);
    
    if (diag->tick_count == 0)
    {
        cli_send_string("  Status: NOT RUNNING (check TIM6 config)\r\n");
    }
    else if (diag->overrun_count > (diag->tick_count / 1000)) // >0.1% overruns
    {
        cli_send_string("  Status: WARNING - High overrun rate\r\n");
    }
    else
    {
        cli_send_string("  Status: Running normally\r\n");
    }
    
    cli_send_string("\r\nUsage: tick reset - Reset counters\r\n");
}

static void cmd_pc(void)
{
    char buffer[100];
    
    if (arg_count >= 2 && strcmp(args[1], "help") == 0)
    {
        cli_send_string("Pulse Count Commands:\r\n");
        cli_send_string("  pc           - Show all motor pulse counts\r\n");
        cli_send_string("  pc <motor>   - Show specific motor pulse count\r\n");
        cli_send_string("  pc all       - Show all motor pulse counts\r\n");
        cli_send_string("  pc reset     - Reset all pulse counters to zero\r\n");
        cli_send_string("  pc pins      - Show timer input pin assignments\r\n");
        cli_send_string("  pc help      - Show this help\r\n");
        cli_send_string("Examples:\r\n");
        cli_send_string("  pc m0        - Show m0 pulse count\r\n");
        cli_send_string("  pc all       - Show all pulse counts\r\n");
        return;
    }
    
    Motor* motors[] = {&m0, &m1, &m2, &m3, &m4, &m5};
    const char* motor_names[] = {"m0", "m1", "m2", "m3", "m4", "m5"};
    
    if (arg_count >= 2 && strcmp(args[1], "reset") == 0)
    {
        // Reset all pulse counters
        Motor* motors[] = {&m0, &m1, &m2, &m3, &m4, &m5};
        cli_send_string("Resetting all pulse counters...\r\n");
        for (int i = 0; i < 6; i++)
        {
            if (motors[i]->hw.pulse_cnt_htim)
            {
                __HAL_TIM_SET_COUNTER(motors[i]->hw.pulse_cnt_htim, 0);
                motors[i]->last_cnt = 0;
                motors[i]->pos_steps = 0;
            }
        }
        cli_send_string("All pulse counters reset to zero\r\n");
        return;
    }
    else if (arg_count >= 2 && strcmp(args[1], "pins") == 0)
    {
        cli_send_string("Timer Input Pin Assignments:\r\n");
        cli_send_string("Counter Timers (External Clock Mode):\r\n");
        cli_send_string("  TIM1 CH1 → PA8  (m0 FR steer counter)\r\n");
        cli_send_string("  TIM2 CH1 → PA5  (m1 FR drive counter)\r\n");
        cli_send_string("  TIM3 CH1 → PA6  (m2 FL steer counter)\r\n");  
        cli_send_string("  TIM4 CH1 → PB6  (m3 FL drive counter)\r\n");
        cli_send_string("  TIM5 CH1 → PA0  (m4 R steer counter)\r\n");
        cli_send_string("  TIM8 CH1 → PC6  (m5 R drive counter)\r\n");
        cli_send_string("PWM Output Timers:\r\n");
        cli_send_string("  TIM9  CH1 → PE5  (m0 FR steer PWM)\r\n");
        cli_send_string("  TIM10 CH1 → PB8  (m1 FR drive PWM)\r\n");
        cli_send_string("  TIM11 CH1 → PB9  (m2 FL steer PWM)\r\n");
        cli_send_string("  TIM12 CH1 → PB14 (m3 FL drive PWM)\r\n");
        cli_send_string("  TIM13 CH1 → PF8  (m4 R steer PWM)\r\n");
        cli_send_string("  TIM14 CH1 → PF9  (m5 R drive PWM)\r\n");
        cli_send_string("\r\nEach counter should ONLY count its corresponding PWM!\r\n");
        return;
    }
    else if (arg_count == 1 || (arg_count >= 2 && strcmp(args[1], "all") == 0))
    {
        // Show all motor pulse counts
        cli_send_string("All Motor Pulse Counts:\r\n");
        for (int i = 0; i < 6; i++)
        {
            if (motors[i]->hw.pulse_cnt_htim)
            {
                uint32_t hw_count = __HAL_TIM_GET_COUNTER(motors[i]->hw.pulse_cnt_htim);
                // Show more detailed info to debug crosstalk
                int32_t current_velocity = motor_get_velocity_sps(motors[i]);
                const char* timer_name = "";
                if (motors[i]->hw.pulse_cnt_htim->Instance == TIM1) timer_name = "TIM1";
                else if (motors[i]->hw.pulse_cnt_htim->Instance == TIM2) timer_name = "TIM2";
                else if (motors[i]->hw.pulse_cnt_htim->Instance == TIM3) timer_name = "TIM3";
                else if (motors[i]->hw.pulse_cnt_htim->Instance == TIM4) timer_name = "TIM4";
                else if (motors[i]->hw.pulse_cnt_htim->Instance == TIM5) timer_name = "TIM5";
                else if (motors[i]->hw.pulse_cnt_htim->Instance == TIM8) timer_name = "TIM8";
                
                sprintf(buffer, "  %s (%s): HW=%lu, Pos=%ld, Vel=%ld, %s\r\n",
                    motor_names[i], get_motor_desc(motor_names[i]),
                    hw_count, motors[i]->pos_steps, current_velocity, timer_name);
            }
            else
            {
                sprintf(buffer, "  %s (%s): No counter timer\r\n",
                    motor_names[i], get_motor_desc(motor_names[i]));
            }
            cli_send_string(buffer);
        }
    }
    else if (arg_count >= 2)
    {
        // Show specific motor pulse count
        Motor* motor = get_motor_by_name(args[1]);
        if (!motor)
        {
            sprintf(buffer, "Unknown motor: %s\r\n", args[1]);
            cli_send_string(buffer);
            cli_send_string("Valid motors: m0, m1, m2, m3, m4, m5\r\n");
            return;
        }
        
        sprintf(buffer, "Motor %s (%s) Pulse Count:\r\n", args[1], get_motor_desc(args[1]));
        cli_send_string(buffer);
        
        if (motor->hw.pulse_cnt_htim)
        {
            uint32_t hw_count = __HAL_TIM_GET_COUNTER(motor->hw.pulse_cnt_htim);
            sprintf(buffer, "  Hardware Counter: %lu\r\n", hw_count);
            cli_send_string(buffer);
            sprintf(buffer, "  Position Steps: %ld\r\n", motor->pos_steps);
            cli_send_string(buffer);
            sprintf(buffer, "  Last Polled: %lu\r\n", motor->last_cnt);
            cli_send_string(buffer);
            sprintf(buffer, "  Delta Since Poll: %ld\r\n", (int32_t)(hw_count - motor->last_cnt));
            cli_send_string(buffer);
        }
        else
        {
            cli_send_string("  No counter timer configured\r\n");
        }
    }
}

static void cli_send_string(const char* str)
{
    if (str)
    {
        comm_io_write((const uint8_t*)str, strlen(str));
    }
}

static int cli_receive_line(char* buffer, int max_len)
{
    static char line_buffer[CLI_MAX_COMMAND_LEN];
    static int line_pos = 0;
    
    int c;
    while ((c = comm_io_getchar()) != -1)
    {
        if (c == '\r' || c == '\n')
        {
            if (line_pos > 0)
            {
                line_buffer[line_pos] = '\0';
                strncpy(buffer, line_buffer, max_len - 1);
                buffer[max_len - 1] = '\0';
                line_pos = 0;
                return strlen(buffer);
            }
        }
        else if (c >= 32 && c <= 126)  // Printable ASCII characters
        {
            if (line_pos < (CLI_MAX_COMMAND_LEN - 1))
            {
                line_buffer[line_pos++] = c;
            }
        }
        else if (c == 8 || c == 127)  // Backspace or DEL
        {
            if (line_pos > 0)
            {
                line_pos--;
            }
        }
    }
    
    return 0;
}

// Initialize swerve modules once
static bool swerve_modules_initialized = false;

static void init_swerve_modules(void)
{
    if (swerve_modules_initialized) return;
    
    // Initialize swerve modules with motor configuration
    // FR: m0=steer, m1=drive
    swerve_module_init(&swerve_fr, &m0, &m1, 
                      1600.0f, 1600.0f,    // steps per rev at motor
                      1000, 5000,          // steer limits  
                      2000, 10000);        // drive limits
    
    // FL: m2=steer, m3=drive  
    swerve_module_init(&swerve_fl, &m2, &m3,
                      1600.0f, 1600.0f,    // steps per rev at motor
                      1000, 5000,          // steer limits
                      2000, 10000);        // drive limits
    
    // R: m4=steer, m5=drive
    swerve_module_init(&swerve_r, &m4, &m5,
                      1600.0f, 1600.0f,    // steps per rev at motor
                      1000, 5000,          // steer limits
                      2000, 10000);        // drive limits
    
    swerve_modules_initialized = true;
    
    // Initialize kinematics system with slew limiting
    if (!kinematics_initialized) {
        SKM_Config cfg = {
            .x = {0.1950f, 0.1950f, -0.1950f},      // FR, FL, R positions (x)
            .y = {-0.1925f, 0.1925f, 0.0000f},      // FR, FL, R positions (y)
            .vx_max = 2.0f,                          // Max chassis forward speed
            .vy_max = 2.0f,                          // Max chassis sideways speed  
            .wz_max = 3.0f,                          // Max yaw rate
            .v_module_max = 2.0f,                    // Max individual wheel speed
            .dvx_max = 0.050f,                       // 5 m/s² forward accel limit
            .dvy_max = 0.050f,                       // 5 m/s² sideways accel limit
            .dwz_max = 0.100f                        // 10 rad/s² yaw accel limit
        };
        skm_init(&swerve_kin, &cfg);
        kinematics_initialized = true;
    }
}

static SwerveModule* get_swerve_module(const char* name)
{
    init_swerve_modules();
    
    if (strcmp(name, "fr") == 0) return &swerve_fr;
    if (strcmp(name, "fl") == 0) return &swerve_fl;
    if (strcmp(name, "r") == 0) return &swerve_r;
    return NULL;
}

static void cmd_swerve(void)
{
    char buffer[100];
    
    if (arg_count == 1 || (arg_count == 2 && strcmp(args[1], "help") == 0))
    {
        cli_send_string("Swerve Module Commands:\r\n");
        cli_send_string("  swerve <module> info                    - Show module status\r\n");
        cli_send_string("  swerve <module> home <offset_steps>     - Home the steer motor\r\n");
        cli_send_string("  swerve <module> cmd <angle_deg> <vel_mps> - Set angle and velocity\r\n");
        cli_send_string("  swerve <module> cal <steer_spr> <drive_spm> - Set calibration\r\n");
        cli_send_string("  swerve <module> tol <angle_deg>         - Set angle tolerance\r\n");
        cli_send_string("  swerve <module> debug                   - Show angle calculation debug\r\n");
        cli_send_string("  swerve <module> reset                   - Emergency reset position counter\r\n");
        cli_send_string("  swerve all info                         - Show all modules\r\n");
        cli_send_string("  swerve all cmd <angle_deg> <vel_mps>    - Command all modules\r\n");
        cli_send_string("  swerve all home <offset_steps>          - Home all modules\r\n");
        cli_send_string("\r\nModules: fr (front-right), fl (front-left), r (rear)\r\n");
        cli_send_string("Examples:\r\n");
        cli_send_string("  swerve fr home 0        - Home FR module\r\n");
        cli_send_string("  swerve fr cmd 45 1.0    - FR: 45°, 1.0 m/s\r\n");
        cli_send_string("  swerve all cmd 0 1.5    - All modules: 0°, 1.5 m/s\r\n");
        cli_send_string("  swerve all home 0       - Home all modules\r\n");
        cli_send_string("  swerve fl cal 3700 1000 - FL: 3700 steps/rad, 1000 steps/m\r\n");
        return;
    }
    
    if (arg_count < 3)
    {
        cli_send_string("Usage: swerve <module> <cmd> [args] or swerve help\r\n");
        return;
    }
    
    // Handle "all" modules
    if (strcmp(args[1], "all") == 0)
    {
        init_swerve_modules();
        SwerveModule* modules[] = {&swerve_fr, &swerve_fl, &swerve_r};
        const char* names[] = {"FR", "FL", "R"};
        
        if (strcmp(args[2], "info") == 0)
        {
            cli_send_string("All Swerve Modules Status:\r\n");
            
            for (int i = 0; i < 3; i++)
            {
                int32_t angle_deg_x10 = (int32_t)(swerve_module_get_current_angle(modules[i]) * 1800.0f / 3.14159f);
                int32_t vel_x100 = (int32_t)(swerve_module_get_current_velocity(modules[i]) * 100);
                snprintf(buffer, sizeof(buffer), "%s: homed=%s, angle=%d.%d deg, vel=%d.%02d m/s, inv=%d\r\n",
                        names[i],
                        modules[i]->is_homed ? "yes" : "no",
                        (int)(angle_deg_x10/10), abs((int)(angle_deg_x10%10)),
                        (int)(vel_x100/100), abs((int)(vel_x100%100)),
                        modules[i]->drive_inverted);
                cli_send_string(buffer);
            }
            return;
        }
        else if (strcmp(args[2], "cmd") == 0 && arg_count >= 5)
        {
            float angle_deg = atof(args[3]);
            float velocity_mps = atof(args[4]);
            float angle_rad = angle_deg * 3.14159f / 180.0f;
            
            // Check if all modules are homed
            bool all_homed = true;
            for (int i = 0; i < 3; i++)
            {
                if (!modules[i]->is_homed)
                {
                    snprintf(buffer, sizeof(buffer), "Module %s not homed\r\n", names[i]);
                    cli_send_string(buffer);
                    all_homed = false;
                }
            }
            
            if (!all_homed)
            {
                cli_send_string("All modules must be homed first\r\n");
                return;
            }
            
            // Command all modules
            for (int i = 0; i < 3; i++)
            {
                swerve_module_set_command(modules[i], angle_rad, velocity_mps);
            }
            
            int32_t angle_deg_x10 = (int32_t)(angle_deg * 10);
            int32_t vel_x100 = (int32_t)(velocity_mps * 100);
            snprintf(buffer, sizeof(buffer), "All modules: cmd %d.%d deg, %d.%02d m/s\r\n", 
                    (int)(angle_deg_x10/10), abs((int)(angle_deg_x10%10)),
                    (int)(vel_x100/100), abs((int)(vel_x100%100)));
            cli_send_string(buffer);
            return;
        }
        else if (strcmp(args[2], "home") == 0 && arg_count >= 4)
        {
            int32_t offset = atoi(args[3]);
            
            for (int i = 0; i < 3; i++)
            {
                swerve_module_home(modules[i], offset);
            }
            
            snprintf(buffer, sizeof(buffer), "All modules homed with offset %ld steps\r\n", (long)offset);
            cli_send_string(buffer);
            return;
        }
        else
        {
            cli_send_string("Unknown 'all' command. Use: info, cmd <angle> <vel>, home <offset>\r\n");
            return;
        }
    }
    
    // Single module commands
    SwerveModule* module = get_swerve_module(args[1]);
    if (!module)
    {
        cli_send_string("Unknown module. Use: fr, fl, r\r\n");
        return;
    }
    
    const char* cmd = args[2];
    
    if (strcmp(cmd, "info") == 0)
    {
        snprintf(buffer, sizeof(buffer), "Module %s:\r\n", args[1]);
        cli_send_string(buffer);
        
        snprintf(buffer, sizeof(buffer), "  Homed: %s\r\n", module->is_homed ? "yes" : "no");
        cli_send_string(buffer);
        
        if (module->is_homed)
        {
            int32_t cur_angle_deg_x10 = (int32_t)(swerve_module_get_current_angle(module) * 1800.0f / 3.14159f);
            int32_t cmd_angle_deg_x10 = (int32_t)(module->angle_cmd_rad * 1800.0f / 3.14159f);
            snprintf(buffer, sizeof(buffer), "  Angle: %d.%d deg (cmd: %d.%d deg)\r\n",
                    (int)(cur_angle_deg_x10/10), abs((int)(cur_angle_deg_x10%10)),
                    (int)(cmd_angle_deg_x10/10), abs((int)(cmd_angle_deg_x10%10)));
            cli_send_string(buffer);
            
            snprintf(buffer, sizeof(buffer), "  At angle: %s\r\n", 
                     swerve_module_is_at_angle(module) ? "yes" : "no");
            cli_send_string(buffer);
        }
        
        int32_t vel_x100 = (int32_t)(swerve_module_get_current_velocity(module) * 100);
        snprintf(buffer, sizeof(buffer), "  Velocity: %d.%02d m/s (inverted: %s)\r\n",
                (int)(vel_x100/100), abs((int)(vel_x100%100)),
                module->drive_inverted ? "yes" : "no");
        cli_send_string(buffer);
        
        int32_t limit_deg = (int32_t)(SWERVE_STEER_LIMIT_RAD * 180.0f / 3.14159f);
        int32_t wheel_x1000 = (int32_t)(SWERVE_WHEEL_RADIUS_M * 1000);
        snprintf(buffer, sizeof(buffer), "  Limits: ±%ld deg, wheel=%ld.%03ld m\r\n",
                limit_deg, wheel_x1000/1000, wheel_x1000%1000);
        cli_send_string(buffer);
        
        int32_t tol_deg_x10 = (int32_t)(SWERVE_ANGLE_TOL_RAD * 1800.0f / 3.14159f);
        int32_t tol_steps = (int32_t)(SWERVE_ANGLE_TOL_RAD * module->steer_steps_per_rad);
        snprintf(buffer, sizeof(buffer), "  Tolerance: %d.%d deg (%ld steps)\r\n",
                (int)(tol_deg_x10/10), abs((int)(tol_deg_x10%10)), (long)tol_steps);
        cli_send_string(buffer);
    }
    else if (strcmp(cmd, "home") == 0)
    {
        if (arg_count < 4)
        {
            cli_send_string("Usage: swerve <module> home <offset_steps>\r\n");
            return;
        }
        
        int32_t offset = atoi(args[3]);
        swerve_module_home(module, offset);
        
        snprintf(buffer, sizeof(buffer), "Module %s homed with offset %ld steps\r\n", 
                 args[1], (long)offset);
        cli_send_string(buffer);
    }
    else if (strcmp(cmd, "cmd") == 0)
    {
        if (arg_count < 5)
        {
            cli_send_string("Usage: swerve <module> cmd <angle_deg> <velocity_mps>\r\n");
            return;
        }
        
        if (!module->is_homed)
        {
            cli_send_string("Module must be homed first\r\n");
            return;
        }
        
        float angle_deg = atof(args[3]);
        float velocity_mps = atof(args[4]);
        float angle_rad = angle_deg * 3.14159f / 180.0f;
        
        swerve_module_set_command(module, angle_rad, velocity_mps);
        
        int32_t angle_deg_x10 = (int32_t)(angle_deg * 10);
        int32_t vel_x100 = (int32_t)(velocity_mps * 100);
        snprintf(buffer, sizeof(buffer), "Module %s: cmd %d.%d deg, %d.%02d m/s\r\n", 
                args[1], (int)(angle_deg_x10/10), abs((int)(angle_deg_x10%10)),
                (int)(vel_x100/100), abs((int)(vel_x100%100)));
        cli_send_string(buffer);
    }
    else if (strcmp(cmd, "cal") == 0)
    {
        if (arg_count < 5)
        {
            cli_send_string("Usage: swerve <module> cal <steer_steps_per_rad> <drive_steps_per_m>\r\n");
            return;
        }
        
        float steer_spr = atof(args[3]);
        float drive_spm = atof(args[4]);
        
        swerve_module_set_calibration(module, steer_spr, drive_spm, WHEEL_RADIUS_M);
        
        int32_t steer_spr_int = (int32_t)steer_spr;
        int32_t drive_spm_int = (int32_t)drive_spm;
        snprintf(buffer, sizeof(buffer), "Module %s calibrated: %ld steps/rad, %ld steps/m\r\n", 
                args[1], steer_spr_int, drive_spm_int);
        cli_send_string(buffer);
    }
    else if (strcmp(cmd, "tol") == 0)
    {
        if (arg_count < 4)
        {
            cli_send_string("Usage: swerve <module> tol <angle_deg>\r\n");
            return;
        }
        
        float tol_deg = atof(args[3]);
        float tol_rad = tol_deg * 3.14159f / 180.0f;
        
        // Note: New swerve module uses fixed tolerance SWERVE_ANGLE_TOL_RAD
        
        int32_t tol_steps = (int32_t)(tol_rad * module->steer_steps_per_rad);
        int32_t tol_deg_x10 = (int32_t)(tol_deg * 10);
        snprintf(buffer, sizeof(buffer), "Module %s tolerance: %d.%d deg (%ld steps)\r\n", 
                args[1], (int)(tol_deg_x10/10), abs((int)(tol_deg_x10%10)), (long)tol_steps);
        cli_send_string(buffer);
    }
    else if (strcmp(cmd, "debug") == 0)
    {
        if (!module->is_homed)
        {
            cli_send_string("Module must be homed first\r\n");
            return;
        }
        
        snprintf(buffer, sizeof(buffer), "Module %s Debug Info:\r\n", args[1]);
        cli_send_string(buffer);
        
        // Get hardware counter and software position
        int32_t hw_count = 0;
        int32_t sw_pos = motor_get_position_steps(module->steer);
        if (module->steer->hw.pulse_cnt_htim) {
            hw_count = (int32_t)__HAL_TIM_GET_COUNTER(module->steer->hw.pulse_cnt_htim);
        }
        
        snprintf(buffer, sizeof(buffer), "  HW Counter: %ld steps\r\n", hw_count);
        cli_send_string(buffer);
        snprintf(buffer, sizeof(buffer), "  SW Position: %ld steps\r\n", sw_pos);
        cli_send_string(buffer);
        snprintf(buffer, sizeof(buffer), "  Zero Offset: %ld steps\r\n", module->steer_zero_offset_steps);
        cli_send_string(buffer);
        
        float current_angle = swerve_module_get_current_angle(module);
        snprintf(buffer, sizeof(buffer), "  Current Angle: approx %ld deg\r\n", 
                (int32_t)(current_angle * 180.0f / 3.14159f));
        cli_send_string(buffer);
        
        snprintf(buffer, sizeof(buffer), "  Command Angle: approx %ld deg\r\n", 
                (int32_t)(module->angle_cmd_rad * 180.0f / 3.14159f));
        cli_send_string(buffer);
        
        float angle_diff = angle_difference(module->angle_cmd_rad, current_angle);
        snprintf(buffer, sizeof(buffer), "  Angle Error: approx %ld deg\r\n", 
                (int32_t)(angle_diff * 180.0f / 3.14159f));
        cli_send_string(buffer);
        
        int32_t target_steps = angle_to_steps(module->angle_cmd_rad, module->steer_steps_per_rad, 
                                              module->steer_zero_offset_steps);
        int32_t steps_needed = target_steps - hw_count;
        snprintf(buffer, sizeof(buffer), "  Target Steps: %ld, Need: %ld steps\r\n", 
                target_steps, steps_needed);
        cli_send_string(buffer);
        
        int32_t threshold_steps = (int32_t)(SWERVE_ANGLE_TOL_RAD * module->steer_steps_per_rad);
        snprintf(buffer, sizeof(buffer), "  Movement Threshold: %ld steps\r\n", threshold_steps);
        cli_send_string(buffer);
    }
    else if (strcmp(cmd, "reset") == 0)
    {
        // Emergency reset - set hardware counter to zero offset position
        if (module->steer->hw.pulse_cnt_htim) {
            __HAL_TIM_SET_COUNTER(module->steer->hw.pulse_cnt_htim, module->steer_zero_offset_steps);
        }
        module->steer->pos_steps = module->steer_zero_offset_steps;
        
        snprintf(buffer, sizeof(buffer), "Module %s position reset to zero offset\r\n", args[1]);
        cli_send_string(buffer);
    }
    else
    {
        cli_send_string("Unknown command. Use: info, home, cmd, cal, tol, debug, reset\r\n");
    }
}

// Function to run kinematics at 100Hz and send to modules
static void update_twist_kinematics(void)
{
    if (!kinematics_initialized) return;
    
    SKM_Out out;
    skm_update_100Hz(&swerve_kin, &out);
    
    // Display computed kinematics for debugging
    char buffer[150];
    const char* names[] = {"FR", "FL", "R"};
    cli_send_string("Kinematics Output:\r\n");
    for (int i = 0; i < 3; i++) {
        int32_t angle_deg_x10 = (int32_t)(out.angle_rad[i] * 1800.0f / 3.14159f);
        int32_t speed_x100 = (int32_t)(out.speed_mps[i] * 100);
        snprintf(buffer, sizeof(buffer), "  %s: angle=%d.%d deg, speed=%d.%02d m/s\r\n",
                names[i],
                (int)(angle_deg_x10/10), abs((int)(angle_deg_x10%10)),
                (int)(speed_x100/100), abs((int)(speed_x100%100)));
        cli_send_string(buffer);
    }
    
    // Show slewing and scaling info
    int32_t vx_cmd_x100 = (int32_t)(swerve_kin.vx_cmd * 100);
    int32_t vx_actual_x100 = (int32_t)(swerve_kin.vx * 100);
    int32_t scale_x100 = (int32_t)(out.scale_factor * 100);
    
    snprintf(buffer, sizeof(buffer), "Slewing: vx_cmd=%d.%02d, vx_actual=%d.%02d, scale=%d.%02d\r\n",
            (int)(vx_cmd_x100/100), abs((int)(vx_cmd_x100%100)),
            (int)(vx_actual_x100/100), abs((int)(vx_actual_x100%100)),
            (int)(scale_x100/100), abs((int)(scale_x100%100)));
    cli_send_string(buffer);
    
    // Send computed angles and speeds to swerve modules
    // Let swerve modules handle shortest path and inversion logic
    SwerveModule* modules[] = {&swerve_fr, &swerve_fl, &swerve_r};
    for (int i = 0; i < 3; i++) {
        if (modules[i] && modules[i]->is_homed) {
            swerve_module_set_angle_abs(modules[i], out.angle_rad[i]);
            swerve_module_set_wheel_speed(modules[i], out.speed_mps[i]);
        }
    }
}

static void cmd_twist(void)
{
    char buffer[100];
    
    init_swerve_modules();
    
    if (arg_count == 1 || (arg_count == 2 && strcmp(args[1], "help") == 0))
    {
        cli_send_string("Twist Commands:\r\n");
        cli_send_string("  twist <vx> <vy> <wz>    - Set body twist (m/s, m/s, rad/s)\r\n");
        cli_send_string("  twist stop              - Stop all motion\r\n");
        cli_send_string("  twist status            - Show current twist command\r\n");
        cli_send_string("  twist update            - Manually update kinematics\r\n");
        cli_send_string("\r\nFrames: +x forward, +y left, +z up (angles CCW+)\r\n");
        cli_send_string("Examples:\r\n");
        cli_send_string("  twist 0.3 0 0       - Pure forward 0.3 m/s\r\n");
        cli_send_string("  twist 0 0.3 0       - Pure left 0.3 m/s\r\n");
        cli_send_string("  twist 0 0 0.5       - Pure yaw 0.5 rad/s CCW\r\n");
        cli_send_string("  twist 0.2 0.2 0     - Diagonal motion\r\n");
        cli_send_string("  twist stop          - Emergency stop\r\n");
        return;
    }
    
    if (arg_count >= 2 && strcmp(args[1], "stop") == 0)
    {
        skm_set_twist(&swerve_kin, 0.0f, 0.0f, 0.0f);
        update_twist_kinematics();
        cli_send_string("Robot motion stopped\r\n");
        return;
    }
    
    if (arg_count >= 2 && strcmp(args[1], "update") == 0)
    {
        update_twist_kinematics();
        cli_send_string("Kinematics updated\r\n");
        return;
    }
    
    if (arg_count >= 2 && strcmp(args[1], "status") == 0)
    {
        int32_t vx_cmd_x100 = (int32_t)(swerve_kin.vx_cmd * 100);
        int32_t vy_cmd_x100 = (int32_t)(swerve_kin.vy_cmd * 100);
        int32_t wz_cmd_x100 = (int32_t)(swerve_kin.wz_cmd * 100);
        
        int32_t vx_actual_x100 = (int32_t)(swerve_kin.vx * 100);
        int32_t vy_actual_x100 = (int32_t)(swerve_kin.vy * 100);
        int32_t wz_actual_x100 = (int32_t)(swerve_kin.wz * 100);
        
        snprintf(buffer, sizeof(buffer), "Twist Commands:\r\n");
        cli_send_string(buffer);
        snprintf(buffer, sizeof(buffer), "  vx_cmd: %d.%02d m/s, vx_actual: %d.%02d m/s\r\n", 
                (int)(vx_cmd_x100/100), abs((int)(vx_cmd_x100%100)),
                (int)(vx_actual_x100/100), abs((int)(vx_actual_x100%100)));
        cli_send_string(buffer);
        snprintf(buffer, sizeof(buffer), "  vy_cmd: %d.%02d m/s, vy_actual: %d.%02d m/s\r\n", 
                (int)(vy_cmd_x100/100), abs((int)(vy_cmd_x100%100)),
                (int)(vy_actual_x100/100), abs((int)(vy_actual_x100%100)));
        cli_send_string(buffer);
        snprintf(buffer, sizeof(buffer), "  wz_cmd: %d.%02d rad/s, wz_actual: %d.%02d rad/s\r\n", 
                (int)(wz_cmd_x100/100), abs((int)(wz_cmd_x100%100)),
                (int)(wz_actual_x100/100), abs((int)(wz_actual_x100%100)));
        cli_send_string(buffer);
        
        // Show module positions and limits
        cli_send_string("Module Positions:\r\n");
        const char* names[] = {"FR", "FL", "R"};
        for (int i = 0; i < 3; i++) {
            int32_t x_x1000 = (int32_t)(swerve_kin.cfg.x[i] * 1000);
            int32_t y_x1000 = (int32_t)(swerve_kin.cfg.y[i] * 1000);
            snprintf(buffer, sizeof(buffer), "  %s: (%d.%03d, %d.%03d) m\r\n", 
                    names[i],
                    (int)(x_x1000/1000), abs((int)(x_x1000%1000)),
                    (int)(y_x1000/1000), abs((int)(y_x1000%1000)));
            cli_send_string(buffer);
        }
        return;
    }
    
    if (arg_count < 4)
    {
        cli_send_string("Usage: twist <vx> <vy> <wz> or twist stop/status/help/update\r\n");
        return;
    }
    
    float vx = atof(args[1]);
    float vy = atof(args[2]); 
    float wz = atof(args[3]);
    
    skm_set_twist(&swerve_kin, vx, vy, wz);
    update_twist_kinematics();  // Apply immediately
    
    int32_t vx_x100 = (int32_t)(vx * 100);
    int32_t vy_x100 = (int32_t)(vy * 100);
    int32_t wz_x100 = (int32_t)(wz * 100);
    
    snprintf(buffer, sizeof(buffer), "Twist command: vx=%d.%02d m/s, vy=%d.%02d m/s, wz=%d.%02d rad/s\r\n", 
            (int)(vx_x100/100), abs((int)(vx_x100%100)),
            (int)(vy_x100/100), abs((int)(vy_x100%100)),
            (int)(wz_x100/100), abs((int)(wz_x100%100)));
    cli_send_string(buffer);
}

static void cmd_mod(void)
{
    char buffer[150];
    
    init_swerve_modules();
    
    if (arg_count < 3 || strcmp(args[2], "stat") != 0)
    {
        cli_send_string("Usage: mod <i> stat\r\n");
        cli_send_string("Modules: 0=FR, 1=FL, 2=R\r\n");
        return;
    }
    
    int module_idx = atoi(args[1]);
    if (module_idx < 0 || module_idx >= 3)
    {
        cli_send_string("Invalid module index. Use 0=FR, 1=FL, 2=R\r\n");
        return;
    }
    
    SwerveModule* modules[] = {&swerve_fr, &swerve_fl, &swerve_r};
    const char* names[] = {"FR", "FL", "R"};
    SwerveModule* mod = modules[module_idx];
    
    snprintf(buffer, sizeof(buffer), "Module %d (%s) Status:\r\n", module_idx, names[module_idx]);
    cli_send_string(buffer);
    
    snprintf(buffer, sizeof(buffer), "  Homed: %s\r\n", mod->is_homed ? "yes" : "no");
    cli_send_string(buffer);
    
    if (mod->is_homed)
    {
        int32_t angle_now_deg_x10 = (int32_t)(swerve_module_get_angle_abs(mod) * 1800.0f / 3.14159f);
        int32_t angle_cmd_deg_x10 = (int32_t)(mod->angle_cmd_rad * 1800.0f / 3.14159f);
        snprintf(buffer, sizeof(buffer), "  angle_now: %d.%d deg\r\n",
                (int)(angle_now_deg_x10/10), abs((int)(angle_now_deg_x10%10)));
        cli_send_string(buffer);
        snprintf(buffer, sizeof(buffer), "  angle_cmd: %d.%d deg\r\n",
                (int)(angle_cmd_deg_x10/10), abs((int)(angle_cmd_deg_x10%10)));
        cli_send_string(buffer);
    }
    
    snprintf(buffer, sizeof(buffer), "  inverted: %s\r\n", mod->drive_inverted ? "yes" : "no");
    cli_send_string(buffer);
    
    // Show kinematic output if available
    SKM_Out out;
    skm_update_100Hz(&swerve_kin, &out);
    int32_t v_cmd_x100 = (int32_t)(out.speed_mps[module_idx] * 100);
    int32_t ang_cmd_deg_x10 = (int32_t)(out.angle_rad[module_idx] * 1800.0f / 3.14159f);
    snprintf(buffer, sizeof(buffer), "  v_cmd: %d.%02d m/s (scaled by %d.%02d)\r\n",
            (int)(v_cmd_x100/100), abs((int)(v_cmd_x100%100)),
            (int)(out.scale_factor*100/100), abs((int)(out.scale_factor*100%100)));
    cli_send_string(buffer);
    snprintf(buffer, sizeof(buffer), "  angle_target: %d.%d deg (from kinematics)\r\n",
            (int)(ang_cmd_deg_x10/10), abs((int)(ang_cmd_deg_x10%10)));
    cli_send_string(buffer);
    
    int32_t v_actual_x100 = (int32_t)(swerve_module_get_current_velocity(mod) * 100);
    snprintf(buffer, sizeof(buffer), "  v_actual: %d.%02d m/s\r\n",
            (int)(v_actual_x100/100), abs((int)(v_actual_x100%100)));
    cli_send_string(buffer);
}

static void cmd_kin(void)
{
    char buffer[100];
    
    init_swerve_modules();
    
    if (arg_count == 1 || (arg_count == 2 && strcmp(args[1], "help") == 0))
    {
        cli_send_string("Kinematics Commands:\r\n");
        cli_send_string("  kin show                    - Show current module positions\r\n");
        cli_send_string("  kin setpos <mod> <x> <y>    - Set module position (meters)\r\n");
        cli_send_string("  kin limits                  - Show velocity and acceleration limits\r\n");
        cli_send_string("\r\nModules: fr, fl, r\r\n");
        cli_send_string("Examples:\r\n");
        cli_send_string("  kin show                - Show all positions\r\n");
        cli_send_string("  kin setpos fr 0.3 -0.2  - Set FR at (0.3, -0.2) m\r\n");
        return;
    }
    
    if (arg_count >= 2 && strcmp(args[1], "show") == 0)
    {
        cli_send_string("Module Positions (from robot center):\r\n");
        
        const char* mod_names[] = {"FR", "FL", "R"};
        for (int i = 0; i < 3; i++)
        {
            float x = swerve_kin.cfg.x[i];
            float y = swerve_kin.cfg.y[i];
            
            int32_t x_x1000 = (int32_t)(x * 1000);
            int32_t y_x1000 = (int32_t)(y * 1000);
            
            snprintf(buffer, sizeof(buffer), "  %s: (%d.%03d, %d.%03d) m\r\n", 
                    mod_names[i],
                    (int)(x_x1000/1000), abs((int)(x_x1000%1000)),
                    (int)(y_x1000/1000), abs((int)(y_x1000%1000)));
            cli_send_string(buffer);
        }
        return;
    }
    
    if (arg_count >= 2 && strcmp(args[1], "limits") == 0)
    {
        cli_send_string("Minimal Kinematics - No built-in limits\r\n");
        cli_send_string("Limits are handled by individual swerve modules:\r\n");
        cli_send_string("  - Steering limits: ±150° (mechanical)\r\n");
        cli_send_string("  - Drive limits: Set per module in swerve init\r\n");
        cli_send_string("  - Use 'swerve <module> info' for module-specific limits\r\n");
        return;
    }
    
    if (arg_count >= 5 && strcmp(args[1], "setpos") == 0)
    {
        int module_idx = -1;
        if (strcmp(args[2], "fr") == 0) module_idx = 0;
        else if (strcmp(args[2], "fl") == 0) module_idx = 1;
        else if (strcmp(args[2], "r") == 0) module_idx = 2;
        
        if (module_idx < 0)
        {
            cli_send_string("Invalid module. Use: fr, fl, r\r\n");
            return;
        }
        
        float x = atof(args[3]);
        float y = atof(args[4]);
        
        // Update the kinematics config directly
        swerve_kin.cfg.x[module_idx] = x;
        swerve_kin.cfg.y[module_idx] = y;
        
        int32_t x_x1000 = (int32_t)(x * 1000);
        int32_t y_x1000 = (int32_t)(y * 1000);
        
        snprintf(buffer, sizeof(buffer), "Module %s position set to (%d.%03d, %d.%03d) m\r\n", 
                args[2],
                (int)(x_x1000/1000), abs((int)(x_x1000%1000)),
                (int)(y_x1000/1000), abs((int)(y_x1000%1000)));
        cli_send_string(buffer);
        return;
    }
    
    cli_send_string("Usage: kin show/limits or kin setpos <module> <x> <y>\r\n");
}