#include "cli.h"
#include "stepgen.h"
#include "motor.h"
#include "app_timers.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern RCC_ClkInitTypeDef RCC_ClkInitStruct;

// External motor instances for testing
extern Motor m0, m1, m2, m3, m4, m5;  // All 6 motors
extern Motor motor_test;  // Legacy test motor

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