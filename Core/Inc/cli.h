#ifndef CLI_H
#define CLI_H

#include "comm_io.h"
#include <stdbool.h>

#define CLI_MAX_COMMAND_LEN 64
#define CLI_MAX_ARGS 8

void cli_init(void);
void cli_process(void);
void cli_print_prompt(void);

// Twist timeout functions (called from 100Hz control loop)
void twist_timeout_check(void);
bool twist_should_send_commands(void);
bool twist_should_send_drive_commands(void);

#endif