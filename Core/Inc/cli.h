#ifndef CLI_H
#define CLI_H

#include "comm_io.h"

#define CLI_MAX_COMMAND_LEN 64
#define CLI_MAX_ARGS 8

void cli_init(void);
void cli_process(void);
void cli_print_prompt(void);

#endif