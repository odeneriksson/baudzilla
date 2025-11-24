#ifndef BAUDZILLA_COMMON_H
#define BAUDZILLA_COMMON_H

#include <stdio.h>
#include <stddef.h>

/*
 * Shared banner function for all Baudzilla tools
 */
static inline void print_banner(void) {
    printf(
" ____                  _     _ _ _\n"
"| __ )  __ _ _   _  __| |___(_) | | __ _ \n"
"|  _ \\ / _` | | | |/ _` |_  / | | |/ _` |\n"
"| |_) | (_| | |_| | (_| |/ /| | | | (_| |\n"
"|____/ \\__,_|\\__,_|\\__,_/___|_|_|_|\\__,_|\n"
    );
}

/*
 * Shared utility functions implemented in baudzilla_common.c
 */

/* Return current timestamp in a human readable form "YYYY-MM-DD HH:MM:SS" */
void get_timestamp(char *buffer, size_t size);

/* Map an integer baudrate (e.g. 9600) to the corresponding termios constant */
int get_baudrate(int baud);

#endif /* BAUDZILLA_COMMON_H */
