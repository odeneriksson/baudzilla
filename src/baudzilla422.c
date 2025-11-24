/*
 * Baudzilla 422 - RS-422 line and protocol test tool
 *
 * (C) Oden Eriksson
 * Licensed under GPLv2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <https://www.gnu.org/licenses/>.
 */

/*
 * Purpose:
 *   Test a single RS-422 serial interface using:
 *     - Loopback data patterns
 *     - Idle line noise check (linecheck)
 *     - Modbus RTU probe
 *     - IEC 60870-5-101 probe
 *
 * Features:
 *   - Configurable baudrate, payload length and loop count
 *   - Multiple pattern generators (ascii, hex, random, walking, checker)
 *   - Linecheck mode to see if the line is noisy when idle
 *   - Modbus RTU function 3 probe (unit 1)
 *   - Simple IEC101 fixed frame and GI probe
 */

#include "config.h"
#include "baudzilla_common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <time.h>
#include <sys/ioctl.h>

#define BUFFER_SIZE 1024

static int verbose = 0;
static const char *current_port = NULL;


/* Timestamp */



/* Baud mapping */



/* Configure RS-422 port (essentially standard 8N1, no flow control) */
static int configure_port(int fd, int baudrate) {
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        return -1;
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;

    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tty.c_cc[VTIME] = 5;   /* 0.5 s read timeout */
    tty.c_cc[VMIN]  = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return -1;
    }

    return 0;
}

/* Patterns */
typedef enum {
    PATTERN_ASCII = 0,
    PATTERN_HEX,
    PATTERN_RANDOM,
    PATTERN_WALKING,
    PATTERN_CHECKER
} pattern_type;

static void fill_pattern(unsigned char *buf, size_t len,
                         pattern_type pat, const char *base_str)
{
    size_t i;

    switch (pat) {
        case PATTERN_ASCII:
            if (!base_str || base_str[0] == '\0')
                base_str = "RS422TEST";
            for (i = 0; i < len; i++)
                buf[i] = (unsigned char)base_str[i % strlen(base_str)];
            break;

        case PATTERN_HEX:
            for (i = 0; i < len; i++)
                buf[i] = (unsigned char)(i & 0xFF);
            break;

        case PATTERN_RANDOM:
            for (i = 0; i < len; i++)
                buf[i] = (unsigned char)(rand() & 0xFF);
            break;

        case PATTERN_WALKING: {
            unsigned char bit = 1;
            for (i = 0; i < len; i++) {
                buf[i] = bit;
                bit <<= 1;
                if (bit == 0)
                    bit = 1;
            }
            break;
        }

        case PATTERN_CHECKER:
            for (i = 0; i < len; i++)
                buf[i] = (i & 1) ? 0xAA : 0x55;
            break;
    }
}

/* Linecheck: see if we get noise on an idle line */
static void run_linecheck(int fd, int timeout, FILE *log) {
    unsigned char buf[BUFFER_SIZE];
    int loops = timeout * 10;
    int total_bytes = 0;

    fprintf(log, "\n=== Line check on port %s (idle noise detection) ===\n",
            current_port ? current_port : "unknown");
    if (verbose)
        printf("\n=== Line check on port %s (idle noise detection) ===\n",
               current_port ? current_port : "unknown");

    tcflush(fd, TCIFLUSH);

    while (loops-- > 0) {
        int n = read(fd, buf, sizeof(buf));
        if (n > 0) {
            total_bytes += n;
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            fprintf(log, "Linecheck: read error\n");
            if (verbose)
                perror("read");
            break;
        }
    }

    fprintf(log, "Linecheck on port %s: total bytes seen on idle line: %d\n",
            current_port ? current_port : "unknown", total_bytes);
    if (verbose)
        printf("Linecheck on port %s: total bytes seen on idle line: %d\n",
               current_port ? current_port : "unknown", total_bytes);
}

/* Loopback test: requires that TX/RX of RS-422 port are physically looped */
static int run_loopback_test(int fd, int loops, int timeout, size_t length,
                             pattern_type pat, const char *teststring,
                             FILE *log)
{
    unsigned char txbuf[BUFFER_SIZE];
    unsigned char rxbuf[BUFFER_SIZE];
    int success = 0;
    int fail = 0;
    double total_time = 0.0;

    if (length > BUFFER_SIZE)
        length = BUFFER_SIZE;

    for (int i = 0; i < loops; i++) {
        char ts[64];
        get_timestamp(ts, sizeof(ts));

        fprintf(log, "\n[%s] --- Loop %d ---\n", ts, i + 1);
        if (verbose)
            printf("\nLoop %d\n", i + 1);

        if (pat == PATTERN_ASCII)
            fill_pattern(txbuf, length, pat, teststring);
        else
            fill_pattern(txbuf, length, pat, NULL);

        tcflush(fd, TCIFLUSH);

        struct timespec start, end;
        clock_gettime(CLOCK_MONOTONIC, &start);

        ssize_t written = write(fd, txbuf, length);
        if (written != (ssize_t)length) {
            fprintf(log, "Loop %d: write failed (%zd/%zu)\n",
                    i + 1, written, length);
            if (verbose)
                printf("Loop %d: write failed\n", i + 1);
            fail++;
            continue;
        }

        size_t total_read = 0;
        int loops_read = timeout * 20;

        while (loops_read-- > 0 && total_read < length) {
            size_t space = sizeof(rxbuf) - total_read;
            if (space == 0)
                break;
            ssize_t n = read(fd,
                             rxbuf + total_read,
                             space);
            if (n > 0)
                total_read += (size_t)n;
            else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                fprintf(log, "Loop %d: read error\n", i + 1);
                if (verbose)
                    perror("read");
                break;
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &end);
        double elapsed = (end.tv_sec - start.tv_sec) +
                         (end.tv_nsec - start.tv_nsec) / 1e9;
        total_time += elapsed;

        if (total_read != length ||
            memcmp(txbuf, rxbuf, length) != 0) {
            fprintf(log, "Loop %d: mismatch (%zu/%zu bytes)\n",
                    i + 1, total_read, length);
            if (verbose)
                printf("Loop %d: mismatch\n", i + 1);
            fail++;
        } else {
            fprintf(log, "Loop %d: OK (%zu bytes, %.3f s)\n",
                    i + 1, total_read, elapsed);
            if (verbose)
                printf("Loop %d: OK (%.3f s)\n", i + 1, elapsed);
            success++;
        }
    }

    double avg = (loops > 0) ? total_time / loops : 0.0;
    fprintf(log, "\nLoopback summary on port %s: success=%d fail=%d avg_time=%.3f s\n",
            current_port ? current_port : "unknown", success, fail, avg);

    return (fail == 0);
}

/* Modbus CRC */
static unsigned short modbus_crc16(const unsigned char *buf, size_t len) {
    unsigned short crc = 0xFFFF;
    for (size_t pos = 0; pos < len; pos++) {
        crc ^= (unsigned short)buf[pos];
        for (int i = 0; i < 8; i++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* Modbus RTU probe */
static void run_modbus_probe(int fd, int timeout, FILE *log) {
    unsigned char req[8];
    unsigned char resp[BUFFER_SIZE];

    fprintf(log, "\n=== Modbus RTU probe ===\n");
    if (verbose)
        printf("\n=== Modbus RTU probe ===\n");

    req[0] = 1;       /* Unit id */
    req[1] = 3;       /* Function code: Read holding registers */
    req[2] = 0x00;    /* Starting address high */
    req[3] = 0x00;    /* Starting address low */
    req[4] = 0x00;    /* Quantity high */
    req[5] = 0x02;    /* Quantity low (2 registers) */

    unsigned short crc = modbus_crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = (crc >> 8) & 0xFF;

    tcflush(fd, TCIFLUSH);

    if (write(fd, req, 8) != 8) {
        fprintf(log, "Modbus: failed to write request\n");
        if (verbose)
            printf("Modbus: failed to write request\n");
        return;
    }

    int loops = timeout * 20;
    size_t total = 0;

    while (loops-- > 0 && total < sizeof(resp)) {
        ssize_t n = read(fd, resp + total, sizeof(resp) - total);
        if (n > 0)
            total += (size_t)n;
        else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
            break;
    }

    if (total == 0) {
        fprintf(log, "Modbus: no response\n");
        if (verbose)
            printf("Modbus: no response\n");
        return;
    }

    fprintf(log, "Modbus response (%zu bytes): ", total);
    if (verbose)
        printf("Modbus response (%zu bytes): ", total);

    for (size_t i = 0; i < total; i++) {
        fprintf(log, "%02X ", resp[i]);
        if (verbose)
            printf("%02X ", resp[i]);
    }
    fprintf(log, "\n");
    if (verbose)
        printf("\n");
}

/* IEC 60870-5-101 helpers */
static unsigned char iec101_checksum(const unsigned char *buf, int len) {
    unsigned int sum = 0;
    for (int i = 0; i < len; i++)
        sum += buf[i];
    return (unsigned char)(sum & 0xFF);
}

static int iec101_send_fixed(int fd, unsigned char control, unsigned char address) {
    unsigned char frame[6];

    frame[0] = 0x10;
    frame[1] = control;
    frame[2] = address;
    frame[3] = 0x00;
    frame[4] = iec101_checksum(&frame[1], 3);
    frame[5] = 0x16;

    return (write(fd, frame, 6) == 6);
}

static int iec101_send_general_interrogation(int fd, int common_addr) {
    unsigned char asdu[16];
    int i = 0;

    asdu[i++] = 100;  /* Type ID 100 */
    asdu[i++] = 1;    /* VSQ */
    asdu[i++] = 6;    /* COT activation */
    asdu[i++] = 0;

    asdu[i++] = common_addr & 0xFF;
    asdu[i++] = (common_addr >> 8) & 0xFF;

    asdu[i++] = 0;
    asdu[i++] = 0;
    asdu[i++] = 0;

    asdu[i++] = 20;   /* QOI global GI */

    unsigned char frame[32];
    int L = 1 + 1 + i;

    frame[0] = 0x68;
    frame[1] = (unsigned char)L;
    frame[2] = (unsigned char)L;
    frame[3] = 0x64;
    frame[4] = common_addr & 0xFF;
    memcpy(&frame[5], asdu, i);
    frame[5 + i] = iec101_checksum(&frame[3], 2 + i);
    frame[6 + i] = 0x16;

    return (write(fd, frame, 7 + i) == (7 + i));
}

static void iec101_read_response(int fd, int timeout, FILE *log) {
    unsigned char buf[BUFFER_SIZE];
    int loops = timeout * 20;
    size_t total = 0;

    while (loops-- > 0 && total < sizeof(buf)) {
        ssize_t n = read(fd, buf + total, sizeof(buf) - total);
        if (n > 0)
            total += (size_t)n;
        else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
            break;
    }

    if (total == 0) {
        fprintf(log, "IEC101: no response\n");
        if (verbose)
            printf("IEC101: no response\n");
        return;
    }

    fprintf(log, "IEC101 response (%zu bytes): ", total);
    if (verbose)
        printf("IEC101 response (%zu bytes): ", total);

    for (size_t i = 0; i < total; i++) {
        fprintf(log, "%02X ", buf[i]);
        if (verbose)
            printf("%02X ", buf[i]);
    }
    fprintf(log, "\n");
    if (verbose)
        printf("\n");
}

static void run_iec101_probe(int fd, int timeout, FILE *log) {
    fprintf(log, "\n=== IEC 60870-5-101 probe ===\n");
    if (verbose)
        printf("\n=== IEC 60870-5-101 probe ===\n");

    tcflush(fd, TCIFLUSH);

    if (!iec101_send_fixed(fd, 0x6B, 0x01)) {
        fprintf(log, "IEC101: failed to send fixed frame\n");
        if (verbose)
            printf("IEC101: failed to send fixed frame\n");
    } else {
        iec101_read_response(fd, timeout, log);
    }

    tcflush(fd, TCIFLUSH);

    if (!iec101_send_general_interrogation(fd, 1)) {
        fprintf(log, "IEC101: failed to send general interrogation\n");
        if (verbose)
            printf("IEC101: failed to send general interrogation\n");
    } else {
        iec101_read_response(fd, timeout, log);
    }
}

/* Help */
static void print_help(const char *progname) {
    print_banner();

    printf("\nbaudzilla422 v%s - RS-422 Test Tool\n", BAUDZILLA_VERSION);

    printf("--------------------------------\n");
    printf("Purpose:\n");
    printf("  Test an RS-422 serial interface using loopback patterns, line\n");
    printf("  stability checks, Modbus RTU probing and IEC 60870-5-101 probing.\n");

    printf("\n(C) Oden Eriksson\n");
    printf("License: GPLv2 - This program is free software under the GNU General Public License v2.\n");
    printf("See: https://www.gnu.org/licenses/old-licenses/gpl-2.0.html\n");

    printf("\nUsage:\n");
    printf("  %s -p <port> -l <logfile> -s <teststring> [-v]\n", progname);
    printf("     [-b <baudrate>] [-n <loops>] [-t <timeout>] [--length <bytes>]\n");
    printf("     [--pattern ascii|hex|random|walking|checker]\n");
    printf("     [--linecheck] [--modbus] [--iec101] [--all]\n");

    printf("\nOptions:\n");
    printf("  -v                 Verbose output\n");
    printf("  -p <port>          Serial device (e.g. /dev/ttyS1)\n");
    printf("  -l <logfile>       Log file path\n");
    printf("  -s <string>        Base test string for ASCII pattern\n");
    printf("  -b <baud>          Baudrate (default 9600)\n");
    printf("  -n <loops>         Loopback iterations (default 5)\n");
    printf("  -t <timeout>       Timeout seconds (default 1)\n");
    printf("  --length <bytes>   Payload length (default 64)\n");
    printf("  --pattern <name>   ascii|hex|random|walking|checker\n");
    printf("  --linecheck        Run idle line noise check\n");
    printf("  --modbus           Modbus RTU probe (unit 1, func 3)\n");
    printf("  --iec101           IEC60870-5-101 probe\n");
    printf("  --all              Enable linecheck, modbus and iec101\n");
    printf("  -h, --help         Show this help\n");

    printf("\nExample:\n");
    printf("  %s -v -p /dev/ttyS1 -b 9600 -n 10 -t 1 -l rs422.log -s \"RS422\" \\\n",
           progname);
    printf("     --pattern ascii --linecheck --modbus --iec101\n");
}

/* Main */
int main(int argc, char *argv[]) {
    const char *port = NULL;
    const char *logfile = NULL;
    const char *teststring = NULL;

    int baudrate = 9600;
    int loops = 5;
    int timeout = 1;
    size_t length = 64;

    int do_linecheck = 0;
    int do_modbus = 0;
    int do_iec101 = 0;
    pattern_type pat = PATTERN_ASCII;

    if (argc == 2 && (strcmp(argv[1], "-h") == 0 ||
                      strcmp(argv[1], "--help") == 0)) {
        print_help(argv[0]);
        return 0;
    }

    srand((unsigned int)time(NULL));

    for (int i = 1; i < argc; ) {
        if (strcmp(argv[i], "-v") == 0) {
            verbose = 1; i++;
        } else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            port = argv[i + 1]; i += 2;
        } else if (strcmp(argv[i], "-l") == 0 && i + 1 < argc) {
            logfile = argv[i + 1]; i += 2;
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            teststring = argv[i + 1]; i += 2;
        } else if (strcmp(argv[i], "-b") == 0 && i + 1 < argc) {
            baudrate = atoi(argv[i + 1]); i += 2;
        } else if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            loops = atoi(argv[i + 1]); i += 2;
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            timeout = atoi(argv[i + 1]); i += 2;
        } else if (strcmp(argv[i], "--length") == 0 && i + 1 < argc) {
            length = (size_t)atoi(argv[i + 1]); i += 2;
        } else if (strcmp(argv[i], "--pattern") == 0 && i + 1 < argc) {
            const char *p = argv[i + 1];
            if (strcmp(p, "ascii") == 0) pat = PATTERN_ASCII;
            else if (strcmp(p, "hex") == 0) pat = PATTERN_HEX;
            else if (strcmp(p, "random") == 0) pat = PATTERN_RANDOM;
            else if (strcmp(p, "walking") == 0) pat = PATTERN_WALKING;
            else if (strcmp(p, "checker") == 0) pat = PATTERN_CHECKER;
            i += 2;
        } else if (strcmp(argv[i], "--linecheck") == 0) {
            do_linecheck = 1; i++;
        } else if (strcmp(argv[i], "--modbus") == 0) {
            do_modbus = 1; i++;
        } else if (strcmp(argv[i], "--iec101") == 0) {
            do_iec101 = 1; i++;
        } else if (strcmp(argv[i], "--all") == 0) {
            do_linecheck = 1;
            do_modbus = 1;
            do_iec101 = 1;
            i++;
        } else {
            /* Unknown argument; skip */
            i++;
        }
    }

    if (!port || !teststring) {
        fprintf(stderr, "Error: missing required arguments.\n");
        print_help(argv[0]);
        return 1;
    }

    FILE *log;
    if (logfile) {
        log = fopen(logfile, "w");
        if (!log) { perror("fopen"); return 1; }
    } else {
        log = stdout;
    }

    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("open");
        if (log != stdout) fclose(log);
        return 1;
    }

    current_port = port;
    fprintf(log, "Using port %s\n", current_port);
    if (verbose)
        printf("Using port %s\n", current_port);

    if (configure_port(fd, get_baudrate(baudrate)) < 0) {
        close(fd);
        if (log != stdout) fclose(log);
        return 1;
    }

    if (do_linecheck)
        run_linecheck(fd, timeout, log);

    run_loopback_test(fd, loops, timeout, length, pat, teststring, log);

    if (do_modbus)
        run_modbus_probe(fd, timeout, log);

    if (do_iec101)
        run_iec101_probe(fd, timeout, log);

    close(fd);
    if (log != stdout) fclose(log);

    return 0;
}
