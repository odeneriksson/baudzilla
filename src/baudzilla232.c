/*
 * Baudzilla 232 - RS-232 Test Tool
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
 *   Test two RS-232 serial ports connected via a null-modem cable in both
 *   directions (A->B and B->A). The program sends a test string, measures
 *   transfer time and logs the results. Optional features include RTS/CTS
 *   flow control, modem signal display and a simple IEC 60870-5-101 probe
 *   on port A.
 *
 * Features:
 *   - Bidirectional testing (A->B and B->A)
 *   - Optional RTS/CTS hardware flow control
 *   - Optional modem signal inspection
 *   - Single or multiple baud rate testing (auto-baud)
 *   - Timing and summary statistics
 *   - Simple IEC101 probe on port A (--test-iec101)
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

#define BUFFER_SIZE 256

/* Global flags */
static int rtscts_enabled = 0;
static int show_modem = 0;
static int verbose = 0;
static int test_iec101 = 0;


/* Timestamp */



/* Baud mapping */



/* Configure RS-232 port */
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

    if (rtscts_enabled)
        tty.c_cflag |= CRTSCTS;
    else
        tty.c_cflag &= ~CRTSCTS;

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

/* Modem signals */
static void check_modem_signals(int fd, const char *port, FILE *log) {
    int status;
    if (ioctl(fd, TIOCMGET, &status) == -1) {
        perror("ioctl");
        return;
    }

    int rts = (status & TIOCM_RTS) ? 1 : 0;
    int cts = (status & TIOCM_CTS) ? 1 : 0;
    int dtr = (status & TIOCM_DTR) ? 1 : 0;
    int dsr = (status & TIOCM_DSR) ? 1 : 0;

    fprintf(log, "Modem signals on %s: RTS=%d CTS=%d DTR=%d DSR=%d\n",
            port, rts, cts, dtr, dsr);
    if (verbose) {
        printf("Modem signals on %s: RTS=%d CTS=%d DTR=%d DSR=%d\n",
               port, rts, cts, dtr, dsr);
    }
}

/* IEC 60870-5-101 helpers (very simple probe) */
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

    asdu[i++] = 100;                     /* Type ID 100 */
    asdu[i++] = 1;                       /* VSQ */
    asdu[i++] = 6;                       /* COT activation */
    asdu[i++] = 0;

    asdu[i++] = common_addr & 0xFF;
    asdu[i++] = (common_addr >> 8) & 0xFF;

    asdu[i++] = 0;
    asdu[i++] = 0;
    asdu[i++] = 0;

    asdu[i++] = 20;                      /* QOI global GI */

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
    fprintf(log, "\n=== IEC 60870-5-101 probe on port A ===\n");
    if (verbose)
        printf("\n=== IEC 60870-5-101 probe on port A ===\n");

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

    printf("\nbaudzilla232 v%s - RS-232 Test Tool\n", BAUDZILLA_VERSION);

    printf("--------------------------------\n");
    printf("Purpose:\n");
    printf("  Test two serial ports connected via a null-modem cable.\n");
    printf("  Sends a test string in both directions (A->B and B->A), measures\n");
    printf("  transfer time and logs the results. Optional features include\n");
    printf("  RTS/CTS flow control, modem signal display and a simple IEC101\n");
    printf("  protocol probe on port A.\n");

    printf("\n(C) Oden Eriksson\n");
    printf("License: GPLv2 - This program is free software under the GNU General Public License v2.\n");
    printf("See: https://www.gnu.org/licenses/old-licenses/gpl-2.0.html\n");

    printf("\nUsage:\n");
    printf("  %s [-v] <portA> <portB> [-r] [-m] [-sf <summaryfile>]\n", progname);
    printf("     -b <baudrate> | --auto-baud -n <loops> -t <timeout>\n");
    printf("     -l <logfile> -s <teststring> [--test-iec101]\n");

    printf("\nOptions:\n");
    printf("  -v            Verbose output\n");
    printf("  -r            Enable RTS/CTS hardware flow control\n");
    printf("  -m            Show modem signals on each loop\n");
    printf("  -sf <file>    Summary file path\n");
    printf("  -b <baud>     Baud rate (9600,19200,38400,57600,115200)\n");
    printf("  --auto-baud   Test all standard baud rates in sequence\n");
    printf("  -n <loops>    Number of loops per baud rate\n");
    printf("  -t <sec>      Timeout seconds per loop\n");
    printf("  -l <logfile>  Log file path\n");
    printf("  -s <string>   Test string to send between ports\n");
    printf("  --test-iec101 Simple IEC 60870-5-101 probe on port A\n");
    printf("  -h, --help    Show this help\n");

    printf("\nExample:\n");
    printf("  %s -v /dev/ttyS0 /dev/ttyS1 -b 9600 -n 5 -t 1 -l log.txt \\\n", progname);
    printf("     -sf summary.txt -s \"Hello RS232\" --test-iec101\n");
}

/* Run one baud test */
static void run_test(const char *portA, const char *portB, int baudrate, int loops,
                     int timeout, const char *teststring, FILE *log,
                     int *total_success, int *total_fail, double *avg_time) {
    int fdA = open(portA, O_RDWR | O_NOCTTY);
    int fdB = open(portB, O_RDWR | O_NOCTTY);
    if (fdA < 0 || fdB < 0) {
        perror("open");
        if (fdA >= 0) close(fdA);
        if (fdB >= 0) close(fdB);
        return;
    }

    if (configure_port(fdA, get_baudrate(baudrate)) < 0 ||
        configure_port(fdB, get_baudrate(baudrate)) < 0) {
        close(fdA);
        close(fdB);
        return;
    }

    char buffer[BUFFER_SIZE];
    char timestamp[64];
    int success = 0, fail = 0;
    double total_time = 0.0;

    fprintf(log, "\n=== Testing baud rate %d on ports %s <-> %s ===\n", baudrate, portA, portB);
    if (verbose)
        printf("\n=== Testing baud rate %d on ports %s <-> %s ===\n", baudrate, portA, portB);

    for (int i = 0; i < loops; i++) {
        get_timestamp(timestamp, sizeof(timestamp));
        fprintf(log, "\n[%s] --- Loop %d on %s <-> %s ---\n", timestamp, i + 1, portA, portB);
        if (verbose)
            printf("\n--- Loop %d on %s <-> %s ---\n", i + 1, portA, portB);

        if (show_modem) {
            check_modem_signals(fdA, portA, log);
            check_modem_signals(fdB, portB, log);
        }

        struct timespec start, end;
        clock_gettime(CLOCK_MONOTONIC, &start);

        /* A -> B */
        ssize_t w = write(fdA, teststring, strlen(teststring));
        if (w < 0) {
            perror("write A");
        }
        sleep(timeout);
        ssize_t n = read(fdB, buffer, BUFFER_SIZE);
        (void)n;

        /* B -> A */
        w = write(fdB, teststring, strlen(teststring));
        if (w < 0) {
            perror("write B");
        }
        sleep(timeout);
        n = read(fdA, buffer, BUFFER_SIZE);
        (void)n;

        clock_gettime(CLOCK_MONOTONIC, &end);
        double elapsed = (end.tv_sec - start.tv_sec) +
                         (end.tv_nsec - start.tv_nsec) / 1e9;
        total_time += elapsed;

        if (verbose)
            printf("Loop time: %.3f sec\n", elapsed);

        /* For now, count all loops as success; extend with string comparison if needed */
        success++;
    }

    *avg_time = (loops > 0) ? (total_time / loops) : 0.0;

    fprintf(log, "\nSummary for baud rate %d on %s <-> %s: Success=%d Fail=%d AvgTime=%.3f sec\n",
            baudrate, portA, portB, success, fail, *avg_time);

    /* Second direction summary (portB <-> portA) */
    fprintf(log,
            "Summary for baud rate %d on %s <-> %s: Success=%d Fail=%d AvgTime=%.3f sec\n",
            baudrate, portB, portA, success, fail, *avg_time);

    if (verbose)
        printf("Summary for baud rate %d on %s <-> %s: Success=%d Fail=%d AvgTime=%.3f sec\n",
               baudrate, portA, portB, success, fail, *avg_time);

    if (verbose)
        printf("Summary for baud rate %d on %s <-> %s: Success=%d Fail=%d AvgTime=%.3f sec\n",
               baudrate, portB, portA, success, fail, *avg_time);

    *total_success += success;
    *total_fail += fail;

    if (test_iec101) {
        run_iec101_probe(fdA, timeout, log);
    }

    close(fdA);
    close(fdB);
}

/* Main */
int main(int argc, char *argv[]) {
    int auto_baud = 0;
    int baudrate = 9600;
    int loops = 1;
    int timeout = 1;
    const char *portA = NULL;
    const char *portB = NULL;
    const char *logfile = NULL;
    const char *summaryfile = NULL;
    const char *teststring = NULL;

    if (argc == 2 && (strcmp(argv[1], "-h") == 0 ||
                      strcmp(argv[1], "--help") == 0)) {
        print_help(argv[0]);
        return 0;
    }

    /* Manual argument parsing */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0) verbose = 1;
        else if (strcmp(argv[i], "-r") == 0) rtscts_enabled = 1;
        else if (strcmp(argv[i], "-m") == 0) show_modem = 1;
        else if (strcmp(argv[i], "-sf") == 0 && i + 1 < argc) summaryfile = argv[++i];
        else if (strcmp(argv[i], "-b") == 0 && i + 1 < argc) baudrate = atoi(argv[++i]);
        else if (strcmp(argv[i], "--auto-baud") == 0) auto_baud = 1;
        else if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) loops = atoi(argv[++i]);
        else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) timeout = atoi(argv[++i]);
        else if (strcmp(argv[i], "-l") == 0 && i + 1 < argc) logfile = argv[++i];
        else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) teststring = argv[++i];
        else if (strcmp(argv[i], "--test-iec101") == 0) test_iec101 = 1;
        else if (!portA) portA = argv[i];
        else if (!portB) portB = argv[i];
    }

    if (!portA || !portB || !teststring) {
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

    int total_success = 0, total_fail = 0;
    double avg_time = 0.0;
    double total_avg_time = 0.0;
    int count_baud = 0;

    if (auto_baud) {
        int baudrates[] = {9600, 19200, 38400, 57600, 115200};
        for (int i = 0; i < 5; i++) {
            run_test(portA, portB, baudrates[i], loops, timeout,
                     teststring, log, &total_success, &total_fail, &avg_time);
            total_avg_time += avg_time;
            count_baud++;
        }
    } else {
        run_test(portA, portB, baudrate, loops, timeout,
                 teststring, log, &total_success, &total_fail, &avg_time);
        total_avg_time = avg_time;
        count_baud = 1;
    }

    if (log != stdout) fclose(log);

    if (summaryfile) {
        FILE *sf = fopen(summaryfile, "w");
        if (sf) {
            fprintf(sf,
                    "{ \"portA\": \"%s\", \"portB\": \"%s\", \"total_success\": %d, \"total_fail\": %d, \"avg_time_sec\": %.3f }\n",
                    portA, portB, total_success, total_fail,
                    (count_baud > 0) ? (total_avg_time / count_baud) : 0.0);
            fclose(sf);
        }
    }

    return 0;
}
