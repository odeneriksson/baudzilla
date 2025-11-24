/*
 * Baudzilla 485 - RS-485 bus and protocol test tool
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
 *   Test RS-485 hardware in three selectable modes:
 *      1. Linux kernel RS485 driver mode
 *      2. GPIO-controlled DE pin
 *      3. Manual DE via RTS toggling
 *
 *   Includes:
 *      - Loopback testing
 *      - Line noise detection
 *      - Modbus RTU probe
 *      - IEC 60870-5-101 probe
 *
 */

#define _GNU_SOURCE

#include "config.h"
#include "baudzilla_common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#define BUFFER_SIZE 1024

/* ----------------------------------------------------------
   Global flags and variables
   ---------------------------------------------------------- */

static int verbose = 0;
static const char *current_port = NULL;

enum rs485_mode {
    RS485_KERNEL = 0,
    RS485_GPIO,
    RS485_MANUAL_RTS
};

static enum rs485_mode mode = RS485_KERNEL;

/* GPIO path */
static const char *gpio_path = NULL;


/* ----------------------------------------------------------
   Timestamp
   ---------------------------------------------------------- */




/* ----------------------------------------------------------
   Baud table
   ---------------------------------------------------------- */




/* ----------------------------------------------------------
   Configure RS-485 kernel mode
   ---------------------------------------------------------- */

#include <linux/serial.h>

static int enable_kernel_rs485(int fd) {
    struct serial_rs485 rs;

    memset(&rs, 0, sizeof(rs));
    rs.flags |= SER_RS485_ENABLED;
    rs.flags |= SER_RS485_RTS_AFTER_SEND;
    rs.flags |= SER_RS485_RTS_ON_SEND;

    if (ioctl(fd, TIOCSRS485, &rs) < 0) {
        perror("TIOCSRS485");
        return -1;
    }

    return 0;
}

/* ----------------------------------------------------------
   GPIO DE control
   ---------------------------------------------------------- */

static int gpio_set(const char *path, int value) {
    int fd = open(path, O_WRONLY);
    if (fd < 0)
        return -1;

    const char *str = value ? "1" : "0";
    ssize_t w = write(fd, str, 1);
    if (w != 1) {
        int saved_errno = errno;
        close(fd);
        errno = saved_errno;
        return -1;
    }

    close(fd);
    return 0;
}

static int gpio_export(const char *path) {
    /* Caller ensures path e.g.: "/sys/class/gpio/gpio23/value" */
    /* We assume gpio is already exported */
    (void)path;
    return 0;
}

/* ----------------------------------------------------------
   Manual RTS DE control
   ---------------------------------------------------------- */

static void rts_set(int fd, int on) {
    int status;

    if (ioctl(fd, TIOCMGET, &status) < 0)
        return;

    if (on)
        status |= TIOCM_RTS;
    else
        status &= ~TIOCM_RTS;

    ioctl(fd, TIOCMSET, &status);
}

/* ----------------------------------------------------------
   Set 8N1 + baud + raw
   ---------------------------------------------------------- */

static int configure_port(int fd, int baudrate) {
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0)
        return -1;

    cfsetispeed(&tty, baudrate);
    cfsetospeed(&tty, baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;

    tty.c_iflag = IGNPAR;
    tty.c_lflag = 0;
    tty.c_oflag = 0;

    tty.c_cc[VTIME] = 5;
    tty.c_cc[VMIN] = 0;

    return tcsetattr(fd, TCSANOW, &tty);
}

/* ----------------------------------------------------------
   Pattern generator
   ---------------------------------------------------------- */

typedef enum {
    PAT_ASCII = 0,
    PAT_HEX,
    PAT_RANDOM,
    PAT_WALKING,
    PAT_CHECKER
} pattern_t;

static void fill_pattern(unsigned char *buf, size_t len,
                         pattern_t pat, const char *base)
{
    size_t i;

    switch (pat) {
        case PAT_ASCII:
            if (!base || base[0] == '\0')
                base = "RS485TEST";
            for (i = 0; i < len; i++)
                buf[i] = (unsigned char)base[i % strlen(base)];
            break;

        case PAT_HEX:
            for (i = 0; i < len; i++)
                buf[i] = (unsigned char)(i & 0xFF);
            break;

        case PAT_RANDOM:
            for (i = 0; i < len; i++)
                buf[i] = (unsigned char)(rand() & 0xFF);
            break;

        case PAT_WALKING: {
            unsigned char v = 1;
            for (i = 0; i < len; i++) {
                buf[i] = v;
                v <<= 1;
                if (v == 0)
                    v = 1;
            }
            break;
        }

        case PAT_CHECKER:
            for (i = 0; i < len; i++)
                buf[i] = (i & 1) ? 0xAA : 0x55;
            break;
    }
}

/* ----------------------------------------------------------
   Linecheck
   ---------------------------------------------------------- */

static void run_linecheck(int fd, int timeout, FILE *log) {
    unsigned char buf[BUFFER_SIZE];
    int loops = timeout * 10;
    int total = 0;

    fprintf(log, "\n=== Line check on port %s (noise on idle bus) ===\n",
            current_port ? current_port : "unknown");
    if (verbose)
        printf("\n=== Line check on port %s (noise on idle bus) ===\n",
               current_port ? current_port : "unknown");

    tcflush(fd, TCIFLUSH);

    while (loops-- > 0) {
        int n = read(fd, buf, sizeof(buf));
        if (n > 0)
            total += n;
    }

    fprintf(log, "Idle noise bytes on port %s: %d\n",
            current_port ? current_port : "unknown", total);
    if (verbose)
        printf("Idle noise bytes on port %s: %d\n",
               current_port ? current_port : "unknown", total);
}

/* ----------------------------------------------------------
   Loopback test
   ---------------------------------------------------------- */

static int do_write(int fd, const unsigned char *buf, size_t len) {
    switch (mode) {
        case RS485_KERNEL:
            return write(fd, buf, len);
        case RS485_GPIO:
            gpio_set(gpio_path, 1);
            {
                int n = write(fd, buf, len);
                usleep(2000);
                gpio_set(gpio_path, 0);
                return n;
            }
        case RS485_MANUAL_RTS:
            rts_set(fd, 1);
            {
                int n = write(fd, buf, len);
                usleep(2000);
                rts_set(fd, 0);
                return n;
            }
    }
    return -1;
}

static int run_loopback(int fd, int loops, int timeout, size_t len,
                        pattern_t pat, const char *ascii, FILE *log)
{
    unsigned char tx[BUFFER_SIZE];
    unsigned char rx[BUFFER_SIZE];

    if (len > BUFFER_SIZE)
        len = BUFFER_SIZE;

    int success = 0;
    int fail = 0;
    double total_time = 0;

    for (int i = 0; i < loops; i++) {
        char ts[64];
        get_timestamp(ts, sizeof(ts));
        fprintf(log, "\n[%s] Loop %d\n", ts, i + 1);

        fill_pattern(tx, len, pat, ascii);
        memset(rx, 0, len);

        tcflush(fd, TCIFLUSH);

        struct timespec s, e;
        clock_gettime(CLOCK_MONOTONIC, &s);

        ssize_t w = do_write(fd, tx, len);
        if (w != (ssize_t)len) {
            fprintf(log, "Write mismatch\n");
            fail++;
            continue;
        }

        size_t r = 0;
        int loops_r = timeout * 20;
        while (loops_r-- > 0 && r < len) {
            size_t space = sizeof(rx) - r;
            if (space == 0)
                break;
            ssize_t n = read(fd, rx + r, space);
            if (n > 0) {
                r += (size_t)n;
            } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                break;
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &e);
        double el = (e.tv_sec - s.tv_sec) +
                    (e.tv_nsec - s.tv_nsec) / 1e9;
        total_time += el;

        if (r != len || memcmp(tx, rx, len) != 0) {
            fprintf(log, "Mismatch (got %zu)\n", r);
            fail++;
        } else {
            fprintf(log, "OK (%.3f s)\n", el);
            success++;
        }
    }

    fprintf(log, "Loopback summary on port %s: success=%d fail=%d\n",
            current_port ? current_port : "unknown", success, fail);
    return fail == 0;
}

/* ----------------------------------------------------------
   Modbus RTU CRC + probe
   ---------------------------------------------------------- */

static unsigned short modbus_crc16(const unsigned char *buf, size_t len) {
    unsigned short crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

static void run_modbus(int fd, int timeout, FILE *log) {
    unsigned char req[8];
    unsigned char resp[BUFFER_SIZE];

    fprintf(log, "\n=== Modbus RTU probe ===\n");
    if (verbose)
        printf("\n=== Modbus RTU probe ===\n");

    req[0] = 1;
    req[1] = 3;
    req[2] = 0;
    req[3] = 0;
    req[4] = 0;
    req[5] = 2;

    unsigned short crc = modbus_crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = (crc >> 8) & 0xFF;

    tcflush(fd, TCIFLUSH);
    do_write(fd, req, 8);

    int loops = timeout * 20;
    size_t total = 0;

    while (loops-- > 0 && total < sizeof(resp)) {
        int n = read(fd, resp + total, sizeof(resp) - total);
        if (n > 0)
            total += n;
    }

    if (total == 0) {
        fprintf(log, "No response\n");
        return;
    }

    fprintf(log, "Response (%zu bytes): ", total);
    for (size_t i = 0; i < total; i++)
        fprintf(log, "%02X ", resp[i]);
    fprintf(log, "\n");
}

/* ----------------------------------------------------------
   IEC 101
   ---------------------------------------------------------- */

static unsigned char chk101(const unsigned char *buf, int n) {
    unsigned int s = 0;
    for (int i = 0; i < n; i++)
        s += buf[i];
    return (unsigned char)(s & 0xFF);
}

static int iec_fixed(int fd) {
    unsigned char f[6];
    f[0] = 0x10;
    f[1] = 0x6B;
    f[2] = 0x01;
    f[3] = 0x00;
    f[4] = chk101(&f[1], 3);
    f[5] = 0x16;
    return do_write(fd, f, 6) == 6;
}

static int iec_gi(int fd) {
    unsigned char asdu[16];
    int i = 0;

    asdu[i++] = 100;
    asdu[i++] = 1;
    asdu[i++] = 6;
    asdu[i++] = 0;
    asdu[i++] = 1;
    asdu[i++] = 0;
    asdu[i++] = 0;
    asdu[i++] = 0;
    asdu[i++] = 0;
    asdu[i++] = 20;

    unsigned char f[32];
    int L = 1 + 1 + i;

    f[0] = 0x68;
    f[1] = L;
    f[2] = L;
    f[3] = 0x64;
    f[4] = 1;
    memcpy(&f[5], asdu, i);
    f[5 + i] = chk101(&f[3], 2 + i);
    f[6 + i] = 0x16;

    return do_write(fd, f, 7 + i) == (7 + i);
}

static void read101(int fd, int timeout, FILE *log) {
    unsigned char buf[BUFFER_SIZE];
    int loops = timeout * 20;
    size_t total = 0;

    while (loops-- && total < sizeof(buf)) {
        int n = read(fd, buf + total, sizeof(buf) - total);
        if (n > 0)
            total += n;
    }

    fprintf(log, "IEC101 response (%zu bytes): ", total);
    for (size_t i = 0; i < total; i++)
        fprintf(log, "%02X ", buf[i]);
    fprintf(log, "\n");
}

static void run_iec101(int fd, int timeout, FILE *log) {
    fprintf(log, "\n=== IEC101 probe ===\n");
    if (verbose)
        printf("\n=== IEC101 probe ===\n");

    tcflush(fd, TCIFLUSH);

    iec_fixed(fd);
    read101(fd, timeout, log);

    tcflush(fd, TCIFLUSH);

    iec_gi(fd);
    read101(fd, timeout, log);
}

/* ----------------------------------------------------------
   Help
   ---------------------------------------------------------- */

static void print_help(const char *prog) {
    print_banner();

    printf("\nbaudzilla485 v%s - RS-485 Test Tool\n", BAUDZILLA_VERSION);

    printf("--------------------------------\n");
    printf("Purpose:\n");
    printf("  Test RS-485 interfaces using kernel RS485 mode, GPIO DE mode or\n");
    printf("  manual RTS-toggled DE. Includes loopback tests, line noise\n");
    printf("  detection, Modbus RTU probe and IEC101 probe.\n");

    printf("\n(C) Oden Eriksson\n");
    printf("License: GPLv2 - This program is free software under the GNU General Public License v2.\n");
    printf("See: https://www.gnu.org/licenses/old-licenses/gpl-2.0.html\n");
    printf("\nUsage:\n");
    printf("  %s -p <port> -l <logfile> -s <teststring> [options]\n", prog);

    printf("\nOptions:\n");
    printf("  -v                   Verbose output\n");
    printf("  -p <port>            Serial device (e.g. /dev/ttyS2)\n");
    printf("  -l <logfile>         Log file\n");
    printf("  -s <string>          ASCII test string\n");
    printf("  -b <baud>            Baudrate (default 9600)\n");
    printf("  -n <loops>           Loop count (default 5)\n");
    printf("  -t <timeout>         Timeout seconds (default 1)\n");
    printf("  --length <bytes>     Payload length\n");
    printf("  --pattern <name>     ascii|hex|random|walking|checker\n");
    printf("  --linecheck          Idle noise detection\n");
    printf("  --modbus             Modbus RTU probe\n");
    printf("  --iec101             IEC101 probe\n");

    printf("\nRS485 modes:\n");
    printf("  --kernel             Use kernel RS485 driver (default)\n");
    printf("  --gpio <path>        Use GPIO-controlled DE pin\n");
    printf("  --manual-rts         Use RTS as DE\n");

    printf("\nExample:\n");
    printf("  %s -v -p /dev/ttyS2 -l log.txt -s \"485\" \\\n", prog);
    printf("     --pattern ascii --manual-rts --modbus --iec101\n");
}

/* ----------------------------------------------------------
   Main
   ---------------------------------------------------------- */

int main(int argc, char *argv[]) {
    const char *port = NULL;
    const char *logfile = NULL;
    const char *teststring = NULL;

    int baudrate = 9600;
    int loops = 5;
    int timeout = 1;
    size_t length = 64;

    pattern_t pattern = PAT_ASCII;
    int do_linecheck = 0;
    int do_modbus = 0;
    int do_iec101 = 0;

    if (argc == 2 &&
       (strcmp(argv[1], "-h") == 0 ||
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
            if (strcmp(p, "ascii") == 0) pattern = PAT_ASCII;
            else if (strcmp(p, "hex") == 0) pattern = PAT_HEX;
            else if (strcmp(p, "random") == 0) pattern = PAT_RANDOM;
            else if (strcmp(p, "walking") == 0) pattern = PAT_WALKING;
            else if (strcmp(p, "checker") == 0) pattern = PAT_CHECKER;
            i += 2;
        } else if (strcmp(argv[i], "--linecheck") == 0) {
            do_linecheck = 1; i++;
        } else if (strcmp(argv[i], "--modbus") == 0) {
            do_modbus = 1; i++;
        } else if (strcmp(argv[i], "--iec101") == 0) {
            do_iec101 = 1; i++;
        } else if (strcmp(argv[i], "--kernel") == 0) {
            mode = RS485_KERNEL; i++;
        } else if (strcmp(argv[i], "--gpio") == 0 && i + 1 < argc) {
            mode = RS485_GPIO;
            gpio_path = argv[i + 1];
            i += 2;
        } else if (strcmp(argv[i], "--manual-rts") == 0) {
            mode = RS485_MANUAL_RTS;
            i++;
        } else {
            i++;
        }
    }

    if (!port || !teststring) {
        fprintf(stderr, "Missing required parameters\n");
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
        perror("configure");
        close(fd);
        if (log != stdout) fclose(log);
        return 1;
    }

    /* Modes setup */
    if (mode == RS485_KERNEL) {
        enable_kernel_rs485(fd);
    } else if (mode == RS485_GPIO) {
        gpio_export(gpio_path);
        gpio_set(gpio_path, 0);
    } else if (mode == RS485_MANUAL_RTS) {
        rts_set(fd, 0);
    }

    if (do_linecheck)
        run_linecheck(fd, timeout, log);

    run_loopback(fd, loops, timeout, length, pattern, teststring, log);

    if (do_modbus)
        run_modbus(fd, timeout, log);

    if (do_iec101)
        run_iec101(fd, timeout, log);

    close(fd);
    if (log != stdout) fclose(log);

    return 0;
}
