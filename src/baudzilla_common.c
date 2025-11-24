/*
 * Baudzilla common helpers
 *
 * Shared utility functions for all Baudzilla frontends.
 */

#include "baudzilla_common.h"

#include <time.h>
#include <string.h>
#include <termios.h>

/*
 * get_timestamp
 *
 * Fill buffer with a human readable timestamp: "YYYY-MM-DD HH:MM:SS".
 */
void get_timestamp(char *buffer, size_t size)
{
    struct timespec ts;
    struct tm tm_buf;

    if (!buffer || size == 0)
        return;

    clock_gettime(CLOCK_REALTIME, &ts);
    localtime_r(&ts.tv_sec, &tm_buf);

    strftime(buffer, size, "%Y-%m-%d %H:%M:%S", &tm_buf);
}

/*
 * get_baudrate
 *
 * Map integer baud rate (e.g. 9600) to the corresponding termios
 * constant (e.g. B9600). Falls back to B9600 on unknown values.
 */
int get_baudrate(int baud)
{
    switch (baud) {
#ifdef B50
    case 50:    return B50;
#endif
#ifdef B75
    case 75:    return B75;
#endif
#ifdef B110
    case 110:   return B110;
#endif
#ifdef B134
    case 134:   return B134;
#endif
#ifdef B150
    case 150:   return B150;
#endif
#ifdef B200
    case 200:   return B200;
#endif
#ifdef B300
    case 300:   return B300;
#endif
#ifdef B600
    case 600:   return B600;
#endif
#ifdef B1200
    case 1200:  return B1200;
#endif
#ifdef B1800
    case 1800:  return B1800;
#endif
#ifdef B2400
    case 2400:  return B2400;
#endif
#ifdef B4800
    case 4800:  return B4800;
#endif
#ifdef B9600
    case 9600:  return B9600;
#endif
#ifdef B19200
    case 19200: return B19200;
#endif
#ifdef B38400
    case 38400: return B38400;
#endif
#ifdef B57600
    case 57600: return B57600;
#endif
#ifdef B115200
    case 115200: return B115200;
#endif
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default:
#ifdef B9600
        return B9600;
#else
        return 0;
#endif
    }
}
