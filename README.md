# Baudzilla 1.0
RS232, RS422 and RS485 test suite  
(C) Oden Eriksson  
License: GPLv2

NOTE: This whole code base was written by the help of ChatGPT

Baudzilla is a complete serial line diagnostic toolkit that supports RS232,
RS422 and RS485. It includes loopback verification, idle bus noise
measurement, byte pattern tests, Modbus RTU probing and IEC 60870-5-101
probing.

The project provides three programs:

- **baudzilla232**  
  RS232 bidirectional test tool with modem signal display, pattern tests and
  optional IEC101 probe on port A.

- **baudzilla422**  
  RS422 loopback and protocol tester with selectable patterns, line noise
  analysis, Modbus RTU probe and IEC101 probe.

- **baudzilla485**  
  RS485 bus test tool supporting three DE control modes:
  kernel RS485 driver, GPIO controlled DE pin and manual RTS toggling.
  Includes loopback, noise tests, Modbus RTU probe and IEC101 probe.

All tools use a common pattern generator with ASCII, hex, random, walking and
checkerboard payloads.

## Features

- RS232, RS422 and RS485 support
- 8N1 serial configuration
- Pattern based payload tests
- Loopback verification
- Idle line noise detection
- Modbus RTU probe (unit 1, function 3)
- IEC 60870-5-101 fixed frame and GI frame probe
- Baud rate selection or auto sweep (RS232)
- RTS/CTS hardware flow control (RS232)
- Modem control status display (RS232)
- RS485 DE control:
  - Kernel RS485 mode
  - GPIO controlled DE pin
  - Manual RTS toggling
- Logging to file
- Summary reporting (RS232)
- Verbose mode
- Built using standard autotools

## Requirements

- GCC or Clang
- Autoconf
- Automake
- Make
- Linux with serial drivers
- Optional for RS485 GPIO mode: access to a GPIO sysfs value file

## Building

Extract the tarball and build using autotools:

