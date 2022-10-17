# SDM120C
PZEM16 ModBus RTU client to read PZEM16 power meter registers

Based on work of https://github.com/gianfrdp/SDM120C

It depends on libmodbus (http://libmodbus.org)

To compile
  make clean && make

To install
  make install

To uninstall
  make uninstall

<PRE>
# PZEM16
PZEM16 ModBus RTU client to read PZEM16 power meter registers


It depends on libmodbus (http://libmodbus.org)

To compile and install
  make clean && make install

<PRE>
pzem16 1.0: ModBus RTU client to read EASTRON SDM120C smart mini power meter registers
Copyright (C) 2022 Pierantonio Tabaro <toni.tabaro@google.com>
based on: Copyright (C) 2015 Gianfranco Di Prinzio <gianfrdp@inwind.it>

Usage: pzem16 [-a address] [-d n] [-x] [-p] [-v] [-c] [-e] [-i] [-t] [-f] [-g] [[-m]|[-q]] [-z num_retries] [-j seconds] [-w seconds] device
       pzem16 [-a address] [-d n] [-x] [-z num_retries] [-j seconds] [-w seconds] -s new_address device
Required:
        device          Serial device (i.e. /dev/ttyUSB0)
        -a address      Meter number (1-247). Default: 1
Reading parameters (no parameter = retrieves all values):
        -p              Get power (W)
        -v              Get voltage (V)
        -c              Get current (A)
        -f              Get frequency (Hz)
        -g              Get power factor
        -t              Get total energy (Wh)
        -m              Output values in IEC 62056 format ID(VALUE*UNIT)
        -q              Output values in compact mode
Writing new settings parameters:
        -s new_address  Set new meter number (1-247)
Fine tuning & debug parameters:
        -z num_retries  Try to read max num_retries times on bus before exiting
                        with error. Default: 1 (no retry)
        -j 1/10 secs    Response timeout. Default: 2=0.2s
        -D 1/1000 secs  Delay before sending commands. Default: 0ms
        -w seconds      Time to wait to lock serial port (1-30s). Default: 0s
        -W 1/1000 secs  Time to wait for 485 line to settle. Default: 0ms
        -y 1/1000 secs  Set timeout between every bytes (1-500). Default: disabled
        -d debug_level  Debug (0=disable, 1=debug, 2=errors to syslog, 3=both)
                        Default: 0
        -x              Trace (libmodbus debug on)</PRE>
