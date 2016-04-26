A simple dual-port test utility for serial port
===============================================
**Author**: Shengkui Leng

**E-mail**: lengshengkui@outlook.com


Description
-----------
This project is a very simple dual-port test utility for serial port under
Linux.  It only supports full-duplex serial port(RS-232/RS-422), and doesn't
support half-duplex serial port(RS-485).

Before run this utility, We must use a serial cable to connect two serial ports
(in the example, we use ttyS0 and ttyS1). The program sends predefined data on
both ports, and receives data on both ports. Then verifies them with predefined
data.

Following are the arguments/options of the program:

    dualport port1 port2 [OPTION] ...
        port1/port2      Serial port: ttyS0(/dev/ttyS0), ttyS1(/dev/ttyS1), ...
        -b baudrate      Baudrate: 9600, 19200, ... (default: 115200)
        -d databits      Databits: 5, 6, 7, 8 (default: 8)
        -c parity        Parity: 0(None), 1(Odd), 2(Even), 3(Mark),
                          4(Space) (default: 0)
        -s stopbits      Stopbits: 1, 2 (default: 1)
        -l loop_count    Loop count: 1~2147483647 (default: 1)
        -i interval      The interval(miliseconds) between 2 packets:
                          0~2147483647 (default: 10)
        -t packet_size   Size of data packet: 1~4096 (default: 256)
        -f               Enable hardware flow control (default: no flow ctrl)
        -h               Print this help message

NOTES: To run this utility, you need to be root user.

* * *

Build
-----------
(1) Open a terminal.

(2) chdir to the source code directory.

(3) Run "make"


Run
-----------
Start program to do read/write test on ttyS0 and ttyS1:

>    $ ./dualport /dev/ttyS0 /dev/ttyS1 -b 115200 -l 100

Try "./dualport -h" to get more help information.
