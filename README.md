A simple dual-port test utility for serial port
===============================================
**Author**: Shengkui Leng

**E-mail**: lengshengkui@outlook.com


Description
-----------
This project is a very simple dual-port test utility for serial port under Linux.
It only supports full-duplex RS-232/RS-422 serial port, and doesn't support
half-duplex RS-485.

Before run this utility, We must use a cable to connect two serial ports (in the
example, we use ttyS0 and ttyS1). Pass these 2 serial ports to the program. The 
program opens two serial ports, then start reading and writing on both ports.

Following are the arguments/options of the program:
<pre>
    dualport port1 port2 [OPTION] ...
        port1/port2   : Serial port, /dev/ttyS0, /dev/ttyS1, ...
        -b baudrate   : Baudrate, 9600, 19200, ... (default: 115200)
        -d databits   : Databits, 5, 6, 7, 8 (default: 8)
        -c parity     : Parity, 0(None), 1(Odd), 2(Even), 3(Mark),
                        4(Space) (default: 0)
        -s stopbits   : Stopbits, 1, 2 (default: 1)
        -l loop_count : Loop count(>0) (default: 1)
        -f            : Enable hardware flow control (default: no flow control)
        -h            : Print this help message
</pre>

NOTES: To run this utility, you need to be root user.

* * *

Build
-----------
(1) Open a terminal.

(2) chdir to the source code directory.

(3) Run "make"


Run
-----------
Start program to do read/write test for ttyS0:

>    $ ./dualport /dev/ttyS0 /dev/ttyS1 -b 115200 -l 100

Try "./dualport -h" to get more help information.