/******************************************************************************
 *
 * FILENAME:
 *     dualport.c
 *
 * DESCRIPTION:
 *     A very simple dual-port test utility for serial port
 *
 * REVISION(MM/DD/YYYY):
 *     08/07/2015  Shengkui Leng (lengshengkui@outlook.com)
 *     - Initial version 
 *
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <limits.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <signal.h>
#include "dualport.h"

/* Version of this program */
#define PROGRAM_VERSION         "0.9"

/* The name of this program */
static char *pname = "dualport";

/* Array of baudrate and flag */
static baud_item_t baud_array[] = {
    {50,     B50},      {75,     B75},      {110,    B110},
    {134,    B134},     {150,    B150},     {200,    B200},
    {300,    B300},     {600,    B600},     {1200,   B1200},
    {1800,   B1800},    {2400,   B2400},    {4800,   B4800},
    {9600,   B9600},    {19200,  B19200},   {38400,  B38400},
    {57600,  B57600},   {115200, B115200},  {230400, B230400},
    {460800, B460800},  {500000, B500000},  {576000, B576000},
    {921600, B921600},
};
static int max_baud_item = DIM(baud_array);

static struct termios orig_term_attr;


/* Friendly-name of parity */
static const char parity_name[] = {
    'N', 'O', 'E', 'M', 'S'
};

/* The flag to inform the program exit from the loop */
static int g_exit_flag = 0;


/******************************************************************************
 * NAME:
 *      print_usage
 *
 * DESCRIPTION: 
 *      Print usage information of the program.
 *
 * PARAMETERS:
 *      None
 *
 * RETURN:
 *      None
 ******************************************************************************/
void print_usage(void)
{
    CLI_OUT("\n"
        "========================================================\n"
        "        A Dual-port Test Utility for Serial Port\n"
        "                     Version %s\n"
        "========================================================\n"
        "\n"
        "Usage:\n"
        "  %s <port1> <port2> [OPTION] ...\n"
        "      port1/port2      Serial port: ttyS0(/dev/ttyS0), ttyS1(/dev/ttyS1), ...\n"
        "      -b baudrate      Baudrate: 9600, 19200, ... (default: %d)\n"
        "      -d databits      Databits: 5, 6, 7, 8 (default: %d)\n"
        "      -c parity        Parity: 0(None), 1(Odd), 2(Even), 3(Mark),\n"
        "                        4(Space) (default: %d)\n"
        "      -s stopbits      Stopbits: 1, 2 (default: %d)\n"
        "      -l loop_count    Loop count: 1~%u (default: %d)\n"
        "      -i interval      The interval(miliseconds) between 2 packets:\n"
        "                        0~%d (default: %u)\n"
        "      -t packet_size   Size of data packet: 1~%d (default: %d)\n"
        "      -f               Enable hardware flow control (default: no flow ctrl)\n"
        "      -h               Print this help message\n"
        "\n"
        "Example:\n"
        "  %s ttyS0 ttyS1 -b 115200 -d 8 -c 0 -s 1 -l 100\n"
        "  %s /dev/ttyS0 /dev/ttyS1 -l 200\n"
        "\n",
        PROGRAM_VERSION, pname, VAL_BAUDRATE, VAL_DATABITS, VAL_PARITY,
        VAL_STOPBITS, UINT_MAX, VAL_LOOPCOUNT, INT_MAX, VAL_INTERVAL,
        MAX_PACKETSIZE, VAL_PACKETSIZE,
        pname, pname);
}


int main(int argc, char *argv[])
{
    int rc = ERR_OK;
    port_param_t param;
    unsigned char *obuf;       /* Buffer to store output data */
    unsigned char *cmp_buf;    /* Buffer to compare with received data */

    pname = argv[0];

    if (argc < 3) {
        print_usage();
        return ERR_INVALID_PARAM;
    }

    rc = check_portname(argv, &param);
    if (rc != ERR_OK) {
        return rc;
    }

    rc = parse_argument(argc-2, &argv[2], &param);
    if (rc != ERR_OK) {
        return rc;
    }
    CLI_OUT("%s<==>%s (%d %d%c%d) (flow ctrl: %s) (loop %u) (interval %u) (packet size %u)\n",
        param.device1, param.device2, param.baudrate, param.databits,
        parity_name[param.parity], param.stopbits, param.hwflow ? "HW" : "No",
        param.loop_count, param.interval, param.packet_size);

    /* Open ports */
    int fd1 = open(param.device1, O_RDWR);
    if (fd1 < 0) {
        CLI_OUT("open <%s> error: %s\n", param.device1, strerror(errno));
        return ERR_OPEN_ERROR;
    }
    int fd2 = open(param.device2, O_RDWR);
    if (fd2 < 0) {
        CLI_OUT("open <%s> error: %s\n", param.device2, strerror(errno));
        return ERR_OPEN_ERROR;
    }

    /* Setup ports */
    if (setup_port(fd1, &param)) {
        CLI_OUT("setup <%s> error\n", param.device1);
        close(fd1);
        close(fd2);
        return ERR_SETUP_ERROR;
    }
    if (setup_port(fd2, &param)) {
        CLI_OUT("setup <%s> error\n", param.device2);
        close(fd1);
        close(fd2);
        return ERR_SETUP_ERROR;
    }

    install_sig_handler();

    /* Allocate memory for output buffer */
    obuf = malloc(param.packet_size);
    if (obuf == NULL) {
        CLI_OUT("Allocate buffer error\n");
        close(fd1);
        close(fd2);
        return ENOMEM;
    }
    cmp_buf = malloc(param.packet_size);
    if (cmp_buf == NULL) {
        CLI_OUT("Allocate buffer error\n");
        close(fd1);
        close(fd2);
        free(obuf);
        return ENOMEM;
    }

    /*
     * Init output buffer with chars in ASCII-table(ASCII code from 0 to 255)
     * NOTES: This must be done after setup_port has been called, because the
     * packet_size and data_bitmask are inited in the function setup_port.
     */
    int i;
    for (i = 0; i < param.packet_size; i++) {
        obuf[i] = i % 0x100;
        cmp_buf[i] = obuf[i] & param.data_bitmask;
    }

    /* The total bytes sent/received */
    unsigned long total_bytes_read1 = 0, total_bytes_write1 = 0;
    unsigned long total_bytes_read2 = 0, total_bytes_write2 = 0;

    long rc_write1 = 0, rc_read1 = 0;
    long rc_write2 = 0, rc_read2 = 0;
    thread_arg_t arg_write1, arg_read1;
    thread_arg_t arg_write2, arg_read2;
    pthread_t thr_id_read1, thr_id_write1;
    pthread_t thr_id_read2, thr_id_write2;

    arg_read1.fd = fd1;
    arg_read1.data_buf = cmp_buf;
    arg_read1.loop = param.loop_count;
    arg_read1.byte_count = &total_bytes_read1;
    arg_read1.interval = 0;
    arg_read1.packet_size = param.packet_size;

    arg_read2.fd = fd2;
    arg_read2.data_buf = cmp_buf;
    arg_read2.loop = param.loop_count;
    arg_read2.byte_count = &total_bytes_read2;
    arg_read2.interval = 0;
    arg_read2.packet_size = param.packet_size;

    arg_write1.fd = fd1;
    arg_write1.data_buf = obuf;
    arg_write1.loop = param.loop_count;
    arg_write1.byte_count = &total_bytes_write1;
    arg_write1.interval = param.interval;
    arg_write1.packet_size = param.packet_size;

    arg_write2.fd = fd2;
    arg_write2.data_buf = obuf;
    arg_write2.loop = param.loop_count;
    arg_write2.byte_count = &total_bytes_write2;
    arg_write2.interval = param.interval;
    arg_write2.packet_size = param.packet_size;

    /* Start threads to do Read&Verify/Write */
    pthread_create(&thr_id_read1, NULL, routine_read, &arg_read1);
    pthread_create(&thr_id_write2, NULL, routine_write, &arg_write2);
    pthread_create(&thr_id_read2, NULL, routine_read, &arg_read2);
    pthread_create(&thr_id_write1, NULL, routine_write, &arg_write1);

    /* Wait until threads exit. */
    pthread_join(thr_id_write1, (void *)&rc_write1);
    pthread_join(thr_id_write2, (void *)&rc_write2);
    pthread_join(thr_id_read1, (void *)&rc_read1);
    pthread_join(thr_id_read2, (void *)&rc_read2);

    CLI_OUT("\n"
        "------------------------------------------------------------\n"
        "%-12s: Sent %lu bytes, received %lu bytes\n"
        "%-12s: Sent %lu bytes, received %lu bytes\n"
        "------------------------------------------------------------\n",
        param.device1, total_bytes_write1, total_bytes_read1,
        param.device2, total_bytes_write2, total_bytes_read2);

    /* Check test result. */
    if (rc_write1 || rc_read1 || rc_write2 || rc_read2) {
        CLI_OUT("\nFAIL\n");
        rc = ERR_TEST_FAIL;
    } else {
        CLI_OUT("\nPASS\n");
        rc = ERR_OK;
    }

    close(fd1);
    close(fd2);
    free(obuf);
    free(cmp_buf);
    return rc;
}


/******************************************************************************
 * NAME:
 *      check_portname
 *
 * DESCRIPTION: 
 *      Build the full path of serial port and check if it's exist or not.
 *
 * PARAMETERS:
 *      argv  - Arguments array
 *      param - Output the port name of serial port. 
 *
 * RETURN:
 *      0 for OK, others for ERROR
 ******************************************************************************/
int check_portname(char *argv[], port_param_t *param)
{
    struct stat st;

    if (argv[1][0] == '/') {
        snprintf(param->device1, sizeof(param->device1), "%s", argv[1]);
    } else {
        snprintf(param->device1, sizeof(param->device1), "/dev/%s", argv[1]);
    }
    if (argv[2][0] == '/') {
        snprintf(param->device2, sizeof(param->device2), "%s", argv[2]);
    } else {
        snprintf(param->device2, sizeof(param->device2), "/dev/%s", argv[2]);
    }

    if (stat(param->device1, &st) != 0) {
        CLI_OUT("The device \"%s\" is not exist or accessable\n", param->device1);
        return ERR_INVALID_PARAM;
    }
    if (stat(param->device2, &st) != 0) {
        CLI_OUT("The device \"%s\" is not exist or accessable\n", param->device2);
        return ERR_INVALID_PARAM;
    }

    return ERR_OK;
}


/******************************************************************************
 * NAME:
 *      parse_argument
 *
 * DESCRIPTION: 
 *      Parse the command line arguments.
 *
 * PARAMETERS:
 *      argc  - Count of arguments
 *      argv  - Arguments array
 *      param - Output the parameters of serial port. 
 *
 * RETURN:
 *      0 for OK, others for ERROR
 ******************************************************************************/
int parse_argument(int argc, char *argv[], port_param_t *param)
{
    int opt;

    if (!param) {
        return ERR_INVALID_PARAM;
    }

    /* Default value of parameters */
    param->baudrate = VAL_BAUDRATE;
    param->databits = VAL_DATABITS;
    param->stopbits = VAL_STOPBITS;
    param->parity = VAL_PARITY;
    param->hwflow = 0;
    param->loop_count = VAL_LOOPCOUNT;
    param->interval = VAL_INTERVAL;
    param->packet_size = VAL_PACKETSIZE;
    param->data_bitmask = 0xFF;

    while ((opt = getopt(argc, argv, ":b:d:c:s:l:i:t:fh")) != -1) {
        switch (opt) {
        case 'b':
            if (!is_all_digit(optarg)) {
                CLI_OUT("Invalid argument, \"baudrate\" should be an integer\n");
                return ERR_INVALID_PARAM;
            }
            param->baudrate = atoi(optarg);
            if (speed_to_flag(param->baudrate) < 0) {
                CLI_OUT("Invalid argument (baudrate = %d)\n", param->baudrate);
                return ERR_INVALID_PARAM;
            }
            break;

        case 'd':
            if (!is_all_digit(optarg)) {
                CLI_OUT("Invalid argument, \"databits\" should be an integer\n");
                return ERR_INVALID_PARAM;
            }
            param->databits = atoi(optarg);
            if ((param->databits < 5) || (param->databits > 8)) {
                CLI_OUT("Invalid argument (databits = %d)\n", param->databits);
                return ERR_INVALID_PARAM;
            }
            break;

        case 'c':
            if (!is_all_digit(optarg)) {
                CLI_OUT("Invalid argument, \"parity\" should be an integer\n");
                return ERR_INVALID_PARAM;
            }
            param->parity = atoi(optarg);
            if ((param->parity < 0) || (param->parity > 4)) {
                CLI_OUT("Invalid argument (parity = %d)\n", param->parity);
                return ERR_INVALID_PARAM;
            }
            break;

        case 's':
            if (!is_all_digit(optarg)) {
                CLI_OUT("Invalid argument, \"stopbits\" should be an integer\n");
                return ERR_INVALID_PARAM;
            }
            param->stopbits = atoi(optarg);
            if ((param->stopbits < 1) || (param->stopbits > 2)) {
                CLI_OUT("Invalid argument (stopbits = %d)\n", param->stopbits);
                return ERR_INVALID_PARAM;
            }
            break;

        case 'l':
            if (!is_all_digit(optarg)) {
                CLI_OUT("Invalid argument, \"loop_count\" should be an integer\n");
                return ERR_INVALID_PARAM;
            }
            param->loop_count = strtoul(optarg, NULL, 0);
            if (param->loop_count < 1) {
                CLI_OUT("Invalid argument (loop_count = %d)\n",
                    param->loop_count);
                return ERR_INVALID_PARAM;
            }
            break;

        case 'i':
            if (!is_all_digit(optarg)) {
                CLI_OUT("Invalid argument, \"interval\" should be an integer\n");
                return ERR_INVALID_PARAM;
            }
            param->interval = strtoul(optarg, NULL, 0);
            if (param->interval < 0) {
                CLI_OUT("Invalid argument (interval = %d)\n", param->interval);
                return ERR_INVALID_PARAM;
            }
            break;

        case 't':
            if (!is_all_digit(optarg)) {
                CLI_OUT("Invalid argument, \"packet size\" should be an integer\n");
                return ERR_INVALID_PARAM;
            }
            param->packet_size = atoi(optarg);
            if ((param->packet_size < 1) || (param->packet_size > MAX_PACKETSIZE)) {
                CLI_OUT("Invalid argument (packet size = %d)\n", param->packet_size);
                return ERR_INVALID_PARAM;
            }
            break;

        case 'f':
            param->hwflow = 1;
            break;

        case 'h':
            print_usage();
            return ERR_INVALID_PARAM;
            break;

        case ':':
            CLI_OUT("Option '%c' needs a value\n\n", optopt);
            print_usage();
            return ERR_INVALID_PARAM;
            break;

        case '?':
            CLI_OUT("No such option: '%c'\n\n", optopt);
            print_usage();
            return ERR_INVALID_PARAM;
            break;

        default:
            return ERR_INVALID_PARAM;
            break;
        }
    }

    if (optind < argc) {
        CLI_OUT("Invalid argument: %s\n\n", argv[optind]);
        print_usage();
        return ERR_INVALID_PARAM;
    }

    return ERR_OK;
}


/******************************************************************************
 * NAME:
 *      speed_to_flag
 *
 * DESCRIPTION: 
 *      Translate baudrate into flag that can be used in termios.c_cflag.
 *
 * PARAMETERS:
 *      speed - The baudrate
 *
 * RETURN:
 *      The flag of baudrate
 *      -1 for error
 ******************************************************************************/
int speed_to_flag(int speed)
{
    int i;

    for (i = 0; i < max_baud_item; i++) {
        if (speed == baud_array[i].speed) {
            DBG_PRINT("baudrate = %d, baud_flag = %07o\n", speed,
                baud_array[i].baud_flag);
            return baud_array[i].baud_flag;
        }
    }

    return -1;
}


/******************************************************************************
 * NAME:
 *      setup_port
 *
 * DESCRIPTION: 
 *      Init serial port (baudrate, line control, flow control, ...)
 *
 * PARAMETERS:
 *      fd    - The fd of serial port
 *      param - The parameters of serial port
 *
 * RETURN:
 *      0 for OK, others for ERROR
 ******************************************************************************/
int setup_port(int fd, port_param_t *param)
{
    struct termios term_attr;
    int baud_flag;

    if (tcflush(fd, TCIOFLUSH) < 0) {
        CLI_OUT("tcflush error: %s\n", strerror(errno));
        return -1;
    }

    /* Backup current setting */
    if (tcgetattr(fd, &orig_term_attr) < 0) {
        CLI_OUT("tcgetattr error: %s\n", strerror(errno));
        return -1;
    }

    memcpy(&term_attr, &orig_term_attr, sizeof(struct termios));

    term_attr.c_oflag &= ~(OPOST | ONLCR | OCRNL);
    term_attr.c_lflag &= ~(ISIG | ECHO | ICANON | NOFLSH);

    /* Enable receiver, ignore modem control lines. */
    term_attr.c_cflag |= CREAD | CLOCAL;

    /*
     * Disable software flow control.  We need to disable XON/XOFF/XANY to
     * send/receive control chars(ASCII code < 0x20)
     */
    term_attr.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | IGNCR | ICRNL | ISTRIP);

    /* Set hardware flow control */
    if (param->hwflow) {
        term_attr.c_cflag |= CRTSCTS;
    } else {
        term_attr.c_cflag &= ~CRTSCTS;
    }

    /* Set baudrate */
    baud_flag = speed_to_flag(param->baudrate);
    if (baud_flag == -1) {
        CLI_OUT("Unsupported baudrate(%d)!\n", param->baudrate);
        return -1;
    }
    if (cfsetispeed(&term_attr, baud_flag) < 0) {
        CLI_OUT("cfsetispeed error: %s\n", strerror(errno));
        return -1;
    }
    if (cfsetospeed(&term_attr, baud_flag) < 0) {
        CLI_OUT("cfsetospeed error: %s\n", strerror(errno));
        return -1;
    }

    /* Set databits */
    term_attr.c_cflag &= ~CSIZE;
    switch (param->databits) {
    case 5:
        term_attr.c_cflag |= CS5;
        break;

    case 6:
        term_attr.c_cflag |= CS6;
        break;

    case 7:
        term_attr.c_cflag |= CS7;
        break;

    case 8:
        term_attr.c_cflag |= CS8;
        break;

    default:
        CLI_OUT("Invalid databits: %d\n", param->databits);
        return -1;
    }
    param->data_bitmask = ((1 << param->databits) - 1) & 0xFF;

    /* Set parity */
    switch (param->parity) {
    case 0:                     /* None parity */
        term_attr.c_cflag &= ~(CMSPAR | PARENB | PARODD);
        //term_attr.c_iflag &= ~INPCK;
        break;

    case 1:                     /* Odd parity */
        term_attr.c_cflag &= ~CMSPAR;
        term_attr.c_cflag |= (PARENB | PARODD);
        term_attr.c_iflag |= INPCK;
        break;

    case 2:                     /* Even parity */
        term_attr.c_cflag &= ~(PARODD | CMSPAR);
        term_attr.c_cflag |= PARENB;
        term_attr.c_iflag |= INPCK;
        break;

    case 3:                     /* Mark parity */
        term_attr.c_cflag |= (PARENB | CMSPAR | PARODD);
        term_attr.c_iflag |= INPCK;
        break;

    case 4:                     /* Space parity */
        term_attr.c_cflag |= (PARENB | CMSPAR);
        term_attr.c_cflag &= ~PARODD;
        term_attr.c_iflag |= INPCK;
        break;

    default:
        CLI_OUT("Invalid parity: %d\n", param->parity);
        return -1;
    }

    /* Set stopbits */
    switch (param->stopbits) {
    case 1:                     /* 1 stopbits */
        term_attr.c_cflag &= ~CSTOPB;
        break;

    case 2:                     /* 2 stopbits */
        term_attr.c_cflag |= CSTOPB;
        break;

    default:
        CLI_OUT("Invalid stopbits: %d\n", param->stopbits);
        return -1;
    }

    /* VMIN = 0 and VTIME > 0
     * VTIME serves as a timeout value(unit: 1/10 second).  The read will be
     * satisfied if a single character has been received or VTIME is exceeded
     * (interval time = VTIME * 0.1 s).  If VTIME is exceeded, no character
     * will be returned.
     */
    term_attr.c_cc[VMIN] = 0;
    term_attr.c_cc[VTIME] = 10;

    if (tcsetattr(fd, TCSANOW, &term_attr) < 0) {
        CLI_OUT("tcsetattr error: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}


/******************************************************************************
 * NAME:
 *      reset_port
 *
 * DESCRIPTION: 
 *      Restore the setting of serial port
 *
 * PARAMETERS:
 *      fd - The fd of the serial port
 *
 * RETURN:
 *      0 for OK, others for ERROR
 ******************************************************************************/
int reset_port(int fd)
{
    if (tcsetattr(fd, TCSANOW, &orig_term_attr) < 0) {
        return -1;
    }

    return 0;
}


/******************************************************************************
 * NAME:
 *      read_data
 *
 * DESCRIPTION: 
 *      Read data from serial port
 *
 * PARAMETERS:
 *      fd   - The fd of serial port
 *      buf  - The buffer to keep readed data
 *      len  - The max count to read
 *
 * RETURN:
 *      Number of bytes readed
 ******************************************************************************/
int read_data(int fd, void *buf, int len)
{
    int ret = 0;
    int bytes = 0;
    int retry_count = 0;

    while (len > 0) {
        ret = read(fd, (unsigned char *)buf + bytes, len);
        DBG_PRINT("read: %d\n", ret);
        if (ret < 0) {
            CLI_OUT("read error: %s\n", strerror(errno));
            break;
        } else if (ret == 0) {
            if (++retry_count > MAX_RETRY_COUNT) {
                break;
            }
            DBG_PRINT("retry: %d\n", retry_count);
        } else { //ret > 0
            DBG_DUMP(buf + bytes, ret);
            
            bytes += ret;
            len = len - ret;
        }
    }

    return bytes;
}


/******************************************************************************
 * NAME:
 *      write_data
 *
 * DESCRIPTION: 
 *      Write data to serial port
 *
 * PARAMETERS:
 *      fd   - The fd of serial port
 *      buf  - The buffer of data
 *      len  - The count of data
 *
 * RETURN:
 *      Number of bytes actually written
 ******************************************************************************/
int write_data(int fd, void *buf, int len)
{
    int bytes = 0;
    int ret = 0;

    while (len > 0) {
        ret = write(fd, (unsigned char *)buf + bytes, len);
        DBG_PRINT("write: %d\n", ret);
        if (ret < 1) {
            CLI_OUT("write error %s\n", strerror(errno));
            break;
        }

        bytes += ret;
        len = len - ret;
    }

    return bytes;
}


/******************************************************************************
 * NAME:
 *      buffer_dump_hex
 *
 * DESCRIPTION: 
 *      Dump buffer in HEX mode
 *
 * PARAMETERS:
 *      buf  - The buffer
 *      len  - The count of data in buf
 *
 * RETURN:
 *      None
 ******************************************************************************/
void buffer_dump_hex(void *buf, int len)
{
    int i;

    for (i = 0; i < len; i++) {
        CLI_OUT("%02X ", *((unsigned char *)buf + i));
        if (((i+1) % 16) == 0) {
            CLI_OUT("\n");
        }
    }
    CLI_OUT("\n");
}


/******************************************************************************
 * NAME:
 *      buffer_compare
 *
 * DESCRIPTION:
 *      Compare 2 buffers, and display data in HEX mode.
 *
 * PARAMETERS:
 *      ibuf - The 1st buffer
 *      obuf - The 2nd buffer
 *      len  - The bytes to compare
 *
 * RETURN:
 *      0 for equal, others for diff
 ******************************************************************************/
int buffer_compare(void *ibuf, void *obuf, size_t len)
{
    int i;
    int ret = 0;
    unsigned char *p1 = ibuf;
    unsigned char *p2 = obuf;

    for (i = 0; i < len; i++) {
        CLI_OUT("%02X  %02X\n", *(p1 + i), *(p2 + i));

        if (p1[i] != p2[i]) {
            // diff !
            ret = i+1;
        }
    }

    return ret;
}


/******************************************************************************
 * NAME:
 *      routine_read
 *
 * DESCRIPTION: 
 *      The thread routine to read data from serial port and verify with predefined
 *      data pattern.
 *
 * PARAMETERS:
 *      thr_arg - Argument of thread routine (data type: thread_arg_t *)
 *
 * RETURN:
 *      Exit code of thread
 ******************************************************************************/
void *routine_read(void *thr_arg)
{
    thread_arg_t *arg = (thread_arg_t *)thr_arg;
    if (!arg) {
        CLI_OUT("%s: Invalid parameter\n", __FUNCTION__);
        pthread_exit((void *)ERR_INVALID_PARAM);
    }
    
    unsigned char *cmp_buf = arg->data_buf;
    if (!cmp_buf) {
        CLI_OUT("%s: Invalid parameter\n", __FUNCTION__);
        pthread_exit((void *)ERR_INVALID_PARAM);
    }

    long rc = ERR_OK;
    unsigned long bytes_read = 0;
    int n;
    int fd = arg->fd;
    unsigned int loop_count = arg->loop;
    unsigned int pack_size = arg->packet_size;
    unsigned char *ibuf = malloc(pack_size);
    if (ibuf == NULL) {
        CLI_OUT("Allocate buffer error\n");
        pthread_exit((void *)ENOMEM);
    }
    memset(ibuf, 0, pack_size);

    while (loop_count--) {
        if (g_exit_flag) {
            break;
        }

        DBG_PRINT("Read data ...\n");

        /* Read data from serial port */
        n = read_data(fd, ibuf, pack_size);
        bytes_read += n;
        if (n != pack_size) {
            CLI_OUT("Received data is less than expected\n");
            buffer_dump_hex(ibuf, n);
            g_exit_flag = 1;
            rc = ERR_READ_ERROR;
            break;
        }
        DBG_DUMP(ibuf, n);
        DBG_PRINT("Read done\n\n");

        /* Verify data */
        if (VERIFY_DATA(ibuf, cmp_buf, pack_size) == 0) {
            CLI_OUT(".");
            fflush(stdout);
        } else {
            CLI_OUT("\nVerify data error\n");
            buffer_dump_hex(ibuf, n);
            g_exit_flag = 1;
            rc = ERR_VERIFY_ERROR;
            break;
        }
        DBG_PRINT("Verify done\n\n");
    }

    free(ibuf);
    *(arg->byte_count) = bytes_read;
    pthread_exit((void*)rc);
}


/******************************************************************************
 * NAME:
 *      routine_write
 *
 * DESCRIPTION: 
 *      The thread routine to write data to serial port
 *
 * PARAMETERS:
 *      thr_arg - Argument of thread routine (data type: thread_arg_t *)
 *
 * RETURN:
 *      Exit code of thread
 ******************************************************************************/
void *routine_write(void *thr_arg)
{
    thread_arg_t *arg = (thread_arg_t *)thr_arg;
    if (!arg) {
        CLI_OUT("%s: Invalid parameter\n", __FUNCTION__);
        pthread_exit((void *)ERR_INVALID_PARAM);
    }
    
    unsigned char *obuf = arg->data_buf;
    if (!obuf) {
        CLI_OUT("%s: Invalid parameter\n", __FUNCTION__);
        pthread_exit((void *)ERR_INVALID_PARAM);
    }

    long rc = ERR_OK;
    unsigned long bytes_write = 0;
    unsigned int loop_count = arg->loop;
    int fd = arg->fd;
    int n;
    unsigned int interval = arg->interval;
    unsigned int pack_size = arg->packet_size;

    while (loop_count--) {
        if (g_exit_flag) {
            break;
        }

        DBG_PRINT("Write data ...\n");

        /* Write data to serial port */
        n = write_data(fd, obuf, pack_size);
        bytes_write += n;
        if (n != pack_size) {
            CLI_OUT("\nNOT all data sent\n");
            g_exit_flag = 1;
            rc = ERR_WRITE_ERROR;
            break;
        }

        DBG_DUMP(obuf, n);
        DBG_PRINT("Write done\n\n");

        sleep_ms(interval);
    }

    *(arg->byte_count) = bytes_write;
    pthread_exit((void*)rc);
}


/******************************************************************************
 * NAME:
 *      my_sig_handler
 *
 * DESCRIPTION: 
 *      Signal handler, when the signal SIGTERM/SIGINT received, set the
 *      exit flag.
 *
 * PARAMETERS:
 *      signo - Signal number 
 *
 * RETURN:
 *      None
 ******************************************************************************/
void my_sig_handler(int signo)
{
    g_exit_flag = 1;
}


/******************************************************************************
 * NAME:
 *      install_sig_handler
 *
 * DESCRIPTION: 
 *      Install signal handler for SIGTERM/SIGINT.
 *
 * PARAMETERS:
 *      None
 *
 * RETURN:
 *      0 for OK, others for ERROR
 ******************************************************************************/
int install_sig_handler(void)
{
    struct sigaction sa;

    sa.sa_handler = my_sig_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGTERM, &sa, NULL) != 0) {
        CLI_OUT("sigaction(SIGTERM) error: %s\n", strerror(errno));
        return -1;
    }
    if (sigaction(SIGINT, &sa, NULL) != 0) {
        CLI_OUT("sigaction(SIGINT) error: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}


/******************************************************************************
 * NAME:
 *      sleep_ms
 *
 * DESCRIPTION: 
 *      Sleep some number of milli-seconds
 *
 * PARAMETERS:
 *      ms - Time in milliseconds to sleep for.
 *
 * RETURN:
 *      0 - OK, Other - interrupted or error
 ******************************************************************************/
int sleep_ms(unsigned int ms)
{
    struct timespec req;
 
    req.tv_sec = ms / 1000;
    req.tv_nsec = (ms % 1000) * 1000000;
    if (nanosleep(&req, NULL) < 0) {
        return -1;
    }
 
    return 0;
}


/******************************************************************************
 * NAME:
 *      is_all_digit
 *
 * DESCRIPTION: 
 *      Check if the string consist of only digit chars.
 *
 * PARAMETERS:
 *      str - The string to check.
 *
 * RETURN:
 *      1 - TRUE
 *      0 - FALSE
 ******************************************************************************/
int is_all_digit(const char *str)
{
    /* Handle empty string. */
    if (!str || !*str) {
        return 0;
    }

    /* Check for non-digit chars in the stirng. */
    while (*str) {
        if (!isdigit(*str)) {
            /* non-digit char found! */
            return 0;
        }

        ++str;
    }

    return 1;
}
