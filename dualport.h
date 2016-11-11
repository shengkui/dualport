/******************************************************************************
 *
 * FILENAME:
 *     dualport.h
 *
 * DESCRIPTION:
 *     Define some data structure and debug macros.
 *
 * REVISION(MM/DD/YYYY):
 *     08/07/2015  Shengkui Leng (lengshengkui@outlook.com)
 *     - Initial version 
 *
 ******************************************************************************/
#ifndef _DUALPORT_H_
#define _DUALPORT_H_


/******************************************************************************
 * Structure definition
 ******************************************************************************/

/* Structure for thread argument */
typedef struct _thread_arg_t {
    int fd;                     /* The fd of serial port */
    unsigned int loop_times;    /* loop times */
    unsigned char *data_buf;    /* The buffer to keep the chars to send or compare */
    unsigned long *byte_count;  /* Bytes received/sent */
    unsigned int interval;      /* The intervals(miliseconds) between 2 packets */
    unsigned int packet_size;   /* The size of data packet */
} thread_arg_t;

/* Structure for baudrate and flag */
typedef struct _baud_item_t {
    int speed;                  /* The baudrate */
    int baud_flag;              /* The flag used to set the baudrate */
} baud_item_t;


/* Structure for parameter of serial port */
typedef struct _port_param_t {
    char device1[PATH_MAX]; /* The 1st serial port */
    char device2[PATH_MAX]; /* The 2nd serial port */
    int baudrate;           /* Baudrate of serial port */
    int databits;           /* Databits of serial port */
    int stopbits;           /* Stopbits of serial port */
    int parity;             /* Parity of serial port */
    int hwflow;             /* Enable hardware flow control or not */
    unsigned int loop_times;    /* Loop times for test */
    unsigned int interval;      /* The intervals(miliseconds) between 2 packets */
    unsigned int packet_size;   /* The size of data packet */
    unsigned char data_bitmask; /* Databits mask */
} port_param_t;


/******************************************************************************
 * Function declaration
 ******************************************************************************/
void print_usage(void);
int check_portname(char *argv[], port_param_t *param);
int parse_argument(int argc, char *argv[], port_param_t *param);
int init_data_packet(unsigned char **out_buf, unsigned char **cmp_buf, port_param_t *param);
int speed_to_flag(int speed);
int setup_port(int fd, port_param_t *param);
int reset_port(int fd);
int read_data(int fd, void *buf, int len);
int write_data(int fd, void *buf, int len);
void buffer_dump_hex(void *buf, int len);
int buffer_compare(void *ibuf, void *obuf, size_t len);
void *routine_read(void *thr_arg);
void *routine_write(void *thr_arg);
void my_sig_handler(int signo);
int install_sig_handler(void);
int sleep_ms(unsigned int ms);
int is_all_digit(const char *str);


/******************************************************************************
 * Macro definition
 ******************************************************************************/
#ifdef DEBUG

/* Macro to print debug message to stdout */
#define DBG_PRINT(format, ...) \
    do { \
        printf("[%s@%s, %d]: " format, \
            __FUNCTION__, __FILE__, __LINE__, ##__VA_ARGS__ ); \
    } while (0)

#define DBG_DUMP                buffer_dump_hex
#define VERIFY_DATA             buffer_compare

#else

#define DBG_PRINT(format, ...)
#define DBG_DUMP(buf, len)
#define VERIFY_DATA             memcmp

#endif /* DEBUG */


/* Macro to print message to stdout */
#define CLI_OUT(format, ...) \
    do { \
        printf(format, ##__VA_ARGS__ ); \
    } while (0)


/* Compute the dimension of an array */
#define DIM(a)                  (sizeof(a) / sizeof((a)[0]))

/* Max retry times on read operation */
#define MAX_RETRY_COUNT         3

/* Default value of arguments/parameters */
#define VAL_BAUDRATE    115200
#define VAL_DATABITS    8
#define VAL_STOPBITS    1
#define VAL_PARITY      0
#define VAL_LOOPTIMES   1
#define VAL_INTERVAL    10
#define VAL_PACKETSIZE  1024

#define MAX_PACKETSIZE  4096
#define MAX_LOOPTIMES   UINT_MAX
#define MAX_INTERVAL    INT_MAX

/* Error codes */
#define ERR_OK                  0
#define ERR_INVALID_PARAM       (ERR_OK + 160)
#define ERR_OPEN_ERROR          (ERR_OK + 161)
#define ERR_SETUP_ERROR         (ERR_OK + 162)
#define ERR_READ_ERROR          (ERR_OK + 163)
#define ERR_WRITE_ERROR         (ERR_OK + 164)
#define ERR_VERIFY_ERROR        (ERR_OK + 165)
#define ERR_TEST_FAIL           (ERR_OK + 166)
#if ERR_OK != 0
#error "ERR_OK has to be defined as 0!"
#endif

#endif /* _DUALPORT_H_ */
