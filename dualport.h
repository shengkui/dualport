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

/* Strucutre for thread argument */
typedef struct _thread_arg_t {
    int fd;                     /* The fd of serial port */
    int loop;                   /* loop times */
    unsigned char *data_buf;    /* The buffer to keep the chars to send or compare */
    unsigned long *byte_count;  /* Bytes received/sent */
} thread_arg_t;

/* Strucutre for baudrate and flag */
typedef struct _baud_item_t {
    int speed;                  /* The baudrate */
    int baud_flag;              /* The flag used to set the baudrate */
} baud_item_t;

/* Strucutre for parameter */
typedef struct _port_param_t {
    char *device1;  /* The 1st serial port */
    char *device2;  /* The 2nd serial port */
    int baudrate;   /* Baudrate of serial port */
    int databits;   /* Databits of serial port */
    int stopbits;   /* Stopbits of serial port */
    int parity;     /* Parity of serial port */
    int hwflow;     /* Enable hardware flow control or not */
    int loop_count; /* Loop count for test */
} port_param_t;


/******************************************************************************
 * Function declaration
 ******************************************************************************/
void print_usage(void);
int parse_argument(int argc, char *argv[], port_param_t *param);
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


/******************************************************************************
 * Macro definition
 ******************************************************************************/
#ifdef DEBUG

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

/* Max retry count on read operation */
#define MAX_RETRY_COUNT         10

/* Size of in/out buffer */
#define BUF_SIZE                256


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
