
#include "config.h"
#include "uart.h"

using namespace std;


/*
 * open serial port
 * return file descriptor on success or -1 on error.
 */
int open_port(char *path)
{
    int fd;
    /*调用 open()函数时，使用 O_NOCTTY 标志，该标志用于告知系统它不会成为进程的控制终端。*/
    /*在读操作时，如果读不到数据，O_NDELAY会使I/O函数马上返回0*/
    fd = open(path, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        /* open failure, print system error msg. */
        SPDLOG_LOGGER_ERROR(mylogger, "open_port: Unable to open port - ");
        SPDLOG_LOGGER_ERROR(console, "open_port: Unable to open port - ");
        perror("open_port: Unable to open port - ");
    }
    else
    {
        /* clear file descriptor flags */
        fcntl(fd, F_SETFL, 0);
        return fd;
    }
}

/*
 * write data to serial port
 * return the length of bytes actually written on success, or -1 on error
 */
ssize_t write_port(int fd, void *buf, size_t len)
{
    ssize_t actual_write;

    actual_write = write(fd, buf, len);
    if (actual_write == -1)
    {
        /* write failure */
        SPDLOG_LOGGER_ERROR(mylogger, "write_port: Unable to write port - ");
        SPDLOG_LOGGER_ERROR(console, "write_port: Unable to write port - ");
        perror("write_port: Unable to write port - ");
    }
    return actual_write;
}

/* set timeout and minimum number of bytes to read */
int read_port_setup(int fd, cc_t timeout_sec, cc_t min_bytes)
{
    struct termios term;

    /* read serial port configure */
    if (tcgetattr(fd, &term) != 0)
    {
        SPDLOG_LOGGER_ERROR(mylogger, "config_port: Unable to read configure - ");
        SPDLOG_LOGGER_ERROR(console, "config_port: Unable to read configure - ");
        perror("config_port: Unable to read configure - ");
        return -1;
    }

    /* set timeout in deciseconds for noncanonical read */
    // term.c_cc[VTIME] = timeout_sec * 10;
    term.c_cc[VTIME] = timeout_sec ;

    /* set minimum number of bytes to read */
    term.c_cc[VMIN] = min_bytes;

    /* write serial port configure */
    if (tcsetattr(fd, TCSADRAIN, &term) != 0)
    {
        SPDLOG_LOGGER_ERROR(mylogger, "config_port: Unable to write configure - ");
        SPDLOG_LOGGER_ERROR(console, "config_port: Unable to write configure - ");
        perror("config_port: Unable to write configure - ");
        return -1;
    }
}

/*
 * read data from serial port
 * return the length of bytes actually read on success, or -1 on error
 *
 * fd - file descriptor
 * buf - pointer of buffer to receive bytes
 * len - buffer
 * 100msec - timeout msecond
 * min - minimum bytes to read
 */
ssize_t read_port(int fd, void *buf, size_t len, cc_t sec, cc_t min)
{
    ssize_t actual_read;

    /* setup serial port */
    if (read_port_setup(fd, sec, min) == -1)
    {
        return -1;
    }

    /* read bytes from serial port */
    actual_read = read(fd, buf, len);
    if (actual_read == -1)
    {
        /* read failure */
        // perror("read_port: Unable to read port - ");
    }

    return actual_read;
}

/*
 * config serial port baud and set data format to 8N1
 * (8bits data, no parity bit, 1 stop bit)
 * return 0 on success, or -1 on error
 */
int config_port(int fd, speed_t baud)
{
    struct termios term;

    /* read serial port configure */
    if (tcgetattr(fd, &term) != 0)
    {
        SPDLOG_LOGGER_ERROR(mylogger, "config_port: Unable to read configure - ");
        SPDLOG_LOGGER_ERROR(console, "config_port: Unable to read configure - ");
        perror("config_port: Unable to read configure - ");
        return -1;
    }

    /* set baudrate */
    cfsetspeed(&term, baud);

    /* 8N1 mode */
    term.c_cflag &= ~PARENB;
    term.c_cflag &= ~CSTOPB;
    term.c_cflag &= ~CSIZE;
    term.c_cflag |= CS8;

    /* enbale raw mode */
    cfmakeraw(&term);

    /* write serial port configure */
    if (tcsetattr(fd, TCSADRAIN, &term) != 0)
    {
        SPDLOG_LOGGER_ERROR(mylogger, "config_port: Unable to write configure - ");
        SPDLOG_LOGGER_ERROR(console, "config_port: Unable to write configure - ");
        perror("config_port: Unable to write configure - ");
        return -1;
    }
}

/*
 * close serial port 
 */
int close_port(int fd)
{
    if (close(fd) == -1)
    {
        SPDLOG_LOGGER_ERROR(mylogger, "close_port: Unable to close port - ");
        SPDLOG_LOGGER_ERROR(console, "close_port: Unable to close port - ");
        perror("close_port: Unable to close port - ");
        return -1;
    }
    return 0;
}