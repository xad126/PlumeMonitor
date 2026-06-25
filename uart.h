#pragma once
#include <iostream>
#include <errno.h> /* Error number definitions */
#include <fcntl.h> /* File control definitions */
#include <stdio.h>
#include <string.h>
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>  /* UNIX standard function definitions */

int open_port(char *path);
ssize_t write_port(int fd, void *buf, size_t len);
int read_port_setup(int fd, cc_t timeout_sec, cc_t min_bytes);
ssize_t read_port(int fd, void *buf, size_t len, cc_t sec, cc_t min);
int config_port(int fd, speed_t baud);
int close_port(int fd);