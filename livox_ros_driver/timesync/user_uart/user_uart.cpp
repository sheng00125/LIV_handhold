//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "user_uart.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

namespace livox_ros {

UserUart::UserUart(uint8_t baudrate_index, uint8_t parity)
    : baudrate_(baudrate_index), parity_(parity) {
  fd_ = 0;
  is_open_ = false;
}

UserUart::~UserUart() {
  is_open_ = false;
  if (fd_ > 0) {
    /** first we flush the port */
    tcflush(fd_, TCOFLUSH);
    tcflush(fd_, TCIFLUSH);

    close(fd_);
  }
}

int UserUart::Open(const char *filename) {
  fd_ = open(filename, O_RDWR | O_NOCTTY);  //| O_NDELAY
  if (fd_ < 0) {
    printf("Open %s fail!\n", filename);
    return -1;
  } else {
    chmod(filename, S_IRWXU | S_IRWXG | S_IRWXO); /* need add here */
    printf("Open %s success!\n", filename);
  }

  if (fd_ > 0) {
    /** set baudrate and parity,etc. */
    if (Setup(baudrate_, parity_)) {
      return -1;
    }
  }

  is_open_ = true;
  return 0;
}

int UserUart::Close() {
  is_open_ = false;
  if (fd_ > 0) {
    /** first we flush the port */
    tcflush(fd_, TCOFLUSH);
    tcflush(fd_, TCIFLUSH);
    return close(fd_);
  }

  return -1;
}

/** sets up the port parameters */
int UserUart::Setup(uint8_t baudrate_index, uint8_t parity) {
  static uint32_t baud_map[19] = {
      B2400,    B4800,    B9600,    B19200,   B38400,  B57600,   B115200,
      B230400,  B460800,  B500000,  B576000,  B921600, B1152000, B1500000,
      B2000000, B2500000, B3000000, B3500000, B4000000};
  tcflag_t baudrate;
  struct termios options;

  if ((baudrate_index > BR4000000) || (parity > P_7S1)) {
    return -1;
  }

  /** clear old setting completely,must add here for CDC serial */
  tcgetattr(fd_, &options);
  memset(&options, 0, sizeof(options));
  tcflush(fd_, TCIOFLUSH);
  tcsetattr(fd_, TCSANOW, &options);
  usleep(10000);

  /** Enable the receiver and set local mode... */
  options.c_cflag |= (CLOCAL | CREAD);

  /** Disable hardware flow */
  // options.c_cflag &= ~CRTSCTS;

  /** Disable software flow */
  // options.c_iflag &= ~(IXON | IXOFF | IXANY);

  // options.c_oflag &= ~OPOST;

  /** set boadrate */
  options.c_cflag &= ~CBAUD;
  baudrate = baud_map[baudrate_index];
  options.c_cflag |= baudrate;

  switch (parity) {
    case P_8N1:
      /** No parity (8N1)  */
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS8;
      break;
    case P_7E1:
      /** Even parity (7E1) */
      options.c_cflag |= PARENB;
      options.c_cflag &= ~PARODD;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS7;
      break;
    case P_7O1:
      /** Odd parity (7O1) */
      options.c_cflag |= PARENB;
      options.c_cflag |= PARODD;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS7;
      break;
    case P_7S1:
      /** Space parity is setup the same as no parity (7S1)  */
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS8;
      break;
    default:
      return -1;
  }

  /** now we setup the values in port's termios */
  options.c_iflag &= ~INPCK;

  /** Enable non-canonical */
  // options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  /** Time to wait for data */
  options.c_cc[VTIME] = 1;

  /** Minimum number of characters to read */
  options.c_cc[VMIN] = 1;

  /** flush the port */
  tcflush(fd_, TCIOFLUSH);

  /** send new config to the port */
  tcsetattr(fd_, TCSANOW, &options);

  return 0;
}

ssize_t UserUart::Write(const char *buffer, size_t size) {
  if (fd_ > 0) {
    return write(fd_, buffer, size);
  } else {
    return 0;
  }
}

ssize_t UserUart::Read(char *buffer, size_t size) {
  if (fd_ > 0) {
    return read(fd_, buffer, size);
  } else {
    return 0;
  }
}

}  // namespace livox_ros
