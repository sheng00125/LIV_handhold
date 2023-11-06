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

#include "synchro.h"
#include <stdio.h>
#include <string.h>

#ifdef WIN32
#include <Windows.h>
#else
#include <unistd.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <fcntl.h>
#endif

#ifdef WIN32
Synchro::Synchro() {
  hfile_ = INVALID_HANDLE_VALUE;
  is_quite_ = false;
  rmc_buff_.resize(128,0);
}
#else
Synchro::Synchro() {
  fd_ = 0; 
  is_quite_ = false;
  rmc_buff_.resize(128,0);
}
#endif

Synchro::~Synchro() {
  Stop();
}

void Synchro::SetBaudRate(BaudRate baudrate) {
  baudrate_ = baudrate;
}

void Synchro::SetParity(Parity parity) {
  parity_ = parity;
}

void Synchro::SetPortName(std::string port_name) {
  port_name_ = port_name;
}

bool Synchro::Start() {
  if (!Open()) {
    return false;
  }
  is_quite_ = false;
  listener_ = std::make_shared<std::thread>(&Synchro::IOLoop, this);
  return true;
}

void Synchro::Stop() {
  is_quite_ = true;
  if (listener_) {
    listener_->join();
    listener_ = nullptr;
  }
  Close();
}

void Synchro::IOLoop() {
  uint8_t buff[READ_BUF];
  size_t size = 0;
  while (!is_quite_) {
    if (0 != (size = Read(buff, READ_BUF))) {
      Decode(buff, size);
    }
  }
}

bool Synchro::ParseGps(uint8_t in_byte) {
  if (cur_len_ < strlen("$GPRMC")) {
    rmc_buff_[0] = rmc_buff_[1];
    rmc_buff_[1] = rmc_buff_[2];
    rmc_buff_[2] = rmc_buff_[3];
    rmc_buff_[3] = rmc_buff_[4];
    rmc_buff_[4] = rmc_buff_[5];
    rmc_buff_[5] = in_byte;
    cur_len_++;

    if (cur_len_ == strlen("$GPRMC")) {
      if (strncmp((const char*)&rmc_buff_[0], "$GPRMC", strlen("$GPRMC")) != 0 &&  \
          strncmp((const char*)&rmc_buff_[0], "$GNRMC", strlen("$GNRMC")) != 0) {
        cur_len_--;
      } 
    }
    return false;
  }

  if (cur_len_ >= rmc_buff_.size()) {
    Clear();
    return false;
  }

  rmc_buff_[cur_len_] = in_byte;
/** Checksum $GPRMC/$GNRMC. */
  if (rmc_buff_[cur_len_ - 2] == '*') {
     uint8_t result = 0;
     for (int i = 1; i < cur_len_ - 2; i++) {
       result ^= rmc_buff_[i];
     }
     char crc[3] = {0};
     uint32_t crc_num = 0;
     strncpy(crc, &rmc_buff_[cur_len_ - 1], 2);
     sscanf(crc, "%x", &crc_num);
     result ^= (uint8_t)crc_num;
     if (result == 0) {
       return true;
     }
  }
  cur_len_++;
  return false; 
}

void Synchro::SetSyncTimerCallback(UtcTimerCallback cb) {
  sync_timer_cb_ = cb;
}

void Synchro::Decode(uint8_t * buff, size_t size) {
  for (size_t i = 0; i < size; i++) {
    if (ParseGps(buff[i])) {
      if (sync_timer_cb_) {
        sync_timer_cb_(&rmc_buff_.at(0), cur_len_);
        Clear();
      }
    }
  }
}

void Synchro::Clear() {
   cur_len_ = 0;
   std::fill(rmc_buff_.begin(), rmc_buff_.end(), 0);
}

#ifdef WIN32
bool Synchro::Open() {
  hfile_ = CreateFile(port_name_.c_str(),
                      GENERIC_READ,
                      0,
                      NULL,
                      OPEN_EXISTING,
                      FILE_ATTRIBUTE_NORMAL,
                      NULL);

  if (hfile_ == INVALID_HANDLE_VALUE) {
    printf("Open %s serials fail!\n", port_name_.c_str());
    return false;
  }

 /* Set baudrate and parity,etc. */
  Setup(baudrate_, parity_);
  printf("Set baudrate success.\n");
  return true;
}

void Synchro::Close() {
  if (hfile_ != INVALID_HANDLE_VALUE) {
    PurgeComm(hfile_, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT); 
    CloseHandle(hfile_);
  }
  hfile_ = INVALID_HANDLE_VALUE;
}

/* Sets up the port parameters */
int Synchro::Setup(enum BaudRate baud, enum Parity parity) {
  static uint32_t baud_map[19] = {\
                                   2400, 4800, 9600, 19200, 38400, 57600,115200, 230400,\
                                   460800, 500000, 576000,921600,1152000, 1500000, 2000000,\
                                   2500000, 3000000, 3500000, 4000000\
                                 };
  DCB dcb;
  GetCommState(hfile_, &dcb);
  dcb.BaudRate = DWORD(baud_map[baud]);
  switch (parity) {
    case P_8N1:
      /* No parity (8N1)  */
      dcb.ByteSize = 8;
      dcb.Parity = NOPARITY;
      dcb.StopBits = ONESTOPBIT;
    break;
    case P_7E1:
      /* Even parity (7E1) */
      dcb.ByteSize = 7;
      dcb.Parity = EVENPARITY;
      dcb.StopBits = ONESTOPBIT;
    break;
    case P_7O1:
      /* Odd parity (7O1) */
      dcb.ByteSize = 7;
      dcb.Parity = ODDPARITY;
      dcb.StopBits = ONESTOPBIT;
    break;
    case P_7S1:
      /* Space parity is setup the same as no parity (7S1)  */
      dcb.ByteSize = 7;
      dcb.Parity = SPACEPARITY;
      dcb.StopBits = ONESTOPBIT;
    break;
    default:
      return -1;
  }
  SetCommState(hfile_, &dcb);
  PurgeComm(hfile_, PURGE_RXCLEAR|PURGE_TXCLEAR|PURGE_RXABORT|PURGE_TXABORT);
  return 0;
}

size_t Synchro::Read(uint8_t *buffer, size_t size) {
  DWORD len = 0;
  if (hfile_ !=  INVALID_HANDLE_VALUE) {
    ReadFile(hfile_, buffer, size, &len, NULL);
    return len;
  } else {
    return 0;
  }
}

#else
bool Synchro::Open() {
  fd_ = open(port_name_.c_str(), O_RDONLY | O_NOCTTY);
  if (fd_ < 0) {
    printf("Open %s serials fail!\n", port_name_.c_str());
    return false;
  }

  /* Set baudrate and parity,etc. */
  Setup(baudrate_, parity_);
  printf("Set baudrate success.\n");
  return true;
}

void Synchro::Close() {
  if (fd_ > 0) {
    /* Flush the port */
    tcflush(fd_,TCOFLUSH); 
    tcflush(fd_,TCIFLUSH); 

    close(fd_);
  }
  fd_ = 0;
}

/* Sets up the port parameters */
int Synchro::Setup(enum BaudRate baud, enum Parity parity) {
  static uint32_t baud_map[19] = {
                                   B2400, B4800, B9600, B19200, B38400, B57600,B115200, B230400
                                 };
  tcflag_t baudrate;
  struct termios options;

  /* Clear old setting completely,must add here for A3 CDC serial */
  tcgetattr(fd_, &options);
  memset(&options, 0, sizeof(options));
  tcflush(fd_, TCIOFLUSH);
  tcsetattr(fd_, TCSANOW, &options);
  usleep(10000);

  /* Minimum number of characters to read */
  options.c_cc[VMIN] = 0;
  /* Time to wait for data */
  options.c_cc[VTIME] = 100;

  /* Enable the receiver and set local mode... */
  options.c_cflag |= (CLOCAL | CREAD);

  /* Set boadrate */
  baudrate = baud_map[baud];
  cfsetispeed(&options, baudrate);
  cfsetospeed(&options, baudrate);
  printf("[Baudrate]: %d %u\r\n", baud, baudrate);

  switch (parity) {
    case P_8N1:
      /* No parity (8N1)  */
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS8;
    break;
    case P_7E1:
      /* Even parity (7E1) */
      options.c_cflag |= PARENB;
      options.c_cflag &= ~PARODD;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS7;
    break;
    case P_7O1:
      /* Odd parity (7O1) */
      options.c_cflag |= PARENB;
      options.c_cflag |= PARODD;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS7;
    break;
    case P_7S1:
      /* Space parity is setup the same as no parity (7S1)  */
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS8;
    break;
    default:
      return -1;
  }    

  /* flush the port */
  tcflush(fd_, TCIOFLUSH);
    
  /* send new config to the port */
  tcsetattr(fd_, TCSANOW, &options);

  return 0;
}

size_t Synchro::Read(uint8_t *buffer, size_t size) {
  if (fd_ > 0) {
    return read(fd_, buffer, size);
  } else {
    return 0;
  }
}
#endif



