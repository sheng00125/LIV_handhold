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

#ifndef USER_UART_H_
#define USER_UART_H_

#include <stdint.h>
#include <sys/fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <termios.h>

namespace livox_ros {

enum Parity {
  P_8N1, /* No parity (8N1)	*/
  P_7E1, /* Even parity (7E1)*/
  P_7O1, /* Odd parity (7O1)	*/
  P_7S1, /* Space parity is setup the same as no parity (7S1)	*/
  ParityUnkown
};

enum BaudRate {
  BR2400,
  BR4800,
  BR9600,
  BR19200,
  BR38400,
  BR57600,
  BR115200,
  BR230400,
  BR460800,
  BR500000,
  BR576000,
  BR921600,
  BR1152000,
  BR1500000,
  BR2000000,
  BR2500000,
  BR3000000,
  BR3500000,
  BR4000000,
  BRUnkown,
};

class UserUart {
 public:
  UserUart(uint8_t baudrate_index, uint8_t parity);
  ~UserUart();

  int Setup(uint8_t baudrate_index, uint8_t parity);
  ssize_t Write(const char *buffer, size_t size);
  ssize_t Read(char *buffer, size_t size);
  int Close();
  int Open(const char *filename);
  bool IsOpen() { return is_open_; };

 private:
  int fd_;
  volatile bool is_open_;

  uint8_t baudrate_;
  uint8_t parity_;
};

}  // namespace livox_ros

#endif
