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

#ifndef MULTIPLE_IO_FACTORY_H_
#define MULTIPLE_IO_FACTORY_H_

#include "multiple_io_base.h"
#include "multiple_io_epoll.h"
#include "multiple_io_kqueue.h"
#include "multiple_io_select.h"
#include "multiple_io_poll.h"
#include <memory>

namespace livox {

class MultipleIOFactory {
 public:
  static std::unique_ptr<MultipleIOBase> CreateMultipleIO() {
#if defined(HAVE_EPOLL)
    return std::unique_ptr<MultipleIOBase>(new MultipleIOEpoll());
#elif defined(HAVE_KQUEUE)
    return std::unique_ptr<MultipleIOBase>(new MultipleIOKqueue());
#elif defined(HAVE_SELECT)
    return std::unique_ptr<MultipleIOBase>(new MultipleIOSelect());
#elif defined(HAVE_POLL)
    return std::unique_ptr<MultipleIOBase>(new MultipleIOPoll());
#else
    return nullptr;
#endif
  }
};

}  // namespace livox

#endif  // MULTIPLE_IO_FACTORY_H_