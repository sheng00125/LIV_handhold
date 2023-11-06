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

#ifndef LIVOX_HUB_DATA_HANDLER_H_
#define LIVOX_HUB_DATA_HANDLER_H_

#include <memory>
#include "base/io_thread.h"
#include "data_handler.h"

namespace livox {
class HubDataHandlerImpl : public DataHandlerImpl {
 public:
  HubDataHandlerImpl(DataHandler *handler)
      : DataHandlerImpl(handler), thread_(new IOThread()), is_valid_(false) {}

  ~HubDataHandlerImpl() { Uninit(); }
  bool Init();
  void Uninit();

  bool AddDevice(const DeviceInfo &info);
  void RemoveDevice(uint8_t t);
  void OnData(socket_t sock, void *client_data);

 private:
  std::unique_ptr<IOThread> thread_;
  socket_t sock_ = -1;
  DeviceInfo hub_info_;
  bool is_valid_;
  std::vector<char> buf_;
};

}  // namespace livox

#endif  // LIVOX_HUB_DATA_HANDLER_H_
