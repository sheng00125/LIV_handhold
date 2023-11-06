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

#ifndef LIVOX_COMMAND_CALLBACK_H
#define LIVOX_COMMAND_CALLBACK_H

#include <functional>
#include <memory>
#include "livox_sdk.h"

namespace livox {

class CommandCallback {
 public:
  virtual ~CommandCallback() {}
  virtual void operator()(livox_status status,uint8_t handle, void *data) = 0;
};

template <class T, class ResponseType>
class MemberFunctionCallback : public CommandCallback {
 public:
  typedef void (T::*MemFn)(livox_status status, uint8_t handle, ResponseType *data);

  MemberFunctionCallback(T *cls, MemFn func) : this_(cls), func_(func) {}
  void operator()(livox_status status, uint8_t handle, void *data) {
    if (this_) {
      (this_->*func_)(status, handle, static_cast<ResponseType *>(data));
    }
  }

 private:
  T *this_;
  MemFn func_;
};

template <class T>
class MemberFunctionCallback<T, uint8_t> : public CommandCallback {
 public:
  typedef void (T::*MemFn)(livox_status status, uint8_t handle, uint8_t response);

  MemberFunctionCallback(T *cls, MemFn func) : this_(cls), func_(func) {}
  void operator()(livox_status status, uint8_t handle, void *data) {
    if (this_) {
      if (data == NULL) {
        (this_->*func_)(status, handle, 0);
      } else {
        (this_->*func_)(status, handle, static_cast<uint8_t>(reinterpret_cast<uintptr_t>(data)));
      }
    }
  }

 private:
  T *this_;
  MemFn func_;
};

template <class T, class ResponseType>
std::shared_ptr<CommandCallback> MakeCommandCallback(T *cls,
                                                       typename MemberFunctionCallback<T, ResponseType>::MemFn func) {
  std::shared_ptr<CommandCallback> cb(new MemberFunctionCallback<T, ResponseType>(cls, func));
  return cb;
}

template <class T>
class FunctionStatusCallback : public CommandCallback {
 public:
  typedef void (*Fn)(livox_status status, uint8_t handle, T *data, void *client_data);

 public:
  FunctionStatusCallback(Fn func, void *client_data) : func_(func), client_data_(client_data) {}
  void operator()(livox_status status, uint8_t handle, void *data) {
    if (func_) {
      (*func_)(status, handle, static_cast<T *>(data), client_data_);
    }
  }

 private:
  Fn func_;
  void *client_data_;
};

template <>
class FunctionStatusCallback<uint8_t> : public CommandCallback {
 public:
  typedef void (*Fn)(livox_status status, uint8_t handle, uint8_t data, void *client_data);

 public:
  FunctionStatusCallback(Fn func, void *client_data) : func_(func), client_data_(client_data) {}
  void operator()(livox_status status, uint8_t handle, void *data) {
    if (func_) {
      if (data == NULL) {
        (*func_)(status, handle, 0, client_data_);
      } else {
        (*func_)(status, handle, *(uint8_t *)data, client_data_);
      }
    }
  }

 private:
  Fn func_;
  void *client_data_;
};

template <class T>
std::shared_ptr<CommandCallback> MakeCommandCallback(typename FunctionStatusCallback<T>::Fn func, void *client_data) {
  std::shared_ptr<CommandCallback> cb(new FunctionStatusCallback<T>(func, client_data));
  return cb;
}

template <class T>
class MessageCallback : public CommandCallback {
 public:
  typedef void (*Fn)(livox_status status, uint8_t handle, T *data);

 public:
  MessageCallback(const Fn &func) : func_(func) {}
  void operator()(livox_status status, uint8_t handle, void *data) {
    if (func_) {
      (*func_)(status, handle, static_cast<T *>(data));
    }
  }

 private:
  Fn func_;
};

template <class ResponseType>
class BoostFunctionMessageCallback : public CommandCallback {
 public:
  typedef std::function<void(livox_status, uint8_t, ResponseType *)> Fn;

 public:
  BoostFunctionMessageCallback(const Fn &func) : func_(func) {}
  void operator()(livox_status status, uint8_t handle, void *data) {
    if (func_) {
      func_(status, handle, static_cast<ResponseType *>(data));
    }
  }

 private:
  Fn func_;
};

template <class T>
std::shared_ptr<CommandCallback> MakeMessageCallback(typename MessageCallback<T>::Fn func) {
  std::shared_ptr<CommandCallback> cb(new MessageCallback<T>(func));
  return cb;
}

template <class T>
std::shared_ptr<CommandCallback> MakeMessageCallback(typename BoostFunctionMessageCallback<T>::Fn func) {
  std::shared_ptr<CommandCallback> cb(new BoostFunctionMessageCallback<T>(func));
  return cb;
}

template <class T, class ResponseType>
class MemberMessageCallback : public CommandCallback {
 public:
  typedef void (T::*MemFn)(livox_status status, uint8_t handle, ResponseType *data);

  MemberMessageCallback(T *cls, MemFn func) : this_(cls), func_(func) {}
  void operator()(livox_status status,uint8_t handle, void *data) {
    if (this_) {
      (this_->*func_)(status, handle, (ResponseType *)data);
    }
  }

 private:
  T *this_;
  MemFn func_;
};

template <class T, class ResponseType>
std::shared_ptr<CommandCallback> MakeMemberMessageCallback(
    T *_this,
    typename MemberMessageCallback<T, ResponseType>::MemFn func) {
  std::shared_ptr<CommandCallback> cb(new MemberMessageCallback<T, ResponseType>(_this, func));
  return cb;
}

template <class ResponseType>
class BoostFunctionCallback : public CommandCallback {
 public:
  typedef std::function<void(uint8_t, uint8_t, ResponseType *)> Fn;
  BoostFunctionCallback(const Fn &func) : func_(func) {}
  void operator()(livox_status status,uint8_t handle, void *data) {
    if (func_) {
      if (data == NULL) {
        func_(status, handle, NULL);
      } else {
        func_(status, handle, (ResponseType *)data);
      }
    }
  }

 private:
  Fn func_;
};

template <>
class BoostFunctionCallback<uint8_t> : public CommandCallback {
 public:
  typedef std::function<void(uint8_t, uint8_t, uint8_t)> Fn;
  BoostFunctionCallback(const Fn &func) : func_(func) {}
  void operator()(livox_status status,uint8_t handle, void *data) {
    if (func_) {
      if (data == NULL) {
        func_(status, handle, 0);
      } else {
        func_(status, handle, static_cast<uint8_t>(reinterpret_cast<uintptr_t>(data)));
      }
    }
  }

 private:
  Fn func_;
};

template <class ResponseType>
std::shared_ptr<CommandCallback> MakeCommandCallback(const typename BoostFunctionCallback<ResponseType>::Fn &func) {
  std::shared_ptr<CommandCallback> cb(new BoostFunctionCallback<ResponseType>(func));
  return cb;
}

}  // namespace livox
#endif  // LIVOX_COMMAND_CALLBACK_H
