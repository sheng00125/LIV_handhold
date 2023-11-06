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

#include "logging.h"

std::shared_ptr<spdlog::logger> logger = NULL;
bool is_save_log_file = false;
bool is_console_log_enable = true;

void InitLogger() {

  if (spdlog::get("console") != nullptr) {
    logger = spdlog::get("console");
    return;
  }

  std::vector<spdlog::sink_ptr> sinkList;
  if (is_console_log_enable) {
    auto consoleSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    consoleSink->set_level(spdlog::level::debug);
    sinkList.push_back(consoleSink);
  }

  if (is_save_log_file) {
    auto rotateSink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("livox_log.txt", 1024 * 1024 * 5, 2);
    rotateSink->set_level(spdlog::level::debug);
    sinkList.push_back(rotateSink);
  }

  logger = std::make_shared<spdlog::logger>("console", begin(sinkList), end(sinkList));
  spdlog::register_logger(logger);
  logger->set_level(spdlog::level::debug);
  logger->flush_on(spdlog::level::debug);
}

void UninitLogger() {
  spdlog::drop_all();
}