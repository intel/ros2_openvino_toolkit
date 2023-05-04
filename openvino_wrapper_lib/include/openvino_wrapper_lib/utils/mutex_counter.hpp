// Copyright (c) 2018-2022 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// @brief a utility class for mutex counter (Thread Safe).
// @file mutex_counter.hpp
//

#ifndef OPENVINO_WRAPPER_LIB__UTILS__MUTEX_COUNTER_HPP_
#define OPENVINO_WRAPPER_LIB__UTILS__MUTEX_COUNTER_HPP_

#include <mutex>

class MutexCounter
{
public:
  explicit MutexCounter(int init_counter = 0)
  {
    std::lock_guard<std::mutex> lk(counter_mutex_);
    counter_ = init_counter;
  }

  void increaseCounter()
  {
    std::lock_guard<std::mutex> lk(counter_mutex_);
    ++counter_;
  }

  void decreaseCounter()
  {
    std::lock_guard<std::mutex> lk(counter_mutex_);
    --counter_;
  }

  int get()
  {
    return counter_;
  }

private:
  std::atomic<int> counter_;
  std::mutex counter_mutex_;
  std::condition_variable cv_;
};

#endif  // OPENVINO_WRAPPER_LIB__UTILS__MUTEX_COUNTER_HPP_
