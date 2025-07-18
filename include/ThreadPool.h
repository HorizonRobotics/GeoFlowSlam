// Copyright (c) 2022，Horizon Robotics.
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

#ifndef SRC_COMMON_THREADPOOL_H_
#define SRC_COMMON_THREADPOOL_H_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <vector>
#include <future>

namespace hobot {
typedef std::function<void()> TaskFunction;
struct Task {
  TaskFunction func;
  std::shared_ptr<std::promise<bool>> promise;
  explicit Task(const TaskFunction &_task) : func(_task) {
    promise = std::make_shared<std::promise<bool>>();
  }
  ~Task() {
//    if (promise) {
//      promise->set_value(true);
//    }
  }
};

class CThreadPool {
 public:
  CThreadPool();
  virtual ~CThreadPool();
  void CreateThread(int threadCount);
  // post an async task
  std::shared_ptr<std::promise<bool>> PostTask(const TaskFunction &task);
  int GetTaskNum();
  void ClearTask();
  void Stop() {}
  void Start() {}

 protected:
  void exec_loop();

 private:
  typedef std::list<std::shared_ptr<Task> > TaskContainer;
  TaskContainer m_setTaskQuenes;
  mutable std::mutex m_mutThread;
  // a mutex for task quene operations only
  mutable std::mutex m_mutTaskQuene;

  std::condition_variable m_varCondition;
  std::atomic<int> m_nNumRunningThreads;
  typedef std::shared_ptr<std::thread> CThreadPtr;

  std::vector<CThreadPtr> m_vecThreads;
  std::atomic<bool> stop_;
  // cound set the value before starting the thread pool only
  int m_nMaxThreads = 0;
};
}  // namespace hobot
#endif  // SRC_COMMON_THREADPOOL_H_
