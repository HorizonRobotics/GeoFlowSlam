#include "ThreadPool.h"
#include <memory>
namespace hobot {
CThreadPool::CThreadPool() { stop_ = false; }

CThreadPool::~CThreadPool() {
  stop_ = true;
  m_varCondition.notify_all();
  std::lock_guard<std::mutex> lck(m_mutThread);
  for (int i = 0; i < m_nMaxThreads; ++i) {
    m_vecThreads[i]->join();
  }
}

void hobot::CThreadPool::CreateThread(int threadCount) {
  std::lock_guard<std::mutex> lck(m_mutThread);
  m_nMaxThreads = threadCount;
  m_nNumRunningThreads = 0;
  m_vecThreads.reserve(m_nMaxThreads);
  for (int i = 0; i < m_nMaxThreads; ++i) {
    auto thread =
        std::make_shared<std::thread>(std::bind(&CThreadPool::exec_loop, this));
    m_vecThreads.push_back(thread);
  }
  //  wait all threads to start, enter main loop
  while (m_nNumRunningThreads < static_cast<int>(m_vecThreads.size())) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void CThreadPool::exec_loop() {
  ++m_nNumRunningThreads;
  while (!stop_) {
    std::shared_ptr<Task> tsk;
    {
      std::unique_lock<std::mutex> lck(m_mutTaskQuene);
      if (!stop_ && m_setTaskQuenes.size() <= 0) {
        m_varCondition.wait(lck);
      }

      if (stop_ || m_setTaskQuenes.size() <= 0) {
        continue;
      }
      tsk = m_setTaskQuenes.front();
      m_setTaskQuenes.pop_front();
    }
    //  Exec one task, wake other threads.
    tsk->func();
    tsk->promise->set_value(true);
  }
}

std::shared_ptr<std::promise<bool>> CThreadPool::PostTask(
    const TaskFunction &fun) {
  std::shared_ptr<std::promise<bool>> promise;
  auto task = std::make_shared<Task>(fun);
  promise = task->promise;
  {
    std::lock_guard<std::mutex> lck(m_mutTaskQuene);
    m_setTaskQuenes.push_back(task);
  }
  m_varCondition.notify_one();  // wake worker thread(s)
  return promise;
}

void CThreadPool::ClearTask() {
  std::lock_guard<std::mutex> lck(m_mutTaskQuene);
  m_setTaskQuenes.clear();
}
int CThreadPool::GetTaskNum() { return m_setTaskQuenes.size(); }

}  // namespace hobot
