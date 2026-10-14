#include <chrono>
#include <atomic>
#include <functional>
#include <future>
#include <gtest/gtest.h>
#include <gazebo/transport/IOManager.hh>
#include <gazebo/transport/TaskGroup.hh>
#include <tbb/global_control.h>

TEST(GazeboIOManager, CanReleaseTheLastOwnerInsideAnIOCallback)
{
  for (int i = 0; i < 100; ++i) {
    std::promise<void> released;
    auto finished = released.get_future();
    auto * manager = new gazebo::transport::IOManager();
    manager->GetIO().post([manager, &released]() {
      delete manager;
      released.set_value();
    });
    ASSERT_EQ(finished.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  }
}

TEST(GazeboIOManager, NormalOwnerJoinsTheIOThread)
{
  for (int i = 0; i < 100; ++i) {
    gazebo::transport::IOManager manager;
    std::promise<void> serviced;
    auto finished = serviced.get_future();
    manager.GetIO().post([&serviced]() { serviced.set_value(); });
    ASSERT_EQ(finished.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  }
}

TEST(GazeboIOManager, PendingTasksDrainOnTheArenaThatQueuedThem)
{
  tbb::global_control single_worker(tbb::global_control::max_allowed_parallelism, 1);
  gazebo::transport::TaskGroup tasks;
  gazebo::transport::IOManager manager;
  std::atomic<int> processed{0};
  std::promise<void> queued;
  auto ready = queued.get_future();
  manager.GetIO().post([&]() {
    for (int i = 0; i < 100; ++i) {
      tasks.run<std::function<void()>>([&]() { ++processed; });
    }
    queued.set_value();
  });
  ASSERT_EQ(ready.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  std::promise<void> drained;
  auto completed = drained.get_future();
  manager.GetIO().dispatch([&]() {
    tasks.Wait();
    drained.set_value();
  });
  ASSERT_EQ(completed.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  EXPECT_EQ(processed, 100);
}
