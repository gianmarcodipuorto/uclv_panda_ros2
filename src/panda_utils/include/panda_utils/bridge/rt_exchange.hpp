#pragma once

#include <cstddef>
#include <cstdint>

#include <realtime_tools/lock_free_queue.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>

namespace panda_bridge {

// Lock-free single producer, single consumer queue: commands from ROS to the real-time loop,
// events from the loop back to ROS.
template <typename T, std::size_t Capacity>
using SpscQueue = realtime_tools::LockFreeSPSCQueue<T, Capacity>;

// Latest value written by one thread and read by another. try_set never blocks the real-time
// loop (the value is skipped for one cycle if the reader holds the lock).
template <typename T>
using SnapshotBox = realtime_tools::RealtimeThreadSafeBox<T>;

// Event raised by the real-time loop and logged by the publisher thread.
struct RtEvent {
  uint16_t code{0};
  uint64_t id{0};
  double value{0.0};
};

}  // namespace panda_bridge
