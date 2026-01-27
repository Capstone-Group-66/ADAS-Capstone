// File: include/adas/queues/SPSCQueue.hpp
// Lock-free Single-Producer Single-Consumer queue
// Per Section 4.4.1 of proposal - preallocated, no dynamic allocation during operation
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <optional>

namespace adas {

/// Lock-free Single-Producer Single-Consumer queue
/// - Preallocated buffer (no malloc during push/pop)
/// - Cache-line aligned head/tail to prevent false sharing
/// - Drop counter for health monitoring
///
/// @tparam T Element type (must be movable)
/// @tparam Capacity Maximum number of elements (default 8)
template <typename T, size_t Capacity = 8> class SPSCQueue {
    static_assert(Capacity > 0, "Capacity must be positive");
    static_assert((Capacity & (Capacity - 1)) == 0 || true,
                  "Capacity need not be power of 2, but modulo is used");

  public:
    SPSCQueue() = default;

    // Non-copyable, non-movable (queues own their buffer)
    SPSCQueue(const SPSCQueue &) = delete;
    SPSCQueue &operator=(const SPSCQueue &) = delete;
    SPSCQueue(SPSCQueue &&) = delete;
    SPSCQueue &operator=(SPSCQueue &&) = delete;

    /// Push item to queue (producer thread only)
    /// @param item Item to push (will be moved)
    /// @return true if successful, false if queue full (increments drop counter)
    bool try_push(T &&item) {
        const size_t curr_tail = tail_.load(std::memory_order_relaxed);
        const size_t next_tail = next_index(curr_tail);

        // Check if full (next_tail would catch up to head)
        if (next_tail == head_.load(std::memory_order_acquire)) {
            drops_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        buffer_[curr_tail] = std::move(item);
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }

    /// Push item to queue (lvalue version)
    bool try_push(const T &item) {
        T copy = item;
        return try_push(std::move(copy));
    }

    /// Pop item from queue (consumer thread only)
    /// @param item Output parameter for popped item
    /// @return true if item retrieved, false if queue empty
    bool try_pop(T &item) {
        const size_t curr_head = head_.load(std::memory_order_relaxed);

        // Check if empty (head caught up to tail)
        if (curr_head == tail_.load(std::memory_order_acquire)) {
            return false;
        }

        item = std::move(buffer_[curr_head]);
        head_.store(next_index(curr_head), std::memory_order_release);
        return true;
    }

    /// Pop item from queue (returns optional)
    std::optional<T> try_pop() {
        T item;
        if (try_pop(item)) {
            return std::move(item);
        }
        return std::nullopt;
    }

    /// Peek at front item without consuming (useful for timestamp checks)
    /// Note: Not thread-safe for modifications, only for reads
    std::optional<const T *> peek() const {
        const size_t curr_head = head_.load(std::memory_order_relaxed);
        if (curr_head == tail_.load(std::memory_order_acquire)) {
            return std::nullopt;
        }
        return &buffer_[curr_head];
    }

    /// Check if queue is empty
    bool empty() const {
        return head_.load(std::memory_order_relaxed) == tail_.load(std::memory_order_relaxed);
    }

    /// Check if queue is full
    bool full() const {
        const size_t curr_tail = tail_.load(std::memory_order_relaxed);
        return next_index(curr_tail) == head_.load(std::memory_order_relaxed);
    }

    /// Current queue size (approximate, may race with producer/consumer)
    size_t size() const {
        const size_t head = head_.load(std::memory_order_relaxed);
        const size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail >= head) {
            return tail - head;
        }
        return (Capacity + 1) - head + tail;
    }

    /// Maximum capacity
    static constexpr size_t capacity() { return Capacity; }

    /// Total dropped items since construction or last reset
    uint64_t drops() const { return drops_.load(std::memory_order_relaxed); }

    /// Reset drop counter (for testing/monitoring resets)
    void reset_drops() { drops_.store(0, std::memory_order_relaxed); }

    /// Clear all items from queue (must be called when no producers/consumers active)
    void clear() {
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
    }

  private:
    /// Compute next index with wrap-around
    static constexpr size_t next_index(size_t idx) { return (idx + 1) % (Capacity + 1); }

    // Buffer has Capacity+1 slots to distinguish full from empty
    std::array<T, Capacity + 1> buffer_;

    // Cache-line aligned to prevent false sharing between producer and consumer
    alignas(64) std::atomic<size_t> head_{0};
    alignas(64) std::atomic<size_t> tail_{0};
    alignas(64) std::atomic<uint64_t> drops_{0};
};

} // namespace adas
