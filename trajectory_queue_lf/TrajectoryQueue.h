// TrajectoryQueue.h
#ifndef TRAJECTORYQUEUE_H
#define TRAJECTORYQUEUE_H

#pragma once

#include <atomic>
#include <cstddef> // For std::size_t
#include <memory>  // For std::unique_ptr
#include <optional> // For std::optional
#include <type_traits> // For std::is_nothrow_copy_assignable, std::is_nothrow_move_assignable

// BEGIN MODIFICATION
namespace RDT {
// END MODIFICATION

/**
 * @brief A Single-Producer, Single-Consumer (SPSC) lock-free ring buffer queue.
 *
 * This queue is designed for high-performance scenarios where one thread produces
 * items and another thread consumes them. It uses atomic operations to achieve
 * lock-freedom.
 *
 * @tparam T The type of elements stored in the queue.
 * @tparam Capacity The maximum number of elements the queue can hold.
 *                  Must be a power of 2. Defaults to 256.
 */
template <typename T, std::size_t Capacity = 256>
class TrajectoryQueue {
    // Ensure Capacity is a power of 2 for efficient modulo operations using bitwise AND
    static_assert((Capacity > 0) && ((Capacity & (Capacity - 1)) == 0),
                  "TrajectoryQueue: Capacity must be a power of 2 and greater than 0.");

public:
    /**
     * @brief Default constructor. Initializes an empty queue.
     */
    TrajectoryQueue() : head_(0), tail_(0) {
        // Allocate buffer. For non-default constructible T, this might need adjustment,
        // but for TrajectoryPoint (which is default constructible) it's fine.
        buffer_ = std::make_unique<T[]>(Capacity);
    }

    // Queues are typically not copyable or movable due to internal buffer and atomic states.
    TrajectoryQueue(const TrajectoryQueue&) = delete;
    TrajectoryQueue& operator=(const TrajectoryQueue&) = delete;
    TrajectoryQueue(TrajectoryQueue&&) = delete;
    TrajectoryQueue& operator=(TrajectoryQueue&&) = delete;


    /**
     * @brief Attempts to push an item to the back of the queue.
     * This operation is designed to be called by a single producer thread.
     *
     * @param item The item to push.
     * @return true if the item was successfully pushed, false if the queue was full.
     * @note Uses std::memory_order_release for the tail update.
     */
    [[nodiscard]] bool try_push(const T& item) noexcept(std::is_nothrow_copy_assignable_v<T>) {
        const auto current_tail = tail_.load(std::memory_order_relaxed);
        const auto next_tail = increment(current_tail);

        if (next_tail == head_.load(std::memory_order_acquire)) {
            return false; // Queue is full
        }

        buffer_[current_tail] = item; // Copy assignment
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }

    /**
     * @brief Attempts to push an item to the back of the queue (move semantics).
     * This operation is designed to be called by a single producer thread.
     *
     * @param item The item to move into the queue.
     * @return true if the item was successfully pushed, false if the queue was full.
     * @note Uses std::memory_order_release for the tail update.
     */
    [[nodiscard]] bool try_push(T&& item) noexcept(std::is_nothrow_move_assignable_v<T>) {
        const auto current_tail = tail_.load(std::memory_order_relaxed);
        const auto next_tail = increment(current_tail);

        if (next_tail == head_.load(std::memory_order_acquire)) {
            return false; // Queue is full
        }

        buffer_[current_tail] = std::move(item); // Move assignment
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }


    /**
     * @brief Attempts to pop an item from the front of the queue.
     * This operation is designed to be called by a single consumer thread.
     *
     * @param out_item Reference to an object where the popped item will be stored.
     * @return true if an item was successfully popped, false if the queue was empty.
     * @note Uses std::memory_order_release for the head update.
     */
    [[nodiscard]] bool try_pop(T& out_item) noexcept(std::is_nothrow_move_assignable_v<T> && std::is_nothrow_default_constructible_v<T>) { // Assuming T can be moved
        const auto current_head = head_.load(std::memory_order_relaxed);
        if (current_head == tail_.load(std::memory_order_acquire)) {
            return false; // Queue is empty
        }

        out_item = std::move(buffer_[current_head]); // Move construct/assign from buffer
        // Optional: if T is non-trivial, you might want to destruct buffer_[current_head]
        // or overwrite it with a default-constructed T. For simple POD-like T, this is often skipped.
        // buffer_[current_head] = T{}; // Example: reset the slot
        head_.store(increment(current_head), std::memory_order_release);
        return true;
    }

    /**
     * @brief Attempts to peek at the item at the front of the queue without removing it.
     * This operation is designed to be called by the consumer thread.
     *
     * @return std::optional<T> containing the front item if the queue is not empty,
     *         otherwise std::nullopt. The item is copied.
     * @note Uses std::memory_order_acquire for both head and tail.
     */
    [[nodiscard]] std::optional<T> try_peek() const noexcept(std::is_nothrow_copy_constructible_v<T>) {
        const auto current_head = head_.load(std::memory_order_acquire);
        if (current_head == tail_.load(std::memory_order_acquire)) {
            return std::nullopt; // Queue is empty
        }
        // Returns a copy of the item
        return buffer_[current_head];
    }

    /**
     * @brief Clears the queue, making it empty.
     * This operation should ideally be called when producer/consumer are paused
     * or by one of them in a controlled manner to avoid race conditions on clear.
     * For SPSC, if consumer calls clear, it's generally safe for head. Producer might still push.
     * If producer calls clear, it's generally safe for tail. Consumer might still pop.
     * A full reset might need external synchronization or specific clear logic.
     * This simple clear resets head and tail, effectively emptying the queue.
     * It is NOT thread-safe if called concurrently by producer AND consumer.
     * It IS thread-safe if called by EITHER producer or consumer when the other is known to be idle.
     * Or if only one thread (e.g., manager thread) ever calls clear.
     */
    void clear() noexcept {
        // This simple clear is not inherently safe if both producer and consumer
        // are active. For a robust clear in a running SPSC queue, more complex
        // synchronization or specific producer/consumer clear protocols are needed.
        // For typical use (e.g., on reset), this is often sufficient if access is controlled.
        head_.store(0, std::memory_order_release); // Consumer sees empty
        tail_.store(0, std::memory_order_release); // Producer sees empty
    }

    /**
     * @brief Checks if the queue is empty.
     * Can be called by either producer or consumer.
     * @return true if the queue is empty, false otherwise.
     * @note Uses std::memory_order_acquire.
     */
    [[nodiscard]] bool empty() const noexcept {
        return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
    }

    /**
     * @brief Gets the current number of items in the queue.
     * Can be called by either producer or consumer. The returned size is approximate
     * if called concurrently with push/pop operations due to the nature of lock-free queues.
     * For an SPSC queue, if called by the consumer, it reflects items available to pop.
     * If called by the producer, it reflects items pushed but not yet popped (approximately).
     * @return The approximate number of items in the queue.
     * @note Uses std::memory_order_acquire.
     */
    [[nodiscard]] std::size_t size() const noexcept {
        // Order of loads matters for consistency in concurrent scenarios.
        // Load tail first then head for a conservative "size" from consumer's perspective.
        // Load head first then tail for a conservative "available space" from producer's.
        // This implementation is a common one.
        const auto current_tail = tail_.load(std::memory_order_acquire); // Get tail first
        const auto current_head = head_.load(std::memory_order_acquire); // Then get head
        return (current_tail + Capacity - current_head) & (Capacity - 1); // Bitwise modulo
    }

private:
    /**
     * @internal
     * @brief Increments an index, wrapping around the buffer capacity.
     * @param idx The index to increment.
     * @return The incremented and wrapped index.
     */
    [[nodiscard]] constexpr std::size_t increment(std::size_t idx) const noexcept {
        return (idx + 1) & (Capacity - 1); // Bitwise AND for modulo, since Capacity is power of 2
    }

    // Pad to prevent false sharing between atomics and buffer or between head/tail themselves
    // This is an advanced optimization and might not be necessary unless profiling shows contention.
    // char pad0_[64 - (sizeof(std::atomic<std::size_t>) % 64)]; // Example padding

    std::unique_ptr<T[]> buffer_; ///< The underlying circular buffer.

    // char pad1_[64 - (sizeof(std::unique_ptr<T[]>) % 64)];

    std::atomic<std::size_t> head_; ///< Index of the front of the queue (for consumer).

    // char pad2_[64 - (sizeof(std::atomic<std::size_t>) % 64)];

    std::atomic<std::size_t> tail_; ///< Index of the back of the queue (where next item is pushed, for producer).

    // char pad3_[64 - (sizeof(std::atomic<std::size_t>) % 64)];
};

// BEGIN MODIFICATION
} // namespace RDT
// END MODIFICATION

#endif // TRAJECTORYQUEUE_H