// File: tests/test_spsc_queue.cpp
// Unit tests for SPSCQueue
#include "adas/queues/SPSCQueue.hpp"

#include <chrono>
#include <iostream>
#include <thread>

using namespace adas;

int main() {
    int passed = 0;
    int failed = 0;

    // Test 1: Basic push/pop
    {
        SPSCQueue<int, 4> q;
        q.try_push(1);
        q.try_push(2);
        q.try_push(3);

        int val;
        bool ok = true;
        ok &= q.try_pop(val) && val == 1;
        ok &= q.try_pop(val) && val == 2;
        ok &= q.try_pop(val) && val == 3;
        ok &= !q.try_pop(val); // Should be empty

        if (ok) {
            std::cout << "[PASS] Test 1: Basic push/pop\n";
            ++passed;
        } else {
            std::cout << "[FAIL] Test 1: Basic push/pop\n";
            ++failed;
        }
    }

    // Test 2: Queue full (drops)
    {
        SPSCQueue<int, 2> q;
        bool ok = true;
        ok &= q.try_push(1);
        ok &= q.try_push(2);
        ok &= !q.try_push(3); // Should fail, queue full
        ok &= q.drops() == 1;

        if (ok) {
            std::cout << "[PASS] Test 2: Queue full (drops)\n";
            ++passed;
        } else {
            std::cout << "[FAIL] Test 2: Queue full (drops)\n";
            ++failed;
        }
    }

    // Test 3: Size tracking
    {
        SPSCQueue<int, 8> q;
        bool ok = q.size() == 0 && q.empty();
        q.try_push(1);
        q.try_push(2);
        ok &= q.size() == 2 && !q.empty();

        int val;
        q.try_pop(val);
        ok &= q.size() == 1;

        if (ok) {
            std::cout << "[PASS] Test 3: Size tracking\n";
            ++passed;
        } else {
            std::cout << "[FAIL] Test 3: Size tracking\n";
            ++failed;
        }
    }

    // Test 4: Concurrent producer/consumer
    {
        SPSCQueue<int, 64> q;
        const int N = 10000;
        std::atomic<int> sum_produced{0};
        std::atomic<int> sum_consumed{0};

        std::thread producer([&]() {
            for (int i = 0; i < N; ++i) {
                while (!q.try_push(i)) {
                    std::this_thread::yield();
                }
                sum_produced += i;
            }
        });

        std::thread consumer([&]() {
            int count = 0;
            while (count < N) {
                int val;
                if (q.try_pop(val)) {
                    sum_consumed += val;
                    ++count;
                } else {
                    std::this_thread::yield();
                }
            }
        });

        producer.join();
        consumer.join();

        if (sum_produced == sum_consumed && q.drops() == 0) {
            std::cout << "[PASS] Test 4: Concurrent producer/consumer\n";
            ++passed;
        } else {
            std::cout << "[FAIL] Test 4: Concurrent producer/consumer\n";
            ++failed;
        }
    }

    // Test 5: Move semantics
    {
        SPSCQueue<std::string, 4> q;
        std::string s = "hello";
        q.try_push(std::move(s));

        std::string out;
        q.try_pop(out);

        if (out == "hello") {
            std::cout << "[PASS] Test 5: Move semantics\n";
            ++passed;
        } else {
            std::cout << "[FAIL] Test 5: Move semantics\n";
            ++failed;
        }
    }

    // Summary
    std::cout << "\n═══════════════════════════════════════\n";
    std::cout << "SPSC Queue Tests: " << passed << " passed, " << failed << " failed\n";
    std::cout << "═══════════════════════════════════════\n";

    return failed > 0 ? 1 : 0;
}
