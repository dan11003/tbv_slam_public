#pragma once
#include <condition_variable>
#include <mutex>
#include <queue>
#include "boost/serialization/queue.hpp"

// A threadsafe-queue.
template <class T>
class SafeQueue
{
public:
    SafeQueue() : q(), m(), c() {}

    ~SafeQueue() {}

    // Add an element to the queue.
    void enqueue(const T& t)
    {
        std::lock_guard<std::mutex> lock(m);
        q.push(t);
        c.notify_one();
    }

    // Get the front element.
    // If the queue is empty, wait till a element is avaiable.
    T dequeue(void)
    {
        std::unique_lock<std::mutex> lock(m);
        while (q.empty())
        {
            // release lock as long as the wait and reaquire it afterwards.
            c.wait(lock);
        }
        T val = q.front();
        q.pop();
        return val;
    }
    bool empty(void)
    {
        std::unique_lock<std::mutex> lock(m);
        return q.empty();
    }



private:
    std::queue<T> q;
    mutable std::mutex m;
    std::condition_variable c;


};
