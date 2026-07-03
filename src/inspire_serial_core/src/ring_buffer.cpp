#include "ring_buffer.hpp"

RingBuffer::RingBuffer(size_t size) : buffer(size), head(0), tail(0), full(false) {}

void RingBuffer::push(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        if (full) {
            // 自动覆盖旧数据：移动tail到下一个位置
            tail = (tail + 1) % buffer.size();
        }
        buffer[head] = data[i];
        head = (head + 1) % buffer.size();
        full = (head == tail);
    }
}

size_t RingBuffer::pop(uint8_t* data, size_t maxlen) {
    size_t count = 0;
    while (count < maxlen && (tail != head || full)) {
        data[count] = buffer[tail];
        tail = (tail + 1) % buffer.size();
        full = false;
        ++count;
    }
    return count;
}

size_t RingBuffer::size() const {
    if (full)
        return buffer.size();
    if (head >= tail)
        return head - tail;
    else
        return buffer.size() - (tail - head);
}

size_t RingBuffer::contiguousDataSize() const {
    if (full || head >= tail)
        return head >= tail ? head - tail : buffer.size() - tail;
    else
        return buffer.size() - tail;
}

const uint8_t* RingBuffer::data() const { return buffer.data(); }

const uint8_t* RingBuffer::dataPtr() const { return &buffer[tail]; }

void RingBuffer::advance(size_t count) {
    if (count > size())
        throw std::runtime_error("超出缓冲区数据");
    tail = (tail + count) % buffer.size();
    full = false;
}

void RingBuffer::clear() {
    head = 0;
    tail = 0;
    full = false;
}
