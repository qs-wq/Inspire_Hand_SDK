#ifndef RING_BUFFER_HPP
#define RING_BUFFER_HPP

#include <vector>
#include <cstdint>
#include <stdexcept>

class RingBuffer {
public:
    explicit RingBuffer(size_t size);
    
    //清除缓冲区
    void clear();
        
    // 入队，将数据添加到环形缓冲区
    void push(const uint8_t* data, size_t len);
    
    // 出队，从缓冲区读取数据
    size_t pop(uint8_t* data, size_t maxlen);
    
    // 获取当前缓冲区中的有效数据长度
    size_t size() const;

    // 获取从tail开始的连续数据长度（未破坏环形结构）
    size_t contiguousDataSize() const;

    // 获取缓冲区的底层数据指针（指向环形缓冲区当前tail位置）
    const uint8_t* data() const;

    // 获取从tail开始的连续区段的指针（指向连续数据区）
    const uint8_t* dataPtr() const;

    // 裁剪已解析的数据，移动tail指针
    void advance(size_t count);

    // 新增：获取buffer底层数据（常量引用）
    const std::vector<uint8_t>& getBuffer() const { return buffer; }
    
    // 新增：获取tail索引
    size_t getTail() const { return tail; }

private:
    std::vector<uint8_t> buffer;
    size_t head;
    size_t tail;
    bool full;
};

#endif // RING_BUFFER_HPP

