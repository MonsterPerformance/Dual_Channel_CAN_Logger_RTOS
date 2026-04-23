#ifndef QUEUE_HPP
#define QUEUE_HPP

#include <cstdint>
#include <stdio.h>

template <typename T, const uint8_t size>
class CQueue
{
    public:
        CQueue() : m_data{}, m_head{0}, m_tail{0}, m_isFull{false}
        {
        }
       ~CQueue() = default;
        CQueue(const CQueue&) = delete;
        CQueue& operator=(const CQueue&) = delete;
        CQueue(CQueue&&) = delete;
        CQueue& operator=(CQueue&&) = delete;

        void push(const T& inValue)
        {
            if (m_isFull)
            {
                moveHead();
            }
            m_data[m_tail] = inValue;
            moveTail();
            m_isFull = (m_head == m_tail);
        }
        bool pop(T& outValue)
        {
            bool result{false};
            if (!empty())
            {
                result = true;
                outValue = m_data[m_head];
                moveHead();
                m_isFull = false;
            }
            return result;
        }
        bool empty() const
        {
            return ((m_head == m_tail) && !m_isFull);
        }
        bool full() const
        {
            return m_isFull;
        }
        void print()
        {
            if (m_head < m_tail)
            {
                for (uint8_t i = m_head; i < m_tail; ++i)
                {
                    m_data[i].print();
                    log(", ");
                }
                log("\r\n");
            }
            else if ((m_tail < m_head) || m_isFull)
            {
                for (uint8_t i = m_head; i < size; ++i)
                {
                    m_data[i].print();
                    log(", ");
                }
                for (uint8_t i = 0; i < m_tail; ++i)
                {
                    m_data[i].print();
                    log(", ");
                }
                log("\r\n");
            }
            else
            {
                log("Queue is empty\r\n");
            }
            log("head = %d, tail = %d, full = %d\r\n", m_head, m_tail, full());
        }

    private:
        void moveHead()
        {
            if (++m_head == size)
            {
                m_head = 0;
            }
        }
        void moveTail()
        {
            if (++m_tail == size)
            {
                m_tail = 0;
            }
        }
        T m_data[size];
        uint8_t m_head;
        uint8_t m_tail;
        bool m_isFull;
};

#endif
