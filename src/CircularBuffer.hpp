/**
 * Copyright (C) 2019  Sergey Morozov <sergey@morozov.ch>
 *
 * Permission is hereby granted, free of charge, to any person 
 * obtaining a copy of this software and associated documentation 
 * files (the "Software"), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, 
 * publish, distribute, sublicense, and/or sell copies of the Software, 
 * and to permit persons to whom the Software is furnished to do so, 
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be 
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH 
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef CAMERA_FUSION_CIRCULARBUFFER_HPP
#define CAMERA_FUSION_CIRCULARBUFFER_HPP

#include <iterator>
#include <cstddef>
#include <array>

template <typename T, size_t N>
class CircularBuffer
{

  static_assert( N > 0, "CircularBuffer size must be greater than one." );

public:

  class iterator
  {
  private:
    size_t beg_offset_;
    CircularBuffer& db_;

  public:
    using value_type = T;
    using difference_type = ptrdiff_t;
    using pointer = T*;
    using reference = T&;

    iterator(size_t offset, CircularBuffer& db);

    reference operator*();
    bool operator==(const iterator& it) const;
    bool operator!=(const iterator& it) const;
    iterator& operator+=(difference_type diff);
    iterator& operator-=(difference_type diff);
    iterator& operator++();
    iterator& operator--();
    iterator operator++(int);
    iterator operator--(int);
    iterator operator+(difference_type diff);
    iterator operator-(difference_type diff);
    difference_type operator-(const iterator& it);
    pointer operator->();

  };

  auto begin();

  auto end();

  constexpr static size_t max_size();

  size_t size() const;

  explicit CircularBuffer();

  void push_back(const T& val);

private:

  inline size_t index(size_t offset);

  std::array<T, N> data_buffer_;
  size_t beg_ind_;
  size_t cur_size_;
};

template<typename T, size_t N>
CircularBuffer<T, N>::iterator::iterator(const size_t offset, CircularBuffer& db) : beg_offset_{offset}, db_{db}
{

}

template<typename T, size_t N>
auto CircularBuffer<T, N>::iterator::operator*() -> reference
{
  return db_.data_buffer_[db_.index(beg_offset_)];
}

template<typename T, size_t N>
bool CircularBuffer<T, N>::iterator::operator==(const iterator& it) const
{
  return &(this->db_) == &(it.db_) and
         this->beg_offset_ == it.beg_offset_;
}

template<typename T, size_t N>
bool CircularBuffer<T, N>::iterator::operator!=(const iterator& it) const
{
  return not (*this == it);
}

template<typename T, size_t N>
auto CircularBuffer<T, N>::iterator::operator+=(const difference_type diff) -> iterator&
{
  beg_offset_ += diff;
  return *this;
}

template<typename T, size_t N>
auto  CircularBuffer<T, N>::iterator::operator-=(const difference_type diff) -> iterator&
{
  beg_offset_ -= diff;
  return *this;
}

template<typename T, size_t N>
auto CircularBuffer<T, N>::iterator::operator++() -> iterator&
{
  *this += 1;
  return *this;
}

template<typename T, size_t N>
auto CircularBuffer<T, N>::iterator::operator--() -> iterator&
{
  *this -= 1;
  return *this;
}

template<typename T, size_t N>
auto CircularBuffer<T, N>::iterator::operator++(int) -> iterator
{
  iterator tmp{*this};
  ++(*this);
  return tmp;
}

template<typename T, size_t N>
auto CircularBuffer<T, N>::iterator::operator--(int) -> iterator
{
  iterator tmp{*this};
  --(*this);
  return tmp;
}

template<typename T, size_t N>
auto CircularBuffer<T, N>::iterator::operator+(const difference_type diff) -> iterator
{
  iterator tmp{*this};
  tmp += diff;
  return tmp;
}

template<typename T, size_t N>
auto CircularBuffer<T, N>::iterator::operator-(const difference_type diff) -> iterator
{
  iterator tmp{*this};
  tmp -= diff;
  return tmp;
}

template<typename T, size_t N>
auto CircularBuffer<T, N>::iterator::operator-(const CircularBuffer::iterator& it) -> difference_type
{
  return this->beg_offset_ - it.beg_offset_;
}

template<typename T, size_t N>
auto CircularBuffer<T, N>::iterator::operator->() -> pointer
{
  return &(**this);
}

template<typename T, size_t N>
CircularBuffer<T, N>::CircularBuffer() : data_buffer_{}, beg_ind_{0}, cur_size_{0}
{

}

template<typename T, size_t N>
void CircularBuffer<T, N>::push_back(const T& val)
{
  *(end()) = val;
  if (cur_size_ < data_buffer_.size())
  {
    ++cur_size_;
  }
  else
  {
    beg_ind_ = index(1);
  }
}

template<typename T, size_t N>
size_t CircularBuffer<T, N>::index(size_t offset)
{
  return (beg_ind_ + offset) % data_buffer_.size();
}

template<typename T, size_t N>
size_t CircularBuffer<T, N>::size() const
{
  return cur_size_;
}

template<typename T, size_t N>
constexpr size_t CircularBuffer<T, N>::max_size()
{
  return N;
}

template<typename T, size_t N>
auto CircularBuffer<T, N>::end()
{
  return iterator(cur_size_, *this);
}

template<typename T, size_t N>
auto CircularBuffer<T, N>::begin()
{
  return iterator(0, *this);
}

#endif //CAMERA_FUSION_CIRCULARBUFFER_HPP
