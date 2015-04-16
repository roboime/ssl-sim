/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef UTILS_STACK_VECTOR_HH_
#define UTILS_STACK_VECTOR_HH_

#include <cstdlib>
#include <utility>
#include <initializer_list>

namespace utils {
using std::declval;
using std::ptrdiff_t;
using std::initializer_list;
using std::forward;

//template <class T, size_t N> class stack_vector {
template <class T, size_t N> class stack_vector {
  size_t count = {0};
  T elems[N] = {};

public:
  // types:
  typedef T &reference;
  typedef const T &const_reference;
  typedef T *iterator;
  typedef const T *const_iterator;
  typedef size_t size_type;
  typedef ptrdiff_t difference_type;
  typedef T value_type;
  typedef T *pointer;
  typedef const T *const_pointer;
#if 0
  typedef reverse_iterator<iterator> reverse_iterator;
  typedef reverse_iterator<const_iterator> const_reverse_iterator;
#endif

  // no explicit construct/copy/destroy for aggregate type

  void fill(const T &u); // TODO
  void swap(stack_vector<T, N> &) noexcept(
      noexcept(swap(declval<T &>(), declval<T &>()))); // TODO

  // iterators:
  iterator begin() noexcept { return elems; }
  const_iterator begin() const noexcept { return elems; }
  iterator end() noexcept { return elems + count; }
  const_iterator end() const noexcept { return elems + count; }

#if 0
  reverse_iterator rbegin() noexcept;
  const_reverse_iterator rbegin() const noexcept;
  reverse_iterator rend() noexcept;
  const_reverse_iterator rend() const noexcept;
#endif

  const_iterator cbegin() noexcept { return elems; }
  const_iterator cend() noexcept { return elems + count; }
#if 0
  const_reverse_iterator crbegin() const noexcept;
  const_reverse_iterator crend() const noexcept;
#endif

  // capacity:
  constexpr size_type size() const noexcept { return count; }
  constexpr size_type max_size() const noexcept { return N; }
  constexpr bool empty() const noexcept { return count == 0; }

  // element access:
  reference operator[](size_type n) { return elems[n]; }
  const_reference operator[](size_type n) const { return elems[n]; }
  const_reference at(size_type n) const { return elems[n]; }
  reference at(size_type n) { return elems[n]; }
  reference front() { return elems[0]; }
  const_reference front() const { return elems[0]; }
  reference back() { return elems[count - 1]; }
  const_reference back() const { return elems[count - 1]; }

  // data-access
  T *data() noexcept { return &elems; }
  const T *data() const noexcept { return &elems; }

  // modifiers:
  template <class... Args> void emplace_back(Args &&... args) {
    elems[count].~T();
    new (elems + count) T(forward<Args>(args)...);
    count++;
  }
  void push_back(const T &x) { elems[count++] = x; }
  void push_back(T &&x);
  void pop_back();

  template <class... Args>
  iterator emplace(const_iterator position, Args &&... args);
  iterator insert(const_iterator position, const T &x);
  iterator insert(const_iterator position, T &&x);
  iterator insert(const_iterator position, size_type n, const T &x);
  template <class InputIterator>
  iterator insert(const_iterator position, InputIterator first,
                  InputIterator last);
  iterator insert(const_iterator position, initializer_list<T>);

  iterator erase(const_iterator position);
  iterator erase(const_iterator first, const_iterator last);
  void clear() noexcept { count = 0; }
};

template <class T, size_t N>
bool operator==(const stack_vector<T, N> &x, const stack_vector<T, N> &y);
template <class T, size_t N>
bool operator!=(const stack_vector<T, N> &x, const stack_vector<T, N> &y);
template <class T, size_t N>
bool operator<(const stack_vector<T, N> &x, const stack_vector<T, N> &y);
template <class T, size_t N>
bool operator>(const stack_vector<T, N> &x, const stack_vector<T, N> &y);
template <class T, size_t N>
bool operator<=(const stack_vector<T, N> &x, const stack_vector<T, N> &y);
template <class T, size_t N>
bool operator>=(const stack_vector<T, N> &x, const stack_vector<T, N> &y);

template <class T, size_t N>
void swap(stack_vector<T, N> &x,
          stack_vector<T, N> &y) noexcept(noexcept(x.swap(y)));
// void swap(stack_vector<T, N>& x, stack_vector<T, N>& y);

#if 0
template <class T> class tuple_size;
template <size_t I, class T> class tuple_element;
template <class T, size_t N> struct tuple_size<stack_vector<T, N>>;
template <size_t I, class T, size_t N>
struct tuple_element<I, stack_vector<T, N>>;
template <size_t I, class T, size_t N> T &get(stack_vector<T, N> &) noexcept;
template <size_t I, class T, size_t N> T &&get(stack_vector<T, N> &&) noexcept;
template <size_t I, class T, size_t N>
const T &get(const stack_vector<T, N> &) noexcept;
#endif
}

#endif
