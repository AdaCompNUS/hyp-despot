/*
 * GPUsplit_buffer.h
 *
 *  Created on: 28 Mar, 2017
 *      Author: panpan
 */

#ifndef GPUSPLIT_BUFFER_H_
#define GPUSPLIT_BUFFER_H_
/*	\file   split_buffer.h
	\author Gregory Diamos <solusstultus@gmail.com>
	\date   November 14, 2012
	\brief  The header file for the split_buffer class
*/

#pragma once

// Archaeopteryx Includes
#include <despot/GPUutil/GPUfunctional.h>
#include <despot/GPUutil/GPUallocator_traits.h>

namespace archaeopteryx
{

namespace util
{

template <bool>
class __split_buffer_common
{
protected:
    __device__ void __throw_length_error() const;
    __device__ void __throw_out_of_range() const;
};

template <class _Tp, class _Allocator = allocator<_Tp> >
struct __split_buffer
    : private __split_buffer_common<true>
{
private:
    __device__ __split_buffer(const __split_buffer&);
    __device__ __split_buffer& operator=(const __split_buffer&);
public:
    typedef _Tp                                             value_type;
    typedef _Allocator                                      allocator_type;
    typedef typename remove_reference<allocator_type>::type __alloc_rr;
    typedef allocator_traits<__alloc_rr>                    __alloc_traits;
    typedef value_type&                                     reference;
    typedef const value_type&                               const_reference;
    typedef typename __alloc_traits::size_type              size_type;
    typedef typename __alloc_traits::difference_type        difference_type;
    typedef typename __alloc_traits::pointer                pointer;
    typedef typename __alloc_traits::const_pointer          const_pointer;
    typedef pointer                                         iterator;
    typedef const_pointer                                   const_iterator;

    pointer                                         __first_;
    pointer                                         __begin_;
    pointer                                         __end_;
    pair<pointer, allocator_type> __end_cap_;

    typedef typename add_lvalue_reference<allocator_type>::type __alloc_ref;
    typedef typename add_lvalue_reference<allocator_type>::type __alloc_const_ref;

    __device__ __alloc_rr&           __alloc()         {return __end_cap_.second;}
    __device__ const __alloc_rr&     __alloc() const   {return __end_cap_.second;}
    __device__ pointer&              __end_cap()       {return __end_cap_.first;}
    __device__ const pointer&        __end_cap() const {return __end_cap_.first;}

    __device__ __split_buffer();
    __device__ explicit __split_buffer(__alloc_rr& __a);
    __device__ explicit __split_buffer(const __alloc_rr& __a);
    __device__ __split_buffer(size_type __cap, size_type __start, __alloc_rr& __a);
    __device__ ~__split_buffer();

    __device__ iterator begin()       {return __begin_;}
    __device__ const_iterator begin() const {return __begin_;}
    __device__ iterator end()         {return __end_;}
    __device__ const_iterator end() const   {return __end_;}

    __device__ void clear()
        {__destruct_at_end(__begin_);}
    __device__ size_type size() const {return static_cast<size_type>(__end_ - __begin_);}
    __device__ bool empty()     const {return __end_ == __begin_;}
    __device__ size_type capacity() const {return static_cast<size_type>(__end_cap() - __first_);}
    __device__ size_type __front_spare() const {return static_cast<size_type>(__begin_ - __first_);}
    __device__ size_type __back_spare() const {return static_cast<size_type>(__end_cap() - __end_);}

    __device__ reference front()       {return *__begin_;}
    __device__ const_reference front() const {return *__begin_;}
    __device__ reference back()        {return *(__end_ - 1);}
    __device__ const_reference back() const  {return *(__end_ - 1);}

    __device__ void reserve(size_type __n);
    __device__ void shrink_to_fit();
    __device__ void push_front(const_reference __x);
    __device__ void push_back(const_reference __x);

    __device__ void pop_front() {__destruct_at_begin(__begin_+1);}
    __device__ void pop_back() {__destruct_at_end(__end_-1);}

    __device__ void __construct_at_end(size_type __n);
    __device__ void __construct_at_end(size_type __n, const_reference __x);
    template <class _InputIter>
        __device__ typename enable_if
        <
            __is_input_iterator<_InputIter>::value &&
           !__is_forward_iterator<_InputIter>::value,
            void
        >::type
        __construct_at_end(_InputIter __first, _InputIter __last);
    template <class _ForwardIterator>
        __device__ typename enable_if
        <
            __is_forward_iterator<_ForwardIterator>::value,
            void
        >::type
        __construct_at_end(_ForwardIterator __first, _ForwardIterator __last);

    __device__ void __destruct_at_begin(pointer __new_begin)
        {__destruct_at_begin(__new_begin, is_trivially_destructible<value_type>());}
    __device__ void __destruct_at_begin(pointer __new_begin, false_type);
    __device__ void __destruct_at_begin(pointer __new_begin, true_type);

    __device__ void __destruct_at_end(pointer __new_last)
        {__destruct_at_end(__new_last, false_type());}
    __device__ void __destruct_at_end(pointer __new_last, false_type);
    __device__ void __destruct_at_end(pointer __new_last, true_type);

    __device__ void swap(__split_buffer& __x);

    __device__ bool __invariants() const;

private:
    __device__ void __move_assign_alloc(__split_buffer& __c, true_type)
        {
            __alloc() = util::move(__c.__alloc());
        }

    __device__ void __move_assign_alloc(__split_buffer&, false_type)
        {}

    __device__ static void __swap_alloc(__alloc_rr& __x, __alloc_rr& __y)
        {__swap_alloc(__x, __y, integral_constant<bool,
                      __alloc_traits::propagate_on_container_swap::value>());}

    __device__ static void __swap_alloc(__alloc_rr& __x, __alloc_rr& __y, true_type)
        {
            using util::swap;
            swap(__x, __y);
        }

    __device__ static void __swap_alloc(__alloc_rr&, __alloc_rr&, false_type)
        {}
};

template <class _Tp, class _Allocator>
__device__ bool
__split_buffer<_Tp, _Allocator>::__invariants() const
{
    if (__first_ == NULL)
    {
        if (__begin_ != NULL)
            return false;
        if (__end_ != NULL)
            return false;
        if (__end_cap() != NULL)
            return false;
    }
    else
    {
        if (__begin_ < __first_)
            return false;
        if (__end_ < __begin_)
            return false;
        if (__end_cap() < __end_)
            return false;
    }
    return true;
}

//  Default constructs __n objects starting at __end_
//  throws if construction throws
//  Precondition:  __n > 0
//  Precondition:  size() + __n <= capacity()
//  Postcondition:  size() == size() + __n
template <class _Tp, class _Allocator>
__device__ void
__split_buffer<_Tp, _Allocator>::__construct_at_end(size_type __n)
{
    __alloc_rr& __a = this->__alloc();
    do
    {
        __alloc_traits::construct(__a, util::__to_raw_pointer(this->__end_));
        ++this->__end_;
        --__n;
    } while (__n > 0);
}

//  Copy constructs __n objects starting at __end_ from __x
//  throws if construction throws
//  Precondition:  __n > 0
//  Precondition:  size() + __n <= capacity()
//  Postcondition:  size() == old size() + __n
//  Postcondition:  [i] == __x for all i in [size() - __n, __n)
template <class _Tp, class _Allocator>
__device__ void
__split_buffer<_Tp, _Allocator>::__construct_at_end(size_type __n, const_reference __x)
{
    __alloc_rr& __a = this->__alloc();
    do
    {
        __alloc_traits::construct(__a, util::__to_raw_pointer(this->__end_), __x);
        ++this->__end_;
        --__n;
    } while (__n > 0);
}

template <class _Tp, class _Allocator>
template <class _InputIter>
__device__ typename enable_if
<
     __is_input_iterator<_InputIter>::value &&
    !__is_forward_iterator<_InputIter>::value,
    void
>::type
__split_buffer<_Tp, _Allocator>::__construct_at_end(_InputIter __first, _InputIter __last)
{
    __alloc_rr& __a = this->__alloc();
    for (; __first != __last; ++__first)
    {
        if (__end_ == __end_cap())
        {
            size_type __old_cap = __end_cap() - __first_;
            size_type __new_cap = util::max<size_type>(2 * __old_cap, 8);
            __split_buffer __buf(__new_cap, 0, __a);
            for (pointer __p = __begin_; __p != __end_; ++__p, ++__buf.__end_)
                __alloc_traits::construct(__buf.__alloc(),
                        util::__to_raw_pointer(__buf.__end_), util::move(*__p));
            swap(__buf);
        }
        __alloc_traits::construct(__a, util::__to_raw_pointer(this->__end_), *__first);
        ++this->__end_;
    }
}

template <class _Tp, class _Allocator>
template <class _ForwardIterator>
__device__ typename enable_if
<
    __is_forward_iterator<_ForwardIterator>::value,
    void
>::type
__split_buffer<_Tp, _Allocator>::__construct_at_end(_ForwardIterator __first, _ForwardIterator __last)
{
    __alloc_rr& __a = this->__alloc();
    for (; __first != __last; ++__first)
    {
        __alloc_traits::construct(__a, util::__to_raw_pointer(this->__end_), *__first);
        ++this->__end_;
    }
}

template <class _Tp, class _Allocator> inline
__device__ void
__split_buffer<_Tp, _Allocator>::__destruct_at_begin(pointer __new_begin, false_type)
{
    while (__begin_ != __new_begin)
        __alloc_traits::destroy(__alloc(), __begin_++);
}

template <class _Tp, class _Allocator> inline
__device__ void
__split_buffer<_Tp, _Allocator>::__destruct_at_begin(pointer __new_begin, true_type)
{
    __begin_ = __new_begin;
}

template <class _Tp, class _Allocator> inline
__device__ void
__split_buffer<_Tp, _Allocator>::__destruct_at_end(pointer __new_last, false_type)
{
    while (__new_last != __end_)
        __alloc_traits::destroy(__alloc(), --__end_);
}

template <class _Tp, class _Allocator> inline
__device__ void
__split_buffer<_Tp, _Allocator>::__destruct_at_end(pointer __new_last, true_type)
{
    __end_ = __new_last;
}

template <class _Tp, class _Allocator>
__device__ __split_buffer<_Tp, _Allocator>::__split_buffer(size_type __cap, size_type __start, __alloc_rr& __a)
    : __end_cap_(0, __a)
{
    __first_ = __cap != 0 ? __alloc_traits::allocate(__alloc(), __cap) : 0;
    __begin_ = __end_ = __first_ + __start;
    __end_cap() = __first_ + __cap;
}

template <class _Tp, class _Allocator> inline
__device__ __split_buffer<_Tp, _Allocator>::__split_buffer()
    : __first_(0), __begin_(0), __end_(0), __end_cap_(0)
{
}

template <class _Tp, class _Allocator> inline
__device__ __split_buffer<_Tp, _Allocator>::__split_buffer(__alloc_rr& __a)
    : __first_(0), __begin_(0), __end_(0), __end_cap_(0, __a)
{
}

template <class _Tp, class _Allocator> inline
__device__ __split_buffer<_Tp, _Allocator>::__split_buffer(const __alloc_rr& __a)
    : __first_(0), __begin_(0), __end_(0), __end_cap_(0, __a)
{
}

template <class _Tp, class _Allocator>
__device__ __split_buffer<_Tp, _Allocator>::~__split_buffer()
{
    clear();
    if (__first_)
        __alloc_traits::deallocate(__alloc(), __first_, capacity());
}

template <class _Tp, class _Allocator>
__device__ void
__split_buffer<_Tp, _Allocator>::swap(__split_buffer& __x)
{
    util::swap(__first_, __x.__first_);
    util::swap(__begin_, __x.__begin_);
    util::swap(__end_, __x.__end_);
    util::swap(__end_cap(), __x.__end_cap());
    __swap_alloc(__alloc(), __x.__alloc());
}

template <class _Tp, class _Allocator>
__device__ void
__split_buffer<_Tp, _Allocator>::reserve(size_type __n)
{
    if (__n < capacity())
    {
        __split_buffer<value_type, __alloc_rr&> __t(__n, 0, __alloc());
        __t.__construct_at_end(move_iterator<pointer>(__begin_),
                               move_iterator<pointer>(__end_));
        util::swap(__first_, __t.__first_);
        util::swap(__begin_, __t.__begin_);
        util::swap(__end_, __t.__end_);
        util::swap(__end_cap(), __t.__end_cap());
    }
}

template <class _Tp, class _Allocator>
__device__ void
__split_buffer<_Tp, _Allocator>::shrink_to_fit()
{
    if (capacity() > size())
    {
        __split_buffer<value_type, __alloc_rr&> __t(size(), 0, __alloc());
        __t.__construct_at_end(move_iterator<pointer>(__begin_),
                               move_iterator<pointer>(__end_));
        __t.__end_ = __t.__begin_ + (__end_ - __begin_);
        util::swap(__first_, __t.__first_);
        util::swap(__begin_, __t.__begin_);
        util::swap(__end_, __t.__end_);
        util::swap(__end_cap(), __t.__end_cap());
    }
}

template <class _Tp, class _Allocator>
__device__ void
__split_buffer<_Tp, _Allocator>::push_front(const_reference __x)
{
    if (__begin_ == __first_)
    {
        if (__end_ < __end_cap())
        {
            difference_type __d = __end_cap() - __end_;
            __d = (__d + 1) / 2;
            __begin_ = util::move_backward(__begin_, __end_, __end_ + __d);
            __end_ += __d;
        }
        else
        {
            size_type __c = max<size_type>(2 * static_cast<size_t>(__end_cap() - __first_), 1);
            __split_buffer<value_type, __alloc_rr&> __t(__c, (__c + 3) / 4, __alloc());
            __t.__construct_at_end(move_iterator<pointer>(__begin_),
                                   move_iterator<pointer>(__end_));
            util::swap(__first_, __t.__first_);
            util::swap(__begin_, __t.__begin_);
            util::swap(__end_, __t.__end_);
            util::swap(__end_cap(), __t.__end_cap());
        }
    }
    __alloc_traits::construct(__alloc(), util::__to_raw_pointer(__begin_-1), __x);
    --__begin_;
}

template <class _Tp, class _Allocator> inline
__device__ void
__split_buffer<_Tp, _Allocator>::push_back(const_reference __x)
{
    if (__end_ == __end_cap())
    {
        if (__begin_ > __first_)
        {
            difference_type __d = __begin_ - __first_;
            __d = (__d + 1) / 2;
            __end_ = util::move(__begin_, __end_, __begin_ - __d);
            __begin_ -= __d;
        }
        else
        {
            size_type __c = max<size_type>(2 * static_cast<size_t>(__end_cap() - __first_), 1);
            __split_buffer<value_type, __alloc_rr&> __t(__c, __c / 4, __alloc());
            __t.__construct_at_end(move_iterator<pointer>(__begin_),
                                   move_iterator<pointer>(__end_));
            util::swap(__first_, __t.__first_);
            util::swap(__begin_, __t.__begin_);
            util::swap(__end_, __t.__end_);
            util::swap(__end_cap(), __t.__end_cap());
        }
    }
    __alloc_traits::construct(__alloc(), util::__to_raw_pointer(__end_), __x);
    ++__end_;
}

template <class _Tp, class _Allocator> inline
__device__ void
swap(__split_buffer<_Tp, _Allocator>& __x, __split_buffer<_Tp, _Allocator>& __y)
{
    __x.swap(__y);
}

}

}


#endif /* GPUSPLIT_BUFFER_H_ */
