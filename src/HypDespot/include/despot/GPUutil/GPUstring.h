/*
 * GPUstring.h
 *
 *  Created on: 28 Mar, 2017
 *      Author: panpan
 */

#ifndef GPUSTRING_H_
#define GPUSTRING_H_
/*! \file   string.h
	\date   Thursday November 15, 2012
	\author Gregory Diamos <gregory.diamos@gatech.edu>
	\brief  The header file for string class.
*/

// Archaeopteryx Includes
#include <despot/GPUutil/GPUallocator_traits.h>
#include <despot/GPUutil/GPUcstring.h>
#include <despot/GPUutil/GPUalgorithm.h>
#include <despot/GPUutil/GPUdebug.h>

#pragma once

#include <iosfwd>
#include <cstring>
#include <cstdio>  // For EOF.
#include <cwchar>

namespace archaeopteryx
{

namespace util
{


template<class _CharT>  struct char_traits;
template<class _Tp>     class allocator;

template <class _CharT,             // for <stdexcept>
          class _Traits = char_traits<_CharT>,
          class _Allocator = allocator<_CharT> >
    class basic_string;
typedef basic_string<char, char_traits<char>, allocator<char> > string;
typedef basic_string<wchar_t, char_traits<wchar_t>, allocator<wchar_t> > wstring;

typedef std::streamoff streamoff;
typedef std::streampos streampos;

// fpos

template <class _StateT>
class fpos
{
private:
    _StateT __st_;
    streamoff __off_;
public:
    __device__ fpos(streamoff __off = streamoff()) : __st_(), __off_(__off) {}

    __device__ operator streamoff() const {return __off_;}

    __device__ _StateT state() const {return __st_;}
    __device__ void state(_StateT __st) {__st_ = __st;}

    __device__ fpos& operator+=(streamoff __off) {__off_ += __off; return *this;}
    __device__ fpos  operator+ (streamoff __off) const {fpos __t(*this); __t += __off; return __t;}
    __device__ fpos& operator-=(streamoff __off) {__off_ -= __off; return *this;}
    __device__ fpos  operator- (streamoff __off) const {fpos __t(*this); __t -= __off; return __t;}
};

template <class _StateT>
__device__ inline
streamoff operator-(const fpos<_StateT>& __x, const fpos<_StateT>& __y)
    {return streamoff(__x) - streamoff(__y);}

template <class _StateT>
__device__ inline
bool operator==(const fpos<_StateT>& __x, const fpos<_StateT>& __y)
    {return streamoff(__x) == streamoff(__y);}

template <class _StateT>
__device__ inline
bool operator!=(const fpos<_StateT>& __x, const fpos<_StateT>& __y)
    {return streamoff(__x) != streamoff(__y);}

// char_traits

template <class _CharT>
struct char_traits
{
    typedef _CharT    char_type;
    typedef int       int_type;
    typedef streamoff off_type;
    typedef streampos pos_type;
    typedef mbstate_t state_type;


    __device__ static void assign(char_type& __c1, const char_type& __c2)
        {__c1 = __c2;}

    __device__ static bool eq(char_type __c1, char_type __c2)
        {return __c1 == __c2;}

    __device__ static bool lt(char_type __c1, char_type __c2)
        {return __c1 < __c2;}

    __device__ static int              compare(const char_type* __s1, const char_type* __s2, size_t __n);
    __device__ static size_t           length(const char_type* __s);
    __device__ static const char_type* find(const char_type* __s, size_t __n, const char_type& __a);
    __device__ static char_type*       move(char_type* __s1, const char_type* __s2, size_t __n);
    __device__ static char_type*       copy(char_type* __s1, const char_type* __s2, size_t __n);
    __device__ static char_type*       assign(char_type* __s, size_t __n, char_type __a);


    __device__ static int_type  not_eof(int_type __c)
        {return eq_int_type(__c, eof()) ? ~eof() : __c;}

    __device__ static char_type to_char_type(int_type __c)
        {return char_type(__c);}

    __device__ static int_type  to_int_type(char_type __c)
        {return int_type(__c);}

    __device__ static bool      eq_int_type(int_type __c1, int_type __c2)
        {return __c1 == __c2;}

    __device__ static int_type  eof()
        {return int_type(EOF);}
};

template <class _CharT>
__device__ int
char_traits<_CharT>::compare(const char_type* __s1, const char_type* __s2, size_t __n)
{
    for (; __n; --__n, ++__s1, ++__s2)
    {
        if (lt(*__s1, *__s2))
            return -1;
        if (lt(*__s2, *__s1))
            return 1;
    }
    return 0;
}

template <class _CharT>
__device__ inline
size_t
char_traits<_CharT>::length(const char_type* __s)
{
    size_t __len = 0;
    for (; !eq(*__s, char_type(0)); ++__s)
        ++__len;
    return __len;
}

template <class _CharT>
__device__ inline
const _CharT*
char_traits<_CharT>::find(const char_type* __s, size_t __n, const char_type& __a)
{
    for (; __n; --__n)
    {
        if (eq(*__s, __a))
            return __s;
        ++__s;
    }
    return 0;
}

template <class _CharT>
__device__ _CharT*
char_traits<_CharT>::move(char_type* __s1, const char_type* __s2, size_t __n)
{
    char_type* __r = __s1;
    if (__s1 < __s2)
    {
        for (; __n; --__n, ++__s1, ++__s2)
            assign(*__s1, *__s2);
    }
    else if (__s2 < __s1)
    {
        __s1 += __n;
        __s2 += __n;
        for (; __n; --__n)
            assign(*--__s1, *--__s2);
    }
    return __r;
}

template <class _CharT>
__device__ inline
_CharT*
char_traits<_CharT>::copy(char_type* __s1, const char_type* __s2, size_t __n)
{
    char_type* __r = __s1;
    for (; __n; --__n, ++__s1, ++__s2)
        assign(*__s1, *__s2);
    return __r;
}

template <class _CharT>
__device__ inline
_CharT*
char_traits<_CharT>::assign(char_type* __s, size_t __n, char_type __a)
{
    char_type* __r = __s;
    for (; __n; --__n, ++__s)
        assign(*__s, __a);
    return __r;
}

// char_traits<char>

template <>
struct char_traits<char>
{
    typedef char      char_type;
    typedef int       int_type;
    typedef streamoff off_type;
    typedef streampos pos_type;
    typedef mbstate_t state_type;

    __device__ static void assign(char_type& __c1, const char_type& __c2)
        {__c1 = __c2;}

    __device__ static bool eq(char_type __c1, char_type __c2)
            {return __c1 == __c2;}

   __device__  static bool lt(char_type __c1, char_type __c2)
        {return (unsigned char)__c1 < (unsigned char)__c2;}


    __device__ static int compare(const char_type* __s1, const char_type* __s2, size_t __n)
        {return util::memcmp(__s1, __s2, __n);}

    __device__ static size_t length(const char_type* __s) {return util::strlen(__s);}

    __device__ static const char_type* find(const char_type* __s, size_t __n, const char_type& __a)
        {return (const char_type*)util::memchr(__s, to_int_type(__a), __n);}

    __device__ static char_type* move(char_type* __s1, const char_type* __s2, size_t __n)
        {return (char_type*)util::memmove(__s1, __s2, __n);}

    __device__ static char_type* copy(char_type* __s1, const char_type* __s2, size_t __n)
        {return (char_type*)util::memcpy(__s1, __s2, __n);}

    __device__ static char_type* assign(char_type* __s, size_t __n, char_type __a)
        {return (char_type*)util::memset(__s, to_int_type(__a), __n);}


    __device__ static int_type  not_eof(int_type __c)
        {return eq_int_type(__c, eof()) ? ~eof() : __c;}

    __device__ static char_type to_char_type(int_type __c)
        {return char_type(__c);}

    __device__ static int_type to_int_type(char_type __c)
        {return int_type((unsigned char)__c);}

    __device__ static bool eq_int_type(int_type __c1, int_type __c2)
        {return __c1 == __c2;}

    __device__ static int_type  eof()
        {return int_type(EOF);}
};

template <bool>
class __basic_string_common
{
protected:
    __device__ void __throw_length_error() const;
    __device__ void __throw_out_of_range() const;
};

template <bool __b>
__device__ void
__basic_string_common<__b>::__throw_length_error() const
{
    //device_assert(!"basic_string length_error");
}

template <bool __b>
__device__ void
__basic_string_common<__b>::__throw_out_of_range() const
{
    //device_assert(!"basic_string out_of_range");
}

template<class _CharT, class _Traits, class _Allocator>
class basic_string
    : private __basic_string_common<true>
{
public:
    typedef basic_string                                 __self;
    typedef _Traits                                      traits_type;
    typedef typename traits_type::char_type              value_type;
    typedef _Allocator                                   allocator_type;
    typedef allocator_traits<allocator_type>             __alloc_traits;
    typedef typename __alloc_traits::size_type           size_type;
    typedef typename __alloc_traits::difference_type     difference_type;
    typedef value_type&                                  reference;
    typedef const value_type&                            const_reference;
    typedef typename __alloc_traits::pointer             pointer;
    typedef typename __alloc_traits::const_pointer       const_pointer;
    typedef pointer                                      iterator;
    typedef const_pointer                                const_iterator;
    typedef util::reverse_iterator<iterator>             reverse_iterator;
    typedef util::reverse_iterator<const_iterator>       const_reverse_iterator;

private:
    struct __long
    {
        size_type __cap_;
        size_type __size_;
        pointer   __data_;
    };

    enum {__short_mask = 0x01};
    enum {__long_mask  = 0x1ul};

    enum {__mask = size_type(~0) >> 1};

    enum {__min_cap = (sizeof(__long) - 1)/sizeof(value_type) > 2 ?
                      (sizeof(__long) - 1)/sizeof(value_type) : 2};

    struct __short
    {
        union
        {
            unsigned char __size_;
            value_type __lx;
        };
        value_type __data_[__min_cap];
    };

    union __lx{__long __lx; __short __lxx;};

    enum {__n_words = sizeof(__lx) / sizeof(size_type)};

    struct __raw
    {
        size_type __words[__n_words];
    };

    struct __rep
    {
        union
        {
            __long  __l;
            __short __s;
            __raw   __r;
        };
    };

    pair<__rep, allocator_type> __r_;

public:
    static const size_type npos = (size_type)-1;

    __device__ basic_string();
    __device__ explicit basic_string(const allocator_type& __a);
    __device__ basic_string(const basic_string& __str);
    __device__ basic_string(const basic_string& __str, const allocator_type& __a);
    __device__ basic_string(const_pointer __s);

    __device__ basic_string(const_pointer __s, const allocator_type& __a);

    __device__ basic_string(const_pointer __s, size_type __n);

    __device__ basic_string(const_pointer __s, size_type __n, const allocator_type& __a);

    __device__ basic_string(size_type __n, value_type __c);

    __device__ basic_string(size_type __n, value_type __c, const allocator_type& __a);
    __device__ basic_string(const basic_string& __str, size_type __pos, size_type __n = npos,
                 const allocator_type& __a = allocator_type());
    template<class _InputIterator>

        __device__ basic_string(_InputIterator __first, _InputIterator __last);
    template<class _InputIterator>

        __device__ basic_string(_InputIterator __first, _InputIterator __last, const allocator_type& __a);

    __device__ ~basic_string();

    __device__ basic_string& operator=(const basic_string& __str);

    __device__ basic_string& operator=(const_pointer __s) {return assign(__s);}
    __device__ basic_string& operator=(value_type __c);

    __device__ iterator begin()
        {return iterator(__get_pointer());}

    __device__ const_iterator begin() const
        {return const_iterator(data());}

    __device__ iterator end()
        {return iterator(__get_pointer() + size());}

    __device__ const_iterator end() const
        {return const_iterator(data() + size());}

    __device__ reverse_iterator rbegin()
        {return reverse_iterator(end());}

    __device__ const_reverse_iterator rbegin() const
        {return const_reverse_iterator(end());}

    __device__ reverse_iterator rend()
        {return reverse_iterator(begin());}

    __device__ const_reverse_iterator rend() const
        {return const_reverse_iterator(begin());}


    __device__ const_iterator cbegin() const
        {return begin();}

    __device__ const_iterator cend() const
        {return end();}

    __device__ const_reverse_iterator crbegin() const
        {return rbegin();}

    __device__ const_reverse_iterator crend() const
        {return rend();}

    __device__ size_type size() const
        {return __is_long() ? __get_long_size() : __get_short_size();}
    __device__ size_type length() const {return size();}

    __device__ size_type max_size() const;
    __device__ size_type capacity() const
        {return (__is_long() ? __get_long_cap() : __min_cap) - 1;}

    __device__ void resize(size_type __n, value_type __c);
    __device__ void resize(size_type __n) {resize(__n, value_type());}

    __device__ void reserve(size_type res_arg = 0);

    __device__ void shrink_to_fit() {reserve();}

    __device__ void clear();
    __device__ bool empty() const {return size() == 0;}

    __device__ const_reference operator[](size_type __pos) const;
    __device__ reference       operator[](size_type __pos);

    __device__ const_reference at(size_type __n) const;
    __device__ reference       at(size_type __n);

    __device__ basic_string& operator+=(const basic_string& __str) {return append(__str);}
    __device__ basic_string& operator+=(const_pointer __s)         {return append(__s);}
    __device__ basic_string& operator+=(value_type __c)            {push_back(__c); return *this;}

    __device__ basic_string& append(const basic_string& __str);
    __device__ basic_string& append(const basic_string& __str, size_type __pos, size_type __n);
    __device__ basic_string& append(const_pointer __s, size_type __n);
    __device__ basic_string& append(const_pointer __s);
    __device__ basic_string& append(size_type __n, value_type __c);
    template<class _InputIterator>
        __device__ typename enable_if
        <
             __is_input_iterator  <_InputIterator>::value &&
            !__is_forward_iterator<_InputIterator>::value,
            basic_string&
        >::type
        append(_InputIterator __first, _InputIterator __last);
    template<class _ForwardIterator>
        __device__ typename enable_if
        <
            __is_forward_iterator<_ForwardIterator>::value,
            basic_string&
        >::type
        append(_ForwardIterator __first, _ForwardIterator __last);

    __device__ void push_back(value_type __c);

    __device__ void pop_back();
    __device__ reference       front();
    __device__ const_reference front() const;
    __device__ reference       back();
    __device__ const_reference back() const;


    __device__ basic_string& assign(const basic_string& __str);
    __device__ basic_string& assign(const basic_string& __str, size_type __pos, size_type __n);
    __device__ basic_string& assign(const_pointer __s, size_type __n);
    __device__ basic_string& assign(const_pointer __s);
    __device__ basic_string& assign(size_type __n, value_type __c);
    template<class _InputIterator>
        __device__ typename enable_if
        <
             __is_input_iterator  <_InputIterator>::value &&
            !__is_forward_iterator<_InputIterator>::value,
            basic_string&
        >::type
        assign(_InputIterator __first, _InputIterator __last);
    template<class _ForwardIterator>
        __device__ typename enable_if
        <
            __is_forward_iterator<_ForwardIterator>::value,
            basic_string&
        >::type
        assign(_ForwardIterator __first, _ForwardIterator __last);

    __device__ basic_string& insert(size_type __pos1, const basic_string& __str);
    __device__ basic_string& insert(size_type __pos1, const basic_string& __str, size_type __pos2, size_type __n);
    __device__ basic_string& insert(size_type __pos, const_pointer __s, size_type __n);
    __device__ basic_string& insert(size_type __pos, const_pointer __s);
    __device__ basic_string& insert(size_type __pos, size_type __n, value_type __c);
    __device__ iterator      insert(const_iterator __pos, value_type __c);

    __device__ iterator      insert(const_iterator __pos, size_type __n, value_type __c);
    template<class _InputIterator>
        __device__ typename enable_if
        <
             __is_input_iterator  <_InputIterator>::value &&
            !__is_forward_iterator<_InputIterator>::value,
            iterator
        >::type
        insert(const_iterator __pos, _InputIterator __first, _InputIterator __last);
    template<class _ForwardIterator>
        __device__ typename enable_if
        <
            __is_forward_iterator<_ForwardIterator>::value,
            iterator
        >::type
        insert(const_iterator __pos, _ForwardIterator __first, _ForwardIterator __last);

    __device__ basic_string& erase(size_type __pos = 0, size_type __n = npos);

    __device__ iterator      erase(const_iterator __pos);

    __device__ iterator      erase(const_iterator __first, const_iterator __last);


    __device__ basic_string& replace(size_type __pos1, size_type __n1, const basic_string& __str);
    __device__ basic_string& replace(size_type __pos1, size_type __n1, const basic_string& __str, size_type __pos2, size_type __n2);
    __device__ basic_string& replace(size_type __pos, size_type __n1, const_pointer __s, size_type __n2);
    __device__ basic_string& replace(size_type __pos, size_type __n1, const_pointer __s);
    __device__ basic_string& replace(size_type __pos, size_type __n1, size_type __n2, value_type __c);

    __device__ basic_string& replace(const_iterator __i1, const_iterator __i2, const basic_string& __str);

    __device__ basic_string& replace(const_iterator __i1, const_iterator __i2, const_pointer __s, size_type __n);

    __device__ basic_string& replace(const_iterator __i1, const_iterator __i2, const_pointer __s);

   __device__  basic_string& replace(const_iterator __i1, const_iterator __i2, size_type __n, value_type __c);
    template<class _InputIterator>
        __device__ typename enable_if
        <
            __is_input_iterator<_InputIterator>::value,
            basic_string&
        >::type
        replace(const_iterator __i1, const_iterator __i2, _InputIterator __j1, _InputIterator __j2);

    __device__ size_type copy(pointer __s, size_type __n, size_type __pos = 0) const;

    __device__ basic_string substr(size_type __pos = 0, size_type __n = npos) const;


    __device__ void swap(basic_string& __str);


    __device__ const_pointer c_str() const {return data();}

    __device__ const_pointer data() const  {return __get_pointer();}


    __device__ allocator_type get_allocator() const {return __alloc();}


    __device__ size_type find(const basic_string& __str, size_type __pos = 0) const;
    __device__ size_type find(const_pointer __s, size_type __pos, size_type __n) const;

    __device__ size_type find(const_pointer __s, size_type __pos = 0) const;
    __device__ size_type find(value_type __c, size_type __pos = 0) const;


    __device__ size_type rfind(const basic_string& __str, size_type __pos = npos) const;
    __device__ size_type rfind(const_pointer __s, size_type __pos, size_type __n) const;

    __device__ size_type rfind(const_pointer __s, size_type __pos = npos) const;
    __device__ size_type rfind(value_type __c, size_type __pos = npos) const;


    __device__ size_type find_first_of(const basic_string& __str, size_type __pos = 0) const;
    __device__ size_type find_first_of(const_pointer __s, size_type __pos, size_type __n) const;

    __device__ size_type find_first_of(const_pointer __s, size_type __pos = 0) const;

    __device__ size_type find_first_of(value_type __c, size_type __pos = 0) const;


    __device__ size_type find_last_of(const basic_string& __str, size_type __pos = npos) const;
    __device__ size_type find_last_of(const_pointer __s, size_type __pos, size_type __n) const;

    __device__ size_type find_last_of(const_pointer __s, size_type __pos = npos) const;

    __device__ size_type find_last_of(value_type __c, size_type __pos = npos) const;


    __device__ size_type find_first_not_of(const basic_string& __str, size_type __pos = 0) const;
    __device__ size_type find_first_not_of(const_pointer __s, size_type __pos, size_type __n) const;

    __device__ size_type find_first_not_of(const_pointer __s, size_type __pos = 0) const;

    __device__ size_type find_first_not_of(value_type __c, size_type __pos = 0) const;


    __device__ size_type find_last_not_of(const basic_string& __str, size_type __pos = npos) const;
    __device__ size_type find_last_not_of(const_pointer __s, size_type __pos, size_type __n) const;

    __device__ size_type find_last_not_of(const_pointer __s, size_type __pos = npos) const;

    __device__ size_type find_last_not_of(value_type __c, size_type __pos = npos) const;


    __device__ int compare(const basic_string& __str) const;

    __device__ int compare(size_type __pos1, size_type __n1, const basic_string& __str) const;
    __device__ int compare(size_type __pos1, size_type __n1, const basic_string& __str, size_type __pos2, size_type __n2) const;
    __device__ int compare(const_pointer __s) const;
    __device__ int compare(size_type __pos1, size_type __n1, const_pointer __s) const;
    __device__ int compare(size_type __pos1, size_type __n1, const_pointer __s, size_type __n2) const;

    __device__ bool __invariants() const;
private:

    __device__ allocator_type& __alloc()
        {return __r_.second;}

    __device__ const allocator_type& __alloc() const
        {return __r_.second;}


    __device__ bool __is_long() const
        {return bool(__r_.first.__s.__size_ & __short_mask);}


    __device__ void __set_short_size(size_type __s)
        {__r_.first.__s.__size_ = (unsigned char)(__s << 1);}

    __device__ size_type __get_short_size() const
        {return __r_.first.__s.__size_ >> 1;}

    __device__ void __set_long_size(size_type __s)
        {__r_.first.__l.__size_ = __s;}

    __device__ size_type __get_long_size() const
        {return __r_.first.__l.__size_;}

    __device__ void __set_size(size_type __s)
        {if (__is_long()) __set_long_size(__s); else __set_short_size(__s);}


    __device__ void __set_long_cap(size_type __s)
        {__r_.first.__l.__cap_  = __long_mask | __s;}

    __device__ size_type __get_long_cap() const
        {return __r_.first.__l.__cap_ & size_type(~__long_mask);}


    __device__ void __set_long_pointer(pointer __p)
        {__r_.first.__l.__data_ = __p;}

    __device__ pointer __get_long_pointer()
        {return __r_.first.__l.__data_;}

    __device__ const_pointer __get_long_pointer() const
        {return __r_.first.__l.__data_;}

    __device__ pointer __get_short_pointer()
        {return __r_.first.__s.__data_;}

    __device__ const_pointer __get_short_pointer() const
        {return __r_.first.__s.__data_;}

    __device__ pointer __get_pointer()
        {return __is_long() ? __get_long_pointer() : __get_short_pointer();}

    __device__ const_pointer __get_pointer() const
        {return __is_long() ? __get_long_pointer() : __get_short_pointer();}


    __device__ void __zero()
        {
            size_type (&__a)[__n_words] = __r_.first.__r.__words;
            for (unsigned __i = 0; __i < __n_words; ++__i)
                __a[__i] = 0;
        }

    template <size_type __a> static

        __device__ size_type __align(size_type __s)
            {return __s + (__a-1) & ~(__a-1);}
    enum {__alignment = 16};
    static
    __device__ size_type __recommend(size_type __s)
        {return (__s < __min_cap ? __min_cap :
                 __align<sizeof(value_type) < __alignment ?
                            __alignment/sizeof(value_type) : 1 > (__s+1)) - 1;}

    __device__ void __init(const_pointer __s, size_type __sz, size_type __reserve);
    __device__ void __init(const_pointer __s, size_type __sz);
    __device__ void __init(size_type __n, value_type __c);

    template <class _InputIterator>
		__device__ typename enable_if
		<
		     __is_input_iterator  <_InputIterator>::value &&
		    !__is_forward_iterator<_InputIterator>::value,
		    void
		>::type
		__init(_InputIterator __first, _InputIterator __last);

    template <class _ForwardIterator>
		__device__ typename enable_if
		<
			__is_forward_iterator<_ForwardIterator>::value,
			void
		>::type
		__init(_ForwardIterator __first, _ForwardIterator __last);

    __device__ void __grow_by(size_type __old_cap, size_type __delta_cap, size_type __old_sz,
                   size_type __n_copy,  size_type __n_del,     size_type __n_add = 0);
    __device__ void __grow_by_and_replace(size_type __old_cap, size_type __delta_cap, size_type __old_sz,
                               size_type __n_copy,  size_type __n_del,
                               size_type __n_add, const_pointer __p_new_stuff);


    __device__ void __erase_to_end(size_type __pos);


    __device__ void __copy_assign_alloc(const basic_string& __str)
        {__copy_assign_alloc(__str, integral_constant<bool,
                      __alloc_traits::propagate_on_container_copy_assignment::value>());}


    __device__ void __copy_assign_alloc(const basic_string& __str, true_type)
        {
            if (__alloc() != __str.__alloc())
            {
                clear();
                shrink_to_fit();
            }
            __alloc() = __str.__alloc();
        }


    __device__ void __copy_assign_alloc(const basic_string&, false_type)
        {}

    __device__ void
    __move_assign_alloc(basic_string& __str)
    {__move_assign_alloc(__str, integral_constant<bool,
                      __alloc_traits::propagate_on_container_move_assignment::value>());}


    __device__ void __move_assign_alloc(basic_string& __c, true_type)
        {
            __alloc() = util::move(__c.__alloc());
        }


    __device__ void __move_assign_alloc(basic_string&, false_type)

        {}


    __device__ static void __swap_alloc(allocator_type& __x, allocator_type& __y)
        {__swap_alloc(__x, __y, integral_constant<bool,
                      __alloc_traits::propagate_on_container_swap::value>());}


    __device__ static void __swap_alloc(allocator_type& __x, allocator_type& __y, true_type)
        {
            using util::swap;
            swap(__x, __y);
        }

    __device__ static void __swap_alloc(allocator_type&, allocator_type&, false_type)
        {}

    __device__ void __invalidate_all_iterators();
    __device__ void __invalidate_iterators_past(size_type);

    //friend __device__ basic_string operator+<>(const basic_string&, const basic_string&);
    //friend __device__ basic_string operator+<>(const value_type*, const basic_string&);
    //friend __device__ basic_string operator+<>(value_type, const basic_string&);
    //friend __device__ basic_string operator+<>(const basic_string&, const value_type*);
    //friend __device__ basic_string operator+<>(const basic_string&, value_type);
};

template <class _CharT, class _Traits, class _Allocator>
__device__ inline void
basic_string<_CharT, _Traits, _Allocator>::__invalidate_all_iterators()
{
}

// basic_string

template<class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>
operator+(const basic_string<_CharT, _Traits, _Allocator>& __x,
          const basic_string<_CharT, _Traits, _Allocator>& __y);

template<class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>
operator+(const _CharT* __x, const basic_string<_CharT,_Traits,_Allocator>& __y);

template<class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>
operator+(_CharT __x, const basic_string<_CharT,_Traits,_Allocator>& __y);

template<class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>
operator+(const basic_string<_CharT, _Traits, _Allocator>& __x, const _CharT* __y);

template<class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>
operator+(const basic_string<_CharT, _Traits, _Allocator>& __x, _CharT __y);


template <class _CharT, class _Traits, class _Allocator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>::basic_string()
{
    __zero();
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>::basic_string(const allocator_type& __a)
    : __r_(__a)
{
    __zero();
}

template <class _CharT, class _Traits, class _Allocator>
__device__ void
basic_string<_CharT, _Traits, _Allocator>::__init(const_pointer __s, size_type __sz, size_type __reserve)
{
    if (__reserve > max_size())
        this->__throw_length_error();
    pointer __p;
    if (__reserve < __min_cap)
    {
        __set_short_size(__sz);
        __p = __get_short_pointer();
    }
    else
    {
        size_type __cap = __recommend(__reserve);
        __p = __alloc_traits::allocate(__alloc(), __cap+1);
        __set_long_pointer(__p);
        __set_long_cap(__cap+1);
        __set_long_size(__sz);
    }
    traits_type::copy(__p, __s, __sz);
    traits_type::assign(__p[__sz], value_type());
}

template <class _CharT, class _Traits, class _Allocator>
__device__ void
basic_string<_CharT, _Traits, _Allocator>::__init(const_pointer __s, size_type __sz)
{
    if (__sz > max_size())
        this->__throw_length_error();
    pointer __p;
    if (__sz < __min_cap)
    {
        __set_short_size(__sz);
        __p = __get_short_pointer();
    }
    else
    {
        size_type __cap = __recommend(__sz);
        __p = __alloc_traits::allocate(__alloc(), __cap+1);
        __set_long_pointer(__p);
        __set_long_cap(__cap+1);
        __set_long_size(__sz);
    }
    traits_type::copy(__p, __s, __sz);
    traits_type::assign(__p[__sz], value_type());
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>::basic_string(const_pointer __s)
{
    device_assert(__s != 0);
    __init(__s, traits_type::length(__s));
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>::basic_string(const_pointer __s, const allocator_type& __a)
    : __r_(__rep(), __a)
{
    device_assert(__s != 0);
    __init(__s, traits_type::length(__s));
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>::basic_string(const_pointer __s, size_type __n)
{
    device_assert(__s != 0);
    __init(__s, __n);
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>::basic_string(const_pointer __s, size_type __n, const allocator_type& __a)
    : __r_(__rep(), __a)
{
    device_assert(__s != 0);
    __init(__s, __n);
}

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>::basic_string(const basic_string& __str)
    : __r_(__rep(), __alloc_traits::select_on_container_copy_construction(__str.__alloc()))
{
    if (!__str.__is_long())
        __r_.first.__r = __str.__r_.first.__r;
    else
        __init(__str.__get_long_pointer(), __str.__get_long_size());
}

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>::basic_string(const basic_string& __str, const allocator_type& __a)
    : __r_(__rep(), __a)
{
    if (!__str.__is_long())
        __r_.first.__r = __str.__r_.first.__r;
    else
        __init(__str.__get_long_pointer(), __str.__get_long_size());
}

template <class _CharT, class _Traits, class _Allocator>
__device__ void
basic_string<_CharT, _Traits, _Allocator>::__init(size_type __n, value_type __c)
{
    if (__n > max_size())
        this->__throw_length_error();
    pointer __p;
    if (__n < __min_cap)
    {
        __set_short_size(__n);
        __p = __get_short_pointer();
    }
    else
    {
        size_type __cap = __recommend(__n);
        __p = __alloc_traits::allocate(__alloc(), __cap+1);
        __set_long_pointer(__p);
        __set_long_cap(__cap+1);
        __set_long_size(__n);
    }
    traits_type::assign(__p, __n, __c);
    traits_type::assign(__p[__n], value_type());
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>::basic_string(size_type __n, value_type __c)
{
    __init(__n, __c);
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>::basic_string(size_type __n, value_type __c, const allocator_type& __a)
    : __r_(__rep(), __a)
{
    __init(__n, __c);
}

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>::basic_string(const basic_string& __str, size_type __pos, size_type __n,
                                                        const allocator_type& __a)
    : __r_(__rep(), __a)
{
    size_type __str_sz = __str.size();
    if (__pos > __str_sz)
        this->__throw_out_of_range();
    __init(__str.data() + __pos, util::min(__n, __str_sz - __pos));
}

template <class _CharT, class _Traits, class _Allocator>
template <class _InputIterator>
__device__ typename enable_if
<
     __is_input_iterator  <_InputIterator>::value &&
    !__is_forward_iterator<_InputIterator>::value,
    void
>::type
basic_string<_CharT, _Traits, _Allocator>::__init(_InputIterator __first, _InputIterator __last)
{
    __zero();
    for (; __first != __last; ++__first)
        push_back(*__first);
}

template <class _CharT, class _Traits, class _Allocator>
template <class _ForwardIterator>
__device__ typename enable_if
<
    __is_forward_iterator<_ForwardIterator>::value,
    void
>::type
basic_string<_CharT, _Traits, _Allocator>::__init(_ForwardIterator __first, _ForwardIterator __last)
{
    size_type __sz = static_cast<size_type>(util::distance(__first, __last));
    if (__sz > max_size())
        this->__throw_length_error();
    pointer __p;
    if (__sz < __min_cap)
    {
        __set_short_size(__sz);
        __p = __get_short_pointer();
    }
    else
    {
        size_type __cap = __recommend(__sz);
        __p = __alloc_traits::allocate(__alloc(), __cap+1);
        __set_long_pointer(__p);
        __set_long_cap(__cap+1);
        __set_long_size(__sz);
    }
    for (; __first != __last; ++__first, ++__p)
        traits_type::assign(*__p, *__first);
    traits_type::assign(*__p, value_type());
}

template <class _CharT, class _Traits, class _Allocator>
template<class _InputIterator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>::basic_string(_InputIterator __first, _InputIterator __last)
{
    __init(__first, __last);
}

template <class _CharT, class _Traits, class _Allocator>
template<class _InputIterator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>::basic_string(_InputIterator __first, _InputIterator __last,
                                                        const allocator_type& __a)
    : __r_(__rep(), __a)
{
    __init(__first, __last);
}

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>::~basic_string()
{
    __invalidate_all_iterators();
    if (__is_long())
        __alloc_traits::deallocate(__alloc(), __get_long_pointer(), __get_long_cap());
}

template <class _CharT, class _Traits, class _Allocator>
__device__ void
basic_string<_CharT, _Traits, _Allocator>::__grow_by_and_replace
    (size_type __old_cap, size_type __delta_cap, size_type __old_sz,
     size_type __n_copy,  size_type __n_del,     size_type __n_add, const_pointer __p_new_stuff)
{
    size_type __ms = max_size();
    if (__delta_cap > __ms - __old_cap - 1)
        this->__throw_length_error();
    pointer __old_p = __get_pointer();
    size_type __cap = __old_cap < __ms / 2 - __alignment ?
                          __recommend(util::max(__old_cap + __delta_cap, 2 * __old_cap)) :
                          __ms - 1;
    pointer __p = __alloc_traits::allocate(__alloc(), __cap+1);
    __invalidate_all_iterators();
    if (__n_copy != 0)
        traits_type::copy(__p, __old_p, __n_copy);
    if (__n_add != 0)
        traits_type::copy(__p + __n_copy, __p_new_stuff, __n_add);
    size_type __sec_cp_sz = __old_sz - __n_del - __n_copy;
    if (__sec_cp_sz != 0)
        traits_type::copy(__p + __n_copy + __n_add, __old_p + __n_copy + __n_del, __sec_cp_sz);
    if (__old_cap+1 != __min_cap)
        __alloc_traits::deallocate(__alloc(), __old_p, __old_cap+1);
    __set_long_pointer(__p);
    __set_long_cap(__cap+1);
    __old_sz = __n_copy + __n_add + __sec_cp_sz;
    __set_long_size(__old_sz);
    traits_type::assign(__p[__old_sz], value_type());
}

template <class _CharT, class _Traits, class _Allocator>
__device__ void
basic_string<_CharT, _Traits, _Allocator>::__grow_by(size_type __old_cap, size_type __delta_cap, size_type __old_sz,
                                                     size_type __n_copy,  size_type __n_del,     size_type __n_add)
{
    size_type __ms = max_size();
    if (__delta_cap > __ms - __old_cap - 1)
        this->__throw_length_error();
    pointer __old_p = __get_pointer();
    size_type __cap = __old_cap < __ms / 2 - __alignment ?
                          __recommend(util::max(__old_cap + __delta_cap, 2 * __old_cap)) :
                          __ms - 1;
    pointer __p = __alloc_traits::allocate(__alloc(), __cap+1);
    __invalidate_all_iterators();
    if (__n_copy != 0)
        traits_type::copy(__p, __old_p, __n_copy);
    size_type __sec_cp_sz = __old_sz - __n_del - __n_copy;
    if (__sec_cp_sz != 0)
        traits_type::copy(__p + __n_copy + __n_add, __old_p + __n_copy + __n_del, __sec_cp_sz);
    if (__old_cap+1 != __min_cap)
        __alloc_traits::deallocate(__alloc(), __old_p, __old_cap+1);
    __set_long_pointer(__p);
    __set_long_cap(__cap+1);
}

// assign

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::assign(const_pointer __s, size_type __n)
{
    device_assert(__s != 0);
    size_type __cap = capacity();
    if (__cap >= __n)
    {
        pointer __p = __get_pointer();
        traits_type::move(__p, __s, __n);
        traits_type::assign(__p[__n], value_type());
        __set_size(__n);
        __invalidate_iterators_past(__n);
    }
    else
    {
        size_type __sz = size();
        __grow_by_and_replace(__cap, __n - __cap, __sz, 0, __sz, __n, __s);
    }
    return *this;
}

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::assign(size_type __n, value_type __c)
{
    size_type __cap = capacity();
    if (__cap < __n)
    {
        size_type __sz = size();
        __grow_by(__cap, __n - __cap, __sz, 0, __sz);
    }
    else
        __invalidate_iterators_past(__n);
    pointer __p = __get_pointer();
    traits_type::assign(__p, __n, __c);
    traits_type::assign(__p[__n], value_type());
    __set_size(__n);
    return *this;
}

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::operator=(value_type __c)
{
    pointer __p;
    if (__is_long())
    {
        __p = __get_long_pointer();
        __set_long_size(1);
    }
    else
    {
        __p = __get_short_pointer();
        __set_short_size(1);
    }
    traits_type::assign(*__p, __c);
    traits_type::assign(*++__p, value_type());
    __invalidate_iterators_past(1);
    return *this;
}

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::operator=(const basic_string& __str)
{
    if (this != &__str)
    {
        __copy_assign_alloc(__str);
        assign(__str);
    }
    return *this;
}

template <class _CharT, class _Traits, class _Allocator>
template<class _InputIterator>
__device__ typename enable_if
<
     __is_input_iterator  <_InputIterator>::value &&
    !__is_forward_iterator<_InputIterator>::value,
    basic_string<_CharT, _Traits, _Allocator>&
>::type
basic_string<_CharT, _Traits, _Allocator>::assign(_InputIterator __first, _InputIterator __last)
{
    clear();
    for (; __first != __last; ++__first)
        push_back(*__first);
    return *this;
}

template <class _CharT, class _Traits, class _Allocator>
template<class _ForwardIterator>
__device__ typename enable_if
<
    __is_forward_iterator<_ForwardIterator>::value,
    basic_string<_CharT, _Traits, _Allocator>&
>::type
basic_string<_CharT, _Traits, _Allocator>::assign(_ForwardIterator __first, _ForwardIterator __last)
{
    size_type __n = static_cast<size_type>(util::distance(__first, __last));
    size_type __cap = capacity();
    if (__cap < __n)
    {
        size_type __sz = size();
        __grow_by(__cap, __n - __cap, __sz, 0, __sz);
    }
    else
        __invalidate_iterators_past(__n);
    pointer __p = __get_pointer();
    for (; __first != __last; ++__first, ++__p)
        traits_type::assign(*__p, *__first);
    traits_type::assign(*__p, value_type());
    __set_size(__n);
    return *this;
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::assign(const basic_string& __str)
{
    return assign(__str.data(), __str.size());
}

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::assign(const basic_string& __str, size_type __pos, size_type __n)
{
    size_type __sz = __str.size();
    if (__pos > __sz)
        this->__throw_out_of_range();
    return assign(__str.data() + __pos, util::min(__n, __sz - __pos));
}

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::assign(const_pointer __s)
{
    device_assert(__s != 0);
    return assign(__s, traits_type::length(__s));
}

// append

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::append(const_pointer __s, size_type __n)
{
    device_assert(__s != 0);
    size_type __cap = capacity();
    size_type __sz = size();
    if (__cap - __sz >= __n)
    {
        if (__n)
        {
            pointer __p = __get_pointer();
            traits_type::copy(__p + __sz, __s, __n);
            __sz += __n;
            __set_size(__sz);
            traits_type::assign(__p[__sz], value_type());
        }
    }
    else
        __grow_by_and_replace(__cap, __sz + __n - __cap, __sz, __sz, 0, __n, __s);
    return *this;
}

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::append(size_type __n, value_type __c)
{
    if (__n)
    {
        size_type __cap = capacity();
        size_type __sz = size();
        if (__cap - __sz < __n)
            __grow_by(__cap, __sz + __n - __cap, __sz, __sz, 0);
        pointer __p = __get_pointer();
        traits_type::assign(__p + __sz, __n, __c);
        __sz += __n;
        __set_size(__sz);
        traits_type::assign(__p[__sz], value_type());
    }
    return *this;
}

template <class _CharT, class _Traits, class _Allocator>
__device__ void
basic_string<_CharT, _Traits, _Allocator>::push_back(value_type __c)
{
    size_type __cap = capacity();
    size_type __sz = size();
    if (__sz == __cap)
        __grow_by(__cap, 1, __sz, __sz, 0);
    pointer __p = __get_pointer() + __sz;
    traits_type::assign(*__p, __c);
    traits_type::assign(*++__p, value_type());
    __set_size(__sz+1);
}

template <class _CharT, class _Traits, class _Allocator>
template<class _InputIterator>
__device__ typename enable_if
<
     __is_input_iterator  <_InputIterator>::value &&
    !__is_forward_iterator<_InputIterator>::value,
    basic_string<_CharT, _Traits, _Allocator>&
>::type
basic_string<_CharT, _Traits, _Allocator>::append(_InputIterator __first, _InputIterator __last)
{
    for (; __first != __last; ++__first)
        push_back(*__first);
    return *this;
}

template <class _CharT, class _Traits, class _Allocator>
template<class _ForwardIterator>
__device__ typename enable_if
<
    __is_forward_iterator<_ForwardIterator>::value,
    basic_string<_CharT, _Traits, _Allocator>&
>::type
basic_string<_CharT, _Traits, _Allocator>::append(_ForwardIterator __first, _ForwardIterator __last)
{
    size_type __sz = size();
    size_type __cap = capacity();
    size_type __n = static_cast<size_type>(util::distance(__first, __last));
    if (__n)
    {
        if (__cap - __sz < __n)
            __grow_by(__cap, __sz + __n - __cap, __sz, __sz, 0);
        pointer __p = __get_pointer() + __sz;
        for (; __first != __last; ++__p, ++__first)
            traits_type::assign(*__p, *__first);
        traits_type::assign(*__p, value_type());
        __set_size(__sz + __n);
    }
    return *this;
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::append(const basic_string& __str)
{
    return append(__str.data(), __str.size());
}

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::append(const basic_string& __str, size_type __pos, size_type __n)
{
    size_type __sz = __str.size();
    if (__pos > __sz)
        this->__throw_out_of_range();
    return append(__str.data() + __pos, util::min(__n, __sz - __pos));
}

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::append(const_pointer __s)
{
    device_assert(__s != 0);
    return append(__s, traits_type::length(__s));
}

// insert

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::insert(size_type __pos, const_pointer __s, size_type __n)
{
    device_assert(__s != 0);
    size_type __sz = size();
    if (__pos > __sz)
        this->__throw_out_of_range();
    size_type __cap = capacity();
    if (__cap - __sz >= __n)
    {
        if (__n)
        {
            pointer __p = __get_pointer();
            size_type __n_move = __sz - __pos;
            if (__n_move != 0)
            {
                if (__p + __pos <= __s && __s < __p + __sz)
                    __s += __n;
                traits_type::move(__p + __pos + __n, __p + __pos, __n_move);
            }
            traits_type::move(__p + __pos, __s, __n);
            __sz += __n;
            __set_size(__sz);
            traits_type::assign(__p[__sz], value_type());
        }
    }
    else
        __grow_by_and_replace(__cap, __sz + __n - __cap, __sz, __pos, 0, __n, __s);
    return *this;
}

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::insert(size_type __pos, size_type __n, value_type __c)
{
    size_type __sz = size();
    if (__pos > __sz)
        this->__throw_out_of_range();
    if (__n)
    {
        size_type __cap = capacity();
        pointer __p;
        if (__cap - __sz >= __n)
        {
            __p = __get_pointer();
            size_type __n_move = __sz - __pos;
            if (__n_move != 0)
                traits_type::move(__p + __pos + __n, __p + __pos, __n_move);
        }
        else
        {
            __grow_by(__cap, __sz + __n - __cap, __sz, __pos, 0, __n);
            __p = __get_long_pointer();
        }
        traits_type::assign(__p + __pos, __n, __c);
        __sz += __n;
        __set_size(__sz);
        traits_type::assign(__p[__sz], value_type());
    }
    return *this;
}

template <class _CharT, class _Traits, class _Allocator>
template<class _InputIterator>
__device__ typename enable_if
<
     __is_input_iterator  <_InputIterator>::value &&
    !__is_forward_iterator<_InputIterator>::value,
    typename basic_string<_CharT, _Traits, _Allocator>::iterator
>::type
basic_string<_CharT, _Traits, _Allocator>::insert(const_iterator __pos, _InputIterator __first, _InputIterator __last)
{
    size_type __old_sz = size();
    difference_type __ip = __pos - begin();
    for (; __first != __last; ++__first)
        push_back(*__first);
    pointer __p = __get_pointer();
    util::rotate(__p + __ip, __p + __old_sz, __p + size());
    return iterator(__p + __ip);
}

template <class _CharT, class _Traits, class _Allocator>
template<class _ForwardIterator>
__device__ typename enable_if
<
    __is_forward_iterator<_ForwardIterator>::value,
    typename basic_string<_CharT, _Traits, _Allocator>::iterator
>::type
basic_string<_CharT, _Traits, _Allocator>::insert(const_iterator __pos, _ForwardIterator __first, _ForwardIterator __last)
{
    size_type __ip = static_cast<size_type>(__pos - begin());
    size_type __sz = size();
    size_type __cap = capacity();
    size_type __n = static_cast<size_type>(util::distance(__first, __last));
    if (__n)
    {
        pointer __p;
        if (__cap - __sz >= __n)
        {
            __p = __get_pointer();
            size_type __n_move = __sz - __ip;
            if (__n_move != 0)
                traits_type::move(__p + __ip + __n, __p + __ip, __n_move);
        }
        else
        {
            __grow_by(__cap, __sz + __n - __cap, __sz, __ip, 0, __n);
            __p = __get_long_pointer();
        }
        __sz += __n;
        __set_size(__sz);
        traits_type::assign(__p[__sz], value_type());
        for (__p += __ip; __first != __last; ++__p, ++__first)
            traits_type::assign(*__p, *__first);
    }
    return begin() + __ip;
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::insert(size_type __pos1, const basic_string& __str)
{
    return insert(__pos1, __str.data(), __str.size());
}

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::insert(size_type __pos1, const basic_string& __str,
                                                  size_type __pos2, size_type __n)
{
    size_type __str_sz = __str.size();
    if (__pos2 > __str_sz)
        this->__throw_out_of_range();
    return insert(__pos1, __str.data() + __pos2, util::min(__n, __str_sz - __pos2));
}

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::insert(size_type __pos, const_pointer __s)
{
    device_assert(__s != 0);
    return insert(__pos, __s, traits_type::length(__s));
}

template <class _CharT, class _Traits, class _Allocator>
__device__ typename basic_string<_CharT, _Traits, _Allocator>::iterator
basic_string<_CharT, _Traits, _Allocator>::insert(const_iterator __pos, value_type __c)
{
    size_type __ip = static_cast<size_type>(__pos - begin());
    size_type __sz = size();
    size_type __cap = capacity();
    pointer __p;
    if (__cap == __sz)
    {
        __grow_by(__cap, 1, __sz, __ip, 0, 1);
        __p = __get_long_pointer();
    }
    else
    {
        __p = __get_pointer();
        size_type __n_move = __sz - __ip;
        if (__n_move != 0)
            traits_type::move(__p + __ip + 1, __p + __ip, __n_move);
    }
    traits_type::assign(__p[__ip], __c);
    traits_type::assign(__p[++__sz], value_type());
    __set_size(__sz);
    return begin() + static_cast<difference_type>(__ip);
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::iterator
basic_string<_CharT, _Traits, _Allocator>::insert(const_iterator __pos, size_type __n, value_type __c)
{
    difference_type __p = __pos - begin();
    insert(static_cast<size_type>(__p), __n, __c);
    return begin() + __p;
}

// replace

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::replace(size_type __pos, size_type __n1, const_pointer __s, size_type __n2)
{
    device_assert(__s != 0);
    size_type __sz = size();
    if (__pos > __sz)
        this->__throw_out_of_range();
    __n1 = util::min(__n1, __sz - __pos);
    size_type __cap = capacity();
    if (__cap - __sz + __n1 >= __n2)
    {
        pointer __p = __get_pointer();
        if (__n1 != __n2)
        {
            size_type __n_move = __sz - __pos - __n1;
            if (__n_move != 0)
            {
                if (__n1 > __n2)
                {
                    traits_type::move(__p + __pos, __s, __n2);
                    traits_type::move(__p + __pos + __n2, __p + __pos + __n1, __n_move);
                    goto __finish;
                }
                if (__p + __pos < __s && __s < __p + __sz)
                {
                    if (__p + __pos + __n1 <= __s)
                        __s += __n2 - __n1;
                    else // __p + __pos < __s < __p + __pos + __n1
                    {
                        traits_type::move(__p + __pos, __s, __n1);
                        __pos += __n1;
                        __s += __n2;
                        __n2 -= __n1;
                        __n1 = 0;
                    }
                }
                traits_type::move(__p + __pos + __n2, __p + __pos + __n1, __n_move);
            }
        }
        traits_type::move(__p + __pos, __s, __n2);
__finish:
        __sz += __n2 - __n1;
        __set_size(__sz);
        __invalidate_iterators_past(__sz);
        traits_type::assign(__p[__sz], value_type());
    }
    else
        __grow_by_and_replace(__cap, __sz - __n1 + __n2 - __cap, __sz, __pos, __n1, __n2, __s);
    return *this;
}

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::replace(size_type __pos, size_type __n1, size_type __n2, value_type __c)
{
    size_type __sz = size();
    if (__pos > __sz)
        this->__throw_out_of_range();
    __n1 = util::min(__n1, __sz - __pos);
    size_type __cap = capacity();
    pointer __p;
    if (__cap - __sz + __n1 >= __n2)
    {
        __p = __get_pointer();
        if (__n1 != __n2)
        {
            size_type __n_move = __sz - __pos - __n1;
            if (__n_move != 0)
                traits_type::move(__p + __pos + __n2, __p + __pos + __n1, __n_move);
        }
    }
    else
    {
        __grow_by(__cap, __sz - __n1 + __n2 - __cap, __sz, __pos, __n1, __n2);
        __p = __get_long_pointer();
    }
    traits_type::assign(__p + __pos, __n2, __c);
    __sz += __n2 - __n1;
    __set_size(__sz);
    __invalidate_iterators_past(__sz);
    traits_type::assign(__p[__sz], value_type());
    return *this;
}

template <class _CharT, class _Traits, class _Allocator>
template<class _InputIterator>
__device__ typename enable_if
<
    __is_input_iterator<_InputIterator>::value,
    basic_string<_CharT, _Traits, _Allocator>&
>::type
basic_string<_CharT, _Traits, _Allocator>::replace(const_iterator __i1, const_iterator __i2,
                                                   _InputIterator __j1, _InputIterator __j2)
{
    for (; true; ++__i1, ++__j1)
    {
        if (__i1 == __i2)
        {
            if (__j1 != __j2)
                insert(__i1, __j1, __j2);
            break;
        }
        if (__j1 == __j2)
        {
            erase(__i1, __i2);
            break;
        }
        traits_type::assign(const_cast<value_type&>(*__i1), *__j1);
    }
    return *this;
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::replace(size_type __pos1, size_type __n1, const basic_string& __str)
{
    return replace(__pos1, __n1, __str.data(), __str.size());
}

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::replace(size_type __pos1, size_type __n1, const basic_string& __str,
                                                   size_type __pos2, size_type __n2)
{
    size_type __str_sz = __str.size();
    if (__pos2 > __str_sz)
        this->__throw_out_of_range();
    return replace(__pos1, __n1, __str.data() + __pos2, util::min(__n2, __str_sz - __pos2));
}

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::replace(size_type __pos, size_type __n1, const_pointer __s)
{
    device_assert(__s != 0);
    return replace(__pos, __n1, __s, traits_type::length(__s));
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::replace(const_iterator __i1, const_iterator __i2, const basic_string& __str)
{
    return replace(static_cast<size_type>(__i1 - begin()), static_cast<size_type>(__i2 - __i1),
                   __str.data(), __str.size());
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::replace(const_iterator __i1, const_iterator __i2, const_pointer __s, size_type __n)
{
    return replace(static_cast<size_type>(__i1 - begin()), static_cast<size_type>(__i2 - __i1), __s, __n);
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::replace(const_iterator __i1, const_iterator __i2, const_pointer __s)
{
    return replace(static_cast<size_type>(__i1 - begin()), static_cast<size_type>(__i2 - __i1), __s);
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::replace(const_iterator __i1, const_iterator __i2, size_type __n, value_type __c)
{
    return replace(static_cast<size_type>(__i1 - begin()), static_cast<size_type>(__i2 - __i1), __n, __c);
}

// erase

template <class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>&
basic_string<_CharT, _Traits, _Allocator>::erase(size_type __pos, size_type __n)
{
    size_type __sz = size();
    if (__pos > __sz)
        this->__throw_out_of_range();
    if (__n)
    {
        pointer __p = __get_pointer();
        __n = util::min(__n, __sz - __pos);
        size_type __n_move = __sz - __pos - __n;
        if (__n_move != 0)
            traits_type::move(__p + __pos, __p + __pos + __n, __n_move);
        __sz -= __n;
        __set_size(__sz);
        __invalidate_iterators_past(__sz);
        traits_type::assign(__p[__sz], value_type());
    }
    return *this;
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::iterator
basic_string<_CharT, _Traits, _Allocator>::erase(const_iterator __pos)
{
    iterator __b = begin();
    size_type __r = static_cast<size_type>(__pos - __b);
    erase(__r, 1);
    return __b + static_cast<difference_type>(__r);
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::iterator
basic_string<_CharT, _Traits, _Allocator>::erase(const_iterator __first, const_iterator __last)
{
    iterator __b = begin();
    size_type __r = static_cast<size_type>(__first - __b);
    erase(__r, static_cast<size_type>(__last - __first));
    return __b + static_cast<difference_type>(__r);
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
void
basic_string<_CharT, _Traits, _Allocator>::pop_back()
{
    device_assert(!empty());
    size_type __sz;
    if (__is_long())
    {
        __sz = __get_long_size() - 1;
        __set_long_size(__sz);
        traits_type::assign(*(__get_long_pointer() + __sz), value_type());
    }
    else
    {
        __sz = __get_short_size() - 1;
        __set_short_size(__sz);
        traits_type::assign(*(__get_short_pointer() + __sz), value_type());
    }
    __invalidate_iterators_past(__sz);
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
void
basic_string<_CharT, _Traits, _Allocator>::clear()
{
    __invalidate_all_iterators();
    if (__is_long())
    {
        traits_type::assign(*__get_long_pointer(), value_type());
        __set_long_size(0);
    }
    else
    {
        traits_type::assign(*__get_short_pointer(), value_type());
        __set_short_size(0);
    }
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
void
basic_string<_CharT, _Traits, _Allocator>::__erase_to_end(size_type __pos)
{
    if (__is_long())
    {
        traits_type::assign(*(__get_long_pointer() + __pos), value_type());
        __set_long_size(__pos);
    }
    else
    {
        traits_type::assign(*(__get_short_pointer() + __pos), value_type());
        __set_short_size(__pos);
    }
    __invalidate_iterators_past(__pos);
}

template <class _CharT, class _Traits, class _Allocator>
__device__ void
basic_string<_CharT, _Traits, _Allocator>::resize(size_type __n, value_type __c)
{
    size_type __sz = size();
    if (__n > __sz)
        append(__n - __sz, __c);
    else
        __erase_to_end(__n);
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::max_size() const
{
    size_type __m = __alloc_traits::max_size(__alloc());
	return __m - 1;
}

template <class _CharT, class _Traits, class _Allocator>
__device__ void
basic_string<_CharT, _Traits, _Allocator>::reserve(size_type __res_arg)
{
    if (__res_arg > max_size())
        this->__throw_length_error();
    size_type __cap = capacity();
    size_type __sz = size();
    __res_arg = util::max(__res_arg, __sz);
    __res_arg = __recommend(__res_arg);
    if (__res_arg != __cap)
    {
        pointer __new_data, __p;
        bool __was_long, __now_long;
        if (__res_arg == __min_cap - 1)
        {
            __was_long = true;
            __now_long = false;
            __new_data = __get_short_pointer();
            __p = __get_long_pointer();
        }
        else
        {
            if (__res_arg > __cap)
                __new_data = __alloc_traits::allocate(__alloc(), __res_arg+1);
            else
            {
                __new_data = __alloc_traits::allocate(__alloc(), __res_arg+1);

                if (__new_data == 0)
                    return;
            }
            __now_long = true;
            __was_long = __is_long();
            __p = __get_pointer();
        }
        traits_type::copy(__new_data, __p, size()+1);
        if (__was_long)
            __alloc_traits::deallocate(__alloc(), __p, __cap+1);
        if (__now_long)
        {
            __set_long_cap(__res_arg+1);
            __set_long_size(__sz);
            __set_long_pointer(__new_data);
        }
        else
            __set_short_size(__sz);
        __invalidate_all_iterators();
    }
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::const_reference
basic_string<_CharT, _Traits, _Allocator>::operator[](size_type __pos) const
{
    device_assert(__pos <= size());
    return *(data() + __pos);
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::reference
basic_string<_CharT, _Traits, _Allocator>::operator[](size_type __pos)
{
    device_assert(__pos < size());
    return *(__get_pointer() + __pos);
}

template <class _CharT, class _Traits, class _Allocator>
__device__ typename basic_string<_CharT, _Traits, _Allocator>::const_reference
basic_string<_CharT, _Traits, _Allocator>::at(size_type __n) const
{
    if (__n >= size())
        this->__throw_out_of_range();
    return (*this)[__n];
}

template <class _CharT, class _Traits, class _Allocator>
__device__ typename basic_string<_CharT, _Traits, _Allocator>::reference
basic_string<_CharT, _Traits, _Allocator>::at(size_type __n)
{
    if (__n >= size())
        this->__throw_out_of_range();
    return (*this)[__n];
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::reference
basic_string<_CharT, _Traits, _Allocator>::front()
{
    device_assert(!empty());
    return *__get_pointer();
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::const_reference
basic_string<_CharT, _Traits, _Allocator>::front() const
{
    device_assert(!empty());
    return *data();
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::reference
basic_string<_CharT, _Traits, _Allocator>::back()
{
    device_assert(!empty());
    return *(__get_pointer() + size() - 1);
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::const_reference
basic_string<_CharT, _Traits, _Allocator>::back() const
{
    device_assert(!empty());
    return *(data() + size() - 1);
}

template <class _CharT, class _Traits, class _Allocator>
__device__ typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::copy(pointer __s, size_type __n, size_type __pos) const
{
    size_type __sz = size();
    if (__pos > __sz)
        this->__throw_out_of_range();
    size_type __rlen = util::min(__n, __sz - __pos);
    traits_type::copy(__s, data() + __pos, __rlen);
    return __rlen;
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
basic_string<_CharT, _Traits, _Allocator>
basic_string<_CharT, _Traits, _Allocator>::substr(size_type __pos, size_type __n) const
{
    return basic_string(*this, __pos, __n, __alloc());
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
void
basic_string<_CharT, _Traits, _Allocator>::swap(basic_string& __str)
{
    util::swap(__r_.first, __str.__r_.first);
    __swap_alloc(__alloc(), __str.__alloc());
}

// find

template <class _Traits>
struct __traits_eq
{
    typedef typename _Traits::char_type char_type;

    __device__ bool operator()(const char_type& __x, const char_type& __y)
        {return _Traits::eq(__x, __y);}
};

template<class _CharT, class _Traits, class _Allocator>
__device__ typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find(const_pointer __s,
                                                size_type __pos,
                                                size_type __n) const
{
    device_assert(__s != 0);
    size_type __sz = size();
    if (__pos > __sz || __sz - __pos < __n)
        return npos;
    if (__n == 0)
        return __pos;
    const_pointer __p = data();
    const_pointer __r = util::search(__p + __pos, __p + __sz, __s, __s + __n,
                                     __traits_eq<traits_type>());
    if (__r == __p + __sz)
        return npos;
    return static_cast<size_type>(__r - __p);
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find(const basic_string& __str,
                                                size_type __pos) const
{
    return find(__str.data(), __pos, __str.size());
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find(const_pointer __s,
                                                size_type __pos) const
{
    device_assert(__s != 0);
    return find(__s, __pos, traits_type::length(__s));
}

template<class _CharT, class _Traits, class _Allocator>
__device__ typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find(value_type __c,
                                                size_type __pos) const
{
    size_type __sz = size();
    if (__pos >= __sz)
        return npos;
    const_pointer __p = data();
    const_pointer __r = traits_type::find(__p + __pos, __sz - __pos, __c);
    if (__r == 0)
        return npos;
    return static_cast<size_type>(__r - __p);
}

// rfind

template<class _CharT, class _Traits, class _Allocator>
__device__ typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::rfind(const_pointer __s,
                                                 size_type __pos,
                                                 size_type __n) const
{
    device_assert(__s != 0);
    size_type __sz = size();
    __pos = util::min(__pos, __sz);
    if (__n < __sz - __pos)
        __pos += __n;
    else
        __pos = __sz;
    const_pointer __p = data();
    const_pointer __r = util::find_end(__p, __p + __pos, __s, __s + __n,
                                       __traits_eq<traits_type>());
    if (__n > 0 && __r == __p + __pos)
        return npos;
    return static_cast<size_type>(__r - __p);
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::rfind(const basic_string& __str,
                                                 size_type __pos) const
{
    return rfind(__str.data(), __pos, __str.size());
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::rfind(const_pointer __s,
                                                 size_type __pos) const
{
    device_assert(__s != 0);
    return rfind(__s, __pos, traits_type::length(__s));
}

template<class _CharT, class _Traits, class _Allocator>
__device__ typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::rfind(value_type __c,
                                                 size_type __pos) const
{
    size_type __sz = size();
    if (__sz)
    {
        if (__pos < __sz)
            ++__pos;
        else
            __pos = __sz;
        const_pointer __p = data();
        for (const_pointer __ps = __p + __pos; __ps != __p;)
        {
            if (traits_type::eq(*--__ps, __c))
                return static_cast<size_type>(__ps - __p);
        }
    }
    return npos;
}

// find_first_of

template<class _CharT, class _Traits, class _Allocator>
__device__ typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find_first_of(const_pointer __s,
                                                         size_type __pos,
                                                         size_type __n) const
{
    device_assert(__s != 0);
    size_type __sz = size();
    if (__pos >= __sz || __n == 0)
        return npos;
    const_pointer __p = data();
    const_pointer __r = util::find_first_of(__p + __pos, __p + __sz, __s,
                                            __s + __n, __traits_eq<traits_type>());
    if (__r == __p + __sz)
        return npos;
    return static_cast<size_type>(__r - __p);
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find_first_of(const basic_string& __str,
                                                         size_type __pos) const
{
    return find_first_of(__str.data(), __pos, __str.size());
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find_first_of(const_pointer __s,
                                                         size_type __pos) const
{
    device_assert(__s != 0);
    return find_first_of(__s, __pos, traits_type::length(__s));
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find_first_of(value_type __c,
                                                         size_type __pos) const
{
    return find(__c, __pos);
}

// find_last_of

template<class _CharT, class _Traits, class _Allocator>
__device__ typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find_last_of(const_pointer __s,
                                                        size_type __pos,
                                                        size_type __n) const
{
    device_assert(__s != 0);
    if (__n != 0)
    {
        size_type __sz = size();
        if (__pos < __sz)
            ++__pos;
        else
            __pos = __sz;
        const_pointer __p = data();
        for (const_pointer __ps = __p + __pos; __ps != __p;)
        {
            const_pointer __r = traits_type::find(__s, __n, *--__ps);
            if (__r)
                return static_cast<size_type>(__ps - __p);
        }
    }
    return npos;
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find_last_of(const basic_string& __str,
                                                        size_type __pos) const
{
    return find_last_of(__str.data(), __pos, __str.size());
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find_last_of(const_pointer __s,
                                                        size_type __pos) const
{
    device_assert(__s != 0);
    return find_last_of(__s, __pos, traits_type::length(__s));
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find_last_of(value_type __c,
                                                        size_type __pos) const
{
    return rfind(__c, __pos);
}

// find_first_not_of

template<class _CharT, class _Traits, class _Allocator>
__device__ typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find_first_not_of(const_pointer __s,
                                                             size_type __pos,
                                                             size_type __n) const
{
    device_assert(__s != 0);
    size_type __sz = size();
    if (__pos < __sz)
    {
        const_pointer __p = data();
        const_pointer __pe = __p + __sz;
        for (const_pointer __ps = __p + __pos; __ps != __pe; ++__ps)
            if (traits_type::find(__s, __n, *__ps) == 0)
                return static_cast<size_type>(__ps - __p);
    }
    return npos;
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find_first_not_of(const basic_string& __str,
                                                             size_type __pos) const
{
    return find_first_not_of(__str.data(), __pos, __str.size());
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find_first_not_of(const_pointer __s,
                                                             size_type __pos) const
{
    device_assert(__s != 0);
    return find_first_not_of(__s, __pos, traits_type::length(__s));
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find_first_not_of(value_type __c,
                                                             size_type __pos) const
{
    size_type __sz = size();
    if (__pos < __sz)
    {
        const_pointer __p = data();
        const_pointer __pe = __p + __sz;
        for (const_pointer __ps = __p + __pos; __p != __pe; ++__ps)
            if (!traits_type::eq(*__ps, __c))
                return static_cast<size_type>(__ps - __p);
    }
    return npos;
}

// find_last_not_of

template<class _CharT, class _Traits, class _Allocator>
__device__ typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find_last_not_of(const_pointer __s,
                                                            size_type __pos,
                                                            size_type __n) const
{
    device_assert(__s != 0);
    size_type __sz = size();
    if (__pos < __sz)
        ++__pos;
    else
        __pos = __sz;
    const_pointer __p = data();
    for (const_pointer __ps = __p + __pos; __ps != __p;)
        if (traits_type::find(__s, __n, *--__ps) == 0)
            return static_cast<size_type>(__ps - __p);
    return npos;
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find_last_not_of(const basic_string& __str,
                                                            size_type __pos) const
{
    return find_last_not_of(__str.data(), __pos, __str.size());
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find_last_not_of(const_pointer __s,
                                                            size_type __pos) const
{
    device_assert(__s != 0);
    return find_last_not_of(__s, __pos, traits_type::length(__s));
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
typename basic_string<_CharT, _Traits, _Allocator>::size_type
basic_string<_CharT, _Traits, _Allocator>::find_last_not_of(value_type __c,
                                                            size_type __pos) const
{
    size_type __sz = size();
    if (__pos < __sz)
        ++__pos;
    else
        __pos = __sz;
    const_pointer __p = data();
    for (const_pointer __ps = __p + __pos; __ps != __p;)
        if (!traits_type::eq(*--__ps, __c))
            return static_cast<size_type>(__ps - __p);
    return npos;
}

// compare

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
int
basic_string<_CharT, _Traits, _Allocator>::compare(const basic_string& __str) const
{
    size_t __lhs_sz = size();
    size_t __rhs_sz = __str.size();
    int __result = traits_type::compare(data(), __str.data(),
                                        util::min(__lhs_sz, __rhs_sz));
    if (__result != 0)
        return __result;
    if (__lhs_sz < __rhs_sz)
        return -1;
    if (__lhs_sz > __rhs_sz)
        return 1;
    return 0;
}

template <class _CharT, class _Traits, class _Allocator>
__device__ inline
int
basic_string<_CharT, _Traits, _Allocator>::compare(size_type __pos1,
                                                   size_type __n1,
                                                   const basic_string& __str) const
{
    return compare(__pos1, __n1, __str.data(), __str.size());
}

template <class _CharT, class _Traits, class _Allocator>
__device__ int
basic_string<_CharT, _Traits, _Allocator>::compare(size_type __pos1,
                                                   size_type __n1,
                                                   const basic_string& __str,
                                                   size_type __pos2,
                                                   size_type __n2) const
{
    size_type __sz = __str.size();
    if (__pos2 > __sz)
        this->__throw_out_of_range();
    return compare(__pos1, __n1, __str.data() + __pos2, util::min(__n2,
                                                                  __sz - __pos2));
}

template <class _CharT, class _Traits, class _Allocator>
__device__ int
basic_string<_CharT, _Traits, _Allocator>::compare(const_pointer __s) const
{
    device_assert(__s != 0);
    return compare(0, npos, __s, traits_type::length(__s));
}

template <class _CharT, class _Traits, class _Allocator>
__device__ int
basic_string<_CharT, _Traits, _Allocator>::compare(size_type __pos1,
                                                   size_type __n1,
                                                   const_pointer __s) const
{
    device_assert(__s != 0);
    return compare(__pos1, __n1, __s, traits_type::length(__s));
}

template <class _CharT, class _Traits, class _Allocator>
__device__ int
basic_string<_CharT, _Traits, _Allocator>::compare(size_type __pos1,
                                                   size_type __n1,
                                                   const_pointer __s,
                                                   size_type __n2) const
{
    device_assert(__s != 0);
    size_type __sz = size();
    if (__pos1 > __sz || __n2 == npos)
        this->__throw_out_of_range();
    size_type __rlen = util::min(__n1, __sz - __pos1);
    int __r = traits_type::compare(data() + __pos1, __s, util::min(__rlen, __n2));
    if (__r == 0)
    {
        if (__rlen < __n2)
            __r = -1;
        else if (__rlen > __n2)
            __r = 1;
    }
    return __r;
}

// __invariants

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
basic_string<_CharT, _Traits, _Allocator>::__invariants() const
{
    if (size() > capacity())
        return false;
    if (capacity() < __min_cap - 1)
        return false;
    if (data() == 0)
        return false;
    if (data()[size()] != value_type(0))
        return false;
    return true;
}

// operator==

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
operator==(const basic_string<_CharT, _Traits, _Allocator>& __lhs,
           const basic_string<_CharT, _Traits, _Allocator>& __rhs)
{
    return __lhs.size() == __rhs.size() && _Traits::compare(__lhs.data(),
                                                            __rhs.data(),
                                                            __lhs.size()) == 0;
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
operator==(const _CharT* __lhs,
           const basic_string<_CharT, _Traits, _Allocator>& __rhs)
{
    return __rhs.compare(__lhs) == 0;
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
operator==(const basic_string<_CharT,_Traits,_Allocator>& __lhs,
           const _CharT* __rhs)
{
    return __lhs.compare(__rhs) == 0;
}

// operator!=

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
operator!=(const basic_string<_CharT,_Traits,_Allocator>& __lhs,
           const basic_string<_CharT, _Traits, _Allocator>& __rhs)
{
    return !(__lhs == __rhs);
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
operator!=(const _CharT* __lhs,
           const basic_string<_CharT, _Traits, _Allocator>& __rhs)
{
    return !(__lhs == __rhs);
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
operator!=(const basic_string<_CharT, _Traits, _Allocator>& __lhs,
           const _CharT* __rhs)
{
    return !(__lhs == __rhs);
}

// operator<

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
operator< (const basic_string<_CharT, _Traits, _Allocator>& __lhs,
           const basic_string<_CharT, _Traits, _Allocator>& __rhs)
{
    return __lhs.compare(__rhs) < 0;
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
operator< (const basic_string<_CharT, _Traits, _Allocator>& __lhs,
           const _CharT* __rhs)
{
    return __lhs.compare(__rhs) < 0;
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
operator< (const _CharT* __lhs,
           const basic_string<_CharT, _Traits, _Allocator>& __rhs)
{
    return __rhs.compare(__lhs) > 0;
}

// operator>

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
operator> (const basic_string<_CharT, _Traits, _Allocator>& __lhs,
           const basic_string<_CharT, _Traits, _Allocator>& __rhs)
{
    return __rhs < __lhs;
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
operator> (const basic_string<_CharT, _Traits, _Allocator>& __lhs,
           const _CharT* __rhs)
{
    return __rhs < __lhs;
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
operator> (const _CharT* __lhs,
           const basic_string<_CharT, _Traits, _Allocator>& __rhs)
{
    return __rhs < __lhs;
}

// operator<=

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
operator<=(const basic_string<_CharT, _Traits, _Allocator>& __lhs,
           const basic_string<_CharT, _Traits, _Allocator>& __rhs)
{
    return !(__rhs < __lhs);
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
operator<=(const basic_string<_CharT, _Traits, _Allocator>& __lhs,
           const _CharT* __rhs)
{
    return !(__rhs < __lhs);
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
operator<=(const _CharT* __lhs,
           const basic_string<_CharT, _Traits, _Allocator>& __rhs)
{
    return !(__rhs < __lhs);
}

// operator>=

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
operator>=(const basic_string<_CharT, _Traits, _Allocator>& __lhs,
           const basic_string<_CharT, _Traits, _Allocator>& __rhs)
{
    return !(__lhs < __rhs);
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
operator>=(const basic_string<_CharT, _Traits, _Allocator>& __lhs,
           const _CharT* __rhs)
{
    return !(__lhs < __rhs);
}

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
bool
operator>=(const _CharT* __lhs,
           const basic_string<_CharT, _Traits, _Allocator>& __rhs)
{
    return !(__lhs < __rhs);
}

// operator +

template<class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>
operator+(const basic_string<_CharT, _Traits, _Allocator>& __lhs,
          const basic_string<_CharT, _Traits, _Allocator>& __rhs)
{
    basic_string<_CharT, _Traits, _Allocator> __r(__lhs.get_allocator());
    typename basic_string<_CharT, _Traits, _Allocator>::size_type __lhs_sz = __lhs.size();
    typename basic_string<_CharT, _Traits, _Allocator>::size_type __rhs_sz = __rhs.size();
    __r.__init(__lhs.data(), __lhs_sz, __lhs_sz + __rhs_sz);
    __r.append(__rhs.data(), __rhs_sz);
    return __r;
}

template<class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>
operator+(const _CharT* __lhs , const basic_string<_CharT,_Traits,_Allocator>& __rhs)
{
    basic_string<_CharT, _Traits, _Allocator> __r(__rhs.get_allocator());
    typename basic_string<_CharT, _Traits, _Allocator>::size_type __lhs_sz = _Traits::length(__lhs);
    typename basic_string<_CharT, _Traits, _Allocator>::size_type __rhs_sz = __rhs.size();
    __r.__init(__lhs, __lhs_sz, __lhs_sz + __rhs_sz);
    __r.append(__rhs.data(), __rhs_sz);
    return __r;
}

template<class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>
operator+(_CharT __lhs, const basic_string<_CharT,_Traits,_Allocator>& __rhs)
{
    basic_string<_CharT, _Traits, _Allocator> __r(__rhs.get_allocator());
    typename basic_string<_CharT, _Traits, _Allocator>::size_type __rhs_sz = __rhs.size();
    __r.__init(&__lhs, 1, 1 + __rhs_sz);
    __r.append(__rhs.data(), __rhs_sz);
    return __r;
}

template<class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>
operator+(const basic_string<_CharT, _Traits, _Allocator>& __lhs, const _CharT* __rhs)
{
    basic_string<_CharT, _Traits, _Allocator> __r(__lhs.get_allocator());
    typename basic_string<_CharT, _Traits, _Allocator>::size_type __lhs_sz = __lhs.size();
    typename basic_string<_CharT, _Traits, _Allocator>::size_type __rhs_sz = _Traits::length(__rhs);
    __r.__init(__lhs.data(), __lhs_sz, __lhs_sz + __rhs_sz);
    __r.append(__rhs, __rhs_sz);
    return __r;
}

template<class _CharT, class _Traits, class _Allocator>
__device__ basic_string<_CharT, _Traits, _Allocator>
operator+(const basic_string<_CharT, _Traits, _Allocator>& __lhs, _CharT __rhs)
{
    basic_string<_CharT, _Traits, _Allocator> __r(__lhs.get_allocator());
    typename basic_string<_CharT, _Traits, _Allocator>::size_type __lhs_sz = __lhs.size();
    __r.__init(__lhs.data(), __lhs_sz, __lhs_sz + 1);
    __r.push_back(__rhs);
    return __r;
}

// swap

template<class _CharT, class _Traits, class _Allocator>
__device__ inline
void
swap(basic_string<_CharT, _Traits, _Allocator>& __lhs,
     basic_string<_CharT, _Traits, _Allocator>& __rhs)
{
    __lhs.swap(__rhs);
}

typedef basic_string<char>    string;

}

}

#endif /* GPUSTRING_H_ */
