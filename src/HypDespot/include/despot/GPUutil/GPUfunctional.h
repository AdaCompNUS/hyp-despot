/*
 * GPUfunctional.h
 *
 *  Created on: 28 Mar, 2017
 *      Author: panpan
 */

#ifndef GPUFUNCTIONAL_H_
#define GPUFUNCTIONAL_H_

/*	\file   functional.h
	\author Gregory Diamos <solusstultus@gmail.com>
	\date   November 14, 2012
	\brief  The header file for the functional operations.
*/

#pragma once
#include <despot/GPUcore/CudaInclude.h>

namespace archaeopteryx
{

namespace util
{

template <class _Arg, class _Result>
struct unary_function
{
    typedef _Arg    argument_type;
    typedef _Result result_type;
};

template <class _Arg1, class _Arg2, class _Result>
struct binary_function
{
    typedef _Arg1   first_argument_type;
    typedef _Arg2   second_argument_type;
    typedef _Result result_type;
};

template <class _Tp> struct hash;

template <class _Tp>
struct __has_result_type
{
private:
    struct __two {char __lx; char __lxx;};
    template <class _Up> static __two __test(...);
    template <class _Up> static char __test(typename _Up::result_type* = 0);
public:
    static const bool value = sizeof(__test<_Tp>(0)) == 1;
};

template <class _Tp>
struct less : binary_function<_Tp, _Tp, bool>
{
    DEVICE bool operator()(const _Tp& __x, const _Tp& __y) const
        {return __x < __y;}
};

template <class _Tp>
struct plus : binary_function<_Tp, _Tp, _Tp>
{
    DEVICE _Tp operator()(const _Tp& __x, const _Tp& __y) const
        {return __x + __y;}
};

template <class _Tp>
struct minus : binary_function<_Tp, _Tp, _Tp>
{
    DEVICE _Tp operator()(const _Tp& __x, const _Tp& __y) const
        {return __x - __y;}
};

template <class _Tp>
struct multiplies : binary_function<_Tp, _Tp, _Tp>
{
    DEVICE _Tp operator()(const _Tp& __x, const _Tp& __y) const
        {return __x * __y;}
};

template <class _Tp>
struct divides : binary_function<_Tp, _Tp, _Tp>
{
    DEVICE _Tp operator()(const _Tp& __x, const _Tp& __y) const
        {return __x / __y;}
};

template <class _Tp>
struct modulus : binary_function<_Tp, _Tp, _Tp>
{
    DEVICE _Tp operator()(const _Tp& __x, const _Tp& __y) const
        {return __x % __y;}
};

template <class _Tp>
struct negate : unary_function<_Tp, _Tp>
{
    DEVICE _Tp operator()(const _Tp& __x) const
        {return -__x;}
};

template <class _Tp>
struct equal_to : binary_function<_Tp, _Tp, bool>
{
    DEVICE bool operator()(const _Tp& __x, const _Tp& __y) const
        {return __x == __y;}
};

template <class _Tp>
struct not_equal_to : binary_function<_Tp, _Tp, bool>
{
    DEVICE bool operator()(const _Tp& __x, const _Tp& __y) const
        {return __x != __y;}
};

template <class _Tp>
struct greater : binary_function<_Tp, _Tp, bool>
{
    DEVICE bool operator()(const _Tp& __x, const _Tp& __y) const
        {return __x > __y;}
};

// less in <__functional_base>

template <class _Tp>
struct greater_equal : binary_function<_Tp, _Tp, bool>
{
    DEVICE bool operator()(const _Tp& __x, const _Tp& __y) const
        {return __x >= __y;}
};

template <class _Tp>
struct less_equal : binary_function<_Tp, _Tp, bool>
{
    DEVICE bool operator()(const _Tp& __x, const _Tp& __y) const
        {return __x <= __y;}
};

template <class _Tp>
struct logical_and : binary_function<_Tp, _Tp, bool>
{
    DEVICE bool operator()(const _Tp& __x, const _Tp& __y) const
        {return __x && __y;}
};

template <class _Tp>
struct logical_or : binary_function<_Tp, _Tp, bool>
{
    DEVICE bool operator()(const _Tp& __x, const _Tp& __y) const
        {return __x || __y;}
};

template <class _Tp>
struct logical_not : unary_function<_Tp, bool>
{
    DEVICE bool operator()(const _Tp& __x) const
        {return !__x;}
};

template <class _Tp>
struct bit_and : binary_function<_Tp, _Tp, _Tp>
{
    DEVICE _Tp operator()(const _Tp& __x, const _Tp& __y) const
        {return __x & __y;}
};

template <class _Tp>
struct bit_or : binary_function<_Tp, _Tp, _Tp>
{
    DEVICE _Tp operator()(const _Tp& __x, const _Tp& __y) const
        {return __x | __y;}
};

template <class _Tp>
struct bit_xor : binary_function<_Tp, _Tp, _Tp>
{
    DEVICE _Tp operator()(const _Tp& __x, const _Tp& __y) const
        {return __x ^ __y;}
};

template <class _Predicate>
class unary_negate
    : public unary_function<typename _Predicate::argument_type, bool>
{
    _Predicate __pred_;
public:
    DEVICE explicit unary_negate(const _Predicate& __pred)
        : __pred_(__pred) {}
    DEVICE bool operator()(const typename _Predicate::argument_type& __x) const
        {return !__pred_(__x);}
};

template <class _Predicate>
DEVICE inline unary_negate<_Predicate>
not1(const _Predicate& __pred) {return unary_negate<_Predicate>(__pred);}

template <class _Predicate>
class binary_negate
    : public binary_function<typename _Predicate::first_argument_type,
                             typename _Predicate::second_argument_type,
                             bool>
{
    _Predicate __pred_;
public:
    DEVICE explicit binary_negate(const _Predicate& __pred)
        : __pred_(__pred) {}
    DEVICE bool operator()(const typename _Predicate::first_argument_type& __x,
                    const typename _Predicate::second_argument_type& __y) const
        {return !__pred_(__x, __y);}
};

template <class _Predicate>
DEVICE inline binary_negate<_Predicate>
not2(const _Predicate& __pred) {return binary_negate<_Predicate>(__pred);}

template <class __Operation>
class binder1st
    : public unary_function<typename __Operation::second_argument_type,
                            typename __Operation::result_type>
{
protected:
    __Operation                               op;
    typename __Operation::first_argument_type value;
public:
    DEVICE binder1st(const __Operation& __x,
                               const typename __Operation::first_argument_type __y)
        : op(__x), value(__y) {}
    DEVICE typename __Operation::result_type operator()
        (typename __Operation::second_argument_type& __x) const
            {return op(value, __x);}
    DEVICE typename __Operation::result_type operator()
        (const typename __Operation::second_argument_type& __x) const
            {return op(value, __x);}
};

template <class __Operation, class _Tp>
DEVICE inline binder1st<__Operation>
bind1st(const __Operation& __op, const _Tp& __x)
    {return binder1st<__Operation>(__op, __x);}

template <class __Operation>
class binder2nd
    : public unary_function<typename __Operation::first_argument_type,
                            typename __Operation::result_type>
{
protected:
    __Operation                                op;
    typename __Operation::second_argument_type value;
public:
    DEVICE binder2nd(const __Operation& __x, const typename __Operation::second_argument_type __y)
        : op(__x), value(__y) {}
    DEVICE typename __Operation::result_type operator()
        (      typename __Operation::first_argument_type& __x) const
            {return op(__x, value);}
    DEVICE typename __Operation::result_type operator()
        (const typename __Operation::first_argument_type& __x) const
            {return op(__x, value);}
};

template <class __Operation, class _Tp>
DEVICE inline binder2nd<__Operation>
bind2nd(const __Operation& __op, const _Tp& __x)
    {return binder2nd<__Operation>(__op, __x);}

template <class _Arg, class _Result>
class pointer_to_unary_function
    : public unary_function<_Arg, _Result>
{
    _Result (*__f_)(_Arg);
public:
    DEVICE explicit pointer_to_unary_function(_Result (*__f)(_Arg))
        : __f_(__f) {}
    DEVICE _Result operator()(_Arg __x) const
        {return __f_(__x);}
};

template <class _Arg, class _Result>
DEVICE inline pointer_to_unary_function<_Arg,_Result>
ptr_fun(_Result (*__f)(_Arg))
    {return pointer_to_unary_function<_Arg,_Result>(__f);}

template <class _Arg1, class _Arg2, class _Result>
class pointer_to_binary_function
    : public binary_function<_Arg1, _Arg2, _Result>
{
    _Result (*__f_)(_Arg1, _Arg2);
public:
    DEVICE explicit pointer_to_binary_function(_Result (*__f)(_Arg1, _Arg2))
        : __f_(__f) {}
    DEVICE _Result operator()(_Arg1 __x, _Arg2 __y) const
        {return __f_(__x, __y);}
};

template <class _Arg1, class _Arg2, class _Result>
DEVICE inline pointer_to_binary_function<_Arg1,_Arg2,_Result>
ptr_fun(_Result (*__f)(_Arg1,_Arg2))
    {return pointer_to_binary_function<_Arg1,_Arg2,_Result>(__f);}

template<class _Sp, class _Tp>
class mem_fun_t : public unary_function<_Tp*, _Sp>
{
    _Sp (_Tp::*__p_)();
public:
    DEVICE explicit mem_fun_t(_Sp (_Tp::*__p)())
        : __p_(__p) {}
    DEVICE _Sp operator()(_Tp* __p) const
        {return (__p->*__p_)();}
};

template<class _Sp, class _Tp, class _Ap>
class mem_fun1_t : public binary_function<_Tp*, _Ap, _Sp>
{
    _Sp (_Tp::*__p_)(_Ap);
public:
    DEVICE explicit mem_fun1_t(_Sp (_Tp::*__p)(_Ap))
        : __p_(__p) {}
    DEVICE _Sp operator()(_Tp* __p, _Ap __x) const
        {return (__p->*__p_)(__x);}
};

template<class _Sp, class _Tp>
DEVICE inline mem_fun_t<_Sp,_Tp>
mem_fun(_Sp (_Tp::*__f)())
    {return mem_fun_t<_Sp,_Tp>(__f);}

template<class _Sp, class _Tp, class _Ap>
DEVICE inline mem_fun1_t<_Sp,_Tp,_Ap>
mem_fun(_Sp (_Tp::*__f)(_Ap))
    {return mem_fun1_t<_Sp,_Tp,_Ap>(__f);}

template<class _Sp, class _Tp>
class mem_fun_ref_t : public unary_function<_Tp, _Sp>
{
    _Sp (_Tp::*__p_)();
public:
    DEVICE explicit mem_fun_ref_t(_Sp (_Tp::*__p)())
        : __p_(__p) {}
    DEVICE _Sp operator()(_Tp& __p) const
        {return (__p.*__p_)();}
};

template<class _Sp, class _Tp, class _Ap>
class mem_fun1_ref_t : public binary_function<_Tp, _Ap, _Sp>
{
    _Sp (_Tp::*__p_)(_Ap);
public:
    DEVICE explicit mem_fun1_ref_t(_Sp (_Tp::*__p)(_Ap))
        : __p_(__p) {}
    DEVICE _Sp operator()(_Tp& __p, _Ap __x) const
        {return (__p.*__p_)(__x);}
};

template<class _Sp, class _Tp>
DEVICE inline mem_fun_ref_t<_Sp,_Tp>
mem_fun_ref(_Sp (_Tp::*__f)())
    {return mem_fun_ref_t<_Sp,_Tp>(__f);}

template<class _Sp, class _Tp, class _Ap>
DEVICE inline mem_fun1_ref_t<_Sp,_Tp,_Ap>
mem_fun_ref(_Sp (_Tp::*__f)(_Ap))
    {return mem_fun1_ref_t<_Sp,_Tp,_Ap>(__f);}

template <class _Sp, class _Tp>
class const_mem_fun_t : public unary_function<const _Tp*, _Sp>
{
    _Sp (_Tp::*__p_)() const;
public:
    DEVICE explicit const_mem_fun_t(_Sp (_Tp::*__p)() const)
        : __p_(__p) {}
    DEVICE _Sp operator()(const _Tp* __p) const
        {return (__p->*__p_)();}
};

template <class _Sp, class _Tp, class _Ap>
class const_mem_fun1_t : public binary_function<const _Tp*, _Ap, _Sp>
{
    _Sp (_Tp::*__p_)(_Ap) const;
public:
    DEVICE explicit const_mem_fun1_t(_Sp (_Tp::*__p)(_Ap) const)
        : __p_(__p) {}
    DEVICE _Sp operator()(const _Tp* __p, _Ap __x) const
        {return (__p->*__p_)(__x);}
};

template <class _Sp, class _Tp>
DEVICE inline const_mem_fun_t<_Sp,_Tp>
mem_fun(_Sp (_Tp::*__f)() const)
    {return const_mem_fun_t<_Sp,_Tp>(__f);}

template <class _Sp, class _Tp, class _Ap>
DEVICE inline const_mem_fun1_t<_Sp,_Tp,_Ap>
mem_fun(_Sp (_Tp::*__f)(_Ap) const)
    {return const_mem_fun1_t<_Sp,_Tp,_Ap>(__f);}

template <class _Sp, class _Tp>
class const_mem_fun_ref_t : public unary_function<_Tp, _Sp>
{
    _Sp (_Tp::*__p_)() const;
public:
    DEVICE explicit const_mem_fun_ref_t(_Sp (_Tp::*__p)() const)
        : __p_(__p) {}
    DEVICE _Sp operator()(const _Tp& __p) const
        {return (__p.*__p_)();}
};

template <class _Sp, class _Tp, class _Ap>
class const_mem_fun1_ref_t
    : public binary_function<_Tp, _Ap, _Sp>
{
    _Sp (_Tp::*__p_)(_Ap) const;
public:
    DEVICE explicit const_mem_fun1_ref_t(_Sp (_Tp::*__p)(_Ap) const)
        : __p_(__p) {}
    DEVICE _Sp operator()(const _Tp& __p, _Ap __x) const
        {return (__p.*__p_)(__x);}
};

template <class _Sp, class _Tp>
DEVICE inline const_mem_fun_ref_t<_Sp,_Tp>
mem_fun_ref(_Sp (_Tp::*__f)() const)
    {return const_mem_fun_ref_t<_Sp,_Tp>(__f);}

template <class _Sp, class _Tp, class _Ap>
DEVICE inline const_mem_fun1_ref_t<_Sp,_Tp,_Ap>
mem_fun_ref(_Sp (_Tp::*__f)(_Ap) const)
    {return const_mem_fun1_ref_t<_Sp,_Tp,_Ap>(__f);}

template <>
struct hash<bool>
    : public unary_function<bool, size_t>
{
        DEVICE size_t operator()(bool __v) const {return static_cast<size_t>(__v);}
};

template <>
struct hash<char>
    : public unary_function<char, size_t>
{
        DEVICE size_t operator()(char __v) const {return static_cast<size_t>(__v);}
};

template <>
struct hash<signed char>
    : public unary_function<signed char, size_t>
{
        size_t operator()(signed char __v) const {return static_cast<size_t>(__v);}
};

template <>
struct hash<unsigned char>
    : public unary_function<unsigned char, size_t>
{
        DEVICE size_t operator()(unsigned char __v) const {return static_cast<size_t>(__v);}
};

template <>
struct hash<wchar_t>
    : public unary_function<wchar_t, size_t>
{
        DEVICE size_t operator()(wchar_t __v) const {return static_cast<size_t>(__v);}
};

template <>
struct hash<short>
    : public unary_function<short, size_t>
{
        DEVICE size_t operator()(short __v) const {return static_cast<size_t>(__v);}
};

template <>
struct hash<unsigned short>
    : public unary_function<unsigned short, size_t>
{
        DEVICE size_t operator()(unsigned short __v) const {return static_cast<size_t>(__v);}
};

template <>
struct hash<int>
    : public unary_function<int, size_t>
{
        DEVICE size_t operator()(int __v) const {return static_cast<size_t>(__v);}
};

template <>
struct hash<unsigned int>
    : public unary_function<unsigned int, size_t>
{
        DEVICE size_t operator()(unsigned int __v) const {return static_cast<size_t>(__v);}
};

template <>
struct hash<long>
    : public unary_function<long, size_t>
{
        DEVICE size_t operator()(long __v) const {return static_cast<size_t>(__v);}
};

template <>
struct hash<unsigned long>
    : public unary_function<unsigned long, size_t>
{
        DEVICE size_t operator()(unsigned long __v) const {return static_cast<size_t>(__v);}
};

}

}



#endif /* GPUFUNCTIONAL_H_ */
