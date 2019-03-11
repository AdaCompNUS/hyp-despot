/*
 * GPUallocator_traits.h
 *
 *  Created on: 28 Mar, 2017
 *      Author: panpan
 */

#ifndef GPUALLOCATOR_TRAITS_H_
#define GPUALLOCATOR_TRAITS_H_

/*! \file   allocator_traits.h
	\date   Thursday November 15, 2012
	\author Gregory Diamos <gregory.diamos@gatech.edu>
	\brief  The header file for allocator_traits class.
*/

#pragma once

// Archaeopteryx Includes
#include <despot/GPUutil/GPUfunctional.h>
#include <despot/GPUutil/GPUiterator.h>
#include <despot/GPUutil/GPUutility.h>
#include <despot/GPUutil/GPUlimits.h>

#include <despot/GPUutil/GPUcstring.h>
#include <despot/GPUcore/CudaInclude.h>


namespace archaeopteryx
{
namespace util
{

template <class _Tp> class allocator;

template <>
class allocator<void>
{
public:
    typedef void*             pointer;
    typedef const void*       const_pointer;
    typedef void              value_type;

    template <class _Up> struct rebind {typedef allocator<_Up> other;};
};

template <>
class allocator<const void>
{
public:
    typedef const void*       pointer;
    typedef const void*       const_pointer;
    typedef const void        value_type;

    template <class _Up> struct rebind {typedef allocator<_Up> other;};
};

struct allocator_arg_t { };

//allocator_arg_t allocator_arg = allocator_arg_t();

template <class T, class Alloc> struct uses_allocator;

// pointer_traits
template <class _Tp>
struct __has_element_type
{
private:
    struct __two {char __lx; char __lxx;};
    template <class _Up> static __two __test(...);
    template <class _Up> static char __test(typename _Up::element_type* = 0);
public:
    static const bool value = sizeof(__test<_Tp>(0)) == 1;
};

template <class _Ptr, bool = __has_element_type<_Ptr>::value>
struct __pointer_traits_element_type;

template <class _Ptr>
struct __pointer_traits_element_type<_Ptr, true>
{
    typedef typename _Ptr::element_type type;
};

template <template <class> class _Sp, class _Tp>
struct __pointer_traits_element_type<_Sp<_Tp>, true>
{
    typedef typename _Sp<_Tp>::element_type type;
};

template <template <class> class _Sp, class _Tp>
struct __pointer_traits_element_type<_Sp<_Tp>, false>
{
    typedef _Tp type;
};

template <template <class, class> class _Sp, class _Tp, class _A0>
struct __pointer_traits_element_type<_Sp<_Tp, _A0>, true>
{
    typedef typename _Sp<_Tp, _A0>::element_type type;
};

template <template <class, class> class _Sp, class _Tp, class _A0>
struct __pointer_traits_element_type<_Sp<_Tp, _A0>, false>
{
    typedef _Tp type;
};

template <template <class, class, class> class _Sp, class _Tp, class _A0, class _A1>
struct __pointer_traits_element_type<_Sp<_Tp, _A0, _A1>, true>
{
    typedef typename _Sp<_Tp, _A0, _A1>::element_type type;
};

template <template <class, class, class> class _Sp, class _Tp, class _A0, class _A1>
struct __pointer_traits_element_type<_Sp<_Tp, _A0, _A1>, false>
{
    typedef _Tp type;
};

template <template <class, class, class, class> class _Sp, class _Tp, class _A0,
                                                           class _A1, class _A2>
struct __pointer_traits_element_type<_Sp<_Tp, _A0, _A1, _A2>, true>
{
    typedef typename _Sp<_Tp, _A0, _A1, _A2>::element_type type;
};

template <template <class, class, class, class> class _Sp, class _Tp, class _A0,
                                                           class _A1, class _A2>
struct __pointer_traits_element_type<_Sp<_Tp, _A0, _A1, _A2>, false>
{
    typedef _Tp type;
};

template <class _Tp>
struct __has_difference_type
{
private:
    struct __two {char __lx; char __lxx;};
    template <class _Up> static __two __test(...);
    template <class _Up> static char __test(typename _Up::difference_type* = 0);
public:
    static const bool value = sizeof(__test<_Tp>(0)) == 1;
};
template <class _Ptr, bool = __has_difference_type<_Ptr>::value>
struct __pointer_traits_difference_type
{
    typedef long int type;
};

template <class _Ptr>
struct __pointer_traits_difference_type<_Ptr, true>
{
    typedef typename _Ptr::difference_type type;
};

template <class _Tp, class _Up>
struct __has_rebind
{
private:
    struct __two {char __lx; char __lxx;};
    template <class _Xp> static __two __test(...);
    template <class _Xp> static char __test(typename _Xp::template rebind<_Up>* = 0);
public:
    static const bool value = sizeof(__test<_Tp>(0)) == 1;
};

template <class _Tp, class _Up, bool = __has_rebind<_Tp, _Up>::value>
struct __pointer_traits_rebind
{
    typedef typename _Tp::template rebind<_Up>::other type;
};

template <template <class> class _Sp, class _Tp, class _Up>
struct __pointer_traits_rebind<_Sp<_Tp>, _Up, true>
{
    typedef typename _Sp<_Tp>::template rebind<_Up>::other type;
};

template <template <class> class _Sp, class _Tp, class _Up>
struct __pointer_traits_rebind<_Sp<_Tp>, _Up, false>
{
    typedef _Sp<_Up> type;
};

template <template <class, class> class _Sp, class _Tp, class _A0, class _Up>
struct __pointer_traits_rebind<_Sp<_Tp, _A0>, _Up, true>
{
    typedef typename _Sp<_Tp, _A0>::template rebind<_Up>::other type;
};

template <template <class, class> class _Sp, class _Tp, class _A0, class _Up>
struct __pointer_traits_rebind<_Sp<_Tp, _A0>, _Up, false>
{
    typedef _Sp<_Up, _A0> type;
};

template <template <class, class, class> class _Sp, class _Tp, class _A0,
                                         class _A1, class _Up>
struct __pointer_traits_rebind<_Sp<_Tp, _A0, _A1>, _Up, true>
{
    typedef typename _Sp<_Tp, _A0, _A1>::template rebind<_Up>::other type;
};

template <template <class, class, class> class _Sp, class _Tp, class _A0,
                                         class _A1, class _Up>
struct __pointer_traits_rebind<_Sp<_Tp, _A0, _A1>, _Up, false>
{
    typedef _Sp<_Up, _A0, _A1> type;
};

template <template <class, class, class, class> class _Sp, class _Tp, class _A0,
                                                class _A1, class _A2, class _Up>
struct __pointer_traits_rebind<_Sp<_Tp, _A0, _A1, _A2>, _Up, true>
{
    typedef typename _Sp<_Tp, _A0, _A1, _A2>::template rebind<_Up>::other type;
};

template <template <class, class, class, class> class _Sp, class _Tp, class _A0,
                                                class _A1, class _A2, class _Up>
struct __pointer_traits_rebind<_Sp<_Tp, _A0, _A1, _A2>, _Up, false>
{
    typedef _Sp<_Up, _A0, _A1, _A2> type;
};

template <class _Ptr>
struct pointer_traits
{
    typedef _Ptr                                                     pointer;
    typedef typename __pointer_traits_element_type<pointer>::type    element_type;
    typedef typename __pointer_traits_difference_type<pointer>::type difference_type;

    template <class _Up> struct rebind
        {typedef typename __pointer_traits_rebind<pointer, _Up>::type other;};

private:
    struct __nat {};
public:
    DEVICE static pointer pointer_to(typename conditional<is_void<element_type>::value,
                                           __nat, element_type>::type& __r)
        {return pointer::pointer_to(__r);}
};

template <class _Tp>
struct pointer_traits<_Tp*>
{
    typedef _Tp*      pointer;
    typedef _Tp       element_type;
    typedef long int difference_type;

    template <class _Up> struct rebind {typedef _Up* other;};

private:
    struct __nat {};
public:
    DEVICE static pointer pointer_to(typename conditional<is_void<element_type>::value,
                                      __nat, element_type>::type& __r)
        {return addressof(__r);}
};

// allocator traits

namespace __has_pointer_type_imp
{
    template <class _Up> static __two test(...);
    template <class _Up> static char test(typename _Up::pointer* = 0);
}

template <class _Tp>
struct __has_pointer_type
    : public integral_constant<bool, sizeof(__has_pointer_type_imp::test<_Tp>(0)) == 1>
{
};

namespace __pointer_type_imp
{

template <class _Tp, class _Dp, bool = __has_pointer_type<_Dp>::value>
struct __pointer_type
{
    typedef typename _Dp::pointer type;
};

template <class _Tp, class _Dp>
struct __pointer_type<_Tp, _Dp, false>
{
    typedef _Tp* type;
};

}  // __pointer_type_imp

template <class _Tp, class _Dp>
struct __pointer_type
{
    typedef typename __pointer_type_imp::__pointer_type<_Tp, typename remove_reference<_Dp>::type>::type type;
};

template <class _Tp>
struct __has_const_pointer
{
private:
    struct __two {char __lx; char __lxx;};
    template <class _Up> static __two __test(...);
    template <class _Up> static char __test(typename _Up::const_pointer* = 0);
public:
    static const bool value = sizeof(__test<_Tp>(0)) == 1;
};

template <class _Tp, class _Ptr, class _Alloc, bool = __has_const_pointer<_Alloc>::value>
struct __const_pointer
{
    typedef typename _Alloc::const_pointer type;
};

template <class _Tp, class _Ptr, class _Alloc>
struct __const_pointer<_Tp, _Ptr, _Alloc, false>
{
    typedef typename pointer_traits<_Ptr>::template rebind<const _Tp>::other type;
};

template <class _Tp>
struct __has_void_pointer
{
private:
    struct __two {char __lx; char __lxx;};
    template <class _Up> static __two __test(...);
    template <class _Up> static char __test(typename _Up::void_pointer* = 0);
public:
    static const bool value = sizeof(__test<_Tp>(0)) == 1;
};

template <class _Ptr, class _Alloc, bool = __has_void_pointer<_Alloc>::value>
struct __void_pointer
{
    typedef typename _Alloc::void_pointer type;
};

template <class _Ptr, class _Alloc>
struct __void_pointer<_Ptr, _Alloc, false>
{
    typedef typename pointer_traits<_Ptr>::template rebind<void>::other type;
};

template <class _Tp>
struct __has_const_void_pointer
{
private:
    struct __two {char __lx; char __lxx;};
    template <class _Up> static __two __test(...);
    template <class _Up> static char __test(typename _Up::const_void_pointer* = 0);
public:
    static const bool value = sizeof(__test<_Tp>(0)) == 1;
};

template <class _Ptr, class _Alloc, bool = __has_const_void_pointer<_Alloc>::value>
struct __const_void_pointer
{
    typedef typename _Alloc::const_void_pointer type;
};

template <class _Ptr, class _Alloc>
struct __const_void_pointer<_Ptr, _Alloc, false>
{
    typedef typename pointer_traits<_Ptr>::template rebind<const void>::other type;
};

template <class _Tp>
DEVICE inline _Tp*
__to_raw_pointer(_Tp* __p)
{
    return __p;
}

template <class _Pointer>
DEVICE inline typename pointer_traits<_Pointer>::element_type*
__to_raw_pointer(_Pointer __p)
{
    return util::__to_raw_pointer(__p.operator->());
}

template <class _Tp>
struct __has_size_type
{
private:
    struct __two {char __lx; char __lxx;};
    template <class _Up> static __two __test(...);
    template <class _Up> static char __test(typename _Up::size_type* = 0);
public:
    static const bool value = sizeof(__test<_Tp>(0)) == 1;
};

template <class _Alloc, class _DiffType, bool = __has_size_type<_Alloc>::value>
struct __size_type
{
    typedef typename make_unsigned<_DiffType>::type type;
};

template <class _Alloc, class _DiffType>
struct __size_type<_Alloc, _DiffType, true>
{
    typedef typename _Alloc::size_type type;
};

template <class _Tp>
struct __has_propagate_on_container_copy_assignment
{
private:
    struct __two {char __lx; char __lxx;};
    template <class _Up> static __two __test(...);
    template <class _Up> static char __test(typename _Up::propagate_on_container_copy_assignment* = 0);
public:
    static const bool value = sizeof(__test<_Tp>(0)) == 1;
};

template <class _Alloc, bool = __has_propagate_on_container_copy_assignment<_Alloc>::value>
struct __propagate_on_container_copy_assignment
{
    typedef false_type type;
};

template <class _Alloc>
struct __propagate_on_container_copy_assignment<_Alloc, true>
{
    typedef typename _Alloc::propagate_on_container_copy_assignment type;
};

template <class _Tp>
struct __has_propagate_on_container_move_assignment
{
private:
    struct __two {char __lx; char __lxx;};
    template <class _Up> static __two __test(...);
    template <class _Up> static char __test(typename _Up::propagate_on_container_move_assignment* = 0);
public:
    static const bool value = sizeof(__test<_Tp>(0)) == 1;
};

template <class _Alloc, bool = __has_propagate_on_container_move_assignment<_Alloc>::value>
struct __propagate_on_container_move_assignment
{
    typedef false_type type;
};

template <class _Alloc>
struct __propagate_on_container_move_assignment<_Alloc, true>
{
    typedef typename _Alloc::propagate_on_container_move_assignment type;
};

template <class _Tp>
struct __has_propagate_on_container_swap
{
private:
    struct __two {char __lx; char __lxx;};
    template <class _Up> static __two __test(...);
    template <class _Up> static char __test(typename _Up::propagate_on_container_swap* = 0);
public:
    static const bool value = sizeof(__test<_Tp>(0)) == 1;
};

template <class _Alloc, bool = __has_propagate_on_container_swap<_Alloc>::value>
struct __propagate_on_container_swap
{
    typedef false_type type;
};

template <class _Alloc>
struct __propagate_on_container_swap<_Alloc, true>
{
    typedef typename _Alloc::propagate_on_container_swap type;
};

template <class _Tp, class _Up, bool = __has_rebind<_Tp, _Up>::value>
struct __has_rebind_other
{
private:
    struct __two {char __lx; char __lxx;};
    template <class _Xp> static __two __test(...);
    template <class _Xp> static char __test(typename _Xp::template rebind<_Up>::other* = 0);
public:
    static const bool value = sizeof(__test<_Tp>(0)) == 1;
};

template <class _Tp, class _Up>
struct __has_rebind_other<_Tp, _Up, false>
{
    static const bool value = false;
};

template <class _Tp, class _Up, bool = __has_rebind_other<_Tp, _Up>::value>
struct __allocator_traits_rebind
{
    typedef typename _Tp::template rebind<_Up>::other type;
};

template <template <class> class _Alloc, class _Tp, class _Up>
struct __allocator_traits_rebind<_Alloc<_Tp>, _Up, true>
{
    typedef typename _Alloc<_Tp>::template rebind<_Up>::other type;
};

template <template <class> class _Alloc, class _Tp, class _Up>
struct __allocator_traits_rebind<_Alloc<_Tp>, _Up, false>
{
    typedef _Alloc<_Up> type;
};

template <template <class, class> class _Alloc, class _Tp, class _A0, class _Up>
struct __allocator_traits_rebind<_Alloc<_Tp, _A0>, _Up, true>
{
    typedef typename _Alloc<_Tp, _A0>::template rebind<_Up>::other type;
};

template <template <class, class> class _Alloc, class _Tp, class _A0, class _Up>
struct __allocator_traits_rebind<_Alloc<_Tp, _A0>, _Up, false>
{
    typedef _Alloc<_Up, _A0> type;
};

template <template <class, class, class> class _Alloc, class _Tp, class _A0,
                                         class _A1, class _Up>
struct __allocator_traits_rebind<_Alloc<_Tp, _A0, _A1>, _Up, true>
{
    typedef typename _Alloc<_Tp, _A0, _A1>::template rebind<_Up>::other type;
};

template <template <class, class, class> class _Alloc, class _Tp, class _A0,
                                         class _A1, class _Up>
struct __allocator_traits_rebind<_Alloc<_Tp, _A0, _A1>, _Up, false>
{
    typedef _Alloc<_Up, _A0, _A1> type;
};

template <template <class, class, class, class> class _Alloc, class _Tp, class _A0,
                                                class _A1, class _A2, class _Up>
struct __allocator_traits_rebind<_Alloc<_Tp, _A0, _A1, _A2>, _Up, true>
{
    typedef typename _Alloc<_Tp, _A0, _A1, _A2>::template rebind<_Up>::other type;
};

template <template <class, class, class, class> class _Alloc, class _Tp, class _A0,
                                                class _A1, class _A2, class _Up>
struct __allocator_traits_rebind<_Alloc<_Tp, _A0, _A1, _A2>, _Up, false>
{
    typedef _Alloc<_Up, _A0, _A1, _A2> type;
};

template <class _Alloc, class _SizeType, class _ConstVoidPtr>
struct __has_allocate_hint
    : true_type
{
};

template <class _Alloc, class _Pointer, class _Args>
struct __has_construct
    : false_type
{
};


template <class _Alloc, class _Pointer>
struct __has_destroy
    : false_type
{
};

template <class _Alloc>
struct __has_max_size
    : true_type
{
};

template <class _Alloc>
struct __has_select_on_container_copy_construction
    : false_type
{
};


template <class _Alloc, class _Ptr, bool = __has_difference_type<_Alloc>::value>
struct __alloc_traits_difference_type
{
    typedef typename pointer_traits<_Ptr>::difference_type type;
};

template <class _Alloc, class _Ptr>
struct __alloc_traits_difference_type<_Alloc, _Ptr, true>
{
    typedef typename _Alloc::difference_type type;
};

template <class _Alloc>
struct allocator_traits
{
    typedef _Alloc                              allocator_type;
    typedef typename allocator_type::value_type value_type;

    typedef typename __pointer_type<value_type, allocator_type>::type pointer;
    typedef typename __const_pointer<value_type, pointer, allocator_type>::type const_pointer;
    typedef typename __void_pointer<pointer, allocator_type>::type void_pointer;
    typedef typename __const_void_pointer<pointer, allocator_type>::type const_void_pointer;

    typedef typename __alloc_traits_difference_type<allocator_type, pointer>::type difference_type;
    typedef typename __size_type<allocator_type, difference_type>::type size_type;

    typedef typename __propagate_on_container_copy_assignment<allocator_type>::type
                     propagate_on_container_copy_assignment;
    typedef typename __propagate_on_container_move_assignment<allocator_type>::type
                     propagate_on_container_move_assignment;
    typedef typename __propagate_on_container_swap<allocator_type>::type
                     propagate_on_container_swap;

    template <class _Tp> struct rebind_alloc
        {typedef typename __allocator_traits_rebind<allocator_type, _Tp>::type other;};
    template <class _Tp> struct rebind_traits
        {typedef allocator_traits<typename rebind_alloc<_Tp>::other> other;};

    DEVICE static pointer allocate(allocator_type& __a, size_type __n)
        	{return __a.allocate(__n);}
    DEVICE static pointer allocate(allocator_type& __a, size_type __n, const_void_pointer __hint)
        	{return allocate(__a, __n, __hint,
            __has_allocate_hint<allocator_type, size_type, const_void_pointer>());}

    DEVICE static void deallocate(allocator_type& __a, pointer __p, size_type __n)
        {__a.deallocate(__p, __n);}

    template <class _Tp>
                DEVICE static void construct(allocator_type& __a, _Tp* __p)
            {
                ::new ((void*)__p) _Tp();
            }
    template <class _Tp, class _A0>
                DEVICE static void construct(allocator_type& __a, _Tp* __p, const _A0& __a0)
            {
                ::new ((void*)__p) _Tp(__a0);
            }
    template <class _Tp, class _A0, class _A1>
                DEVICE static void construct(allocator_type& __a, _Tp* __p, const _A0& __a0,
                              const _A1& __a1)
            {
                ::new ((void*)__p) _Tp(__a0, __a1);
            }
    template <class _Tp, class _A0, class _A1, class _A2>
                DEVICE static void construct(allocator_type& __a, _Tp* __p, const _A0& __a0,
                              const _A1& __a1, const _A2& __a2)
            {
                ::new ((void*)__p) _Tp(__a0, __a1, __a2);
            }

    template <class _Tp>
                DEVICE static void destroy(allocator_type& __a, _Tp* __p)
            {__destroy(__has_destroy<allocator_type, _Tp*>(), __a, __p);}

       DEVICE  static size_type max_size(const allocator_type& __a)
        	{return __max_size(__has_max_size<const allocator_type>(), __a);}

        DEVICE static allocator_type
        	select_on_container_copy_construction(const allocator_type& __a)
            {return select_on_container_copy_construction(
                __has_select_on_container_copy_construction<const allocator_type>(),
                __a);}

    template <class _Ptr>
        DEVICE static void
        __construct_forward(allocator_type& __a, _Ptr __begin1, _Ptr __end1, _Ptr& __begin2)
        {
            for (; __begin1 != __end1; ++__begin1, ++__begin2)
                construct(__a, util::__to_raw_pointer(__begin2), util::move_if_noexcept(*__begin1));
        }

    template <class _Tp>
                DEVICE static
        typename enable_if
        <
            (is_same<allocator_type, allocator<_Tp> >::value
                || !__has_construct<allocator_type, _Tp*, _Tp>::value) &&
             is_trivially_move_constructible<_Tp>::value,
            void
        >::type
        __construct_forward(allocator_type& __a, _Tp* __begin1, _Tp* __end1, _Tp*& __begin2)
        {
            long int _Np = __end1 - __begin1;
            util::memcpy(__begin2, __begin1, _Np * sizeof(_Tp));
            __begin2 += _Np;
        }

    template <class _Ptr>
                DEVICE static
        void
        __construct_backward(allocator_type& __a, _Ptr __begin1, _Ptr __end1, _Ptr& __end2)
        {
            while (__end1 != __begin1)
                construct(__a, util::__to_raw_pointer(--__end2), util::move_if_noexcept(*--__end1));
        }

    template <class _Tp>
                DEVICE static
        typename enable_if
        <
            (is_same<allocator_type, allocator<_Tp> >::value
                || !__has_construct<allocator_type, _Tp*, _Tp>::value) &&
             is_trivially_move_constructible<_Tp>::value,
            void
        >::type
        __construct_backward(allocator_type& __a, _Tp* __begin1, _Tp* __end1, _Tp*& __end2)
        {
            long int _Np = __end1 - __begin1;
            __end2 -= _Np;
            util::memcpy(__end2, __begin1, _Np * sizeof(_Tp));
        }

private:

        DEVICE static pointer allocate(allocator_type& __a, size_type __n,
        	const_void_pointer __hint, true_type)
       		{return __a.allocate(__n, __hint);}
        DEVICE static pointer allocate(allocator_type& __a, size_type __n,
		    const_void_pointer, false_type)
		    {return __a.allocate(__n);}

    template <class _Tp>
                DEVICE static void __destroy(true_type, allocator_type& __a, _Tp* __p)
            {__a.destroy(__p);}
    template <class _Tp>
                DEVICE static void __destroy(false_type, allocator_type&, _Tp* __p)
            {
                __p->~_Tp();
            }

        DEVICE static size_type __max_size(true_type, const allocator_type& __a)
            {return __a.max_size();}
        DEVICE static size_type __max_size(false_type, const allocator_type&)
            {return numeric_limits<size_type>::max();}

        DEVICE static allocator_type
		    select_on_container_copy_construction(true_type, const allocator_type& __a)
		        {return __a.select_on_container_copy_construction();}
        DEVICE static allocator_type
		    select_on_container_copy_construction(false_type, const allocator_type& __a)
		        {return __a;}
};

// addressof

template <class _Tp>
DEVICE inline _Tp*
addressof(_Tp& __x)
{
    return (_Tp*)&(char&)__x;
}

// allocator

template <class _Tp>
class allocator
{
public:
    typedef size_t            size_type;
    typedef long int         difference_type;
    typedef _Tp*              pointer;
    typedef const _Tp*        const_pointer;
    typedef _Tp&              reference;
    typedef const _Tp&        const_reference;
    typedef _Tp               value_type;

    typedef true_type propagate_on_container_move_assignment;

    template <class _Up> struct rebind {typedef allocator<_Up> other;};

    DEVICE allocator() {}
    template <class _Up> allocator(const allocator<_Up>&) {}
    DEVICE pointer address(reference __x) const
        {return util::addressof(__x);}
    DEVICE const_pointer address(const_reference __x) const
        {return util::addressof(__x);}
    DEVICE pointer allocate(size_type __n, allocator<void>::const_pointer = 0)
        {return static_cast<pointer>(::operator new(__n * sizeof(_Tp)));}
    DEVICE void deallocate(pointer __p, size_type)
        {::operator delete((void*)__p);}
    DEVICE size_type max_size() const
        {return size_type(~0) / sizeof(_Tp);}
        DEVICE void
        construct(pointer __p)
        {
            ::new((void*)__p) _Tp();
        }

    template <class _A0>
        DEVICE void
        construct(pointer __p, _A0& __a0)
        {
            ::new((void*)__p) _Tp(__a0);
        }
    template <class _A0>
        DEVICE void
        construct(pointer __p, const _A0& __a0)
        {
            ::new((void*)__p) _Tp(__a0);
        }

    template <class _A0, class _A1>
        DEVICE void
        construct(pointer __p, _A0& __a0, _A1& __a1)
        {
            ::new((void*)__p) _Tp(__a0, __a1);
        }
    template <class _A0, class _A1>
        DEVICE void
        construct(pointer __p, const _A0& __a0, _A1& __a1)
        {
            ::new((void*)__p) _Tp(__a0, __a1);
        }
    template <class _A0, class _A1>
        DEVICE void
        construct(pointer __p, _A0& __a0, const _A1& __a1)
        {
            ::new((void*)__p) _Tp(__a0, __a1);
        }
    template <class _A0, class _A1>
        DEVICE void
        construct(pointer __p, const _A0& __a0, const _A1& __a1)
        {
            ::new((void*)__p) _Tp(__a0, __a1);
        }
    DEVICE void destroy(pointer __p) {__p->~_Tp();}
};

template <class _Tp>
class allocator<const _Tp>
{
public:
    typedef size_t            size_type;
    typedef long int         difference_type;
    typedef const _Tp*        pointer;
    typedef const _Tp*        const_pointer;
    typedef const _Tp&        reference;
    typedef const _Tp&        const_reference;
    typedef _Tp               value_type;

    typedef true_type propagate_on_container_move_assignment;

    template <class _Up> struct rebind {typedef allocator<_Up> other;};

    DEVICE allocator() {}
    template <class _Up> allocator(const allocator<_Up>&) {}
    DEVICE const_pointer address(const_reference __x) const
        {return util::addressof(__x);}
    DEVICE pointer allocate(size_type __n, allocator<void>::const_pointer = 0)
        {return static_cast<pointer>(::operator new(__n * sizeof(_Tp)));}
    DEVICE void deallocate(pointer __p, size_type)
        {::operator delete((void*)__p);}
    DEVICE size_type max_size() const
        {return size_type(~0) / sizeof(_Tp);}

        DEVICE void
        construct(pointer __p)
        {
            ::new((void*)__p) _Tp();
        }

    template <class _A0>
        DEVICE void
        construct(pointer __p, _A0& __a0)
        {
            ::new((void*)__p) _Tp(__a0);
        }
    template <class _A0>
        DEVICE void
        construct(pointer __p, const _A0& __a0)
        {
            ::new((void*)__p) _Tp(__a0);
        }

    template <class _A0, class _A1>
        DEVICE void
        construct(pointer __p, _A0& __a0, _A1& __a1)
        {
            ::new((void*)__p) _Tp(__a0, __a1);
        }
    template <class _A0, class _A1>
        DEVICE void
        construct(pointer __p, const _A0& __a0, _A1& __a1)
        {
            ::new((void*)__p) _Tp(__a0, __a1);
        }
    template <class _A0, class _A1>
        DEVICE void
        construct(pointer __p, _A0& __a0, const _A1& __a1)
        {
            ::new((void*)__p) _Tp(__a0, __a1);
        }
    template <class _A0, class _A1>
        DEVICE void
        construct(pointer __p, const _A0& __a0, const _A1& __a1)
        {
            ::new((void*)__p) _Tp(__a0, __a1);
        }
    DEVICE void destroy(pointer __p) {__p->~_Tp();}
};


template <class T, class U>
DEVICE bool operator==(const allocator<T>&, const allocator<U>&);

template <class T, class U>
DEVICE bool operator!=(const allocator<T>&, const allocator<U>&);

template <class OutputIterator, class T>
class raw_storage_iterator
    : public iterator<output_iterator_tag,
                      T,                               // purposefully not C++03
                      long int,                       // purposefully not C++03
                      T*,                              // purposefully not C++03
                      raw_storage_iterator<OutputIterator, T>&>           // purposefully not C++03
{
public:
    DEVICE explicit raw_storage_iterator(OutputIterator x);
    DEVICE raw_storage_iterator& operator*();
    DEVICE raw_storage_iterator& operator=(const T& element);
    DEVICE raw_storage_iterator& operator++();
    DEVICE raw_storage_iterator  operator++(int);
};

template <class T> DEVICE pair<T*,long int> get_temporary_buffer(long int n);
template <class T> DEVICE void               return_temporary_buffer(T* p);

template <class T> DEVICE T* addressof(T& r);

template <class InputIterator, class ForwardIterator>
DEVICE ForwardIterator
uninitialized_copy(InputIterator first, InputIterator last, ForwardIterator result);

template <class InputIterator, class Size, class ForwardIterator>
DEVICE ForwardIterator
uninitialized_copy_n(InputIterator first, Size n, ForwardIterator result);

template <class ForwardIterator, class T>
DEVICE void uninitialized_fill(ForwardIterator first, ForwardIterator last, const T& x);

template <class ForwardIterator, class Size, class T>
DEVICE ForwardIterator
uninitialized_fill_n(ForwardIterator first, Size n, const T& x);

// Unique Ptr
template <class _Tp>
struct default_delete
{
    DEVICE default_delete() {}
    template <class _Up>
        DEVICE default_delete(const default_delete<_Up>&) {}
    DEVICE void operator() (_Tp* __ptr) const
    {
        delete __ptr;
    }
};

template <class _Tp>
struct default_delete<_Tp[]>
{
public:
    DEVICE default_delete() {}
    template <class _Up>
        DEVICE default_delete(const default_delete<_Up[]>&) {}
    template <class _Up>
        DEVICE void operator() (_Up* __ptr) const
        {
            delete [] __ptr;
        }
};

template <class _Tp, class _Dp = default_delete<_Tp> >
class unique_ptr
{
public:
    typedef _Tp element_type;
    typedef _Dp deleter_type;
    typedef typename __pointer_type<_Tp, deleter_type>::type pointer;
private:
    pair<pointer, deleter_type> __ptr_;

    struct __nat {int __for_bool_;};

    typedef       typename remove_reference<deleter_type>::type& _Dp_reference;
    typedef const typename remove_reference<deleter_type>::type& _Dp_const_reference;
public:
    DEVICE unique_ptr()
        : __ptr_(pointer())
        {
        }

    DEVICE explicit unique_ptr(pointer __p)
        : __ptr_(__p)
        {
        }

    DEVICE operator __rv<unique_ptr>()
    {
        return __rv<unique_ptr>(*this);
    }

    DEVICE unique_ptr(__rv<unique_ptr> __u)
        : __ptr_(__u->release(), util::forward<deleter_type>(__u->get_deleter())) {}

    template <class _Up, class _Ep>
    DEVICE unique_ptr& operator=(unique_ptr<_Up, _Ep> __u)
    {
        reset(__u.release());
        __ptr_.second = util::forward<deleter_type>(__u.get_deleter());
        return *this;
    }

    DEVICE unique_ptr(pointer __p, deleter_type __d)
        : __ptr_(util::move(__p), util::move(__d)) {}

    DEVICE ~unique_ptr() {reset();}

    DEVICE typename add_lvalue_reference<_Tp>::type operator*() const
        {return *__ptr_.first();}
    DEVICE pointer operator->() const {return __ptr_.first;}
    DEVICE pointer get() const {return __ptr_.first;}
    DEVICE _Dp_reference get_deleter()
        {return __ptr_.second;}
    DEVICE _Dp_const_reference get_deleter() const
        {return __ptr_.second;}
    DEVICE operator bool() const
        {return __ptr_.first != 0;}

    DEVICE pointer release()
    {
        pointer __t = __ptr_.first;
        __ptr_.first = pointer();
        return __t;
    }

    DEVICE void reset(pointer __p = pointer())
    {
        pointer __tmp = __ptr_.first;
        __ptr_.first = __p;
        if (__tmp)
            __ptr_.second(__tmp);
    }

    DEVICE void swap(unique_ptr& __u)
        {__ptr_.swap(__u.__ptr_);}
};

template <class _Tp, class _Dp>
class unique_ptr<_Tp[], _Dp>
{
public:
    typedef _Tp element_type;
    typedef _Dp deleter_type;
    typedef typename __pointer_type<_Tp, deleter_type>::type pointer;
private:
    pair<pointer, deleter_type> __ptr_;

    struct __nat {int __for_bool_;};

    typedef       typename remove_reference<deleter_type>::type& _Dp_reference;
    typedef const typename remove_reference<deleter_type>::type& _Dp_const_reference;
public:
    DEVICE unique_ptr()
        : __ptr_(pointer())
        {
        }


    DEVICE explicit unique_ptr(pointer __p)
        : __ptr_(__p)
        {
        }

    DEVICE unique_ptr(pointer __p, deleter_type __d)
        : __ptr_(__p, util::forward<deleter_type>(__d)) {}

    DEVICE operator __rv<unique_ptr>()
    {
        return __rv<unique_ptr>(*this);
    }

    DEVICE unique_ptr(__rv<unique_ptr> __u)
        : __ptr_(__u->release(), util::forward<deleter_type>(__u->get_deleter())) {}

    DEVICE unique_ptr& operator=(__rv<unique_ptr> __u)
    {
        reset(__u->release());
        __ptr_.second() = util::forward<deleter_type>(__u->get_deleter());
        return *this;
    }

    DEVICE ~unique_ptr() {reset();}

    DEVICE typename add_lvalue_reference<_Tp>::type operator[](size_t __i) const
        {return __ptr_.first()[__i];}
    DEVICE pointer get() const {return __ptr_.first();}

    DEVICE _Dp_reference get_deleter()
        {return __ptr_.second();}
    DEVICE _Dp_const_reference get_deleter() const
        {return __ptr_.second();}
    DEVICE operator bool() const
        {return __ptr_.first() != 0;}

    DEVICE pointer release()
    {
        pointer __t = __ptr_.first();
        __ptr_.first() = pointer();
        return __t;
    }

    DEVICE void reset(pointer __p = pointer())
    {
        pointer __tmp = __ptr_.first();
        __ptr_.first() = __p;
        if (__tmp)
            __ptr_.second()(__tmp);
    }

    DEVICE void swap(unique_ptr& __u) {__ptr_.swap(__u.__ptr_);}
};

template <class _Tp, class _Dp>
DEVICE inline void
swap(unique_ptr<_Tp, _Dp>& __x, unique_ptr<_Tp, _Dp>& __y) {__x.swap(__y);}

template <class _T1, class _D1, class _T2, class _D2>
DEVICE inline bool
operator==(const unique_ptr<_T1, _D1>& __x, const unique_ptr<_T2, _D2>& __y) {return __x.get() == __y.get();}

template <class _T1, class _D1, class _T2, class _D2>
DEVICE inline bool
operator!=(const unique_ptr<_T1, _D1>& __x, const unique_ptr<_T2, _D2>& __y) {return !(__x == __y);}

template <class _T1, class _D1, class _T2, class _D2>
DEVICE inline bool
operator< (const unique_ptr<_T1, _D1>& __x, const unique_ptr<_T2, _D2>& __y)
{
    typedef typename unique_ptr<_T1, _D1>::pointer _P1;
    typedef typename unique_ptr<_T2, _D2>::pointer _P2;
    typedef typename common_type<_P1, _P2>::type _V;
    return less<_V>()(__x.get(), __y.get());
}

template <class _T1, class _D1, class _T2, class _D2>
DEVICE inline bool
operator> (const unique_ptr<_T1, _D1>& __x, const unique_ptr<_T2, _D2>& __y) {return __y < __x;}

template <class _T1, class _D1, class _T2, class _D2>
DEVICE inline bool
operator<=(const unique_ptr<_T1, _D1>& __x, const unique_ptr<_T2, _D2>& __y) {return !(__y < __x);}

template <class _T1, class _D1, class _T2, class _D2>
DEVICE inline bool
operator>=(const unique_ptr<_T1, _D1>& __x, const unique_ptr<_T2, _D2>& __y) {return !(__x < __y);}

template <class _Tp, class _Dp>
DEVICE inline unique_ptr<_Tp, _Dp>
move(unique_ptr<_Tp, _Dp>& __t)
{
    return unique_ptr<_Tp, _Dp>(__rv<unique_ptr<_Tp, _Dp> >(__t));
}

struct __destruct_n
{
private:
    size_t size;

    template <class _Tp>
    DEVICE void __process(_Tp* __p, false_type)
        {for (size_t __i = 0; __i < size; ++__i, ++__p) __p->~_Tp();}

    template <class _Tp>
    DEVICE void __process(_Tp*, true_type)
        {}

    DEVICE void __incr(false_type)
        {++size;}
    DEVICE void __incr(true_type)
        {}

    DEVICE void __set(size_t __s, false_type)
        {size = __s;}
    DEVICE void __set(size_t, true_type)
        {}
public:
    DEVICE explicit __destruct_n(size_t __s)
        : size(__s) {}

    template <class _Tp>
    DEVICE void __incr(_Tp*)
        {__incr(integral_constant<bool, is_trivially_destructible<_Tp>::value>());}

    template <class _Tp>
    DEVICE void __set(size_t __s, _Tp*)
        {__set(__s, integral_constant<bool, is_trivially_destructible<_Tp>::value>());}

    template <class _Tp>
    DEVICE void operator()(_Tp* __p)
        {__process(__p, integral_constant<bool, is_trivially_destructible<_Tp>::value>());}
};


}

}



#endif /* GPUALLOCATOR_TRAITS_H_ */
