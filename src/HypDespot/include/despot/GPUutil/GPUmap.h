/*
 * GPUmap.h
 *
 *  Created on: 28 Mar, 2017
 *      Author: panpan
 */

#ifndef GPUMAP_H_
#define GPUMAP_H_

/*	\file   map.h
	\author Gregory Diamos <solusstultus@gmail.com>
	\date   November 14, 2012
	\brief  The header file for the map class
*/

#pragma once

// Archaeopteryx Includes
#include <despot/GPUutil/GPUfunctional.h>
#include <despot/GPUutil/GPUallocator_traits.h>
#include <despot/GPUutil/GPURedBlackTree.h>
#include <despot/GPUcore/CudaInclude.h>

namespace archaeopteryx
{

namespace util
{

template <class _Key, class _Tp, class _Compare, bool = is_empty<_Compare>::value>
class __map_value_compare
    : private _Compare
{
    typedef pair<typename remove_const<_Key>::type, _Tp> _Pp;
    typedef pair<const _Key, _Tp> _CP;
public:
    DEVICE __map_value_compare()
        : _Compare() {}
    DEVICE __map_value_compare(_Compare c)
        : _Compare(c) {}
    DEVICE const _Compare& key_comp() const {return *this;}
    DEVICE bool operator()(const _CP& __x, const _CP& __y) const
        {return static_cast<const _Compare&>(*this)(__x.first, __y.first);}
    DEVICE bool operator()(const _CP& __x, const _Pp& __y) const
        {return static_cast<const _Compare&>(*this)(__x.first, __y.first);}
    DEVICE bool operator()(const _CP& __x, const _Key& __y) const
        {return static_cast<const _Compare&>(*this)(__x.first, __y);}
    DEVICE bool operator()(const _Pp& __x, const _CP& __y) const
        {return static_cast<const _Compare&>(*this)(__x.first, __y.first);}
    DEVICE bool operator()(const _Pp& __x, const _Pp& __y) const
        {return static_cast<const _Compare&>(*this)(__x.first, __y.first);}
    DEVICE bool operator()(const _Pp& __x, const _Key& __y) const
        {return static_cast<const _Compare&>(*this)(__x.first, __y);}
    DEVICE bool operator()(const _Key& __x, const _CP& __y) const
        {return static_cast<const _Compare&>(*this)(__x, __y.first);}
    DEVICE bool operator()(const _Key& __x, const _Pp& __y) const
        {return static_cast<const _Compare&>(*this)(__x, __y.first);}
    DEVICE bool operator()(const _Key& __x, const _Key& __y) const
        {return static_cast<const _Compare&>(*this)(__x, __y);}
};

template <class _Key, class _Tp, class _Compare>
class __map_value_compare<_Key, _Tp, _Compare, false>
{
    _Compare comp;

    typedef pair<typename remove_const<_Key>::type, _Tp> _Pp;
    typedef pair<const _Key, _Tp> _CP;

public:
    DEVICE __map_value_compare()
        : comp() {}
    DEVICE __map_value_compare(_Compare c)
        : comp(c) {}
    DEVICE const _Compare& key_comp() const {return comp;}

    DEVICE bool operator()(const _CP& __x, const _CP& __y) const
        {return comp(__x.first, __y.first);}
    DEVICE bool operator()(const _CP& __x, const _Pp& __y) const
        {return comp(__x.first, __y.first);}
    DEVICE bool operator()(const _CP& __x, const _Key& __y) const
        {return comp(__x.first, __y);}
    DEVICE bool operator()(const _Pp& __x, const _CP& __y) const
        {return comp(__x.first, __y.first);}
    DEVICE bool operator()(const _Pp& __x, const _Pp& __y) const
        {return comp(__x.first, __y.first);}
    DEVICE bool operator()(const _Pp& __x, const _Key& __y) const
        {return comp(__x.first, __y);}
    DEVICE bool operator()(const _Key& __x, const _CP& __y) const
        {return comp(__x, __y.first);}
    DEVICE bool operator()(const _Key& __x, const _Pp& __y) const
        {return comp(__x, __y.first);}
    DEVICE bool operator()(const _Key& __x, const _Key& __y) const
        {return comp(__x, __y);}
};

template <class _Allocator>
class __map_node_destructor
{
    typedef _Allocator                          allocator_type;
    typedef allocator_traits<allocator_type>    __alloc_traits;
    typedef typename __alloc_traits::value_type::value_type value_type;
public:
    typedef typename __alloc_traits::pointer    pointer;
private:
    typedef typename value_type::first_type     first_type;
    typedef typename value_type::second_type    second_type;

    allocator_type& __na_;

    DEVICE __map_node_destructor& operator=(const __map_node_destructor&);

public:
    bool __first_constructed;
    bool __second_constructed;

    explicit DEVICE __map_node_destructor(allocator_type& __na)
        : __na_(__na),
          __first_constructed(false),
          __second_constructed(false)
        {}

    DEVICE void operator()(pointer __p)
    {
        if (__second_constructed)
            __alloc_traits::destroy(__na_, addressof(__p->__value_.second));
        if (__first_constructed)
            __alloc_traits::destroy(__na_, addressof(__p->__value_.first));
        if (__p)
            __alloc_traits::deallocate(__na_, __p, 1);
    }
};

template <class _Key, class _Tp, class _Compare, class _Allocator>
    class map;
template <class _Key, class _Tp, class _Compare, class _Allocator>
    class multimap;
template <class _TreeIterator> class __map_const_iterator;

template <class _TreeIterator>
class __map_iterator
{
    _TreeIterator __i_;

    typedef typename _TreeIterator::__pointer_traits             __pointer_traits;
    typedef const typename _TreeIterator::value_type::first_type __key_type;
    typedef typename _TreeIterator::value_type::second_type      __mapped_type;
public:
    typedef bidirectional_iterator_tag                           iterator_category;
    typedef pair<__key_type, __mapped_type>                      value_type;
    typedef typename _TreeIterator::difference_type              difference_type;
    typedef value_type&                                          reference;
    typedef typename __pointer_traits::template
            rebind<value_type>::other                      pointer;

    DEVICE __map_iterator() {}

    DEVICE __map_iterator(_TreeIterator __i) : __i_(__i) {}

    DEVICE reference operator*() const {return *operator->();}
    DEVICE pointer operator->() const {return (pointer)__i_.operator->();}

    DEVICE __map_iterator& operator++() {++__i_; return *this;}
    DEVICE __map_iterator operator++(int)
    {
        __map_iterator __t(*this);
        ++(*this);
        return __t;
    }

    DEVICE __map_iterator& operator--() {--__i_; return *this;}
    DEVICE __map_iterator operator--(int)
    {
        __map_iterator __t(*this);
        --(*this);
        return __t;
    }

    friend DEVICE bool operator==(const __map_iterator& __x, const __map_iterator& __y)
        {return __x.__i_ == __y.__i_;}
    friend
    DEVICE bool operator!=(const __map_iterator& __x, const __map_iterator& __y)
        {return __x.__i_ != __y.__i_;}

    template <class, class, class, class> friend class map;
    template <class, class, class, class> friend class multimap;
    template <class> friend class __map_const_iterator;
};

template <class _TreeIterator>
class __map_const_iterator
{
    _TreeIterator __i_;

    typedef typename _TreeIterator::__pointer_traits             __pointer_traits;
    typedef const typename _TreeIterator::value_type::first_type __key_type;
    typedef typename _TreeIterator::value_type::second_type      __mapped_type;
public:
    typedef bidirectional_iterator_tag                           iterator_category;
    typedef pair<__key_type, __mapped_type>                      value_type;
    typedef typename _TreeIterator::difference_type              difference_type;
    typedef const value_type&                                    reference;
    typedef typename __pointer_traits::template
            rebind<const value_type>::other                      pointer;

    DEVICE __map_const_iterator() {}

    DEVICE __map_const_iterator(_TreeIterator __i) : __i_(__i) {}
    DEVICE __map_const_iterator(
            __map_iterator<typename _TreeIterator::__non_const_iterator> __i)

                : __i_(__i.__i_) {}

    DEVICE reference operator*() const {return *operator->();}
    DEVICE pointer operator->() const {return (pointer)__i_.operator->();}

    DEVICE __map_const_iterator& operator++() {++__i_; return *this;}
    DEVICE __map_const_iterator operator++(int)
    {
        __map_const_iterator __t(*this);
        ++(*this);
        return __t;
    }

    DEVICE __map_const_iterator& operator--() {--__i_; return *this;}
    DEVICE __map_const_iterator operator--(int)
    {
        __map_const_iterator __t(*this);
        --(*this);
        return __t;
    }

    friend DEVICE bool operator==(const __map_const_iterator& __x, const __map_const_iterator& __y)
        {return __x.__i_ == __y.__i_;}
    friend DEVICE bool operator!=(const __map_const_iterator& __x, const __map_const_iterator& __y)
        {return __x.__i_ != __y.__i_;}

    template <class, class, class, class> friend class map;
    template <class, class, class, class> friend class multimap;
    template <class, class, class> friend class __tree_const_iterator;
};


template <class Key, class T, class Compare = less<Key>,
          class Allocator = allocator<pair<const Key, T> > >
class map
{
public:
    // types:
    typedef Key                                      key_type;
    typedef T                                        mapped_type;
    typedef pair<const key_type, mapped_type>        value_type;
    typedef Compare                                  key_compare;
    typedef Allocator                                allocator_type;
    typedef value_type&                              reference;
    typedef const value_type&                        const_reference;

    class value_compare
        : public binary_function<value_type, value_type, bool>
    {
        friend class map;
    protected:
        key_compare comp;

        DEVICE value_compare(key_compare c) : comp(c) {}
    public:
        DEVICE bool operator()(const value_type& __x, const value_type& __y) const
            {return comp(__x.first, __y.first);}
    };

private:
    typedef pair<key_type, mapped_type>                             __value_type;
    typedef __map_value_compare<key_type, mapped_type, key_compare> __vc;
    typedef typename allocator_traits<allocator_type>::template
            rebind_alloc<__value_type>::other
                                                           __allocator_type;
    typedef RedBlackTree<__value_type, __vc, __allocator_type>   __base;
    typedef typename __base::__node_traits                 __node_traits;
    typedef allocator_traits<allocator_type>               __alloc_traits;


public:
    typedef typename __alloc_traits::pointer            pointer;
    typedef typename __alloc_traits::const_pointer      const_pointer;
    typedef typename __alloc_traits::size_type          size_type;
    typedef typename __alloc_traits::difference_type    difference_type;
    typedef __map_iterator<typename __base::iterator>   iterator;
    typedef __map_const_iterator<typename __base::const_iterator> const_iterator;
    typedef util::reverse_iterator<iterator>                  reverse_iterator;
    typedef util::reverse_iterator<const_iterator>            const_reverse_iterator;

public:
	explicit DEVICE map(const key_compare& __comp = key_compare())
        : __tree_(__vc(__comp)) {}

    explicit DEVICE map(const key_compare& __comp, const allocator_type& __a)
        : __tree_(__vc(__comp), __a) {}

    template <class _InputIterator>
        DEVICE map(_InputIterator __f, _InputIterator __l,
            const key_compare& __comp = key_compare())
        : __tree_(__vc(__comp))
        {
            insert(__f, __l);
        }

    template <class _InputIterator>
        DEVICE map(_InputIterator __f, _InputIterator __l,
            const key_compare& __comp, const allocator_type& __a)
        : __tree_(__vc(__comp), __a)
        {
            insert(__f, __l);
        }

    DEVICE map(const map& __m)
        : __tree_(__m.__tree_)
        {
            insert(__m.begin(), __m.end());
        }

    DEVICE map& operator=(const map& __m)
        {
            __tree_ = __m.__tree_;
            return *this;
        }

    DEVICE explicit map(const allocator_type& __a)
        : __tree_(__a)
        {
        }

    DEVICE map(const map& __m, const allocator_type& __a)
        : __tree_(__m.__tree_.value_comp(), __a)
        {
            insert(__m.begin(), __m.end());
        }

public:
	// Iteration
    DEVICE iterator begin() {return __tree_.begin();}
    DEVICE  const_iterator begin() const {return __tree_.begin();}
    DEVICE iterator end() {return __tree_.end();}
    DEVICE const_iterator end() const {return __tree_.end();}

    DEVICE reverse_iterator rbegin() {return reverse_iterator(end());}
    DEVICE const_reverse_iterator rbegin() const
        {return const_reverse_iterator(end());}
    DEVICE reverse_iterator rend()
            {return       reverse_iterator(begin());}
    DEVICE const_reverse_iterator rend() const
        {return const_reverse_iterator(begin());}

public:
	// Size
    DEVICE bool      empty() const {return __tree_.size() == 0;}
    DEVICE size_type size() const {return __tree_.size();}
    DEVICE size_type max_size() const {return __tree_.max_size();}

public:
	// Element Access
    DEVICE mapped_type& operator[](const key_type& __k);

          mapped_type& at(const key_type& __k);
    DEVICE const mapped_type& at(const key_type& __k) const;

    DEVICE pair<iterator, bool>
        insert(const value_type& __v) {return __tree_.__insert_unique(__v);}

    DEVICE iterator
        insert(const_iterator __p, const value_type& __v)
            {return __tree_.__insert_unique(__p.__i_, __v);}

    template <class _InputIterator>
            DEVICE void insert(_InputIterator __f, _InputIterator __l)
        {
            for (const_iterator __e = end(); __f != __l; ++__f)
                insert(__e.__i_, *__f);
        }

    DEVICE iterator erase(const_iterator __p) {return __tree_.erase(__p.__i_);}
    DEVICE size_type erase(const key_type& __k)
        {return __tree_.__erase_unique(__k);}
    DEVICE iterator  erase(const_iterator __f, const_iterator __l)
        {return __tree_.erase(__f.__i_, __l.__i_);}
    DEVICE void clear() {__tree_.clear();}

    DEVICE void swap(map& __m)
        {__tree_.swap(__m.__tree_);}

    DEVICE iterator find(const key_type& __k)             {return __tree_.find(__k);}
    DEVICE const_iterator find(const key_type& __k) const {return __tree_.find(__k);}
    DEVICE size_type      count(const key_type& __k) const
        {return __tree_.__count_unique(__k);}
    DEVICE iterator lower_bound(const key_type& __k)
        {return __tree_.lower_bound(__k);}
    DEVICE const_iterator lower_bound(const key_type& __k) const
        {return __tree_.lower_bound(__k);}
    DEVICE iterator upper_bound(const key_type& __k)
        {return __tree_.upper_bound(__k);}
    DEVICE const_iterator upper_bound(const key_type& __k) const
        {return __tree_.upper_bound(__k);}
    DEVICE pair<iterator,iterator> equal_range(const key_type& __k)
        {return __tree_.__equal_range_unique(__k);}
   DEVICE  pair<const_iterator,const_iterator> equal_range(const key_type& __k) const
        {return __tree_.__equal_range_unique(__k);}

public:
	// Member access
    DEVICE allocator_type get_allocator() const {return __tree_.__alloc();}
    DEVICE key_compare    key_comp()      const {return __tree_.value_comp().key_comp();}
    DEVICE value_compare  value_comp()    const {return value_compare(__tree_.value_comp().key_comp());}


private:
	__base __tree_;

private:
    typedef typename __base::__node                    __node;
    typedef typename __base::__node_allocator          __node_allocator;
    typedef typename __base::__node_pointer            __node_pointer;
    typedef typename __base::__node_const_pointer      __node_const_pointer;
    typedef typename __base::__node_base_pointer       __node_base_pointer;
    typedef typename __base::__node_base_const_pointer __node_base_const_pointer;
    typedef __map_node_destructor<__node_allocator> _Dp;
    typedef unique_ptr<__node, _Dp> __node_holder;

private:
	DEVICE __node_holder __construct_node(const key_type& __k);
	DEVICE __node_base_pointer&
		__find_equal_key(__node_base_pointer& __parent, const key_type& __k);
	DEVICE __node_base_pointer& __find_equal_key(const_iterator __hint,
		             __node_base_pointer& __parent, const key_type& __k);
	DEVICE __node_base_const_pointer
		__find_equal_key(__node_base_const_pointer& __parent, const key_type& __k) const;

};

// Find place to insert if __k doesn't exist
// Set __parent to parent of null leaf
// Return reference to null leaf
// If __k exists, set parent to node of __k and return reference to node of __k
template <class _Key, class _Tp, class _Compare, class _Allocator>
DEVICE typename map<_Key, _Tp, _Compare, _Allocator>::__node_base_pointer&
map<_Key, _Tp, _Compare, _Allocator>::__find_equal_key(__node_base_pointer& __parent,
                                                       const key_type& __k)
{
    __node_pointer __nd = __tree_.__root();
    if (__nd != 0)
    {
        while (true)
        {
            if (__tree_.value_comp().key_comp()(__k, __nd->__value_.first))
            {
                if (__nd->__left_ != 0)
                    __nd = static_cast<__node_pointer>(__nd->__left_);
                else
                {
                    __parent = __nd;
                    return __parent->__left_;
                }
            }
            else if (__tree_.value_comp().key_comp()(__nd->__value_.first, __k))
            {
                if (__nd->__right_ != 0)
                    __nd = static_cast<__node_pointer>(__nd->__right_);
                else
                {
                    __parent = __nd;
                    return __parent->__right_;
                }
            }
            else
            {
                __parent = __nd;
                return __parent;
            }
        }
    }
    __parent = __tree_.__end_node();
    return __parent->__left_;
}

// Find place to insert if __k doesn't exist
// First check prior to __hint.
// Next check after __hint.
// Next do O(log N) search.
// Set __parent to parent of null leaf
// Return reference to null leaf
// If __k exists, set parent to node of __k and return reference to node of __k
template <class _Key, class _Tp, class _Compare, class _Allocator>
DEVICE typename map<_Key, _Tp, _Compare, _Allocator>::__node_base_pointer&
map<_Key, _Tp, _Compare, _Allocator>::__find_equal_key(const_iterator __hint,
                                                       __node_base_pointer& __parent,
                                                       const key_type& __k)
{
    if (__hint == end() || __tree_.value_comp().key_comp()(__k, __hint->first))  // check before
    {
        // __k < *__hint
        const_iterator __prior = __hint;
        if (__prior == begin() || __tree_.value_comp().key_comp()((--__prior)->first, __k))
        {
            // *prev(__hint) < __k < *__hint
            if (__hint.__ptr_->__left_ == NULL)
            {
                __parent = const_cast<__node_pointer&>(__hint.__ptr_);
                return __parent->__left_;
            }
            else
            {
                __parent = const_cast<__node_pointer&>(__prior.__ptr_);
                return __parent->__right_;
            }
        }
        // __k <= *prev(__hint)
        return __find_equal_key(__parent, __k);
    }
    else if (__tree_.value_comp().key_comp()(__hint->first, __k))  // check after
    {
        // *__hint < __k

        const_iterator __next = util::next<const_iterator>(__hint);
        if (__next == end() || __tree_.value_comp().key_comp()(__k, __next->first))
        {
            // *__hint < __k < *next(__hint)
            if (__hint.__ptr_->__right_ == 0)
            {
                __parent = const_cast<__node_pointer&>(__hint.__ptr_);
                return __parent->__right_;
            }
            else
            {
                __parent = const_cast<__node_pointer&>(__next.__ptr_);
                return __parent->__left_;
            }
        }
        // *next(__hint) <= __k
        return __find_equal_key(__parent, __k);
    }
    // else __k == *__hint
    __parent = const_cast<__node_pointer&>(__hint.__ptr_);
    return __parent;
}

// Find __k
// Set __parent to parent of null leaf and
//    return reference to null leaf iv __k does not exist.
// If __k exists, set parent to node of __k and return reference to node of __k
template <class _Key, class _Tp, class _Compare, class _Allocator>
DEVICE typename map<_Key, _Tp, _Compare, _Allocator>::__node_base_const_pointer
map<_Key, _Tp, _Compare, _Allocator>::__find_equal_key(__node_base_const_pointer& __parent,
                                                       const key_type& __k) const
{
    __node_const_pointer __nd = __tree_.__root();
    if (__nd != 0)
    {
        while (true)
        {
            if (__tree_.value_comp().key_comp()(__k, __nd->__value_.first))
            {
                if (__nd->__left_ != NULL)
                    __nd = static_cast<__node_pointer>(__nd->__left_);
                else
                {
                    __parent = __nd;
                    return const_cast<const __node_base_const_pointer&>(__parent->__left_);
                }
            }
            else if (__tree_.value_comp().key_comp()(__nd->__value_.first, __k))
            {
                if (__nd->__right_ != NULL)
                    __nd = static_cast<__node_pointer>(__nd->__right_);
                else
                {
                    __parent = __nd;
                    return const_cast<const __node_base_const_pointer&>(__parent->__right_);
                }
            }
            else
            {
                __parent = __nd;
                return __parent;
            }
        }
    }
    __parent = __tree_.__end_node();
    return const_cast<const __node_base_const_pointer&>(__parent->__left_);
}

template <class _Key, class _Tp, class _Compare, class _Allocator>
DEVICE typename map<_Key, _Tp, _Compare, _Allocator>::__node_holder
map<_Key, _Tp, _Compare, _Allocator>::__construct_node(const key_type& __k)
{
    __node_allocator& __na = __tree_.__node_alloc();
    __node_holder __h(__node_traits::allocate(__na, 1), _Dp(__na));
    __node_traits::construct(__na, addressof(__h->__value_.first), __k);
    __h.get_deleter().__first_constructed = true;
    __node_traits::construct(__na, addressof(__h->__value_.second));
    __h.get_deleter().__second_constructed = true;
    return move(__h);
}

template <class _Key, class _Tp, class _Compare, class _Allocator>
DEVICE _Tp& map<_Key, _Tp, _Compare, _Allocator>::operator[](const key_type& __k)
{
    __node_base_pointer __parent;
    __node_base_pointer& __child = __find_equal_key(__parent, __k);
    __node_pointer __r = static_cast<__node_pointer>(__child);
    if (__child == 0)
    {
        __node_holder __h = __construct_node(__k);
        __tree_.__insert_node_at(__parent, __child, __h.get());
        __r = __h.release();
    }
    return __r->__value_.second;
}

}

}



#endif /* GPUMAP_H_ */
