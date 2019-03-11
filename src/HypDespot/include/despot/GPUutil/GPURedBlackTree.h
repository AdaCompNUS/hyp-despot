/*
 * GPURedBlackTree.h
 *
 *  Created on: 28 Mar, 2017
 *      Author: panpan
 */

#ifndef GPUREDBLACKTREE_H_
#define GPUREDBLACKTREE_H_

/*	\file   RedBlackTree.h
	\author Gregory Diamos <solusstultus@gmail.com>
	\date   November 26, 2012
	\brief  The header file for the RedBlackTree class
*/

#pragma once
#include <despot/GPUcore/CudaInclude.h>

// Archaeopteryx Includes

namespace archaeopteryx
{

namespace util
{


template <class _Tp, class _Compare, class _Allocator> class __tree;
template <class _Tp, class _NodePtr, class _DiffType>
    class __tree_iterator;
template <class _Tp, class _ConstNodePtr, class _DiffType>
    class __tree_const_iterator;
template <class _Key, class _Tp, class _Compare, class _Allocator>
    class map;
template <class _Key, class _Tp, class _Compare, class _Allocator>
    class multimap;
template <class _Key, class _Compare, class _Allocator>
    class set;
template <class _Key, class _Compare, class _Allocator>
    class multiset;

/*

_NodePtr algorithms

The algorithms taking _NodePtr are red black tree algorithms.  Those
algorithms taking a parameter named __root should assume that __root
points to a proper red black tree (unless otherwise specified).

Each algorithm herein assumes that __root->__parent_ points to a non-null
structure which has a member __left_ which points back to __root.  No other
member is read or written to at __root->__parent_.

__root->__parent_ will be referred to below (in comments only) as end_node.
end_node->__left_ is an externably accessible lvalue for __root, and can be
changed by node insertion and removal (without explicit reference to end_node).

All nodes (with the exception of end_node), even the node referred to as
__root, have a non-null __parent_ field.

*/

// Returns:  true if __x is a left child of its parent, else false
// Precondition:  __x != 0.
template <class _NodePtr>
DEVICE inline bool
__tree_is_left_child(_NodePtr __x)
{
    return __x == __x->__parent_->__left_;
}

// Determintes if the subtree rooted at __x is a proper red black subtree.  If
//    __x is a proper subtree, returns the black height (null counts as 1).  If
//    __x is an improper subtree, returns 0.
template <class _NodePtr>
DEVICE unsigned
__tree_sub_invariant(_NodePtr __x)
{
    if (__x == 0)
        return 1;
    // parent consistency checked by caller
    // check __x->__left_ consistency
    if (__x->__left_ != 0 && __x->__left_->__parent_ != __x)
        return 0;
    // check __x->__right_ consistency
    if (__x->__right_ != 0 && __x->__right_->__parent_ != __x)
        return 0;
    // check __x->__left_ != __x->__right_ unless both are 0
    if (__x->__left_ == __x->__right_ && __x->__left_ != 0)
        return 0;
    // If this is red, neither child can be red
    if (!__x->__is_black_)
    {
        if (__x->__left_ && !__x->__left_->__is_black_)
            return 0;
        if (__x->__right_ && !__x->__right_->__is_black_)
            return 0;
    }
    unsigned __h = __tree_sub_invariant(__x->__left_);
    if (__h == 0)
        return 0;  // invalid left subtree
    if (__h != __tree_sub_invariant(__x->__right_))
        return 0;  // invalid or different height right subtree
    return __h + __x->__is_black_;  // return black height of this node
}

// Determintes if the red black tree rooted at __root is a proper red black tree.
//    __root == 0 is a proper tree.  Returns true is __root is a proper
//    red black tree, else returns false.
template <class _NodePtr>
DEVICE bool
__tree_invariant(_NodePtr __root)
{
    if (__root == 0)
        return true;
    // check __x->__parent_ consistency
    if (__root->__parent_ == 0)
        return false;
    if (!__tree_is_left_child(__root))
        return false;
    // root must be black
    if (!__root->__is_black_)
        return false;
    // do normal node checks
    return __tree_sub_invariant(__root) != 0;
}

// Returns:  pointer to the left-most node under __x.
// Precondition:  __x != 0.
template <class _NodePtr>
DEVICE inline _NodePtr
__tree_min(_NodePtr __x)
{
    while (__x->__left_ != 0)
        __x = __x->__left_;
    return __x;
}

// Returns:  pointer to the right-most node under __x.
// Precondition:  __x != 0.
template <class _NodePtr>
DEVICE inline _NodePtr
__tree_max(_NodePtr __x)
{
    while (__x->__right_ != 0)
        __x = __x->__right_;
    return __x;
}

// Returns:  pointer to the next in-order node after __x.
// Precondition:  __x != 0.
template <class _NodePtr>
DEVICE _NodePtr
__tree_next(_NodePtr __x)
{
    if (__x->__right_ != 0)
        return __tree_min(__x->__right_);
    while (!__tree_is_left_child(__x))
        __x = __x->__parent_;
    return __x->__parent_;
}

// Returns:  pointer to the previous in-order node before __x.
// Precondition:  __x != 0.
template <class _NodePtr>
DEVICE _NodePtr
__tree_prev(_NodePtr __x)
{
    if (__x->__left_ != 0)
        return __tree_max(__x->__left_);
    while (__tree_is_left_child(__x))
        __x = __x->__parent_;
    return __x->__parent_;
}

// Returns:  pointer to a node which has no children
// Precondition:  __x != 0.
template <class _NodePtr>
DEVICE _NodePtr
__tree_leaf(_NodePtr __x)
{
    while (true)
    {
        if (__x->__left_ != 0)
        {
            __x = __x->__left_;
            continue;
        }
        if (__x->__right_ != 0)
        {
            __x = __x->__right_;
            continue;
        }
        break;
    }
    return __x;
}

// Effects:  Makes __x->__right_ the subtree root with __x as its left child
//           while preserving in-order order.
// Precondition:  __x->__right_ != 0
template <class _NodePtr>
DEVICE void
__tree_left_rotate(_NodePtr __x)
{
    _NodePtr __y = __x->__right_;
    __x->__right_ = __y->__left_;
    if (__x->__right_ != 0)
        __x->__right_->__parent_ = __x;
    __y->__parent_ = __x->__parent_;
    if (__tree_is_left_child(__x))
        __x->__parent_->__left_ = __y;
    else
        __x->__parent_->__right_ = __y;
    __y->__left_ = __x;
    __x->__parent_ = __y;
}

// Effects:  Makes __x->__left_ the subtree root with __x as its right child
//           while preserving in-order order.
// Precondition:  __x->__left_ != 0
template <class _NodePtr>
DEVICE void
__tree_right_rotate(_NodePtr __x)
{
    _NodePtr __y = __x->__left_;
    __x->__left_ = __y->__right_;
    if (__x->__left_ != 0)
        __x->__left_->__parent_ = __x;
    __y->__parent_ = __x->__parent_;
    if (__tree_is_left_child(__x))
        __x->__parent_->__left_ = __y;
    else
        __x->__parent_->__right_ = __y;
    __y->__right_ = __x;
    __x->__parent_ = __y;
}

// Effects:  Rebalances __root after attaching __x to a leaf.
// Precondition:  __root != nulptr && __x != 0.
//                __x has no children.
//                __x == __root or == a direct or indirect child of __root.
//                If __x were to be unlinked from __root (setting __root to
//                  0 if __root == __x), __tree_invariant(__root) == true.
// Postcondition: __tree_invariant(end_node->__left_) == true.  end_node->__left_
//                may be different than the value passed in as __root.
template <class _NodePtr>
DEVICE void
__tree_balance_after_insert(_NodePtr __root, _NodePtr __x)
{
    __x->__is_black_ = __x == __root;
    while (__x != __root && !__x->__parent_->__is_black_)
    {
        // __x->__parent_ != __root because __x->__parent_->__is_black == false
        if (__tree_is_left_child(__x->__parent_))
        {
            _NodePtr __y = __x->__parent_->__parent_->__right_;
            if (__y != 0 && !__y->__is_black_)
            {
                __x = __x->__parent_;
                __x->__is_black_ = true;
                __x = __x->__parent_;
                __x->__is_black_ = __x == __root;
                __y->__is_black_ = true;
            }
            else
            {
                if (!__tree_is_left_child(__x))
                {
                    __x = __x->__parent_;
                    __tree_left_rotate(__x);
                }
                __x = __x->__parent_;
                __x->__is_black_ = true;
                __x = __x->__parent_;
                __x->__is_black_ = false;
                __tree_right_rotate(__x);
                break;
            }
        }
        else
        {
            _NodePtr __y = __x->__parent_->__parent_->__left_;
            if (__y != 0 && !__y->__is_black_)
            {
                __x = __x->__parent_;
                __x->__is_black_ = true;
                __x = __x->__parent_;
                __x->__is_black_ = __x == __root;
                __y->__is_black_ = true;
            }
            else
            {
                if (__tree_is_left_child(__x))
                {
                    __x = __x->__parent_;
                    __tree_right_rotate(__x);
                }
                __x = __x->__parent_;
                __x->__is_black_ = true;
                __x = __x->__parent_;
                __x->__is_black_ = false;
                __tree_left_rotate(__x);
                break;
            }
        }
    }
}

// Precondition:  __root != 0 && __z != 0.
//                __tree_invariant(__root) == true.
//                __z == __root or == a direct or indirect child of __root.
// Effects:  unlinks __z from the tree rooted at __root, rebalancing as needed.
// Postcondition: __tree_invariant(end_node->__left_) == true && end_node->__left_
//                nor any of its children refer to __z.  end_node->__left_
//                may be different than the value passed in as __root.
template <class _NodePtr>
DEVICE void
__tree_remove(_NodePtr __root, _NodePtr __z)
{
    // __z will be removed from the tree.  Client still needs to destruct/deallocate it
    // __y is either __z, or if __z has two children, __tree_next(__z).
    // __y will have at most one child.
    // __y will be the initial hole in the tree (make the hole at a leaf)
    _NodePtr __y = (__z->__left_ == 0 || __z->__right_ == 0) ?
                    __z : __tree_next(__z);
    // __x is __y's possibly null single child
    _NodePtr __x = __y->__left_ != 0 ? __y->__left_ : __y->__right_;
    // __w is __x's possibly null uncle (will become __x's sibling)
    _NodePtr __w = 0;
    // link __x to __y's parent, and find __w
    if (__x != 0)
        __x->__parent_ = __y->__parent_;
    if (__tree_is_left_child(__y))
    {
        __y->__parent_->__left_ = __x;
        if (__y != __root)
            __w = __y->__parent_->__right_;
        else
            __root = __x;  // __w == 0
    }
    else
    {
        __y->__parent_->__right_ = __x;
        // __y can't be root if it is a right child
        __w = __y->__parent_->__left_;
    }
    bool __removed_black = __y->__is_black_;
    // If we didn't remove __z, do so now by splicing in __y for __z,
    //    but copy __z's color.  This does not impact __x or __w.
    if (__y != __z)
    {
        // __z->__left_ != nulptr but __z->__right_ might == __x == 0
        __y->__parent_ = __z->__parent_;
        if (__tree_is_left_child(__z))
            __y->__parent_->__left_ = __y;
        else
            __y->__parent_->__right_ = __y;
        __y->__left_ = __z->__left_;
        __y->__left_->__parent_ = __y;
        __y->__right_ = __z->__right_;
        if (__y->__right_ != 0)
            __y->__right_->__parent_ = __y;
        __y->__is_black_ = __z->__is_black_;
        if (__root == __z)
            __root = __y;
    }
    // There is no need to rebalance if we removed a red, or if we removed
    //     the last node.
    if (__removed_black && __root != 0)
    {
        // Rebalance:
        // __x has an implicit black color (transferred from the removed __y)
        //    associated with it, no matter what its color is.
        // If __x is __root (in which case it can't be null), it is supposed
        //    to be black anyway, and if it is doubly black, then the double
        //    can just be ignored.
        // If __x is red (in which case it can't be null), then it can absorb
        //    the implicit black just by setting its color to black.
        // Since __y was black and only had one child (which __x points to), __x
        //   is either red with no children, else null, otherwise __y would have
        //   different black heights under left and right pointers.
        // if (__x == __root || __x != 0 && !__x->__is_black_)
        if (__x != 0)
            __x->__is_black_ = true;
        else
        {
            //  Else __x isn't root, and is "doubly black", even though it may
            //     be null.  __w can not be null here, else the parent would
            //     see a black height >= 2 on the __x side and a black height
            //     of 1 on the __w side (__w must be a non-null black or a red
            //     with a non-null black child).
            while (true)
            {
                if (!__tree_is_left_child(__w))  // if x is left child
                {
                    if (!__w->__is_black_)
                    {
                        __w->__is_black_ = true;
                        __w->__parent_->__is_black_ = false;
                        __tree_left_rotate(__w->__parent_);
                        // __x is still valid
                        // reset __root only if necessary
                        if (__root == __w->__left_)
                            __root = __w;
                        // reset sibling, and it still can't be null
                        __w = __w->__left_->__right_;
                    }
                    // __w->__is_black_ is now true, __w may have null children
                    if ((__w->__left_  == 0 || __w->__left_->__is_black_) &&
                        (__w->__right_ == 0 || __w->__right_->__is_black_))
                    {
                        __w->__is_black_ = false;
                        __x = __w->__parent_;
                        // __x can no longer be null
                        if (__x == __root || !__x->__is_black_)
                        {
                            __x->__is_black_ = true;
                            break;
                        }
                        // reset sibling, and it still can't be null
                        __w = __tree_is_left_child(__x) ?
                                    __x->__parent_->__right_ :
                                    __x->__parent_->__left_;
                        // continue;
                    }
                    else  // __w has a red child
                    {
                        if (__w->__right_ == 0 || __w->__right_->__is_black_)
                        {
                            // __w left child is non-null and red
                            __w->__left_->__is_black_ = true;
                            __w->__is_black_ = false;
                            __tree_right_rotate(__w);
                            // __w is known not to be root, so root hasn't changed
                            // reset sibling, and it still can't be null
                            __w = __w->__parent_;
                        }
                        // __w has a right red child, left child may be null
                        __w->__is_black_ = __w->__parent_->__is_black_;
                        __w->__parent_->__is_black_ = true;
                        __w->__right_->__is_black_ = true;
                        __tree_left_rotate(__w->__parent_);
                        break;
                    }
                }
                else
                {
                    if (!__w->__is_black_)
                    {
                        __w->__is_black_ = true;
                        __w->__parent_->__is_black_ = false;
                        __tree_right_rotate(__w->__parent_);
                        // __x is still valid
                        // reset __root only if necessary
                        if (__root == __w->__right_)
                            __root = __w;
                        // reset sibling, and it still can't be null
                        __w = __w->__right_->__left_;
                    }
                    // __w->__is_black_ is now true, __w may have null children
                    if ((__w->__left_  == 0 || __w->__left_->__is_black_) &&
                        (__w->__right_ == 0 || __w->__right_->__is_black_))
                    {
                        __w->__is_black_ = false;
                        __x = __w->__parent_;
                        // __x can no longer be null
                        if (!__x->__is_black_ || __x == __root)
                        {
                            __x->__is_black_ = true;
                            break;
                        }
                        // reset sibling, and it still can't be null
                        __w = __tree_is_left_child(__x) ?
                                    __x->__parent_->__right_ :
                                    __x->__parent_->__left_;
                        // continue;
                    }
                    else  // __w has a red child
                    {
                        if (__w->__left_ == 0 || __w->__left_->__is_black_)
                        {
                            // __w right child is non-null and red
                            __w->__right_->__is_black_ = true;
                            __w->__is_black_ = false;
                            __tree_left_rotate(__w);
                            // __w is known not to be root, so root hasn't changed
                            // reset sibling, and it still can't be null
                            __w = __w->__parent_;
                        }
                        // __w has a left red child, right child may be null
                        __w->__is_black_ = __w->__parent_->__is_black_;
                        __w->__parent_->__is_black_ = true;
                        __w->__left_->__is_black_ = true;
                        __tree_right_rotate(__w->__parent_);
                        break;
                    }
                }
            }
        }
    }
}

template <class _Allocator> class __map_node_destructor;

template <class _Allocator>
class __tree_node_destructor
{
    typedef _Allocator                                      allocator_type;
    typedef allocator_traits<allocator_type>                __alloc_traits;
    typedef typename __alloc_traits::value_type::value_type value_type;
public:
    typedef typename __alloc_traits::pointer                pointer;
private:

    allocator_type& __na_;

    DEVICE __tree_node_destructor& operator=(const __tree_node_destructor&);

public:
    bool __value_constructed;

    DEVICE explicit __tree_node_destructor(allocator_type& __na)
        : __na_(__na),
          __value_constructed(false)
        {}

    DEVICE void operator()(pointer __p)
    {
        if (__value_constructed)
            __alloc_traits::destroy(__na_, util::addressof(__p->__value_));
        if (__p)
            __alloc_traits::deallocate(__na_, __p, 1);
    }

    template <class> friend class __map_node_destructor;
};

// node

template <class _Pointer>
class __tree_end_node
{
public:
    typedef _Pointer pointer;
    pointer __left_;

        DEVICE __tree_end_node() : __left_() {}
};

template <class _VoidPtr>
class __tree_node_base
    : public __tree_end_node
             <
                typename pointer_traits<_VoidPtr>::template
                     rebind<__tree_node_base<_VoidPtr> >::other
             >
{
    DEVICE __tree_node_base(const __tree_node_base&);
    DEVICE __tree_node_base& operator=(const __tree_node_base&);
public:
    typedef typename pointer_traits<_VoidPtr>::template
            rebind<__tree_node_base>::other
                                                pointer;
    typedef typename pointer_traits<_VoidPtr>::template
            rebind<const __tree_node_base>::other
                                                const_pointer;
    typedef __tree_end_node<pointer> base;

    pointer __right_;
    pointer __parent_;
    bool __is_black_;

    DEVICE __tree_node_base()
        : __right_(), __parent_(), __is_black_(false) {}
};

template <class _Tp, class _VoidPtr>
class __tree_node
    : public __tree_node_base<_VoidPtr>
{
public:
    typedef __tree_node_base<_VoidPtr> base;
    typedef _Tp value_type;

    value_type __value_;

    DEVICE explicit __tree_node(const value_type& __v)
        : __value_(__v) {}
};

template <class _TreeIterator> class __map_iterator;
template <class _TreeIterator> class __map_const_iterator;

template <class _Tp, class _NodePtr, class _DiffType>
class __tree_iterator
{
    typedef _NodePtr                                              __node_pointer;
    typedef typename pointer_traits<__node_pointer>::element_type __node;
    typedef typename __node::base                                 __node_base;
    typedef typename __node_base::pointer                         __node_base_pointer;

    __node_pointer __ptr_;

    typedef pointer_traits<__node_pointer> __pointer_traits;
public:
    typedef bidirectional_iterator_tag iterator_category;
    typedef _Tp                        value_type;
    typedef _DiffType                  difference_type;
    typedef value_type&                reference;
    typedef typename pointer_traits<__node_pointer>::template
            rebind<value_type>::other         pointer;

    DEVICE __tree_iterator() {}

    DEVICE reference operator*() const {return __ptr_->__value_;}
    DEVICE pointer operator->() const {return &__ptr_->__value_;}

    DEVICE __tree_iterator& operator++()
        {__ptr_ = static_cast<__node_pointer>(__tree_next(static_cast<__node_base_pointer>(__ptr_)));
         return *this;}
    DEVICE __tree_iterator operator++(int)
        {__tree_iterator __t(*this); ++(*this); return __t;}

    DEVICE __tree_iterator& operator--()
        {__ptr_ = static_cast<__node_pointer>(__tree_prev(static_cast<__node_base_pointer>(__ptr_)));
         return *this;}
    DEVICE __tree_iterator operator--(int)
        {__tree_iterator __t(*this); --(*this); return __t;}

    friend
        DEVICE bool operator==(const __tree_iterator& __x, const __tree_iterator& __y)
        {return __x.__ptr_ == __y.__ptr_;}
    friend         DEVICE bool operator!=(const __tree_iterator& __x, const __tree_iterator& __y)
        {return !(__x == __y);}

private:
       DEVICE explicit __tree_iterator(__node_pointer __p) : __ptr_(__p) {}
    template <class, class, class> friend class __tree;
    template <class, class, class> friend class __tree_const_iterator;
    template <class> friend class __map_iterator;
    template <class, class, class> friend class RedBlackTree;
    template <class, class, class, class> friend class multimap;
    template <class, class, class> friend class set;
    template <class, class, class> friend class multiset;
};

template <class _Tp, class _ConstNodePtr, class _DiffType>
class __tree_const_iterator
{
    typedef _ConstNodePtr                                         __node_pointer;
    typedef typename pointer_traits<__node_pointer>::element_type __node;
    typedef const typename __node::base                           __node_base;
    typedef typename pointer_traits<__node_pointer>::template
            rebind<__node_base>::other

                                                                  __node_base_pointer;

    __node_pointer __ptr_;

    typedef pointer_traits<__node_pointer> __pointer_traits;
public:
    typedef bidirectional_iterator_tag       iterator_category;
    typedef _Tp                              value_type;
    typedef _DiffType                        difference_type;
    typedef const value_type&                reference;
    typedef typename pointer_traits<__node_pointer>::template
            rebind<const value_type>::other

                                       pointer;

    DEVICE __tree_const_iterator() {}
private:
    typedef typename remove_const<__node>::type  __non_const_node;
    typedef typename pointer_traits<__node_pointer>::template
            rebind<__non_const_node>::other

                                                 __non_const_node_pointer;
    typedef __tree_iterator<value_type, __non_const_node_pointer, difference_type>
                                                 __non_const_iterator;
public:
   DEVICE  __tree_const_iterator(__non_const_iterator __p)
        : __ptr_(__p.__ptr_) {}

    DEVICE reference operator*() const {return __ptr_->__value_;}
    DEVICE pointer operator->() const {return &__ptr_->__value_;}

    DEVICE __tree_const_iterator& operator++()
        {__ptr_ = static_cast<__node_pointer>(__tree_next(static_cast<__node_base_pointer>(__ptr_)));
         return *this;}
    DEVICE __tree_const_iterator operator++(int)
        {__tree_const_iterator __t(*this); ++(*this); return __t;}

    DEVICE __tree_const_iterator& operator--()
        {__ptr_ = static_cast<__node_pointer>(__tree_prev(static_cast<__node_base_pointer>(__ptr_)));
         return *this;}
    DEVICE __tree_const_iterator operator--(int)
        {__tree_const_iterator __t(*this); --(*this); return __t;}

    friend         DEVICE bool operator==(const __tree_const_iterator& __x, const __tree_const_iterator& __y)
        {return __x.__ptr_ == __y.__ptr_;}
    friend         DEVICE bool operator!=(const __tree_const_iterator& __x, const __tree_const_iterator& __y)
        {return !(__x == __y);}

private:
        DEVICE explicit __tree_const_iterator(__node_pointer __p)
        : __ptr_(__p) {}
    template <class, class, class> friend class RedBlackTree;
    template <class, class, class, class> friend class map;
    template <class, class, class, class> friend class multimap;
    template <class, class, class> friend class set;
    template <class, class, class> friend class multiset;
    template <class> friend class __map_const_iterator;
};

template <class _Tp, class _Compare, class _Allocator>
class RedBlackTree
{
public:
    typedef _Tp                                      value_type;
    typedef _Compare                                 value_compare;
    typedef _Allocator                               allocator_type;
    typedef allocator_traits<allocator_type>         __alloc_traits;
    typedef typename __alloc_traits::pointer         pointer;
    typedef typename __alloc_traits::const_pointer   const_pointer;
    typedef typename __alloc_traits::size_type       size_type;
    typedef typename __alloc_traits::difference_type difference_type;

    typedef __tree_node<value_type, typename __alloc_traits::void_pointer> __node;
    typedef __tree_node_base<typename __alloc_traits::void_pointer> __node_base;
    typedef typename __alloc_traits::template
            rebind_alloc<__node>::other

                                                     __node_allocator;
    typedef allocator_traits<__node_allocator>       __node_traits;
    typedef typename __node_traits::pointer          __node_pointer;
    typedef typename __node_traits::const_pointer    __node_const_pointer;
    typedef typename __node_base::pointer            __node_base_pointer;
    typedef typename __node_base::const_pointer      __node_base_const_pointer;
private:
    typedef typename __node_base::base __end_node_t;
    typedef typename pointer_traits<__node_pointer>::template
            rebind<__end_node_t>::other
                                                     __end_node_ptr;
    typedef typename pointer_traits<__node_pointer>::template
            rebind<const __end_node_t>::other
                                                     __end_node_const_ptr;

    __node_pointer                                          __begin_node_;
    pair<__end_node_t, __node_allocator>  __pair1_;
    pair<size_type, value_compare>        __pair3_;

public:
    DEVICE __node_pointer __end_node()
    {
        return static_cast<__node_pointer>
               (
                   pointer_traits<__end_node_ptr>::pointer_to(__pair1_.first)
               );
    }
    DEVICE __node_const_pointer __end_node() const
    {
        return static_cast<__node_const_pointer>
               (
                   pointer_traits<__end_node_const_ptr>::pointer_to(__pair1_.first)
               );
    }

	DEVICE __node_allocator& __node_alloc() {return __pair1_.second;}

private:
        DEVICE const __node_allocator& __node_alloc() const
        {return __pair1_.second;}
              DEVICE __node_pointer& __begin_node() {return __begin_node_;}
        DEVICE const __node_pointer& __begin_node() const {return __begin_node_;}
public:
        DEVICE allocator_type __alloc() const
        	{return allocator_type(__node_alloc());}
private:
              DEVICE size_type& size() {return __pair3_.first;}
public:
        DEVICE const size_type& size() const {return __pair3_.first;}
		DEVICE value_compare& value_comp() {return __pair3_.second;}
        DEVICE const value_compare& value_comp() const
        	{return __pair3_.second;}
public:
		DEVICE __node_pointer __root()
        {return static_cast<__node_pointer>      (__end_node()->__left_);}
		DEVICE __node_const_pointer __root() const
        {return static_cast<__node_const_pointer>(__end_node()->__left_);}

    typedef __tree_iterator<value_type, __node_pointer, difference_type>             iterator;
    typedef __tree_const_iterator<value_type, __node_const_pointer, difference_type> const_iterator;

    DEVICE explicit RedBlackTree(const value_compare& __comp);
    DEVICE explicit RedBlackTree(const allocator_type& __a);
    DEVICE RedBlackTree(const value_compare& __comp, const allocator_type& __a);
    DEVICE RedBlackTree(const RedBlackTree& __t);
    DEVICE RedBlackTree& operator=(const RedBlackTree& __t);
    template <class _InputIterator>
        DEVICE void __assign_unique(_InputIterator __first, _InputIterator __last);
    template <class _InputIterator>
        DEVICE void __assign_multi(_InputIterator __first, _InputIterator __last);

    DEVICE ~RedBlackTree();

	DEVICE iterator begin()  {return       iterator(__begin_node());}
	DEVICE const_iterator begin() const {return const_iterator(__begin_node());}
	DEVICE iterator end() {return       iterator(__end_node());}
	DEVICE const_iterator end() const {return const_iterator(__end_node());}

	DEVICE size_type max_size() const
        {return __node_traits::max_size(__node_alloc());}

    DEVICE void clear();

    DEVICE void swap(RedBlackTree& __t);

    DEVICE pair<iterator, bool> __insert_unique(const value_type& __v);
    DEVICE iterator __insert_unique(const_iterator __p, const value_type& __v);
    DEVICE iterator __insert_multi(const value_type& __v);
    DEVICE iterator __insert_multi(const_iterator __p, const value_type& __v);

    DEVICE pair<iterator, bool> __node_insert_unique(__node_pointer __nd);
    DEVICE iterator             __node_insert_unique(const_iterator __p,
                                              __node_pointer __nd);

    DEVICE iterator __node_insert_multi(__node_pointer __nd);
    DEVICE iterator __node_insert_multi(const_iterator __p, __node_pointer __nd);

    DEVICE iterator erase(const_iterator __p);
    DEVICE iterator erase(const_iterator __f, const_iterator __l);
    template <class _Key>
        DEVICE size_type __erase_unique(const _Key& __k);
    template <class _Key>
        DEVICE size_type __erase_multi(const _Key& __k);

    DEVICE void __insert_node_at(__node_base_pointer __parent,
                          __node_base_pointer& __child,
                          __node_base_pointer __new_node);

    template <class _Key>
        DEVICE iterator find(const _Key& __v);
    template <class _Key>
        DEVICE const_iterator find(const _Key& __v) const;

    template <class _Key>
        DEVICE size_type __count_unique(const _Key& __k) const;
    template <class _Key>
        DEVICE size_type __count_multi(const _Key& __k) const;

    template <class _Key>
    	DEVICE iterator lower_bound(const _Key& __v)
            {return __lower_bound(__v, __root(), __end_node());}
    template <class _Key>
    	DEVICE iterator __lower_bound(const _Key& __v,
                               __node_pointer __root,
                               __node_pointer __result);
    template <class _Key>
   		DEVICE const_iterator lower_bound(const _Key& __v) const
            {return __lower_bound(__v, __root(), __end_node());}
    template <class _Key>
    	DEVICE const_iterator __lower_bound(const _Key& __v,
                                     __node_const_pointer __root,
                                     __node_const_pointer __result) const;
    template <class _Key>
    	DEVICE iterator upper_bound(const _Key& __v)
            {return __upper_bound(__v, __root(), __end_node());}
    template <class _Key>
    	DEVICE iterator __upper_bound(const _Key& __v,
                               __node_pointer __root,
                               __node_pointer __result);
    template <class _Key>
    	DEVICE const_iterator upper_bound(const _Key& __v) const
            {return __upper_bound(__v, __root(), __end_node());}
    template <class _Key>
    	DEVICE const_iterator __upper_bound(const _Key& __v,
                                     __node_const_pointer __root,
                                     __node_const_pointer __result) const;
    template <class _Key>
    	DEVICE pair<iterator, iterator>
        __equal_range_unique(const _Key& __k);
    template <class _Key>
    	DEVICE pair<const_iterator, const_iterator>
        __equal_range_unique(const _Key& __k) const;

    template <class _Key>
    	DEVICE pair<iterator, iterator>
        __equal_range_multi(const _Key& __k);
    template <class _Key>
    	DEVICE pair<const_iterator, const_iterator>
        __equal_range_multi(const _Key& __k) const;

    typedef __tree_node_destructor<__node_allocator> _Dp;
    typedef unique_ptr<__node, _Dp> __node_holder;

    DEVICE __node_holder remove(const_iterator __p);
private:
    DEVICE typename __node_base::pointer&
        __find_leaf_low(typename __node_base::pointer& __parent, const value_type& __v);
    DEVICE typename __node_base::pointer&
        __find_leaf_high(typename __node_base::pointer& __parent, const value_type& __v);
    DEVICE typename __node_base::pointer&
        __find_leaf(const_iterator __hint,
                    typename __node_base::pointer& __parent, const value_type& __v);
    template <class _Key>
        DEVICE typename __node_base::pointer&
        __find_equal(typename __node_base::pointer& __parent, const _Key& __v);
    template <class _Key>
        DEVICE typename __node_base::pointer&
        __find_equal(const_iterator __hint, typename __node_base::pointer& __parent,
                     const _Key& __v);

    DEVICE __node_holder __construct_node(const value_type& __v);

    DEVICE void destroy(__node_pointer __nd);

    DEVICE void __copy_assign_alloc(const RedBlackTree& __t)
        {__copy_assign_alloc(__t, integral_constant<bool,
             __node_traits::propagate_on_container_copy_assignment::value>());}

    DEVICE void __copy_assign_alloc(const RedBlackTree& __t, true_type)
        {__node_alloc() = __t.__node_alloc();}
        void __copy_assign_alloc(const RedBlackTree& __t, false_type) {}

    DEVICE void __move_assign(RedBlackTree& __t, false_type);
    DEVICE void __move_assign(RedBlackTree& __t, true_type);

    DEVICE void __move_assign_alloc(RedBlackTree& __t)
    {__move_assign_alloc(__t, integral_constant<bool,
         __node_traits::propagate_on_container_move_assignment::value>());}

    DEVICE void __move_assign_alloc(RedBlackTree& __t, true_type)
    {__node_alloc() = util::move(__t.__node_alloc());}
    DEVICE void __move_assign_alloc(RedBlackTree& __t, false_type) {}

    DEVICE static void __swap_alloc(__node_allocator& __x, __node_allocator& __y)
    {__swap_alloc(__x, __y, integral_constant<bool,
                  __node_traits::propagate_on_container_swap::value>());}
    DEVICE static void __swap_alloc(__node_allocator& __x, __node_allocator& __y, true_type)
    {
        using util::swap;
        swap(__x, __y);
    }
    DEVICE static void __swap_alloc(__node_allocator& __x, __node_allocator& __y, false_type)
    {}

    DEVICE __node_pointer __detach();
    DEVICE static __node_pointer __detach(__node_pointer);
};

template <class _Tp, class _Compare, class _Allocator>
DEVICE RedBlackTree<_Tp, _Compare, _Allocator>::RedBlackTree(const value_compare& __comp)
    : __pair3_(0, __comp)
{
    __begin_node() = __end_node();
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE RedBlackTree<_Tp, _Compare, _Allocator>::RedBlackTree(const allocator_type& __a)
    : __pair1_(__node_allocator(__a)),
      __begin_node_(__node_pointer()),
      __pair3_(0)
{
    __begin_node() = __end_node();
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE RedBlackTree<_Tp, _Compare, _Allocator>::RedBlackTree(const value_compare& __comp,
                                           const allocator_type& __a)
    : __pair1_(__node_allocator(__a)),
      __begin_node_(__node_pointer()),
      __pair3_(0, __comp)
{
    __begin_node() = __end_node();
}

// Precondition:  size() != 0
template <class _Tp, class _Compare, class _Allocator>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::__node_pointer
RedBlackTree<_Tp, _Compare, _Allocator>::__detach()
{
    __node_pointer __cache = __begin_node();
    __begin_node() = __end_node();
    __end_node()->__left_->__parent_ = 0;
    __end_node()->__left_ = 0;
    size() = 0;
    // __cache->__left_ == 0
    if (__cache->__right_ != 0)
        __cache = static_cast<__node_pointer>(__cache->__right_);
    // __cache->__left_ == 0
    // __cache->__right_ == 0
    return __cache;
}

// Precondition:  __cache != 0
//    __cache->left_ == 0
//    __cache->right_ == 0
//    This is no longer a red-black tree
template <class _Tp, class _Compare, class _Allocator>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::__node_pointer
RedBlackTree<_Tp, _Compare, _Allocator>::__detach(__node_pointer __cache)
{
    if (__cache->__parent_ == 0)
        return 0;
    if (__tree_is_left_child(__cache))
    {
        __cache->__parent_->__left_ = 0;
        __cache = static_cast<__node_pointer>(__cache->__parent_);
        if (__cache->__right_ == 0)
            return __cache;
        return static_cast<__node_pointer>(__tree_leaf(__cache->__right_));
    }
    // __cache is right child
    __cache->__parent_->__right_ = 0;
    __cache = static_cast<__node_pointer>(__cache->__parent_);
    if (__cache->__left_ == 0)
        return __cache;
    return static_cast<__node_pointer>(__tree_leaf(__cache->__left_));
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE RedBlackTree<_Tp, _Compare, _Allocator>&
RedBlackTree<_Tp, _Compare, _Allocator>::operator=(const RedBlackTree& __t)
{
    if (this != &__t)
    {
        value_comp() = __t.value_comp();
        __copy_assign_alloc(__t);
        __assign_multi(__t.begin(), __t.end());
    }
    return *this;
}

template <class _Tp, class _Compare, class _Allocator>
template <class _InputIterator>
DEVICE void
RedBlackTree<_Tp, _Compare, _Allocator>::__assign_unique(_InputIterator __first, _InputIterator __last)
{
    if (size() != 0)
    {
        __node_pointer __cache = __detach();
            for (; __cache != 0 && __first != __last; ++__first)
            {
                __cache->__value_ = *__first;
                __node_pointer __next = __detach(__cache);
                __node_insert_unique(__cache);
                __cache = __next;
            }
        if (__cache != 0)
        {
            while (__cache->__parent_ != 0)
                __cache = static_cast<__node_pointer>(__cache->__parent_);
            destroy(__cache);
        }
    }
    for (; __first != __last; ++__first)
        __insert_unique(*__first);
}

template <class _Tp, class _Compare, class _Allocator>
template <class _InputIterator>
DEVICE void
RedBlackTree<_Tp, _Compare, _Allocator>::__assign_multi(_InputIterator __first, _InputIterator __last)
{
    if (size() != 0)
    {
        __node_pointer __cache = __detach();
            for (; __cache != 0 && __first != __last; ++__first)
            {
                __cache->__value_ = *__first;
                __node_pointer __next = __detach(__cache);
                __node_insert_multi(__cache);
                __cache = __next;
            }
        if (__cache != 0)
        {
            while (__cache->__parent_ != 0)
                __cache = static_cast<__node_pointer>(__cache->__parent_);
            destroy(__cache);
        }
    }
    for (; __first != __last; ++__first)
        __insert_multi(*__first);
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE RedBlackTree<_Tp, _Compare, _Allocator>::RedBlackTree(const RedBlackTree& __t)
    : __begin_node_(__node_pointer()),
      __pair1_(__node_traits::select_on_container_copy_construction(__t.__node_alloc())),
      __pair3_(0, __t.value_comp())
{
    __begin_node() = __end_node();
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE RedBlackTree<_Tp, _Compare, _Allocator>::~RedBlackTree()
{
    destroy(__root());
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE void
RedBlackTree<_Tp, _Compare, _Allocator>::destroy(__node_pointer __nd)
{
    if (__nd != 0)
    {
        destroy(static_cast<__node_pointer>(__nd->__left_));
        destroy(static_cast<__node_pointer>(__nd->__right_));
        __node_allocator& __na = __node_alloc();
        __node_traits::destroy(__na, util::addressof(__nd->__value_));
        __node_traits::deallocate(__na, __nd, 1);
    }
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE void
RedBlackTree<_Tp, _Compare, _Allocator>::swap(RedBlackTree& __t)
{
    using util::swap;
    swap(__begin_node_, __t.__begin_node_);
    swap(__pair1_.first(), __t.__pair1_.first());
    __swap_alloc(__node_alloc(), __t.__node_alloc());
    __pair3_.swap(__t.__pair3_);
    if (size() == 0)
        __begin_node() = __end_node();
    else
        __end_node()->__left_->__parent_ = __end_node();
    if (__t.size() == 0)
        __t.__begin_node() = __t.__end_node();
    else
        __t.__end_node()->__left_->__parent_ = __t.__end_node();
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE void
RedBlackTree<_Tp, _Compare, _Allocator>::clear()
{
    destroy(__root());
    size() = 0;
    __begin_node() = __end_node();
    __end_node()->__left_ = 0;
}

// Find lower_bound place to insert
// Set __parent to parent of null leaf
// Return reference to null leaf
template <class _Tp, class _Compare, class _Allocator>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::__node_base::pointer&
RedBlackTree<_Tp, _Compare, _Allocator>::__find_leaf_low(typename __node_base::pointer& __parent,
                                                   const value_type& __v)
{
    __node_pointer __nd = __root();
    if (__nd != 0)
    {
        while (true)
        {
            if (value_comp()(__nd->__value_, __v))
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
                if (__nd->__left_ != 0)
                    __nd = static_cast<__node_pointer>(__nd->__left_);
                else
                {
                    __parent = __nd;
                    return __parent->__left_;
                }
            }
        }
    }
    __parent = __end_node();
    return __parent->__left_;
}

// Find upper_bound place to insert
// Set __parent to parent of null leaf
// Return reference to null leaf
template <class _Tp, class _Compare, class _Allocator>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::__node_base::pointer&
RedBlackTree<_Tp, _Compare, _Allocator>::__find_leaf_high(typename __node_base::pointer& __parent,
                                                    const value_type& __v)
{
    __node_pointer __nd = __root();
    if (__nd != 0)
    {
        while (true)
        {
            if (value_comp()(__v, __nd->__value_))
            {
                if (__nd->__left_ != 0)
                    __nd = static_cast<__node_pointer>(__nd->__left_);
                else
                {
                    __parent = __nd;
                    return __parent->__left_;
                }
            }
            else
            {
                if (__nd->__right_ != 0)
                    __nd = static_cast<__node_pointer>(__nd->__right_);
                else
                {
                    __parent = __nd;
                    return __parent->__right_;
                }
            }
        }
    }
    __parent = __end_node();
    return __parent->__left_;
}

// Find leaf place to insert closest to __hint
// First check prior to __hint.
// Next check after __hint.
// Next do O(log N) search.
// Set __parent to parent of null leaf
// Return reference to null leaf
template <class _Tp, class _Compare, class _Allocator>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::__node_base::pointer&
RedBlackTree<_Tp, _Compare, _Allocator>::__find_leaf(const_iterator __hint,
                                               typename __node_base::pointer& __parent,
                                               const value_type& __v)
{
    if (__hint == end() || !value_comp()(*__hint, __v))  // check before
    {
        // __v <= *__hint
        const_iterator __prior = __hint;
        if (__prior == begin() || !value_comp()(__v, *--__prior))
        {
            // *prev(__hint) <= __v <= *__hint
            if (__hint.__ptr_->__left_ == 0)
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
        // __v < *prev(__hint)
        return __find_leaf_high(__parent, __v);
    }
    // else __v > *__hint
    return __find_leaf_low(__parent, __v);
}

// Find place to insert if __v doesn't exist
// Set __parent to parent of null leaf
// Return reference to null leaf
// If __v exists, set parent to node of __v and return reference to node of __v
template <class _Tp, class _Compare, class _Allocator>
template <class _Key>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::__node_base::pointer&
RedBlackTree<_Tp, _Compare, _Allocator>::__find_equal(typename __node_base::pointer& __parent,
                                                const _Key& __v)
{
    __node_pointer __nd = __root();
    if (__nd != 0)
    {
        while (true)
        {
            if (value_comp()(__v, __nd->__value_))
            {
                if (__nd->__left_ != 0)
                    __nd = static_cast<__node_pointer>(__nd->__left_);
                else
                {
                    __parent = __nd;
                    return __parent->__left_;
                }
            }
            else if (value_comp()(__nd->__value_, __v))
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
    __parent = __end_node();
    return __parent->__left_;
}

// Find place to insert if __v doesn't exist
// First check prior to __hint.
// Next check after __hint.
// Next do O(log N) search.
// Set __parent to parent of null leaf
// Return reference to null leaf
// If __v exists, set parent to node of __v and return reference to node of __v
template <class _Tp, class _Compare, class _Allocator>
template <class _Key>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::__node_base::pointer&
RedBlackTree<_Tp, _Compare, _Allocator>::__find_equal(const_iterator __hint,
                                                typename __node_base::pointer& __parent,
                                                const _Key& __v)
{
    if (__hint == end() || value_comp()(__v, *__hint))  // check before
    {
        // __v < *__hint
        const_iterator __prior = __hint;
        if (__prior == begin() || value_comp()(*--__prior, __v))
        {
            // *prev(__hint) < __v < *__hint
            if (__hint.__ptr_->__left_ == 0)
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
        // __v <= *prev(__hint)
        return __find_equal(__parent, __v);
    }
    else if (value_comp()(*__hint, __v))  // check after
    {
        // *__hint < __v
        const_iterator __next = util::next(__hint);
        if (__next == end() || value_comp()(__v, *__next))
        {
            // *__hint < __v < *util::next(__hint)
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
        // *next(__hint) <= __v
        return __find_equal(__parent, __v);
    }
    // else __v == *__hint
    __parent = const_cast<__node_pointer&>(__hint.__ptr_);
    return __parent;
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE void
RedBlackTree<_Tp, _Compare, _Allocator>::__insert_node_at(__node_base_pointer __parent,
                                                    __node_base_pointer& __child,
                                                    __node_base_pointer __new_node)
{
    __new_node->__left_   = 0;
    __new_node->__right_  = 0;
    __new_node->__parent_ = __parent;
    __child = __new_node;
    if (__begin_node()->__left_ != 0)
        __begin_node() = static_cast<__node_pointer>(__begin_node()->__left_);
    __tree_balance_after_insert(__end_node()->__left_, __child);
    ++size();
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE pair<typename RedBlackTree<_Tp, _Compare, _Allocator>::iterator, bool>
RedBlackTree<_Tp, _Compare, _Allocator>::__insert_unique(const value_type& __v)
{
    __node_base_pointer __parent;
    __node_base_pointer& __child = __find_equal(__parent, __v);
    __node_pointer __r = static_cast<__node_pointer>(__child);
    bool __inserted = false;
    if (__child == 0)
    {
        __node_holder __h = __construct_node(__v);
        __insert_node_at(__parent, __child, __h.get());
        __r = __h.release();
        __inserted = true;
    }
    return pair<iterator, bool>(iterator(__r), __inserted);
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::iterator
RedBlackTree<_Tp, _Compare, _Allocator>::__insert_unique(const_iterator __p, const value_type& __v)
{
    __node_base_pointer __parent;
    __node_base_pointer& __child = __find_equal(__p, __parent, __v);
    __node_pointer __r = static_cast<__node_pointer>(__child);
    if (__child == 0)
    {
        __node_holder __h = __construct_node(__v);
        __insert_node_at(__parent, __child, __h.get());
        __r = __h.release();
    }
    return iterator(__r);
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::iterator
RedBlackTree<_Tp, _Compare, _Allocator>::__insert_multi(const value_type& __v)
{
    __node_base_pointer __parent;
    __node_base_pointer& __child = __find_leaf_high(__parent, __v);
    __node_holder __h = __construct_node(__v);
    __insert_node_at(__parent, __child, __h.get());
    return iterator(__h.release());
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::iterator
RedBlackTree<_Tp, _Compare, _Allocator>::__insert_multi(const_iterator __p, const value_type& __v)
{
    __node_base_pointer __parent;
    __node_base_pointer& __child = __find_leaf(__p, __parent, __v);
    __node_holder __h = __construct_node(__v);
    __insert_node_at(__parent, __child, __h.get());
    return iterator(__h.release());
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE pair<typename RedBlackTree<_Tp, _Compare, _Allocator>::iterator, bool>
RedBlackTree<_Tp, _Compare, _Allocator>::__node_insert_unique(__node_pointer __nd)
{
    __node_base_pointer __parent;
    __node_base_pointer& __child = __find_equal(__parent, __nd->__value_);
    __node_pointer __r = static_cast<__node_pointer>(__child);
    bool __inserted = false;
    if (__child == 0)
    {
        __insert_node_at(__parent, __child, __nd);
        __r = __nd;
        __inserted = true;
    }
    return pair<iterator, bool>(iterator(__r), __inserted);
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::iterator
RedBlackTree<_Tp, _Compare, _Allocator>::__node_insert_unique(const_iterator __p,
                                                        __node_pointer __nd)
{
    __node_base_pointer __parent;
    __node_base_pointer& __child = __find_equal(__p, __parent, __nd->__value_);
    __node_pointer __r = static_cast<__node_pointer>(__child);
    if (__child == 0)
    {
        __insert_node_at(__parent, __child, __nd);
        __r = __nd;
    }
    return iterator(__r);
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::iterator
RedBlackTree<_Tp, _Compare, _Allocator>::__node_insert_multi(__node_pointer __nd)
{
    __node_base_pointer __parent;
    __node_base_pointer& __child = __find_leaf_high(__parent, __nd->__value_);
    __insert_node_at(__parent, __child, __nd);
    return iterator(__nd);
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::iterator
RedBlackTree<_Tp, _Compare, _Allocator>::__node_insert_multi(const_iterator __p,
                                                       __node_pointer __nd)
{
    __node_base_pointer __parent;
    __node_base_pointer& __child = __find_leaf(__p, __parent, __nd->__value_);
    __insert_node_at(__parent, __child, __nd);
    return iterator(__nd);
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::iterator
RedBlackTree<_Tp, _Compare, _Allocator>::erase(const_iterator __p)
{
    __node_pointer __np = const_cast<__node_pointer>(__p.__ptr_);
    iterator __r(__np);
    ++__r;
    if (__begin_node() == __np)
        __begin_node() = __r.__ptr_;
    --size();
    __node_allocator& __na = __node_alloc();
    __node_traits::destroy(__na, const_cast<value_type*>(util::addressof(*__p)));
    __tree_remove(__end_node()->__left_,
                  static_cast<__node_base_pointer>(__np));
    __node_traits::deallocate(__na, __np, 1);
    return __r;
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::iterator
RedBlackTree<_Tp, _Compare, _Allocator>::erase(const_iterator __f, const_iterator __l)
{
    while (__f != __l)
        __f = erase(__f);
    return iterator(const_cast<__node_pointer>(__l.__ptr_));
}

template <class _Tp, class _Compare, class _Allocator>
template <class _Key>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::size_type
RedBlackTree<_Tp, _Compare, _Allocator>::__erase_unique(const _Key& __k)
{
    iterator __i = find(__k);
    if (__i == end())
        return 0;
    erase(__i);
    return 1;
}

template <class _Tp, class _Compare, class _Allocator>
template <class _Key>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::size_type
RedBlackTree<_Tp, _Compare, _Allocator>::__erase_multi(const _Key& __k)
{
    pair<iterator, iterator> __p = __equal_range_multi(__k);
    size_type __r = 0;
    for (; __p.first != __p.second; ++__r)
        __p.first = erase(__p.first);
    return __r;
}

template <class _Tp, class _Compare, class _Allocator>
template <class _Key>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::iterator
RedBlackTree<_Tp, _Compare, _Allocator>::find(const _Key& __v)
{
    iterator __p = __lower_bound(__v, __root(), __end_node());
    if (__p != end() && !value_comp()(__v, *__p))
        return __p;
    return end();
}

template <class _Tp, class _Compare, class _Allocator>
template <class _Key>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::const_iterator
RedBlackTree<_Tp, _Compare, _Allocator>::find(const _Key& __v) const
{
    const_iterator __p = __lower_bound(__v, __root(), __end_node());
    if (__p != end() && !value_comp()(__v, *__p))
        return __p;
    return end();
}

template <class _Tp, class _Compare, class _Allocator>
template <class _Key>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::size_type
RedBlackTree<_Tp, _Compare, _Allocator>::__count_unique(const _Key& __k) const
{
    __node_const_pointer __result = __end_node();
    __node_const_pointer __rt = __root();
    while (__rt != 0)
    {
        if (value_comp()(__k, __rt->__value_))
        {
            __result = __rt;
            __rt = static_cast<__node_const_pointer>(__rt->__left_);
        }
        else if (value_comp()(__rt->__value_, __k))
            __rt = static_cast<__node_const_pointer>(__rt->__right_);
        else
            return 1;
    }
    return 0;
}

template <class _Tp, class _Compare, class _Allocator>
template <class _Key>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::size_type
RedBlackTree<_Tp, _Compare, _Allocator>::__count_multi(const _Key& __k) const
{
    typedef pair<const_iterator, const_iterator> _Pp;
    __node_const_pointer __result = __end_node();
    __node_const_pointer __rt = __root();
    while (__rt != 0)
    {
        if (value_comp()(__k, __rt->__value_))
        {
            __result = __rt;
            __rt = static_cast<__node_const_pointer>(__rt->__left_);
        }
        else if (value_comp()(__rt->__value_, __k))
            __rt = static_cast<__node_const_pointer>(__rt->__right_);
        else
            return util::distance(
                __lower_bound(__k, static_cast<__node_const_pointer>(__rt->__left_), __rt),
                __upper_bound(__k, static_cast<__node_const_pointer>(__rt->__right_), __result)
            );
    }
    return 0;
}

template <class _Tp, class _Compare, class _Allocator>
template <class _Key>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::iterator
RedBlackTree<_Tp, _Compare, _Allocator>::__lower_bound(const _Key& __v,
                                                 __node_pointer __root,
                                                 __node_pointer __result)
{
    while (__root != 0)
    {
        if (!value_comp()(__root->__value_, __v))
        {
            __result = __root;
            __root = static_cast<__node_pointer>(__root->__left_);
        }
        else
            __root = static_cast<__node_pointer>(__root->__right_);
    }
    return iterator(__result);
}

template <class _Tp, class _Compare, class _Allocator>
template <class _Key>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::const_iterator
RedBlackTree<_Tp, _Compare, _Allocator>::__lower_bound(const _Key& __v,
                                                 __node_const_pointer __root,
                                                 __node_const_pointer __result) const
{
    while (__root != 0)
    {
        if (!value_comp()(__root->__value_, __v))
        {
            __result = __root;
            __root = static_cast<__node_const_pointer>(__root->__left_);
        }
        else
            __root = static_cast<__node_const_pointer>(__root->__right_);
    }
    return const_iterator(__result);
}

template <class _Tp, class _Compare, class _Allocator>
template <class _Key>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::iterator
RedBlackTree<_Tp, _Compare, _Allocator>::__upper_bound(const _Key& __v,
                                                 __node_pointer __root,
                                                 __node_pointer __result)
{
    while (__root != 0)
    {
        if (value_comp()(__v, __root->__value_))
        {
            __result = __root;
            __root = static_cast<__node_pointer>(__root->__left_);
        }
        else
            __root = static_cast<__node_pointer>(__root->__right_);
    }
    return iterator(__result);
}

template <class _Tp, class _Compare, class _Allocator>
template <class _Key>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::const_iterator
RedBlackTree<_Tp, _Compare, _Allocator>::__upper_bound(const _Key& __v,
                                                 __node_const_pointer __root,
                                                 __node_const_pointer __result) const
{
    while (__root != 0)
    {
        if (value_comp()(__v, __root->__value_))
        {
            __result = __root;
            __root = static_cast<__node_const_pointer>(__root->__left_);
        }
        else
            __root = static_cast<__node_const_pointer>(__root->__right_);
    }
    return const_iterator(__result);
}

template <class _Tp, class _Compare, class _Allocator>
template <class _Key>
DEVICE pair<typename RedBlackTree<_Tp, _Compare, _Allocator>::iterator,
     typename RedBlackTree<_Tp, _Compare, _Allocator>::iterator>
RedBlackTree<_Tp, _Compare, _Allocator>::__equal_range_unique(const _Key& __k)
{
    typedef pair<iterator, iterator> _Pp;
    __node_pointer __result = __end_node();
    __node_pointer __rt = __root();
    while (__rt != 0)
    {
        if (value_comp()(__k, __rt->__value_))
        {
            __result = __rt;
            __rt = static_cast<__node_pointer>(__rt->__left_);
        }
        else if (value_comp()(__rt->__value_, __k))
            __rt = static_cast<__node_pointer>(__rt->__right_);
        else
            return _Pp(iterator(__rt),
                      iterator(
                          __rt->__right_ != 0 ?
                              static_cast<__node_pointer>(__tree_min(__rt->__right_))
                            : __result));
    }
    return _Pp(iterator(__result), iterator(__result));
}

template <class _Tp, class _Compare, class _Allocator>
template <class _Key>
DEVICE pair<typename RedBlackTree<_Tp, _Compare, _Allocator>::const_iterator,
     typename RedBlackTree<_Tp, _Compare, _Allocator>::const_iterator>
RedBlackTree<_Tp, _Compare, _Allocator>::__equal_range_unique(const _Key& __k) const
{
    typedef pair<const_iterator, const_iterator> _Pp;
    __node_const_pointer __result = __end_node();
    __node_const_pointer __rt = __root();
    while (__rt != 0)
    {
        if (value_comp()(__k, __rt->__value_))
        {
            __result = __rt;
            __rt = static_cast<__node_const_pointer>(__rt->__left_);
        }
        else if (value_comp()(__rt->__value_, __k))
            __rt = static_cast<__node_const_pointer>(__rt->__right_);
        else
            return _Pp(const_iterator(__rt),
                      const_iterator(
                          __rt->__right_ != 0 ?
                              static_cast<__node_const_pointer>(__tree_min(__rt->__right_))
                            : __result));
    }
    return _Pp(const_iterator(__result), const_iterator(__result));
}

template <class _Tp, class _Compare, class _Allocator>
template <class _Key>
DEVICE pair<typename RedBlackTree<_Tp, _Compare, _Allocator>::iterator,
     typename RedBlackTree<_Tp, _Compare, _Allocator>::iterator>
RedBlackTree<_Tp, _Compare, _Allocator>::__equal_range_multi(const _Key& __k)
{
    typedef pair<iterator, iterator> _Pp;
    __node_pointer __result = __end_node();
    __node_pointer __rt = __root();
    while (__rt != 0)
    {
        if (value_comp()(__k, __rt->__value_))
        {
            __result = __rt;
            __rt = static_cast<__node_pointer>(__rt->__left_);
        }
        else if (value_comp()(__rt->__value_, __k))
            __rt = static_cast<__node_pointer>(__rt->__right_);
        else
            return _Pp(__lower_bound(__k, static_cast<__node_pointer>(__rt->__left_), __rt),
                      __upper_bound(__k, static_cast<__node_pointer>(__rt->__right_), __result));
    }
    return _Pp(iterator(__result), iterator(__result));
}

template <class _Tp, class _Compare, class _Allocator>
template <class _Key>
DEVICE pair<typename RedBlackTree<_Tp, _Compare, _Allocator>::const_iterator,
     typename RedBlackTree<_Tp, _Compare, _Allocator>::const_iterator>
RedBlackTree<_Tp, _Compare, _Allocator>::__equal_range_multi(const _Key& __k) const
{
    typedef pair<const_iterator, const_iterator> _Pp;
    __node_const_pointer __result = __end_node();
    __node_const_pointer __rt = __root();
    while (__rt != 0)
    {
        if (value_comp()(__k, __rt->__value_))
        {
            __result = __rt;
            __rt = static_cast<__node_const_pointer>(__rt->__left_);
        }
        else if (value_comp()(__rt->__value_, __k))
            __rt = static_cast<__node_const_pointer>(__rt->__right_);
        else
            return _Pp(__lower_bound(__k, static_cast<__node_const_pointer>(__rt->__left_), __rt),
                      __upper_bound(__k, static_cast<__node_const_pointer>(__rt->__right_), __result));
    }
    return _Pp(const_iterator(__result), const_iterator(__result));
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::__node_holder
RedBlackTree<_Tp, _Compare, _Allocator>::__construct_node(const value_type& __v)
{
    __node_allocator& __na = __node_alloc();
    __node_holder __h(__node_traits::allocate(__na, 1), _Dp(__na));
    __node_traits::construct(__na, util::addressof(__h->__value_), __v);
    __h.get_deleter().__value_constructed = true;
    return util::move(__h);
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE typename RedBlackTree<_Tp, _Compare, _Allocator>::__node_holder
RedBlackTree<_Tp, _Compare, _Allocator>::remove(const_iterator __p)
{
    __node_pointer __np = const_cast<__node_pointer>(__p.__ptr_);
    if (__begin_node() == __np)
    {
        if (__np->__right_ != 0)
            __begin_node() = static_cast<__node_pointer>(__np->__right_);
        else
            __begin_node() = static_cast<__node_pointer>(__np->__parent_);
    }
    --size();
    __tree_remove(__end_node()->__left_,
                  static_cast<__node_base_pointer>(__np));
    return __node_holder(__np, _Dp(__node_alloc()));
}

template <class _Tp, class _Compare, class _Allocator>
DEVICE inline void
swap(RedBlackTree<_Tp, _Compare, _Allocator>& __x,
     RedBlackTree<_Tp, _Compare, _Allocator>& __y)
{
    __x.swap(__y);
}

}

}




#endif /* GPUREDBLACKTREE_H_ */
