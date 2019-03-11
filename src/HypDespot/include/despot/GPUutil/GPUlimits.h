/*
 * GPUlimits.h
 *
 *  Created on: 28 Mar, 2017
 *      Author: panpan
 */

#ifndef GPULIMITS_H_
#define GPULIMITS_H_
/*! \file   limits.h
	\date   November 30, 2012
	\author Gregory Diamos <gregory.diamos@gatech.edu>
	\brief  The header file for the numeric_limits class.
*/

#pragma once

// Archaeopteryx Includes
#include <despot/GPUutil/GPUtype_traits.h>
#include <despot/GPUcore/CudaInclude.h>

#include <cfloat>

namespace archaeopteryx
{

namespace util
{

enum float_round_style
{
    round_indeterminate       = -1,
    round_toward_zero         =  0,
    round_to_nearest          =  1,
    round_toward_infinity     =  2,
    round_toward_neg_infinity =  3
};

enum float_denorm_style
{
    denorm_indeterminate = -1,
    denorm_absent = 0,
    denorm_present = 1
};

template <class _Tp, bool = is_arithmetic<_Tp>::value>
class __libcpp_numeric_limits
{
protected:
    typedef _Tp type;

    static const  bool is_specialized = false;
    DEVICE static type min() {return type();}
    DEVICE static type max() {return type();}
    DEVICE static type lowest() {return type();}

    static const int  digits = 0;
    static const int  digits10 = 0;
    static const int  max_digits10 = 0;
    static const bool is_signed = false;
    static const bool is_integer = false;
    static const bool is_exact = false;
    static const int  radix = 0;
    DEVICE static type epsilon() {return type();}
    DEVICE static type round_error() {return type();}

    static const int  min_exponent = 0;
    static const int  min_exponent10 = 0;
    static const int  max_exponent = 0;
    static const int  max_exponent10 = 0;

    static const bool has_infinity = false;
    static const bool has_quiet_NaN = false;
    static const bool has_signaling_NaN = false;
    static const float_denorm_style has_denorm = denorm_absent;
    static const bool has_denorm_loss = false;
    DEVICE static type infinity() {return type();}
    DEVICE static type quiet_NaN() {return type();}
    DEVICE static type signaling_NaN() {return type();}
    DEVICE static type denorm_min() {return type();}

    static const bool is_iec559 = false;
    static const bool is_bounded = false;
    static const bool is_modulo = false;

    static const bool traps = false;
    static const bool tinyness_before = false;
    static const float_round_style round_style = round_toward_zero;
};

template <class _Tp, int digits, bool is_signed>
struct __libcpp_compute_min
{
    static const _Tp value = _Tp(_Tp(1) << digits);
};

template <class _Tp, int digits>
struct __libcpp_compute_min<_Tp, digits, false>
{
    static const _Tp value = _Tp(0);
};

template <class _Tp>
class __libcpp_numeric_limits<_Tp, true>
{
protected:
    typedef _Tp type;

    static const bool is_specialized = true;

    static const bool is_signed = type(-1) < type(0);
    static const int  digits = static_cast<int>(sizeof(type) * __CHAR_BIT__ - is_signed);
    static const int  digits10 = digits * 3 / 10;
    static const int  max_digits10 = 0;
    static const type __min = __libcpp_compute_min<type, digits, is_signed>::value;
    static const type __max = is_signed ? type(type(~0) ^ __min) : type(~0);
    DEVICE static type min() {return __min;}
    DEVICE static type max() {return __max;}
    DEVICE static type lowest() {return min();}

    static const bool is_integer = true;
    static const bool is_exact = true;
    static const int  radix = 2;
    DEVICE static type epsilon() {return type(0);}
    DEVICE static type round_error() {return type(0);}

    static const int  min_exponent = 0;
    static const int  min_exponent10 = 0;
    static const int  max_exponent = 0;
    static const int  max_exponent10 = 0;

    static const bool has_infinity = false;
    static const bool has_quiet_NaN = false;
    static const bool has_signaling_NaN = false;
    static const float_denorm_style has_denorm = denorm_absent;
    static const bool has_denorm_loss = false;
    DEVICE static type infinity() {return type(0);}
    DEVICE static type quiet_NaN() {return type(0);}
    DEVICE static type signaling_NaN() {return type(0);}
    DEVICE static type denorm_min() {return type(0);}

    static const bool is_iec559 = false;
    static const bool is_bounded = true;
    static const bool is_modulo = true;

    static const bool traps = false;
    static const bool tinyness_before = false;
    static const float_round_style round_style = round_toward_zero;
};

template <>
class __libcpp_numeric_limits<bool, true>
{
protected:
    typedef bool type;

    static const bool is_specialized = true;

    static const bool is_signed = false;
    static const int  digits = 1;
    static const int  digits10 = 0;
    static const int  max_digits10 = 0;
    static const type __min = false;
    static const type __max = true;
    DEVICE static type min() {return __min;}
    DEVICE static type max() {return __max;}
    DEVICE static type lowest() {return min();}

    static const bool is_integer = true;
    static const bool is_exact = true;
    static const int  radix = 2;
    DEVICE static type epsilon() {return type(0);}
    DEVICE static type round_error() {return type(0);}

    static const int  min_exponent = 0;
    static const int  min_exponent10 = 0;
    static const int  max_exponent = 0;
    static const int  max_exponent10 = 0;

    static const bool has_infinity = false;
    static const bool has_quiet_NaN = false;
    static const bool has_signaling_NaN = false;
    static const float_denorm_style has_denorm = denorm_absent;
    static const bool has_denorm_loss = false;
    DEVICE static type infinity() {return type(0);}
    DEVICE static type quiet_NaN() {return type(0);}
    DEVICE static type signaling_NaN() {return type(0);}
    DEVICE static type denorm_min() {return type(0);}

    static const bool is_iec559 = false;
    static const bool is_bounded = true;
    static const bool is_modulo = false;

    static const bool traps = false;
    static const bool tinyness_before = false;
    static const float_round_style round_style = round_toward_zero;
};

template <>
class __libcpp_numeric_limits<float, true>
{
protected:
    typedef float type;

    static const bool is_specialized = true;

    static const bool is_signed = true;
    static const int  digits = __FLT_MANT_DIG__;
    static const int  digits10 = __FLT_DIG__;
    static const int  max_digits10 = 2+(digits * 30103)/100000;
    DEVICE static type min() {return __FLT_MIN__;}
    DEVICE static type max() {return __FLT_MAX__;}
    DEVICE static type lowest() {return -max();}

    static const bool is_integer = false;
    static const bool is_exact = false;
    static const int  radix = __FLT_RADIX__;
    DEVICE static type epsilon() {return __FLT_EPSILON__;}
    DEVICE static type round_error() {return 0.5F;}

    static const int  min_exponent = __FLT_MIN_EXP__;
    static const int  min_exponent10 = __FLT_MIN_10_EXP__;
    static const int  max_exponent = __FLT_MAX_EXP__;
    static const int  max_exponent10 = __FLT_MAX_10_EXP__;

    static const bool has_infinity = true;
    static const bool has_quiet_NaN = true;
    static const bool has_signaling_NaN = true;
    static const float_denorm_style has_denorm = denorm_present;
    static const bool has_denorm_loss = false;
    DEVICE static type infinity() {return __builtin_huge_valf();}
    DEVICE static type quiet_NaN() {return __builtin_nanf("");}
    DEVICE static type signaling_NaN() {return __builtin_nansf("");}
    DEVICE static type denorm_min() {return __FLT_DENORM_MIN__;}

    static const bool is_iec559 = true;
    static const bool is_bounded = true;
    static const bool is_modulo = false;

    static const bool traps = false;
    static const bool tinyness_before = false;
    static const float_round_style round_style = round_to_nearest;
};

template <>
class __libcpp_numeric_limits<double, true>
{
protected:
    typedef double type;

    static const bool is_specialized = true;

    static const bool is_signed = true;
    static const int  digits = __DBL_MANT_DIG__;
    static const int  digits10 = __DBL_DIG__;
    static const int  max_digits10 = 2+(digits * 30103)/100000;
    DEVICE static type min() {return __DBL_MIN__;}
    DEVICE static type max() {return __DBL_MAX__;}
    DEVICE static type lowest() {return -max();}

    static const bool is_integer = false;
    static const bool is_exact = false;
    static const int  radix = __FLT_RADIX__;
    DEVICE static type epsilon() {return __DBL_EPSILON__;}
    DEVICE static type round_error() {return 0.5;}

    static const int  min_exponent = __DBL_MIN_EXP__;
    static const int  min_exponent10 = __DBL_MIN_10_EXP__;
    static const int  max_exponent = __DBL_MAX_EXP__;
    static const int  max_exponent10 = __DBL_MAX_10_EXP__;

    static const bool has_infinity = true;
    static const bool has_quiet_NaN = true;
    static const bool has_signaling_NaN = true;
    static const float_denorm_style has_denorm = denorm_present;
    static const bool has_denorm_loss = false;
    DEVICE static type infinity() {return __builtin_huge_val();}
    DEVICE static type quiet_NaN() {return __builtin_nan("");}
    DEVICE static type signaling_NaN() {return __builtin_nans("");}
    DEVICE static type denorm_min() {return __DBL_DENORM_MIN__;}

    static const bool is_iec559 = true;
    static const bool is_bounded = true;
    static const bool is_modulo = false;

    static const bool traps = false;
    static const bool tinyness_before = false;
    static const float_round_style round_style = round_to_nearest;
};

template <>
class __libcpp_numeric_limits<long double, true>
{
protected:
    typedef long double type;

    static const bool is_specialized = true;

    static const bool is_signed = true;
    static const int  digits = __LDBL_MANT_DIG__;
    static const int  digits10 = __LDBL_DIG__;
    static const int  max_digits10 = 2+(digits * 30103)/100000;
    DEVICE static type min() {return __LDBL_MIN__;}
    DEVICE static type max() {return __LDBL_MAX__;}
    DEVICE static type lowest() {return -max();}

    static const bool is_integer = false;
    static const bool is_exact = false;
    static const int  radix = __FLT_RADIX__;
    DEVICE static type epsilon() {return __LDBL_EPSILON__;}
    DEVICE static type round_error() {return 0.5;}

    static const int  min_exponent = __LDBL_MIN_EXP__;
    static const int  min_exponent10 = __LDBL_MIN_10_EXP__;
    static const int  max_exponent = __LDBL_MAX_EXP__;
    static const int  max_exponent10 = __LDBL_MAX_10_EXP__;

    static const bool has_infinity = true;
    static const bool has_quiet_NaN = true;
    static const bool has_signaling_NaN = true;
    static const float_denorm_style has_denorm = denorm_present;
    static const bool has_denorm_loss = false;
    DEVICE static type infinity() {return __builtin_huge_vall();}
    DEVICE static type quiet_NaN() {return __builtin_nanl("");}
    DEVICE static type signaling_NaN() {return __builtin_nansl("");}
    DEVICE static type denorm_min() {return __LDBL_DENORM_MIN__;}


    static const bool is_iec559 = true;
    static const bool is_bounded = true;
    static const bool is_modulo = false;

    static const bool traps = false;
    static const bool tinyness_before = false;
    static const float_round_style round_style = round_to_nearest;
};

template <class _Tp>
class numeric_limits
    : private __libcpp_numeric_limits<typename remove_cv<_Tp>::type>
{
    typedef __libcpp_numeric_limits<typename remove_cv<_Tp>::type> __base;
    typedef typename __base::type type;
public:
    static const bool is_specialized = __base::is_specialized;
    DEVICE static type min() {return __base::min();}
    DEVICE static type max() {return __base::max();}
    DEVICE static type lowest() {return __base::lowest();}

    static const int  digits = __base::digits;
    static const int  digits10 = __base::digits10;
    static const int  max_digits10 = __base::max_digits10;
    static const bool is_signed = __base::is_signed;
    static const bool is_integer = __base::is_integer;
    static const bool is_exact = __base::is_exact;
    static const int  radix = __base::radix;
    DEVICE static type epsilon() {return __base::epsilon();}
    DEVICE static type round_error() {return __base::round_error();}

    static const int  min_exponent = __base::min_exponent;
    static const int  min_exponent10 = __base::min_exponent10;
    static const int  max_exponent = __base::max_exponent;
    static const int  max_exponent10 = __base::max_exponent10;

    static const bool has_infinity = __base::has_infinity;
    static const bool has_quiet_NaN = __base::has_quiet_NaN;
    static const bool has_signaling_NaN = __base::has_signaling_NaN;
    static const float_denorm_style has_denorm = __base::has_denorm;
    static const bool has_denorm_loss = __base::has_denorm_loss;
    DEVICE static type infinity() {return __base::infinity();}
    DEVICE static type quiet_NaN() {return __base::quiet_NaN();}
    DEVICE static type signaling_NaN() {return __base::signaling_NaN();}
    DEVICE static type denorm_min() {return __base::denorm_min();}

    static const bool is_iec559 = __base::is_iec559;
    static const bool is_bounded = __base::is_bounded;
    static const bool is_modulo = __base::is_modulo;

    static const bool traps = __base::traps;
    static const bool tinyness_before = __base::tinyness_before;
    static const float_round_style round_style = __base::round_style;
};

template <class _Tp>
class numeric_limits<const _Tp>
    : private numeric_limits<_Tp>
{
    typedef numeric_limits<_Tp> __base;
    typedef _Tp type;
public:
    static const bool is_specialized = __base::is_specialized;
    DEVICE static type min() {return __base::min();}
    DEVICE static type max() {return __base::max();}
    DEVICE static type lowest() {return __base::lowest();}

    static const int  digits = __base::digits;
    static const int  digits10 = __base::digits10;
    static const int  max_digits10 = __base::max_digits10;
    static const bool is_signed = __base::is_signed;
    static const bool is_integer = __base::is_integer;
    static const bool is_exact = __base::is_exact;
    static const int  radix = __base::radix;
    DEVICE static type epsilon() {return __base::epsilon();}
    DEVICE static type round_error() {return __base::round_error();}

    static const int  min_exponent = __base::min_exponent;
    static const int  min_exponent10 = __base::min_exponent10;
    static const int  max_exponent = __base::max_exponent;
    static const int  max_exponent10 = __base::max_exponent10;

    static const bool has_infinity = __base::has_infinity;
    static const bool has_quiet_NaN = __base::has_quiet_NaN;
    static const bool has_signaling_NaN = __base::has_signaling_NaN;
    static const float_denorm_style has_denorm = __base::has_denorm;
    static const bool has_denorm_loss = __base::has_denorm_loss;
    DEVICE static type infinity() {return __base::infinity();}
    DEVICE static type quiet_NaN() {return __base::quiet_NaN();}
    DEVICE static type signaling_NaN() {return __base::signaling_NaN();}
    DEVICE static type denorm_min() {return __base::denorm_min();}

    static const bool is_iec559 = __base::is_iec559;
    static const bool is_bounded = __base::is_bounded;
    static const bool is_modulo = __base::is_modulo;

    static const bool traps = __base::traps;
    static const bool tinyness_before = __base::tinyness_before;
    static const float_round_style round_style = __base::round_style;
};

template <class _Tp>
class numeric_limits<volatile _Tp>
    : private numeric_limits<_Tp>
{
    typedef numeric_limits<_Tp> __base;
    typedef _Tp type;
public:
    static const bool is_specialized = __base::is_specialized;
    DEVICE static type min() {return __base::min();}
    DEVICE static type max() {return __base::max();}
    DEVICE static type lowest() {return __base::lowest();}

    static const int  digits = __base::digits;
    static const int  digits10 = __base::digits10;
    static const int  max_digits10 = __base::max_digits10;
    static const bool is_signed = __base::is_signed;
    static const bool is_integer = __base::is_integer;
    static const bool is_exact = __base::is_exact;
    static const int  radix = __base::radix;
    DEVICE static type epsilon() {return __base::epsilon();}
    DEVICE static type round_error() {return __base::round_error();}

    static const int  min_exponent = __base::min_exponent;
    static const int  min_exponent10 = __base::min_exponent10;
    static const int  max_exponent = __base::max_exponent;
    static const int  max_exponent10 = __base::max_exponent10;

    static const bool has_infinity = __base::has_infinity;
    static const bool has_quiet_NaN = __base::has_quiet_NaN;
    static const bool has_signaling_NaN = __base::has_signaling_NaN;
    static const float_denorm_style has_denorm = __base::has_denorm;
    static const bool has_denorm_loss = __base::has_denorm_loss;
    DEVICE static type infinity() {return __base::infinity();}
    DEVICE static type quiet_NaN() {return __base::quiet_NaN();}
    DEVICE static type signaling_NaN() {return __base::signaling_NaN();}
    DEVICE static type denorm_min() {return __base::denorm_min();}

    static const bool is_iec559 = __base::is_iec559;
    static const bool is_bounded = __base::is_bounded;
    static const bool is_modulo = __base::is_modulo;

    static const bool traps = __base::traps;
    static const bool tinyness_before = __base::tinyness_before;
    static const float_round_style round_style = __base::round_style;
};

template <class _Tp>
class numeric_limits<const volatile _Tp>
    : private numeric_limits<_Tp>
{
    typedef numeric_limits<_Tp> __base;
    typedef _Tp type;
public:
    static const bool is_specialized = __base::is_specialized;
    DEVICE static type min() {return __base::min();}
    DEVICE static type max() {return __base::max();}
    DEVICE static type lowest() {return __base::lowest();}

    static const int  digits = __base::digits;
    static const int  digits10 = __base::digits10;
    static const int  max_digits10 = __base::max_digits10;
    static const bool is_signed = __base::is_signed;
    static const bool is_integer = __base::is_integer;
    static const bool is_exact = __base::is_exact;
    static const int  radix = __base::radix;
    DEVICE static type epsilon() {return __base::epsilon();}
    DEVICE static type round_error() {return __base::round_error();}

    static const int  min_exponent = __base::min_exponent;
    static const int  min_exponent10 = __base::min_exponent10;
    static const int  max_exponent = __base::max_exponent;
    static const int  max_exponent10 = __base::max_exponent10;

    static const bool has_infinity = __base::has_infinity;
    static const bool has_quiet_NaN = __base::has_quiet_NaN;
    static const bool has_signaling_NaN = __base::has_signaling_NaN;
    static const float_denorm_style has_denorm = __base::has_denorm;
    static const bool has_denorm_loss = __base::has_denorm_loss;
    DEVICE static type infinity() {return __base::infinity();}
    DEVICE static type quiet_NaN() {return __base::quiet_NaN();}
    DEVICE static type signaling_NaN() {return __base::signaling_NaN();}
    DEVICE static type denorm_min() {return __base::denorm_min();}

    static const bool is_iec559 = __base::is_iec559;
    static const bool is_bounded = __base::is_bounded;
    static const bool is_modulo = __base::is_modulo;

    static const bool traps = __base::traps;
    static const bool tinyness_before = __base::tinyness_before;
    static const float_round_style round_style = __base::round_style;
};
}

}



#endif /* GPULIMITS_H_ */
