/*
 * GPUdebug.h
 *
 *  Created on: 28 Mar, 2017
 *      Author: panpan
 */

#ifndef GPUDEBUG_H_
#define GPUDEBUG_H_
/*! \file   debug.h
	\date   Sunday July 24, 2011
	\author Gregory Diamos <gregory.diamos@gatech.edu>
	\brief  The header file for archaeopteryx debug functions.
*/

#pragma once

// Standard Library Includes
#include <iostream>
#include <cstdio>

#ifdef REPORT_BASE
#undef REPORT_BASE
#endif

#define REPORT_BASE 0

// Preprocessor macros
#ifdef device_assert
#undef device_assert
#endif

#define device_assert(x) util::_assert(x, #x, __FILE__, __LINE__)
#define device_assert_m(x, y) device_report(y "\n"); device_assert(x)

#ifndef NDEBUG
	#define report(y) \
		if(REPORT_BASE > 0)\
		{ \
			{\
			std::cout << __FILE__ << ":"  << __LINE__  \
					<< ": " << y << "\n";\
			}\
		 \
		}
#else
	#define report(y)
#endif

#ifdef device_report
#undef device_report
#endif

#define device_report(...) \
	if(REPORT_BASE > 0)\
	{ \
		printf(__VA_ARGS__);\
	}

#ifdef cta_report
#undef cta_report
#endif

#define cta_report(...) \
	if(threadIdx.x == 0)\
	{ \
		device_report(__VA_ARGS__);\
	}

#ifdef kernel_report
#undef kernel_report
#endif

#define kernel_report(...) \
	if(blockIdx.x == 0)\
	{ \
		cta_report(__VA_ARGS__);\
	}

namespace archaeopteryx
{

namespace util
{

__device__ void _assert(bool condition, const char* expression,
	const char* filename, int line);

}

}


#endif /* GPUDEBUG_H_ */
