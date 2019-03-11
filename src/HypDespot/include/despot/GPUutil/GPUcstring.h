/*
 * GPUcstring.h
 *
 *  Created on: 28 Mar, 2017
 *      Author: panpan
 */

#ifndef GPUCSTRING_H_
#define GPUCSTRING_H_
/*! \file   cstring.h
	\date   Tuesday June 28, 2011
	\author Gregory Diamos <gregory.diamos@gatech.edu>
	\brief  The header file for device string functions.
*/
#include <despot/GPUcore/CudaInclude.h>

#pragma once

namespace archaeopteryx
{

namespace util
{

/*! \brief Safe string copy

	\param destination The target string
	\param source The source string
	\param max The max number of characters to copy
*/
DEVICE void strlcpy(char* destination, const char* source, size_t max);

/*! \brief string compare

	\param left The target string
	\param right The source string

	\return 0 if all bytes match, some random int otherwise
*/
DEVICE int strcmp(const char* left, const char* right);

DEVICE int memcmp(const void* s1, const void* s2, size_t n);
DEVICE size_t strlen(const char* s);
DEVICE const void* memchr(const void* s, int c, size_t n);
DEVICE void* memchr(      void* s, int c, size_t n);
DEVICE void* memmove(void* s1, const void* s2, size_t n);

DEVICE void* memcpy(void* s1, const void* s2, size_t n);
DEVICE void* memset(void* s, int c, size_t n);

}

}


#endif /* GPUCSTRING_H_ */
