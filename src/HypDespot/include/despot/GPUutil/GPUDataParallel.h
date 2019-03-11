/*
 * GPUDataParallel.h
 *
 *  Created on: 28 Mar, 2017
 *      Author: panpan
 */

#ifndef GPUDATAPARALLEL_H_
#define GPUDATAPARALLEL_H_
/*! \file   DataParallel.h
	\author Gregory Diamos <gregory.diamos@gatech.edu>
	\date   Thursday September 22, 2011
	\brief  The header file for the DataParallel primitive functions.
*/

#pragma once

namespace util
{

class DataParallelConfig
{
public:
	unsigned int threads;
	unsigned int ctas;

	unsigned int sharedSize;
};

template<typename Type, DataParallelConfig config>
void transpose(Type* begin, Type* end, unsigned int stride, Type* out)
{
	__shared__ Type buffer[config.sharedSize];

	unsigned int size = end - begin;
	unsigned int rows = size / stride;

	unsigned int step  = config.threads;
	unsigned int start = threadIdx.x;

	for(unsigned int i = start; i < size; i+=step)
	{
		offset = (i / stride) % stride;

		buffer[i + offset] = begin[i];
	}

	__syncthreads();

	for(unsigned int i = start; i < size; i+=step)
	{
		unsigned int row    = i % stride;
		unsigned int column = i / stride;

		unsigned int index = (row * stride) + (row % stride) + column;

		out[i] = buffer[index];
	}
}

}



#endif /* GPUDATAPARALLEL_H_ */
