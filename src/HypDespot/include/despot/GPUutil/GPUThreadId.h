/*
 * GPUThreadId.h
 *
 *  Created on: 28 Mar, 2017
 *      Author: panpan
 */

#ifndef GPUTHREADID_H_
#define GPUTHREADID_H_

#pragma once

namespace archaeopteryx
{

namespace util
{

__device__ unsigned int threadId();

}

}

#endif /* GPUTHREADID_H_ */
