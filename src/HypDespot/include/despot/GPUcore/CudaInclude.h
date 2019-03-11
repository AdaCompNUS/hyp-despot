/*
 * CudaInclude.h
 *
 *  Created on: 22 Mar, 2017
 *      Author: panpan
 */

#ifndef CUDAINCLUDE_H_
#define CUDAINCLUDE_H_
//Use this file to make sure the portability of the CUDA header files.
//When CUDACC is not defined (in .cpp files), DEVICE and HOST will be ignored
#include <stdio.h>

#pragma once
#ifdef __CUDACC__
	#define HOST __host__
	#define DEVICE __device__
	#define ALIGN(n) __align__(n)



	//#include <thrust/device_vector.h>
	//#define ThrustVector thrust::host_vector
	static void HandleError( cudaError_t err,
							 const char *file,
							 int line ) {
		if (err != cudaSuccess) {
			printf( "%s in %s at line %d\n", cudaGetErrorString( err ),
					file, line );
			exit( EXIT_FAILURE );
		}
	}
	#define HANDLE_ERROR( err ) (HandleError( err, __FILE__, __LINE__ ))

#include <curand.h>
#include<curand_kernel.h>

#else
	#define HOST
	#define DEVICE
#if defined(__GNUC__) // GCC
  #define ALIGN(n) __attribute__((aligned(n)))
#elif defined(_MSC_VER) // MSVC
  #define ALIGN(n) __declspec(align(n))
#else
  #error "CudaInclude.h: Please provide a definition for ALIGN macro for your host compiler!"
#endif
#endif


#endif /* CUDAINCLUDE_H_ */
