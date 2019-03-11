#ifndef DVCMEMORYPOOL_H
#define DVCMEMORYPOOL_H

#include <cassert>
#include <vector>
#include <iostream>
#include <despot/GPUcore/CudaInclude.h>
#ifdef __CUDACC__
#include <cuda.h>
#include <cuda_runtime_api.h>
#endif
namespace despot {
#define MAX_CHUNK_NUM 5000

/* =============================================================================
 * Dvc_MemoryPool class
 * =============================================================================*/
/**
 * Chunked list of GPU memory for efficient GPU memory allocation (in device code).
 * Typically used to allocate particles
 */

template<class T>
class Dvc_MemoryPool {
public:

	int num_allocated() const {
		return num_allocated_;
	}
private:

	class Dvc_Chunk {
	public:
		static const int Size=131072;
		T* Objects;
	#ifdef __CUDACC__

		DEVICE Dvc_Chunk()
		{
			Objects=(T*)malloc( sizeof(T)*Size);
		}
		DEVICE ~Dvc_Chunk()
		{
			free(Objects);
		}
	#endif
	};

public:
	DEVICE Dvc_MemoryPool() :
		num_allocated_(0),
		freehead_(NULL),
		freepos_(NULL)
		{
		chunks_=new Dvc_Chunk*[MAX_CHUNK_NUM];
		current_chunck=-1;
	}

	DEVICE T* Allocate(int size) {

		assert(size<=Dvc_Chunk/*<T>*/::Size);
		if (chunck_full(size))
		{
			NewChunk();
		}

		T* obj = freepos_;
		freepos_+=size;
		num_allocated_++;
		return obj;
	}
	DEVICE void DeleteAll() {
		for (int i=0;i<current_chunck+1;i++)
		{
			if(chunks_[i]) delete chunks_[i];chunks_[i]=NULL;
		}
		delete [] chunks_;
		freehead_=NULL;
		freepos_=NULL;
		num_allocated_ = 0;
		current_chunck=-1;
	}
	DEVICE void DeleteContents() {
		for (int i=0;i<current_chunck+1;i++)
		{
			if(chunks_[i]) delete chunks_[i];chunks_[i]=NULL;
		}
		freehead_=NULL;
		freepos_=NULL;
		num_allocated_ = 0;
		current_chunck=-1;
	}
	DEVICE ~Dvc_MemoryPool(){
		DeleteAll();
	}
private:


	DEVICE bool chunck_full(int size)
	{
		return (freepos_+size-freehead_>=Dvc_Chunk::Size)|| (freepos_==NULL);
	}

	DEVICE void NewChunk() {
		current_chunck++;
		chunks_[current_chunck] = new Dvc_Chunk;
		freehead_=chunks_[current_chunck]->Objects;
		freepos_=freehead_;
	}

	T* freehead_;
	T* freepos_;
	Dvc_Chunk** chunks_;
	int current_chunck;

public:
	int num_allocated_;
};

} // namespace despot

#endif // MEMORYPOOL_H
