#ifndef GPUMEMORYPOOL_H
#define GPUMEMORYPOOL_H

#include <cassert>
#include <vector>
#include <iostream>
#include <despot/GPUcore/CudaInclude.h>
#include <despot/GPUcore/thread_globals.h>

#ifdef __CUDACC__
#include <cuda.h>
#include <cuda_runtime_api.h>
#endif
namespace despot {

/* =============================================================================
 * GPU_MemoryPool class
 * =============================================================================*/
/**
 * Chunked list of GPU memory for efficient GPU memory allocation (in host code).
 * Typically used to allocate particles
 */

template<class T>
class GPU_MemoryPool {
public:

	T* init_head_;
	T* init_pos_;
	int init_chunk_;
	int current_chunck;

	int num_allocated() const {
		return num_allocated_;
	}
private:

	class Dvc_Chunk {
	public:
		static const int Size=131072*5;
		T* Objects;
	#ifdef __CUDACC__

		Dvc_Chunk()
		{
			HANDLE_ERROR(cudaMalloc((void**)&Objects, sizeof(T)*Size));
		}
		~Dvc_Chunk()
		{
			HANDLE_ERROR(cudaFree(Objects));
		}

		void ResetValue(int v)
		{
			HANDLE_ERROR(cudaMemset(Objects, 0, Size*sizeof(T)));
		}
	#endif
	};

public:
	GPU_MemoryPool() :
		num_allocated_(0),
		freehead_(NULL),
		freepos_(NULL),
		current_chunck(-1),
		init_head_(NULL),
		init_pos_(NULL),
		init_chunk_(0)
		{
	}

	void RecordHead()
	{
		init_head_=freehead_;
		init_pos_=freepos_;
		init_chunk_=current_chunck;
	}
	T* Allocate(int size) {
		Globals::lock_process();

		assert(size<=Dvc_Chunk::Size);
		if (chunck_full(size))
		{
			if(NextChunckEmpty())
			{
				NewChunk();
			}
			else
			{
				ReuseChunk();
			}
		}

		T* obj = freepos_;
		freepos_+=size;
		num_allocated_+=size;

		Globals::unlock_process();
		return obj;
	}
	void DeleteAll() {
		for (chunk_iterator_ i_chunk = chunks_.begin();
			i_chunk != chunks_.end(); ++i_chunk)
			delete *i_chunk;
		chunks_.clear();
		freehead_=NULL;
		freepos_=NULL;
		num_allocated_ = 0;
		current_chunck=-1;
		init_head_=NULL;init_pos_=NULL;init_chunk_=0;
	}

	void ResetChuncks()
	{
		for (chunk_iterator_ i_chunk = chunks_.begin();
			i_chunk != chunks_.end(); ++i_chunk)
		{
			if(*i_chunk!=NULL)
				(*i_chunk)->ResetValue(0);
		}
		freehead_=init_head_;
		freepos_=init_pos_;
		current_chunck=init_chunk_;
	}
	~GPU_MemoryPool(){
		DeleteAll();
	}
private:


	bool chunck_full(int size)
	{
		return (freepos_+size-freehead_>=Dvc_Chunk::Size)|| (freepos_==NULL);
	}

	void NewChunk() {
		current_chunck++;
		Dvc_Chunk/*<T>*/* chunk = new Dvc_Chunk;
		chunks_.push_back(chunk);
		freehead_=chunk->Objects;
		freepos_=freehead_;
	}
	bool NextChunckEmpty()
	{
		if(chunks_.size()==0) return true;
		return chunks_.size()<=current_chunck+1;
	}

	void ReuseChunk() {
		current_chunck++;
		freehead_=chunks_[current_chunck]->Objects;
		freepos_=freehead_;
	}
	T* freehead_;
	T* freepos_;
	std::vector<Dvc_Chunk*> chunks_;

	typedef typename std::vector<Dvc_Chunk*>::iterator chunk_iterator_;

public:
	int num_allocated_;
};


/* =============================================================================
 * Array_MemoryPool class
 * =============================================================================*/
/**
 * Chunked list of GPU memory for efficient GPU array memory allocation (in host code).
 * Typically used to allocated arrays in particles, e.g., list of pedestrians in a car driving POMDP state
 */

template<class T>
class Array_MemoryPool {
public:

	int num_allocated() const {
		return num_allocated_;
	}
private:

	class Array_Chunk {
	public:
		static const int Size=131072*5;
		T* Objects;
		bool mt_;
		Array_Chunk(bool mt)
		{
			mt_=mt;
#ifdef __CUDACC__
			if(mt)
				HANDLE_ERROR(cudaHostAlloc((void**)&Objects, sizeof(T)*Size,cudaHostAllocPortable));
			else
#endif
				Objects=(T*)malloc( sizeof(T)*Size);
		}
		~Array_Chunk()
		{
#ifdef __CUDACC__
			if(mt_)
				HANDLE_ERROR(cudaFreeHost(Objects));
			else
#endif
				free(Objects);
		}
		void ResetValue(int v)
		{
			memset(Objects, 0, Size*sizeof(T));
		}
	};

public:
	Array_MemoryPool(bool b_mt) :
		num_allocated_(0),
		freehead_(NULL),
		freepos_(NULL)
		{
		chunks_=new Array_Chunk*[5000];
		for(int i=0;i<5000;i++)
			chunks_[i]=NULL;
		current_chunck=-1;
		mt_=b_mt;
	}

	T* Allocate(int size) {

		assert(size<=Array_Chunk::Size);
		if (chunck_full(size))
		{
			if(NextChunckEmpty())
				NewChunk();
			else
				ReuseChunk();
		}

		T* obj = freepos_;
		freepos_+=size;
		num_allocated_++;
		return obj;
	}
	void DeleteAll() {
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
	void DeleteContents() {
		for (int i=0;i<current_chunck+1;i++)
		{
			if(chunks_[i]) delete chunks_[i];chunks_[i]=NULL;
		}
		freehead_=NULL;
		freepos_=NULL;
		num_allocated_ = 0;
		current_chunck=-1;
	}

	void ResetChuncks()
	{
		for (int i=0;i<current_chunck+1;i++)
		{
			if(chunks_[i])
				chunks_[i]->ResetValue(0);
		}
		freehead_=chunks_[0]->Objects;
		freepos_=freehead_;
		num_allocated_ = 0;
		current_chunck=0;
	}

	~Array_MemoryPool(){
		DeleteAll();
	}
private:


	bool chunck_full(int size)
	{
		return (freepos_+size-freehead_>=Array_Chunk::Size)|| (freepos_==NULL);
	}

	void NewChunk() {
		current_chunck++;
		chunks_[current_chunck] = new Array_Chunk(mt_);
		freehead_=chunks_[current_chunck]->Objects;
		freepos_=freehead_;
	}


	bool NextChunckEmpty()
	{
		return chunks_[current_chunck+1]==NULL;
	}

	void ReuseChunk() {
		current_chunck++;
		freehead_=chunks_[current_chunck]->Objects;
		freepos_=freehead_;
	}

	T* freehead_;
	T* freepos_;
	Array_Chunk** chunks_;
	int current_chunck;
	bool mt_;//multi_threading support
public:
	int num_allocated_;
};

} // namespace despot

#endif // MEMORYPOOL_H
