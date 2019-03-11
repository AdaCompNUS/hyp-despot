#ifndef MEMORYPOOL_H
#define MEMORYPOOL_H

#include <cassert>
#include <vector>
#include <ostream>
#include <despot/GPUcore/thread_globals.h>

namespace despot {

class MemoryObject {
public:
	void SetAllocated() {
		allocated_ = true;
	}
	void ClearAllocated() {
		allocated_ = false;
	}
	bool IsAllocated() const {
		return allocated_;
	}

public:
	bool allocated_;
};

template<class T>
class MemoryPool {
public:
	MemoryPool() :
		num_allocated_(0) {
	}

	~MemoryPool() {
		DeleteAll();
	}

	T* Construct() {
		T* obj = Allocate();
		return new (obj) T;
	}

	void Destroy(T* obj) {
		obj.T::~T();
		Free(obj);
	}

	T* Allocate() {
		Globals::lock_process();

		if (freelist_.empty())
			NewChunk();
		T* obj = freelist_.back();
		freelist_.pop_back();


		assert(!obj->IsAllocated());
		obj->SetAllocated();
		num_allocated_++;

		Globals::unlock_process();

		return obj;
	}

	void Free(T* obj) {
		Globals::lock_process();

		assert(obj->IsAllocated());
		obj->ClearAllocated();
		freelist_.push_back(obj);
		num_allocated_--;

		Globals::unlock_process();
	}

	void DeleteAll() {
		for (chunk_iterator_ i_chunk = chunks_.begin();
			i_chunk != chunks_.end(); ++i_chunk)
			delete *i_chunk;
		chunks_.clear();
		freelist_.clear();
		num_allocated_ = 0;
	}

	int num_allocated() const {
		return num_allocated_;
	}

private:
	struct Chunk {
		static const int Size = 256;
		T Objects[Size];
	};

	void NewChunk() {
		Chunk* chunk = new Chunk;
		chunks_.push_back(chunk);
		for (int i = Chunk::Size - 1; i >= 0; --i) {
			freelist_.push_back(&chunk->Objects[i]);
			chunk->Objects[i].ClearAllocated();
		}
	}

	std::vector<Chunk*> chunks_;
	std::vector<T*> freelist_;
	typedef typename std::vector<Chunk*>::iterator chunk_iterator_;

public:
	int num_allocated_;
};

} // namespace despot

#endif // MEMORYPOOL_H
