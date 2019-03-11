#ifndef GPUHISTORY_H
#define GPUHISTORY_H

#include <vector>
#include <despot/util/util.h>
#include <despot/GPUcore/GPUglobals.h>
#include <despot/core/history.h>
#include <despot/GPUcore/CudaInclude.h>

namespace despot {

/**
 * Action-observation history.
 */
class Dvc_History {
public:

	HOST static void InitInGPU(int num_particles, Dvc_History* Dvc_history, int length,void* cuda_stream=NULL);

	HOST static void CopyToGPU(int num_particles,int* particleIDs, Dvc_History* Dvc_history, History* history);
	HOST static void Dvc_Add(Dvc_History* Dvc_history,ACT_TYPE action, OBS_TYPE obs, void* cudaStream=NULL);
	HOST static void Dvc_Trunc(Dvc_History* Dvc_history, int size, void* cudaStream=NULL);

	DEVICE void Add(ACT_TYPE action, OBS_TYPE obs) {

		if(actions_)actions_[currentSize_]=action;
		if(observations_)observations_[currentSize_]=obs;
		currentSize_++;
	}

	DEVICE void RemoveLast() {
		if(actions_)actions_[currentSize_-1]=-1;
		if(observations_)observations_[currentSize_-1]=-1;
		currentSize_--;
	}

	DEVICE ACT_TYPE Action(int t) const {
		return actions_[t];
	}

	DEVICE OBS_TYPE Observation(int t) const {
		return observations_[t];
	}

	DEVICE size_t Size() const {
		return currentSize_;
	}

	DEVICE void Truncate(int d) {
		currentSize_=d;
	}

	DEVICE ACT_TYPE LastAction() const {
		return actions_[currentSize_-1];
	}

	DEVICE OBS_TYPE LastObservation() const {
		return observations_[currentSize_-1];
	}

	void CreateMemoryPool(int mode=0) const;
	void DestroyMemoryPool(int mode=0) const;
public:
	ACT_TYPE* actions_;
	OBS_TYPE* observations_;
	int currentSize_;
};

} // namespace despot

#endif
