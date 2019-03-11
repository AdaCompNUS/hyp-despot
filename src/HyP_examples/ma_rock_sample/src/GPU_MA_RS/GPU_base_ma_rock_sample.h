#ifndef GPU_MA_BASEROCKSAMPLE_H
#define GPU_MA_BASEROCKSAMPLE_H

#include <despot/GPUinterface/GPUpomdp.h>
#include <despot/GPUutil/GPUcoord.h>
#include <despot/GPUcore/CudaInclude.h>

namespace despot {

/* =============================================================================
 * MARockSampleState class
 * =============================================================================*/
class MARockSampleState;

class Dvc_MARockSampleState: public Dvc_State {
public:
	int joint_pos;

	DEVICE
	Dvc_MARockSampleState();

	/*DEVICE
	void SetAllocated() {
		allocated_ = true;
	}*/

	HOST
	static void CopyToGPU(Dvc_MARockSampleState* Dvc, int scenarioID,
			const MARockSampleState*, bool copy_cells = true);

	HOST
	static void ReadBackToCPU(const Dvc_MARockSampleState* Dvc,
			MARockSampleState* Hst,
			bool copy_cells=true);

};

/* =============================================================================
 * Dvc_RockSampleApproxParticleUpperBound class
 * =============================================================================*/
class Dvc_MARockSampleApproxParticleUpperBound/*: public ParticleUpperBound*/{
protected:
public:

	DEVICE
	static float Value(const Dvc_State* state, int scenarioID,
			Dvc_History& history);
};

class Dvc_MARockSampleMDPParticleUpperBound/*: public ParticleUpperBound*/{
public:

	DEVICE
	static float Value(const Dvc_State* particles, int scenarioID,
			Dvc_History& history);
};


class Dvc_MARockSampleTrivialParticleUpperBound/*: public ParticleUpperBound*/{
public:

	DEVICE
	static float Value(const Dvc_State* particles, int scenarioID,
			Dvc_History& history);
};


class Dvc_MARockSampleEastScenarioLowerBound/* : public ScenarioLowerBound */{

public:
	DEVICE
	static Dvc_ValuedAction Value(Dvc_State* particles,
			Dvc_RandomStreams& streams, Dvc_History& history, int dummy);

};

} // namespace despot

#endif
