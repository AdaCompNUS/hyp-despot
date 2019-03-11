#ifndef MA_ROCKSAMPLE_H
#define MA_ROCKSAMPLE_H

#include <despot/interface/pomdp.h>
#include <despot/core/mdp.h>
#include "base_ma_rock_sample.h"
#include <despot/util/coord.h>
#include <despot/util/grid.h>

namespace despot {

/* =============================================================================
 * MultiAgentRockSample class
 * =============================================================================*/

class MultiAgentRockSample: public BaseMultiAgentRockSample {
public:
	MultiAgentRockSample(std::string map);
	MultiAgentRockSample(int size, int rocks);

	bool Step(State& state, double rand_num, int action, double& reward,
		OBS_TYPE& obs) const;
	int NumActions() const;
	double ObsProb(OBS_TYPE obs, const State& state, int action) const;
	void PrintObs(const State& state, OBS_TYPE observation,
		std::ostream& out = std::cout) const;

	void InitGPUModel();
	void InitGPUUpperBound(std::string name,	std::string particle_bound_name) const;
	void InitGPULowerBound(std::string name,	std::string particle_bound_name) const ;

	void DeleteGPUModel();
	void DeleteGPUUpperBound(std::string name, std::string particle_bound_name);
	void DeleteGPULowerBound(std::string name, std::string particle_bound_name);

};

} // namespace despot

#endif
