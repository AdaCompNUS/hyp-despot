#ifndef UNCNAVIGATION_H
#define UNCNAVIGATION_H

#include <despot/interface/pomdp.h>
#include <despot/core/mdp.h>
#include "base_unc_navigation.h"
#include <despot/util/coord.h>
#include <despot/util/grid.h>

namespace despot {

/* =============================================================================
 * UncNavigation class
 * =============================================================================*/

class UncNavigation: public BaseUncNavigation {
public:
	//UncNavigation(std::string map);
	UncNavigation(int size, int obstacles);
	~UncNavigation(){;}
	bool Step(State& state, double rand_num, int action, double& reward,
		OBS_TYPE& obs) const;
	int NumActions() const;
	double ObsProb(OBS_TYPE obs, const State& state, int action) const;
	void PrintObs(const State& state, OBS_TYPE observation,
		std::ostream& out = std::cout) const;
	void TestObsProb(const State& state) const;


	void InitGPUModel();
	void InitGPUUpperBound(std::string name,	std::string particle_bound_name) const;
	void InitGPULowerBound(std::string name,	std::string particle_bound_name) const ;

	void DeleteGPUModel();
	void DeleteGPUUpperBound(std::string name, std::string particle_bound_name);
	void DeleteGPULowerBound(std::string name, std::string particle_bound_name);

};

} // namespace despot

#endif
