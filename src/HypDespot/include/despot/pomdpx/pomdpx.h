#ifndef POMDPX_H
#define POMDPX_H

#include <despot/interface/pomdp.h>
#include <despot/core/mdp.h>
#include <despot/pomdpx/parser/parser.h>

namespace despot {

/* ==============================================================================
 * POMDPXState class
 * ==============================================================================*/
class POMDPXState: public State {
public:
	std::vector<int> vec_id;

	POMDPXState();
	~POMDPXState();

	POMDPXState(std::vector<int> state);

	std::string text() const;
};

/* ==============================================================================
 * POMDPX class
 * ==============================================================================*/

class POMDPX: public MDP,
	public DSPOMDP,
	public StateIndexer,
	public StatePolicy {
	friend class POMDPXBeliefRevitalise;
	friend class POMDPXBeliefRandomConsistent;
	friend class POMDPXBeliefMix;
	friend class POMDPXGreedyActionPolicy;

private:
	Parser* parser_;
	bool is_small_;

	ValuedAction min_reward_action_;
	ValuedAction max_reward_action_;

	std::vector<POMDPXState*> states_;
	std::vector<std::vector<std::vector<State> > > transition_probabilities_;
	mutable std::vector<std::vector<double> > rewards_;

	mutable MemoryPool<POMDPXState> memory_pool_;

	void InitStates();
	void InitTransitions();
	void PrintTransitions();
	void InitRewards();

	mutable std::vector<ACT_TYPE> default_action_;
	void ComputeDefaultActions(std::string type) const;
	void PrintDefaultActions();

	void PrintModel(std::ostream& out = std::cout) const;

public:
	static POMDPX* current_;
	static int STATE_NUM_THRESHOLD;

	POMDPX();
	POMDPX(std::string file);

	inline Parser* parser() const {
		return parser_;
	}

	bool NoisyStep(State& s, double random_num, ACT_TYPE action) const;
	bool Step(State& s, double random_num, ACT_TYPE action, double& reward,
		OBS_TYPE& obs) const;
	int NumActions() const;
	int NumObservations() const;
	int NumStates() const;
	int GetIndex(const State* state) const;
	const State* GetState(int index) const;

	const std::vector<State>& TransitionProbability(int s, ACT_TYPE a) const;
	double Reward(int s, ACT_TYPE action) const;
	double Reward(const State& state, ACT_TYPE action) const;

	double ObsProb(OBS_TYPE obs, const State& s, ACT_TYPE a) const;

	State* CreateStartState(std::string type) const;
	std::vector<State*> ExactInitialParticleSet() const;
	std::vector<State*> ApproxInitialParticleSet() const;
	Belief* InitialBelief(const State* start, std::string type = "DEFAULT") const;

	inline int GetAction(const State& state) const {
		const POMDPXState& pomdpx_state = static_cast<const POMDPXState&>(state);
		return default_action_[parser_->ComputeIndex(pomdpx_state.vec_id)];
	}

	inline double GetMaxReward() const {
		return max_reward_action_.value;
	}
	ParticleUpperBound* CreateParticleUpperBound(std::string name = "DEFAULT") const;
	ScenarioUpperBound* CreateScenarioUpperBound(std::string name = "DEFAULT",
	        std::string particle_bound_name = "DEFAULT") const;

	inline ValuedAction GetBestAction() const {
		return min_reward_action_;
	}
	ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT",
	        std::string particle_bound_name = "DEFAULT") const;

	void PrintState(const State& state, std::ostream& out = std::cout) const;
	void ExportState(const State& state, std::ostream& out = std::cout) const {;}
	State* ImportState(std::istream& in) const {return NULL;}
	void ImportStateList(std::vector<State*>& particles, std::istream& in) const {;}

	void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
	void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const;
	void PrintAction(ACT_TYPE action, std::ostream& out = std::cout) const;

	State* Allocate(int state_id, double weight) const;
	State* Copy(const State* particle) const;
	void Free(State* particle) const;
	int NumActiveParticles() const;

	virtual DSPOMDP* MakeCopy() const;

	void PrintMDPBound(const std::vector<ValuedAction>& policy, const char* fn);

	// For server-client messages in IPPC competition
	const std::string& GetActionName();
	const std::string& GetEnumedAction(ACT_TYPE action);
	OBS_TYPE GetPOMDPXObservation(std::map<std::string, std::string>& observe);

	/* ========================================================================
	 * Functions for HyP-DESPOT
	 * ========================================================================*/

	virtual Dvc_State* AllocGPUParticles(int numParticles, MEMORY_MODE mode,  Dvc_State*** particles_all_a = NULL ) const {return NULL;}

	virtual void DeleteGPUParticles( MEMORY_MODE mode, Dvc_State** particles_all_a = NULL) const {;}

	virtual void CopyGPUParticlesFromParent(Dvc_State* des, Dvc_State* src, int src_offset, int* IDs,
	                                        int num_particles, bool interleave,
	                                        Dvc_RandomStreams* streams, int stream_pos,
	                                        void* CUDAstream = NULL, int shift = 0) const {;}

	virtual Dvc_State* CopyParticlesToGPU(Dvc_State* dvc_particles, const std::vector<State*>& particles , bool deep_copy) const {return NULL;}

	virtual void CopyParticleIDsToGPU(int* dvc_IDs, const std::vector<int>& particleIDs, void* CUDAstream = NULL) const {;}

	virtual void ReadParticlesBackToCPU(std::vector<State*>& particles , const Dvc_State* parent_particles,
	                                    bool deepcopy) const {
		std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
	}

	virtual void InitGPUModel() {;}

	virtual void InitGPUUpperBound(std::string name,	std::string particle_bound_name) const {;}

	virtual void InitGPULowerBound(std::string name,	std::string particle_bound_name) const {;}

	virtual void DeleteGPUModel() {;}

	virtual void DeleteGPUUpperBound(std::string name, std::string particle_bound_name) {;}

	virtual void DeleteGPULowerBound(std::string name, std::string particle_bound_name) {;}

	virtual void CreateMemoryPool() const {;}
	virtual void DestroyMemoryPool(MEMORY_MODE mode) const {;}

	virtual int ParallelismInStep() const {return 0;}

};

} // namespace despot

#endif
