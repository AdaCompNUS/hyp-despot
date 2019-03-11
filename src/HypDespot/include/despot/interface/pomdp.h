#ifndef POMDP_H
#define POMDP_H

#include <despot/core/globals.h>
#include <despot/interface/belief.h>
#include <despot/random_streams.h>
#include <despot/core/history.h>
#include <despot/interface/default_policy.h>
#include <despot/interface/lower_bound.h>
#include <despot/interface/upper_bound.h>
#include <despot/util/memorypool.h>
#include <despot/util/seeds.h>
#include <despot/util/util.h>

namespace despot {

/* =============================================================================
 * State class
 * =============================================================================*/
/**
 * [Optional]
 * Base state class. (define your custom state by inheriting this class)
 */
class State: public MemoryObject {
public:
	int state_id;
	int scenario_id;
	double weight;

	State();
	State(int _state_id, double weight);
	virtual ~State();

	friend std::ostream& operator<<(std::ostream& os, const State& state);

	virtual std::string text() const;

	static double Weight(const std::vector<State*>& particles);

	State* operator()(int state_id, double weight) {
		this->state_id = state_id;
		this->weight = weight;
		return this;
	}
};

/* =============================================================================
 * StateIndexer class
 * =============================================================================*/
/**
 * [Optional]
 * Interface for a mapping between states and indices.
 */
class StateIndexer {
public:
	virtual ~StateIndexer();

	virtual int NumStates() const = 0;
	virtual int GetIndex(const State* state) const = 0;
	virtual const State* GetState(int index) const = 0;
};

/* =============================================================================
 * StatePolicy class
 * =============================================================================*/
/**
 * [Optional]
 * Interface for a mapping from states to actions.
 */
class StatePolicy {
public:
	virtual ~StatePolicy();
	virtual int GetAction(const State& state) const = 0;
};

/* =============================================================================
 * MMAPinferencer class
 * =============================================================================*/
/**
 * [Optional]
 * Interface for computing marginal MAP state from a set of particles.
 */
class MMAPInferencer {
public:
	virtual ~MMAPInferencer();

	virtual const State* GetMMAP(const std::vector<State*>& particles) const = 0;
};

class POMCPPrior;
class Dvc_State;
class Dvc_RandomStreams;

/* =============================================================================
 * DSPOMDP class
 * =============================================================================*/
/**
 * Interface for a deterministic simulative model for POMDP.
 */
class DSPOMDP {
public:
	DSPOMDP();

	virtual ~DSPOMDP();

	/* ========================================================================
	 * Deterministic simulative model and related functions
	 * ========================================================================*/
	/**
	 * [Essential]
	 * Determistic simulative model for POMDP.
	 * Returns whether the terminal state has been reached
	 * @param state      Current state of the world in a scenario
	 * @param random_num Random number in a scenario
	 * @param action     Action to be taken
	 * @param reward     Reward received after taking action from state
	 * @param obs        Observation received after taking action
	 */
	virtual bool Step(State& state, double random_num, ACT_TYPE action,
		double& reward, OBS_TYPE& obs) const = 0;

	/**
	 * [Optional]
	 * Override this to get speedup for LookaheadUpperBound. No observation generated.
	 * Returns whether the terminal state has been reached
	 * @param state      Current state of the world in a scenario
	 * @param random_num Random number in a scenario
	 * @param action     Action to be taken
	 * @param reward     Reward received after taking action from state
	 */
	virtual bool Step(State& state, double random_num, ACT_TYPE action,
		double& reward) const;

	/**
	 * [Optional]
	 * Simulative model for POMCP. No random number required.
	 * Returns whether the terminal state has been reached
	 * @param state      Current state of the world in a scenario
	 * @param action     Action to be taken
	 * @param reward     Reward received after taking action from state
	 * @param obs        Observation received after taking action
	 */
	virtual bool Step(State& state, ACT_TYPE action, double& reward,
		OBS_TYPE& obs) const;

	/* ========================================================================
	 * Action
	 * ========================================================================*/
	/**
	 * [Essential]
	 * Returns number of actions.
	 */
	virtual int NumActions() const = 0;

	/* ========================================================================
	 * Observation
	 * ========================================================================*/
	/**
	 * [Essential]
	 * Returns number of observations.
	 */
	virtual int NumObservations() const = 0;

	/* ========================================================================
	 * Reward
	 * ========================================================================*/
	/**
	 * [Optional]
	 * Returns the reward for taking an action at a state ( to help evaluate the planning process)
	 * @param state  Current state of the world
	 * @param action Action to be taken
	 */
	virtual double Reward(const State& state, ACT_TYPE action) const;

	/* ========================================================================
	 * Functions related to beliefs and starting states.
	 * ========================================================================*/
	/**
	 * [Essential]
	 * Returns the observation probability.
	 * @param obs    Observation candidate
	 * @param state  Current state of the world
	 * @param action Action that has been taken
	 */
	virtual double ObsProb(OBS_TYPE obs, const State& state,
		ACT_TYPE action) const = 0;

	/**
	 * [Optional]
	 * Returns a starting state of simulation.
	 * Used to generate states in the initial belief or the true starting state of a POMDP-based world
	 */
	virtual State* CreateStartState(std::string type = "DEFAULT") const;

	/**
	 * [Essential]
	 * Returns the initial belief.
	 * @param start The start state of the world
	 * @param type  Type of the initial belief
	 */
	virtual Belief* InitialBelief(const State* start,
		std::string type = "DEFAULT") const = 0;

	/* ========================================================================
	 * Bound-related functions.
	 * ========================================================================*/
	/**
	 * [Essential]
	 * Returns the maximum reward.
	 */
	virtual double GetMaxReward() const = 0;

	/**
	 * [Essential]
	 * Returns (a, v), where a is an action with largest minimum reward when it is
	 * executed, and v is its minimum reward, that is, a = \max_{a'} \min_{s}
	 * R(a', s), and v = \min_{s} R(a, s).
	 */
	virtual ValuedAction GetBestAction() const = 0;

	/**
	 * [Optional]
	 * Override to create custom ParticleUpperBounds for solvers
	 * @param name Name of the particle upper bound to be used
	 */
	virtual ParticleUpperBound* CreateParticleUpperBound(std::string name = "DEFAULT") const;

	/**
	 * [Optional]
	 * Override to create custom ScenarioUpperBounds for solvers
	 * @param name 				  Name of the upper bound to be used
	 * @param particle_bound_name Name of the ParticleUpperBound to be used as the base of ScenarioUpperBound
	 */
	virtual ScenarioUpperBound* CreateScenarioUpperBound(std::string name = "DEFAULT",
		std::string particle_bound_name = "DEFAULT") const;

	/**
	 * [Optional]
	 * Override to create custom ParticleLowerBounds for solvers
	 * @param name Name of the particle lower bound to be used
	 */
	virtual ParticleLowerBound* CreateParticleLowerBound(std::string name = "DEFAULT") const;

	/**
	 * [Optional]
	 * Override to create custom ScenarioLowerBounds for solvers
	 * @param name 				  Name of the lower bound to be used
	 * @param particle_bound_name Name of the ParticleLowerBound to be used as the base of ScenarioLowerBound
	 */
	virtual ScenarioLowerBound* CreateScenarioLowerBound(std::string bound_name = "DEFAULT",
		std::string particle_bound_name = "DEFAULT") const;

	/**
	 * [Optional]
	 * Override to create custom priors for the POMCP solver
	 * @param name Name of the prior to be used
	 */
	virtual POMCPPrior* CreatePOMCPPrior(std::string name = "DEFAULT") const;

	/* ========================================================================
	 * Display
	 * ========================================================================*/
	/**
	 * [Essential]
	 * Prints a state.
	 * @param state The state to be printed
	 * @param out   The destination stream
	 */
	virtual void PrintState(const State& state, std::ostream& out = std::cout) const = 0;

	/**
	 * [Essential]
	 * Prints an observation.
	 * @param state The current state
	 * @param obs   The observation to be printed
	 * @param out   The destination stream
	 */
	virtual void PrintObs(const State& state, OBS_TYPE obs,
		std::ostream& out = std::cout) const = 0;

	/**
	 * [Essential]
	 * Prints an action.
	 * @param action The action to be printed
	 * @param out    The destination stream
	 */
	virtual void PrintAction(ACT_TYPE action, std::ostream& out = std::cout) const = 0;

	/**
	 * [Essential]
	 * Prints a belief.
	 * @param belief The belief to be printed
	 * @param out    The destination stream
	 */
	virtual void PrintBelief(const Belief& belief,
		std::ostream& out = std::cout) const = 0;

	/**
	 * [Optional]
	 * To print the particles, for testing purpose
	 */
	 virtual void PrintParticles(const std::vector<State*> particles, std::ostream& out = std::cout) const;

	/**
	 * [Optional]
	 * Explort and import a state or a list of states from file.
	 */
	 virtual void ExportState(const State& state, std::ostream& out = std::cout) const;
	 virtual State* ImportState(std::istream& in) const;
	 virtual void ImportStateList(std::vector<State*>& particles, std::istream& in) const;

	/* ========================================================================
	 * Memory management.
	 * ========================================================================*/
	/**
	 * [Essential]
	 * Allocate a state.
	 * @param state_id ID of the allocated state in the state space
	 * @param weight   Weight of the allocated state
	 */
	virtual State* Allocate(int state_id = -1, double weight = 0) const = 0;

	/**
	 * [Essential]
	 * Returns a copy of the state.
	 * @param state The state to be copied
	 */
	virtual State* Copy(const State* state) const = 0;

	/**
	 * [Essential]
	 * Free the memory of a state.
	 * @param state The state to be freed
	 */
	virtual void Free(State* state) const = 0;

	/**
	 * [Optional]
	 * Returns a copy of a list of particles.
	 * @param particles The particles to be copied
	 */
	std::vector<State*> Copy(const std::vector<State*>& particles) const;

	/**
	 * [Essential]
	 * Returns number of allocated particles (sampled states).
	 */
	virtual int NumActiveParticles() const = 0;

	/**
	 * [Optional]
	 * Returns a copy of this POMDP model.
	 */
	inline virtual DSPOMDP* MakeCopy() const {
		return NULL;
	}


	/* ========================================================================
	 * Functions for HyP-DESPOT
	 * ========================================================================*/

	virtual Dvc_State* AllocGPUParticles(int numParticles, MEMORY_MODE mode,  Dvc_State*** particles_all_a = NULL ) const = 0;

	virtual void DeleteGPUParticles( MEMORY_MODE mode, Dvc_State** particles_all_a = NULL) const = 0;

	virtual void CopyGPUParticlesFromParent(Dvc_State* des, Dvc_State* src, int src_offset, int* IDs,
	                                        int num_particles, bool interleave,
	                                        Dvc_RandomStreams* streams, int stream_pos,
	                                        void* CUDAstream = NULL, int shift = 0) const = 0;

	virtual Dvc_State* CopyParticlesToGPU(Dvc_State* dvc_particles, const std::vector<State*>& particles , bool deep_copy) const = 0;

	virtual void CopyParticleIDsToGPU(int* dvc_IDs, const std::vector<int>& particleIDs, void* CUDAstream = NULL) const = 0;

	virtual void ReadParticlesBackToCPU(std::vector<State*>& particles , const Dvc_State* parent_particles,
	                                    bool deepcopy) const {
		std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
	}

	virtual void InitGPUModel() = 0;

	virtual void InitGPUUpperBound(std::string name,	std::string particle_bound_name) const = 0;

	virtual void InitGPULowerBound(std::string name,	std::string particle_bound_name) const = 0;

	virtual void DeleteGPUModel() = 0;

	virtual void DeleteGPUUpperBound(std::string name, std::string particle_bound_name) = 0;

	virtual void DeleteGPULowerBound(std::string name, std::string particle_bound_name) = 0;

	virtual void CreateMemoryPool() const = 0;
	virtual void DestroyMemoryPool(MEMORY_MODE mode) const = 0;

	virtual int ParallelismInStep() const=0;

	
};

enum Exploration_Mode
{
	VIRTUAL_LOSS,
	UCT,
};

/* =============================================================================
 * BeliefMDP class
 * =============================================================================*/
/**
 * [Optional]
 * The BeliefMDP class provides an interface for the belief MDP, which is
 * commonly used in belief tree search algorithms.
 *
 * @see AEMS
 */

class BeliefMDP: public DSPOMDP {
public:
	BeliefMDP();
	virtual ~BeliefMDP();

	virtual BeliefLowerBound* CreateBeliefLowerBound(std::string name) const;
	virtual BeliefUpperBound* CreateBeliefUpperBound(std::string name) const;

  /**
   * Transition function for the belief MDP.
   */
	virtual Belief* Tau(const Belief* belief, ACT_TYPE action,
		OBS_TYPE obs) const = 0;

  /**
   * Observation function for the belief MDP.
   */
	virtual void Observe(const Belief* belief, ACT_TYPE action,
		std::map<OBS_TYPE, double>& obss) const = 0;

  /**
   * Reward function for the belief MDP.
   */
	virtual double StepReward(const Belief* belief, ACT_TYPE action) const = 0;
};

} // namespace despot

#endif
