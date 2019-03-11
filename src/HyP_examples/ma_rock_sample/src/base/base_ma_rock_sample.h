#ifndef BASE_MA_ROCKSAMPLE_H
#define BASE_MA_ROCKSAMPLE_H

#include <despot/interface/pomdp.h>
#include <despot/solver/pomcp.h>
#include <despot/core/mdp.h>
#include <despot/util/coord.h>
#include <despot/util/grid.h>

#define NUM_ROCKS 20
#define MAX_COORD_BIT 10
#define COORD_BIT_MASK ((1 << MAX_COORD_BIT)-1)
#define ROB_TERMINAL_ID COORD_BIT_MASK
#define MAX_ACTION_BIT 5
#define ACTION_BIT_MASK ((1 << MAX_ACTION_BIT)-1)
#define MAX_OBS_BIT 3
#define OBS_BIT_MASK ((1 << MAX_OBS_BIT)-1)
#define MAX_NUM_AGENTS 3


namespace despot {
//#define POLICY_GRAPH_SIZE 1000
/* =============================================================================
 * MARockSampleState class
 * =============================================================================*/

class MARockSampleState: public State {
public:
	int joint_pos;
	MARockSampleState();
	MARockSampleState(int _state_id, int joint_pos);

	std::string text() const;

	MARockSampleState& operator=(const MARockSampleState& other) // copy assignment
	{
		if (this != &other) { // self-assignment check expected
						 // storage can be reused
			state_id=other.state_id;
			joint_pos=other.joint_pos;
			scenario_id=other.scenario_id;
			weight=other.weight;
		}
		return *this;
	}
};


#define POLICY_GRAPH_SIZE 1000
/* =============================================================================
 * BaseMultiAgentRockSample class
 * =============================================================================*/

class BaseMultiAgentRockSample: public MDP,
	//public BeliefMDP,
	public DSPOMDP,
	public StateIndexer,
	public StatePolicy {
	friend class MARockSampleENTScenarioLowerBound;
	friend class MARockSampleMMAPStateScenarioLowerBound;
	friend class MARockSampleEastScenarioLowerBound;
	friend class MARockSampleParticleUpperBound1;
	friend class MARockSampleParticleUpperBound2;
	friend class MARockSampleMDPParticleUpperBound;
	friend class MARockSampleApproxParticleUpperBound;
	friend class MARockSampleEastBeliefPolicy;
	friend class MARockSampleMDPBeliefUpperBound;
	friend class MARockSamplePOMCPPrior;

public:
	Grid<int> grid_;
	std::vector<Coord> rock_pos_;
	int size_, num_rocks_, num_agents_;
	Coord* start_poses_;
	double half_efficiency_distance_;

	MARockSampleState* rock_state_;
protected:

	mutable MemoryPool<MARockSampleState> memory_pool_;

	std::vector<MARockSampleState*> states_;
protected:
	void InitGeneral();
	void Init_4_4();
	void Init_5_5();
	void Init_5_7();
	void Init_7_8();
	void Init_11_11();
	void Init_15_15();
	void InitStates();
	bool GetObservation(double rand_num, const MARockSampleState& rockstate, int rid,
		int rock) const;

	std::vector<std::vector<std::vector<State> > > transition_probabilities_;
	std::vector<std::vector<double> > alpha_vectors_; // For blind policy
	mutable std::vector<ValuedAction> mdp_policy_;

public:
	enum { // FRAGILE: Don't change!
		E_BAD = 0,
		E_GOOD = 1,
		E_NONE = 2
	};

	enum { // FRAGILE: Don't change!
		E_SAMPLE = 4
	};

public:
	BaseMultiAgentRockSample(std::string map);
	BaseMultiAgentRockSample(int size, int rocks);

	virtual bool Step(State& state, double rand_num, int action,
		double& reward, OBS_TYPE& obs) const = 0;
	virtual int NumActions() const = 0;
	virtual double ObsProb(OBS_TYPE obs, const State& state, int action) const = 0;

	const std::vector<State>& TransitionProbability(int s, int a) const;
	//int NextState(int s, int a) const;
	double Reward(int s, int a) const;
	bool Terminal(int s) const;

	State* CreateStartState(std::string type = "DEFAULT") const;
	std::vector<State*> InitialParticleSet() const;
	std::vector<State*> NoisyInitialParticleSet() const;
	Belief* InitialBelief(const State* start, std::string type = "PARTICLE") const;

	inline double GetMaxReward() const {
		return 10;
	}
	ScenarioUpperBound* CreateScenarioUpperBound(std::string name = "DEFAULT",
		std::string particle_bound_name = "DEFAULT") const;
	//BeliefUpperBound* CreateBeliefUpperBound(std::string name = "DEFAULT") const;

	inline ValuedAction GetBestAction() const {
		return ValuedAction(E_SAMPLE+1, 0);
	}
	ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT",
		std::string particle_bound_name = "DEFAULT") const;
	//BeliefLowerBound* CreateBeliefLowerBound(std::string name = "DEFAULT") const;

	//POMCPPrior* CreatePOMCPPrior(std::string name = "DEFAULT") const;

	void PrintState(const State& state, std::ostream& out = std::cout) const;
	void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
	virtual void PrintObs(const State& state, OBS_TYPE observation, std::ostream& out = std::cout) const = 0;
	void PrintAction(int action, std::ostream& out = std::cout) const;

	State* Allocate(int state_id, double weight) const;

	State* Allocate(int state_id, int joint_pos, double weight) const;
	State* Copy(const State* particle) const;
	void Free(State* particle) const;
	int NumActiveParticles() const;

	Belief* Tau(const Belief* belief, int action, OBS_TYPE obs) const;
	//void Observe(const Belief* belief, int action, std::map<OBS_TYPE, double>& obss) const;
	//double StepReward(const Belief* belief, int action) const;

	int NumStates() const;
	int NumRockStates() const;
	int NumPosStates() const;
	int RobNumCoordState() const{return grid_.xsize()*grid_.ysize();};
	int RobNumAction() const{return num_rocks_+5;};
	bool CheckStateValidity(int s) const;
	bool CheckActionValidity(int a) const;

	const State* GetState(int index) const;
	int GetIndex(const State* state) const;

	inline int GetAction(const State& tagstate) const {
		return 0;
	}

	int GetRobPosIndex(const MARockSampleState* state, int rid) const;
	int GetRobPosIndex(int joint_pos_id, int rid) const;

	int SetRobPosIndex(int& joint_pos_id, int rid, int index) const;

	Coord GetRobPos(const MARockSampleState* state, int rid) const;
	bool GetRock(const State* state, int rock) const;
	void SampleRock(State* state, int rock) const;
	//int RobActionCount() const;
	int GetRobAction(int action, int rid) const;
	int GetRobObs(OBS_TYPE obs, int rid) const;
	void SetRobObs(OBS_TYPE& obs,int rob_obs,  int rid) const;

	int GetX(const MARockSampleState* state, int rid) const;
	void IncX(MARockSampleState* state, int rid) const;
	void DecX(MARockSampleState* state, int rid) const;
	int GetY(const MARockSampleState* state, int rid) const;
	void IncY(MARockSampleState* state, int rid) const;
	void DecY(MARockSampleState* state, int rid) const;

protected:
	Coord IndexToCoord(int pos) const;
	int CoordToIndex(Coord c) const;
	MARockSampleState* MajorityMARockSampleState(const std::vector<State*>& particles) const;

public:
	virtual int NumObservations() const;
	virtual int ParallelismInStep() const{return /*num_agents_*/1;}
	void ExportMap(std::ostream& out = std::cout) const;
	void ImportMap(std::istream& in = std::cin);
	void ExportState(const State& state, std::ostream& out = std::cout) const;
	State* ImportState(std::istream& in) const;
	void ImportStateList(std::vector<State*>& particles, std::istream& in) const;

	virtual Dvc_State* AllocGPUParticles(int numParticles, MEMORY_MODE mode, Dvc_State*** particles_for_all_actions) const;
	virtual void CopyGPUParticlesFromParent(Dvc_State* des,Dvc_State* src,int src_offset,int* IDs,
			int num_particles,bool interleave,
			Dvc_RandomStreams* streams, int stream_pos,
			void* CUDAstream=NULL, int shift=0) const;

	virtual void DeleteGPUParticles(MEMORY_MODE mode, Dvc_State** particles_for_all_actions) const;

	virtual Dvc_State* CopyParticlesToGPU(Dvc_State* dvc_particles, const std::vector<State*>& particles , bool copy_cells) const;
	virtual void CopyParticleIDsToGPU(int* Dvc_ptr, const std::vector<int>& particleIDs,void* CUDAstream=NULL) const;

	virtual void ReadParticlesBackToCPU(std::vector<State*>& particles ,
			const Dvc_State* dvc_particles,
			bool copycells) const;
	virtual void CreateMemoryPool() const;
	virtual void DestroyMemoryPool(MEMORY_MODE mode) const;

	virtual void PrintParticles(const std::vector<State*> particles, std::ostream& out = std::cout) const;

};
class PolicyGraph;
extern PolicyGraph* policy_graph;
} // namespace despot



#endif
