#ifndef DESPOT_H
#define DESPOT_H

#include <despot/core/solver.h>
#include <despot/interface/pomdp.h>
#include <despot/interface/belief.h>
#include <despot/core/node.h>
#include <despot/core/globals.h>
#include <despot/core/history.h>
#include <despot/random_streams.h>
#include <despot/GPUcore/shared_node.h>
#include <despot/GPUcore/shared_solver.h>
#include <despot/util/memorypool.h>

namespace despot {
class Dvc_RandomStreams;

class DESPOT: public Solver {
friend class VNode;

	static void CPU_MakeNodes(double start, int NumParticles,
			const std::vector<int>& particleIDs, const DSPOMDP* model,
			const std::vector<State*>& particles, QNode* qnode, VNode* parent,
			History& history, ScenarioLowerBound* lb, ScenarioUpperBound* ub,
			RandomStreams& streams);

	/************** HyP-DESPOT ************/
	static void GPU_MakeNodes(double start, int NumParticles,
			const std::vector<int>& particleIDs, const DSPOMDP* model,
			const std::vector<State*>& particles, QNode* qnode, VNode* parent,
			History& history, ScenarioLowerBound* lb, ScenarioUpperBound* ub,
			RandomStreams& streams);
	static int CalSharedMemSize();

	static void EnableDebugInfo(QNode* qnode);
	static void DisableDebugInfo();
	static void EnableDebugInfo(VNode* vnode, QNode* qnode);

	/************** HyP-DESPOT ************/


protected:
	VNode* root_;
	Shared_SearchStatistics statistics_;

	ScenarioLowerBound* lower_bound_;
	ScenarioUpperBound* upper_bound_;

	/************** HyP-DESPOT ************/
	std::vector<double> Initial_upper;
	std::vector<double> Initial_lower;
	std::vector<double> Final_upper;
	std::vector<double> Final_lower;
	/************** HyP-DESPOT ************/


public:
	DESPOT(const DSPOMDP* model, ScenarioLowerBound* lb, ScenarioUpperBound* ub, Belief* belief = NULL, bool use_GPU=false);
	virtual ~DESPOT();

	ValuedAction Search();

	void belief(Belief* b);
	void BeliefUpdate(ACT_TYPE action, OBS_TYPE obs);

	ScenarioLowerBound* lower_bound() const;
	ScenarioUpperBound* upper_bound() const;

	static VNode* ConstructTree(std::vector<State*>& particles, RandomStreams& streams,
		ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
		const DSPOMDP* model, History& history, double timeout,
		SearchStatistics* statistics = NULL);

protected:
	static VNode* Trial(VNode* root, RandomStreams& streams,
		ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
		const DSPOMDP* model, History& history, SearchStatistics* statistics =
			NULL);
	static Shared_VNode* Trial(Shared_VNode* root, RandomStreams& streams,
		ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
		const DSPOMDP* model, History& history, bool & Expansion_done, Shared_SearchStatistics* statistics =
			NULL);
	static void InitLowerBound(VNode* vnode, ScenarioLowerBound* lower_bound,
		RandomStreams& streams, History& history);
	static void InitUpperBound(VNode* vnode, ScenarioUpperBound* upper_bound,
		RandomStreams& streams, History& history);
	static void InitBounds(VNode* vnode, ScenarioLowerBound* lower_bound,
		ScenarioUpperBound* upper_bound, RandomStreams& streams, History& history);

	static void Expand(VNode* vnode,
		ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
		const DSPOMDP* model, RandomStreams& streams, History& history);
	static void Expand(Shared_VNode* vnode,
		ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
		const DSPOMDP* model, RandomStreams& streams, History& history);
	static void Backup(VNode* vnode, bool real);

	static double Gap(VNode* vnode);
	static double Gap(Shared_VNode* vnode, bool use_Vloss);

	double CheckDESPOT(const VNode* vnode, double regularized_value);
	double CheckDESPOTSTAR(const VNode* vnode, double regularized_value);
	void Compare();

	static void ExploitBlockers(VNode* vnode);
	static void ExploitBlockers(Shared_VNode* vnode);
	static VNode* FindBlocker(VNode* vnode);
	static void Expand(QNode* qnode, ScenarioLowerBound* lower_bound,
		ScenarioUpperBound* upper_bound, const DSPOMDP* model,
		RandomStreams& streams, History& history);
	static void Update(VNode* vnode, bool real);
	static void Update(QNode* qnode, bool real);
	static void Update(Shared_VNode* vnode, bool real);
	static void Update(Shared_QNode* qnode, bool real);
	static VNode* Prune(VNode* vnode, ACT_TYPE& pruned_action, double& pruned_value);
	static QNode* Prune(QNode* qnode, double& pruned_value);
	static double WEU(VNode* vnode);
	static double WEU(Shared_VNode* vnode);
	static double WEU(VNode* vnode, double epsilon);
	static double WEU(Shared_VNode* vnode, double xi);
	static VNode* SelectBestWEUNode(QNode* qnode);
	static QNode* SelectBestUpperBoundNode(VNode* vnode);
	static Shared_QNode* SelectBestUpperBoundNode(Shared_VNode* vnode);
	static ValuedAction OptimalAction(VNode* vnode);

	/*Debug*/
	static void OutputWeight(QNode* qnode);
	static void OptimalAction2(VNode* vnode);
	/*Debug*/

	static ValuedAction Evaluate(VNode* root, std::vector<State*>& particles,
		RandomStreams& streams, POMCPPrior* prior, const DSPOMDP* model);

	/************** HyP-DESPOT ************/

	static void ReadBackData(int ThreadID);

	static void MCSimulation(VNode* vnode, int ThreadID,
			const DSPOMDP* model, RandomStreams& streams,History& history, bool Do_rollout=true);
	static void GPU_Expand_Action(VNode* vnode, ScenarioLowerBound* lb,
		ScenarioUpperBound* ub, const DSPOMDP* model,
		RandomStreams& streams,
		History& history);
	static void GPU_InitBounds(VNode* vnode, ScenarioLowerBound* lower_bound,
		ScenarioUpperBound* upper_bound,const DSPOMDP* model, RandomStreams& streams,
		History& history);
	static void GPU_UpdateParticles(VNode* vnode, ScenarioLowerBound* lb,
		ScenarioUpperBound* ub, const DSPOMDP* model, RandomStreams& streams,
		History& history);


	void PrepareGPUMemory(const DSPOMDP* model, int num_actions, int num_obs);
	void PrepareGPUStreams(const RandomStreams& streams);
	void ClearGPUMemory(const DSPOMDP* model);
	void PrintGPUData(int num_searches);
	void PrintCPUTime(int num_searches);

	static void PrepareGPUDataForRoot(VNode* node, const DSPOMDP* model, const std::vector<int>& particleIDs, std::vector<State*>& particles);
	static void PrepareGPUDataForNode(VNode* vnode, const DSPOMDP* model, int ThreadID ,RandomStreams& streams);

	double AverageInitLower() const;
	double StderrInitLower() const;
	double AverageFinalLower() const;
	double StderrFinalLower() const;

	double AverageInitUpper() const;
	double StderrInitUpper() const;
	double AverageFinalUpper() const;
	double StderrFinalUpper() const;

	void PrintStatisticResult();


	static void ExpandTreeServer(RandomStreams streams,
			ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
			const DSPOMDP* model, History history, Shared_SearchStatistics* statistics,
			double& used_time,double& explore_time,double& backup_time,int& num_trials,double timeout,
			MsgQueque<Shared_VNode>& node_queue, MsgQueque<Shared_VNode>& print_queue, int threadID);

	static float CalExplorationValue(int depth);
	static void CalExplorationValue(Shared_QNode* node);
	static void CalExplorationValue(Shared_VNode* node);

	/************** HyP-DESPOT ************/


public:

	/************** HyP-DESPOT ************/
	const DSPOMDP* model_ ;

	static bool use_GPU_;
	static int num_Obs_element_in_GPU;

	static double Initial_root_gap;
	static bool Debug_mode;
	static bool Print_nodes;
	/************** HyP-DESPOT ************/


public:
	/************** HyP-DESPOT ************/
	static MemoryPool<QNode> qnode_pool_;
	static MemoryPool<Shared_QNode> s_qnode_pool_;

	static MemoryPool<VNode> vnode_pool_;
	static MemoryPool<Shared_VNode> s_vnode_pool_;
	/************** HyP-DESPOT ************/

};




} // namespace despot

#endif
