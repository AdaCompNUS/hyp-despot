#ifndef POMDPLITE_H
#define POMDPLITE_H

#include <despot/interface/pomdp.h>
#include <despot/core/history.h>
#include <despot/core/ext_node.h>
#include <despot/core/globals.h>

namespace despot {


/* =============================================================================
 * SolverPrior class
 * =============================================================================*/

class SolverPrior {
protected:
	const MeanMDP* model_;
	ActionStateHistory as_history_;
	VariableActionStateHistory as_history_in_search_;
	std::vector<double> action_probs_;

public:
	SolverPrior(const MeanMDP* model);
	virtual ~SolverPrior();

	inline virtual int SmartCount(ACT_TYPE action) const {
		return 10;
	}

	inline virtual double SmartValue(ACT_TYPE action) const {
		return 1;
	}

	inline virtual const ActionStateHistory& history() const {
		return as_history_;
	}

	inline virtual VariableActionStateHistory& history_in_search() {
		return as_history_in_search_;
	}

	inline virtual void history_in_search(VariableActionStateHistory h) {
		as_history_in_search_ = h;
		Print_in_search(__FUNCTION__, __LINE__);
	}

	inline virtual void history(ActionStateHistory h) {
		as_history_ = h;
	}

	inline const std::vector<const State*>& history_states() {
		return as_history_.states();
	}

	inline std::vector<State*>& history_states_for_search() {
		return as_history_in_search_.states();
	}

	inline virtual void Add(ACT_TYPE action, const State* state) {
		as_history_.Add(action, state);
	}
	inline virtual void Add_in_search(ACT_TYPE action, State* state) {
		as_history_in_search_.Add(action, state);
		Print_in_search(__FUNCTION__, __LINE__);
	}


	inline virtual void Print_in_search(const char* func, int line) {
		//std::cout<<func<<"@"<<line<<" ";
		//std::cout<<"as_history_in_search_ length: "<<as_history_in_search_.Size()<<std::endl;
		//if(as_history_in_search_.Size()>Globals::config.max_policy_sim_len*2)
		//	assert(false);
	}

	inline virtual void PopLast(bool insearch) {
		(insearch)? as_history_in_search_.RemoveLast(): as_history_.RemoveLast();
		if(insearch) Print_in_search(__FUNCTION__, __LINE__);
	}

    inline virtual void PopAll(bool insearch) {
	  (insearch)? as_history_in_search_.Truncate(0): as_history_.Truncate(0);
	  if(insearch) Print_in_search(__FUNCTION__, __LINE__);
	}

	virtual const std::vector<double>& ComputePreference(const Belief* belief, const State* state) = 0;

	virtual double ComputeValue(const Belief* belief, const State* state,std::vector<int>& rollout_scenarios,
			RandomStreams& streams, VNode* vnode)=0;
	const std::vector<double>& action_probs() const;
};

class DefaultPOMDPLitePrior: public SolverPrior {
protected:
	StatePolicy* default_policy_;
	int initial_depth_;
public:

	DefaultPOMDPLitePrior(const MeanMDP* model, StatePolicy* policy):
		SolverPrior(model)
	{
		default_policy_=policy;
		initial_depth_=0;
	}
	virtual const std::vector<double>& ComputePreference(const Belief* belief, const State* state);

	virtual double ComputeValue(const Belief* belief, const State* state,std::vector<int>& rollout_scenarios,
			RandomStreams& streams, VNode* vnode);
	double Rollout(State* particle, RandomStreams& streams,int depth);
	void default_policy(StatePolicy* policy){
		default_policy_=policy;
	}
};

/* =============================================================================
 * POMDPLite class
 * =============================================================================*/
class World;

class POMDPLite: public Solver {
public:
	const World* world_;

private:

    int TreeDepth, PeakTreeDepth;
    Ext_VNode* root_;
    SolverPrior* prior_;
    /*
     * In POMDPLite, belief_ only on hidden variables.
     * Therefore, a state need to be maintained for observable variables
     * */
    State* state_;
    SearchStatistics statistics_;

public:

    POMDPLite(const MeanMDP* model, const World* world, SolverPrior* prior, Belief* belief);
    ~POMDPLite();

    virtual ValuedAction Search();
    virtual void BeliefUpdate(ACT_TYPE action, OBS_TYPE obs);
    static Ext_VNode* ConstructTree(Belief* belief, const State* state, RandomStreams& streams,
    		const MeanMDP* model, SolverPrior* prior, double timeout,
    		SearchStatistics* statistics=NULL);

    static double Rollout(Ext_VNode* vnode,std::vector<int>& rollout_scenarios,RandomStreams& streams,
    		SolverPrior* prior);

    static void Update(Ext_QNode* qnode);
    static void Update(Ext_QNode* qnode, double value);
    static void Update(Ext_QNode* qnode,double old_child_value, double new_child_value,
    		int old_count, int new_count, int old_qcount, int new_qcount);
    static void Update(Ext_VNode* vnode, double new_value);

    const ActionStateHistory& history() const {
    	return prior_->history(); }
    const VariableActionStateHistory& history_in_search() const {
        return prior_->history_in_search(); }
    /*void HistoryUpdate(ACT_TYPE action, const State* state)
    {
    	prior_->Add(action, state);
    }*/

    static void InitFastPUCB();
	static ValuedAction OptimalAction(const Ext_VNode* vnode, bool use_usb);
	static double CalculateScenarioRewardBonus(const Belief* belief, const State* state, const ACT_TYPE action,
			const State* next_state,bool fast_update);

private:
    static Ext_VNode* SelectVNode(Ext_QNode* qnode);
    static Ext_QNode* SelectQNode(Ext_VNode* vnode, bool use_ucb);

    static double Simulate(Ext_VNode* cur, RandomStreams& streams, SolverPrior* prior,
    		const MeanMDP* model, SearchStatistics* statistics);

    static Ext_VNode* CreateVNode(int depth, Belief* belief, State* state,RandomStreams& streams,
    		SolverPrior* prior, const DSPOMDP* model, const std::vector<int>& scenarios,
			std::vector<int>& rollout_scenarios,
			QNode* parent,OBS_TYPE edge) ;

    // Fast lookup table for UCB
    static const int UCB_N = 10000, UCB_n = 100;
    static double UCB[UCB_N][UCB_n];
    static bool InitialisedFastUCB;

    static double FastPUCB(int N, int n, double logN, Ext_QNode* qnode);
    static double FastPUCB(int N, int n, Ext_QNode* qnode);

};


} // namespace despot

#endif // POMDPLITE_H
