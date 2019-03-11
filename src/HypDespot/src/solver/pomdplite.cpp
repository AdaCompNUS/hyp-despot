#include <despot/config.h>
#include <despot/core/history.h>
#include <despot/core/solver.h>
#include <despot/interface/belief.h>
#include <despot/interface/world.h>
#include <despot/interface/default_policy.h>
#include <despot/core/particle_belief.h>
#include <despot/random_streams.h>
#include <despot/solver/pomdplite.h>
#include <despot/util/logging.h>
#include <despot/util/random.h>
#include <despot/util/util.h>
#include <sys/time.h>
#include <cassert>
#include <cmath>
#include <ctime>
#include <iostream>
#include <vector>


class Simulator;

using namespace std;

namespace despot {

/* =============================================================================
 * SolverPrior class
 * =============================================================================*/
static Belief* next_belief=NULL;
bool isDebug=false;

SolverPrior::SolverPrior(const MeanMDP* model) :
	model_(model) {
}

SolverPrior::~SolverPrior() {
}

const vector<double>& SolverPrior::action_probs() const {
	return action_probs_;
}


const std::vector<double>& DefaultPOMDPLitePrior::ComputePreference(const Belief* belief, const State* state){
	//Uniform action probabilities
	action_probs_.resize(model_->NumActions());
	std::fill(action_probs_.begin(), action_probs_.end(), 1.0/model_->NumActions());

	return action_probs_;
}

double DefaultPOMDPLitePrior::ComputeValue(const Belief* belief, const State* state,std::vector<int>& rollout_scenarios,
		RandomStreams& streams, VNode* vnode){
	//Conduct roll-outs to approximate the value
	assert(static_cast<const DiscreteBelief*>(belief)!=NULL);//Sanity check
	double total_reward=0;
	for(int i=0;i<rollout_scenarios.size();i++){
		State* particle=model_->Copy(state);
		particle->scenario_id=rollout_scenarios[i];
		static_cast<const DiscreteBelief*>(belief)->SampleHiddenState(particle,
				streams.Entry(particle->scenario_id));
		initial_depth_=as_history_in_search_.Size();
		total_reward+=Rollout(particle, streams, vnode->depth())/rollout_scenarios.size();
		model_->Free(particle);
	}
	return total_reward;
}

double DefaultPOMDPLitePrior::Rollout(State* particle, RandomStreams& streams, int depth) {
	/*if (streams.Exhausted()) {
		return 0;
	}*/
	if (streams.Exhausted()
		|| (as_history_in_search_.Size() - initial_depth_
			>= Globals::config.max_policy_sim_len)) {
		return 0;
	}

	ACT_TYPE action = default_policy_->GetAction(*particle);

	logv << *particle << endl;
	logv << "depth = " << depth << "; action = " << action << endl;

	double reward;
	OBS_TYPE obs;
	bool terminal = model_->Step(*particle, streams.Entry(particle->scenario_id),
		action, reward, obs);
	if (!terminal) {
		Add_in_search(action, particle);
		streams.Advance();
		reward += Globals::Discount()
			* Rollout( particle, streams, depth + 1);
		streams.Back();
		PopLast(true);
	}

	return reward;
}



/* =============================================================================
 * POMDPLite class
 * =============================================================================*/

POMDPLite::POMDPLite(const MeanMDP* model, const World* world,
		SolverPrior* prior, Belief* belief)
:  Solver(model,belief),
	world_(world),
	prior_(prior),
    TreeDepth(0),
	PeakTreeDepth(0),
	root_(NULL)
{
	assert(model!=NULL);
	assert(world!=NULL);//need to use world to update the current state
	assert(prior!=NULL);//need to use prior to maintain the history
	InitFastPUCB();
	state_=static_cast<const MeanMDP*>(model_)->CopyForSearch(world_->GetCurrentState());
}

POMDPLite::~POMDPLite()
{
}

ValuedAction POMDPLite::OptimalAction(const Ext_VNode* vnode, bool use_usb)
{
	static std::vector<int> besta;
	besta.clear();
	double bestq = Globals::NEG_INFTY;
	const vector<QNode*>& qnodes = vnode->children();
	ValuedAction astar(-1, Globals::NEG_INFTY);

	for (ACT_TYPE action = 0; action < qnodes.size(); action++) {
		Ext_QNode* qnode=static_cast<Ext_QNode*>(qnodes[action]);
		double q=qnode->value();
		if(use_usb)
		{
			int n = qnode->count();
			int N = vnode->count();
			//double logN = log(N + 1);
			q += FastPUCB(N, n, /*logN,*/ qnode);
		}

		if (q > bestq) {
			if(q > bestq)
				besta.clear();
			bestq = q;
			besta.push_back(action);
		}
	}
	assert(!besta.empty());
	int action = besta[Random::RANDOM.NextInt(besta.size())];
	astar = ValuedAction(action, bestq);
	return astar;
}


ValuedAction POMDPLite::Search()
{
	state_=static_cast<const MeanMDP*>(model_)->CopyForSearch(world_->GetCurrentState());

	if (logging::level() >= logging::DEBUG) {
		model_->PrintState(*state_);
		model_->PrintBelief(*belief_);
	}

	if (Globals::config.time_per_move <= 0) // Return a random action if no time is allocated for planning
		return ValuedAction(Random::RANDOM.NextInt(model_->NumActions()),
			Globals::NEG_INFTY);

	statistics_ = SearchStatistics();
	RandomStreams streams(Globals::config.num_scenarios,
			Globals::config.search_depth);

    double start = get_time_second();

    root_ = ConstructTree(belief_, state_,streams, static_cast<const MeanMDP*>(model_), prior_,
    		Globals::config.time_per_move, &statistics_);
	logi << "[DESPOT::Search] Time for tree construction: "
		<< (get_time_second() - start) << "s" << endl;

	start = get_time_second();
	root_->Free(*model_);
	logi << "[DESPOT::Search] Time for freeing particles in search tree: "
		<< (get_time_second() - start) << "s" << endl;

	ValuedAction astar = OptimalAction(root_, false);
	start = get_time_second();
	delete root_;
	model_->Free(state_);

	logi << "[DESPOT::Search] Time for deleting tree: "
		<< (get_time_second() - start) << "s" << endl;
	logi << "[DESPOT::Search] Search statistics:" << endl << statistics_
		<< endl;

	return astar;
}

Ext_VNode* POMDPLite::ConstructTree(Belief* belief, const State* state, RandomStreams& streams,
		const MeanMDP* model, SolverPrior* prior, double timeout,
		SearchStatistics* statistics) {

	State* rootstate = model->Copy(state);

	vector<int> scenarios; scenarios.resize(Globals::config.num_scenarios);
	for (int i = 0; i < Globals::config.num_scenarios; i++) {
		scenarios[i] = i;
	}

	// b' to be used in search
	next_belief=belief->MakeCopy();
	if (logging::level() >= logging::DEBUG) {
		model->PrintBelief(*next_belief);
	}

	Ext_VNode* root=CreateVNode(0/*depth*/,belief/*doesn't update belief*/,
			rootstate, streams, prior, model,scenarios,scenarios,NULL/*parent*/, 0/*edge*/);
	if (statistics != NULL) {
		statistics->initial_value = root->value();
		statistics->num_expanded_nodes=1;
		statistics->num_nodes_in_level.resize(Globals::config.search_depth);
		statistics->num_nodes_in_level[0]=1;
	}
    //int hist_size = prior->history_in_search().Size();

	double used_time = 0;
	int num_trials = 0;
	do {
		double start = clock();
		//Start trial from root
		Simulate(root, streams, prior, model, statistics);
		//root->count(root->count()+root->scenarios().size());
		used_time += double(clock() - start) / CLOCKS_PER_SEC;
		num_trials++;

	} while (used_time * (num_trials + 1.0) / num_trials < timeout);

	if (statistics != NULL) {
		statistics->final_value = root->value();
		statistics->num_policy_nodes = root->PolicyTreeSize();
		statistics->num_tree_nodes = root->Size();
		statistics->time_search = used_time;
		statistics->num_trials = num_trials;
	}

	delete next_belief;

	return root;
}
/*void Debug(const Belief* b, const char* function_name){
	const PedPomdpBelief* belief=static_cast<const PedPomdpBelief*>(b);
	for (const element &e: belief->DiscreteBel)
	{
		int size=e.distribution.size();
		if(size>3){
			cout<< function_name<<": wrong distributions size: "<< size<<endl;
			assert(false);
		}
	}
}*/

double POMDPLite::Simulate(Ext_VNode* vnode, RandomStreams& streams, SolverPrior* prior,
	const MeanMDP* model, SearchStatistics* statistics) {

	/*Bottom of the search tree: return 0 value*/
	if (streams.Exhausted())
		return 0;

	if (statistics != NULL
		&& vnode->depth() > statistics->longest_trial_length) {
		statistics->longest_trial_length = vnode->depth();
	}

	double start = clock();

	/*Not bottom yet: choose action branch to proceed*/
	ACT_TYPE action = OptimalAction(vnode, true).action;
	Ext_QNode* qnode = static_cast<Ext_QNode*>(vnode->Child(action));
	map<OBS_TYPE, VNode*>& vnodes = qnode->children();
	//logd << *vnode->observable_state() << endl;
	//logd << "depth = " << vnode->depth() << "; action = " << action << "; " << endl;
	logd << "[Traversal @Simulate] Traverse to q-node with action " << qnode->edge()
			<<" at depth "<< vnode->depth()+1<< endl;


	/*Sample a v-node branch to proceed (for internal q-nodes);
	 * or construct new v-nodes (for leaf q-nodes)*/
	Ext_VNode* next=NULL;
	//double next_value_old=0;// For incremental update of node value
	//int qcount_old=0;
	//qcount_old=qnode->count();
	//int vcount_old=0;

	double AccumReward;


	if(qnode->MeanInfoReady()){
		/*Existing q-node: sample a child node to proceed*/

		/*Termination q-node: terminate the simulation*/
		if(qnode->children().size()==0)
			return qnode->internal_reward();

		/*Normal q-node: sample a child belief node using mean transition information*/
		next=qnode->SampleMeanTrans();
		assert(next);
		logd << "[Traversal @Simulate] Traverse to v-node with obs " << next->edge()
			<<" at depth "<< vnode->depth()+1<< endl;

		if (statistics != NULL) {
			statistics->time_path += (clock() - start) / CLOCKS_PER_SEC;
		}

		//next_value_old=next->value();
		//vcount_old=next->count();

		/*Record step internal reward*/
		AccumReward=qnode->internal_reward();

		/*Traverse to next belief node*/
		prior->Add_in_search(action, vnode->observable_state());
		streams.Advance();
		assert (next != NULL);
		AccumReward += Globals::Discount()
			* Simulate(next,streams,prior, model, statistics);

		/*Leave the child belief node*/
		streams.Back();
		prior->PopLast(true);

		//next->count(next->count()+next->scenarios().size());
		//qnode->count(qnode->count()+next->scenarios().size());
	}
	else{

		if (statistics != NULL
			&& vnode->depth()+1 > statistics->longest_trial_length) {
			statistics->longest_trial_length = vnode->depth()+1;
		}

		start=clock();
		/*Leaf q-node: expand the q-node and create children belief nodes*/
		logd << "[Expand @Simulate] Initializing q-node for action " << action <<" at depth "<< vnode->depth()+1 << endl;

		map<OBS_TYPE, vector<State*> > partitions;
		map<OBS_TYPE, vector<int> > partition_scenarios;

		/*Sample next state for each scenario*/
		double scenario_reward=0, scenario_prob=1.0/vnode->scenarios().size();

		/*The q-node is traversed by K scenarios*/
		//qnode->count(qnode->count()+vnode->scenarios().size());
		//But it is only regared as one simulation

		for (int i=0;i<vnode->scenarios().size();i++) {
			State* current_state = vnode->observable_state();
			State* next_state=model->Copy(current_state);
			next_state->scenario_id=vnode->scenarios()[i];

			//long double trans_prob;

			/*Initialize information for fast simulation and belief update*/
			if(!vnode->faststep_ready()){
				bool fast_step=model->InitMeanStep(*current_state/*, action*/);
				assert(fast_step);
				vnode->faststep_ready(fast_step);
				logd << "   [Expand @Simulate] Initialized v-node fast-step info! "<< endl;
			}

			/*Sample next state for the scenario*/
			bool terminal = model->MeanStep(*vnode->belief(),*next_state, streams.Entry(next_state->scenario_id)
					,action, scenario_reward, /*trans_prob,*/ vnode->faststep_ready());

			/*Prepare the discrete flag for the children belief nodes*/
			OBS_TYPE obs=model->StateToIndex(next_state);

			/*Calculate reward bonus for the sampled next state*/
			scenario_reward += CalculateScenarioRewardBonus(vnode->belief(),
					current_state, action, next_state, vnode->faststep_ready());

			/*Record MeanMDP information*/
			qnode->RecordScenarioTrans(obs, /*trans_prob*/scenario_prob);
			qnode->RecordScenarioReward(scenario_reward, scenario_prob);

			/*Construct discretized partitions*/
			if (!terminal) {
				partitions[obs].push_back(next_state);
				partition_scenarios[obs].push_back(next_state->scenario_id);
			} else {
				model->Free(next_state);
			}
		}

		/*Normalize mean transition probabilities for children nodes*/
		double norm_factor=qnode->NormalizeScenarioTrans();

		/*Add step reward to node value*/
		AccumReward=qnode->internal_reward();

		/*Create children belief nodes (sampled)*/
		for (map<OBS_TYPE, vector<State*> >::iterator it = partitions.begin();
				it != partitions.end(); it++) {
			OBS_TYPE obs=it->first;
			/*only use one representative state from the bin*/
			State* state=partitions[obs][0];

			/*Create the belief node and initialize the value*/
			prior->Add_in_search(action, state);
			streams.Advance();

			logd << "   [Expand @Simulate] Creating v-node" <<" at depth "<< vnode->depth()+1<< endl;
			Ext_VNode* child_vnode = CreateVNode(vnode->depth() + 1,vnode->belief()/*doesn't update belief*/,
					state, streams, prior, model, vnode->scenarios(),
					partition_scenarios[obs],qnode, obs);
			//Doesn't include roll-out in simulation count
			//child_vnode->count(partitions[obs].size());
			logd << "   [Expand @Simulate] New v-node created!" << endl;

			streams.Back();
			prior->PopLast(true);
			qnode->children()[obs] = child_vnode;

			/*Add discounted and weighted child value to node value*/
			AccumReward += Globals::Discount() // discounted
				* partition_scenarios[obs].size() /vnode->scenarios().size() // weighted
				* child_vnode->value();

			/*logi << "   [Expand @Simulate] New v-node contribute "<<Globals::Discount() // discounted
			* partition_scenarios[obs].size() /vnode->scenarios().size() // weighted
			* child_vnode->value()<<" to total discounted reward!" << endl;*/
		}

		if (statistics != NULL) {
			statistics->time_node_expansion += (double) (clock() - start)
				/ CLOCKS_PER_SEC;
			statistics->num_expanded_nodes+=partitions.size();
			statistics->num_nodes_in_level[vnode->depth()+1]+=partitions.size();
		}

		//Update(qnode, AccumReward);
		qnode->Add(AccumReward);
		//vnode->Add(AccumReward);

		logd << "   q-node initialized:";
		logd << " (reward, value, count)=" <<
				"("<<qnode->internal_reward()<<","<<qnode->value()<<","<<qnode->count()<<")" << endl;

		/*q-node is ready for future traversal*/
		qnode->MeanInfoReady(true);
	}

	start=clock();

	if(next!=NULL) {// internal node
		qnode->Add(AccumReward);
		logd << "[Update @Simulate] Update q-node with action " << qnode->edge()
			<<" at depth "<< vnode->depth()+1
			<<" (reward, value, count)=" << "("<<qnode->internal_reward()<<","
			<<qnode->value()<<","<<qnode->count()<<")"
			<< endl;
	}
	vnode->Add(AccumReward);
	logd << "[Update @Simulate] Update v-node";
	if(vnode->edge()==0)
		logd << " at root";
	else
		logd /*<< " with obs " << vnode->edge() */<< " at depth "<< vnode->depth();
	logd <<" (value, count)=" << "("<<vnode->value()<<","<<vnode->count()<<")"
		<< endl;

	if (statistics != NULL) {
		statistics->time_backup += double(clock() - start) / CLOCKS_PER_SEC;
	}

	/* Back-up value along the traversal path
	if(next!=NULL) {// internal node
		Update(qnode, next_value_old, next->value(), vcount_old, next->count(),
			qcount_old, qnode->count());
		logi << "[Update @Simulate] Update q-node with action " << qnode->edge()
			<<" at depth "<< vnode->depth()+1
			<<" (value, count)=" << "("<<qnode->value()<<","<<qnode->count()<<")"
			<< endl;
	}

	Update(vnode, qnode->value());
	logi << "[Update @Simulate] Update v-node with obs " << vnode->edge()
		<<" at depth "<< vnode->depth()
		<<" (value, count)=" << "("<<vnode->value()<<","<<vnode->count()<<")"
		<< endl;*/

	/*Return estimated optimal value of the belief node*/
	return AccumReward;
}

double POMDPLite::CalculateScenarioRewardBonus(const Belief* belief, const State* state, const ACT_TYPE action,
		const State* next_state, bool fast_update){
	/*Calculate b_s'*/
    const DiscreteBelief* dis_belief=static_cast<const DiscreteBelief*>(belief);
    DiscreteBelief* next_dis_belief=static_cast<DiscreteBelief*>(next_belief);
    assert(dis_belief);
    assert(next_dis_belief);
    next_dis_belief->CopyContent(dis_belief);
    next_dis_belief->Update(state, next_state, action, fast_update);

    /*Calculate c||b_s'-b_s||*/
    return next_dis_belief->Distance(dis_belief) * Globals::config.reward_bonus_constant;
}

Ext_VNode* POMDPLite::CreateVNode(int depth, Belief* belief, State* state,
		RandomStreams& streams, SolverPrior* prior, const DSPOMDP* model,
		const std::vector<int>& scenarios, std::vector<int>& rollout_scenarios,
		QNode* parent,OBS_TYPE edge) {
	/*Create the belief node*/
	Ext_VNode* vnode = new Ext_VNode(belief, state, true/*external belief*/,
			0/*count*/, Globals::NEG_INFTY/*value*/, scenarios,
			depth, parent, edge);

	/*Initialize the value and count*/
	vnode->value( Rollout(vnode,rollout_scenarios, streams, prior) );
	//Dosen't include roll-out in simulation count
	//vnode->count( rollout_scenarios.size() );
	if(edge==0)// root
		logd << "[CreateVNode] root-node info: (value, count)=" <<
				"("<<vnode->value()<<","<<vnode->count()<<")" << endl;
	else
		logd << "      [CreateVNode] v-node info: (value, count)=" <<
				"("<<vnode->value()<<","<<vnode->count()<<")" << endl;

	/*Create children q-nodes for the belief node*/
	const vector<double>& action_probs=
			prior->ComputePreference(belief, state);

	for (ACT_TYPE action = 0; action < model->NumActions(); action++) {
		Ext_QNode* qnode = new Ext_QNode(vnode, action);
		qnode->count(0);
		qnode->value(-10000000000);
		qnode->prior_probability(action_probs[action]);

		vnode->children().push_back(qnode);
	}

	return vnode;
}

double POMDPLite::Rollout(Ext_VNode* vnode,std::vector<int>& rollout_scenarios,
		RandomStreams& streams,
		SolverPrior* prior)
{
	double approximateValue=0;
	/*Initialize the value*/
	approximateValue=prior->ComputeValue(vnode->belief(), vnode->observable_state(),rollout_scenarios,
			streams, vnode)
		/** vnode->Weight()*/;

    return approximateValue;
}

void POMDPLite::Update(Ext_VNode* vnode, double new_value) {
	if (vnode->IsLeaf()) {
		return;
	}

	if (vnode->value() <= new_value) {// new max
		vnode->value(new_value);
	}
	else // not new max, but possible changing the old max, recalculate max
	{
		for (ACT_TYPE action = 0; action < vnode->children().size(); action++) {
				QNode* qnode = vnode->Child(action);
				vnode->value( max(vnode->value(), qnode->value()) );
		}
	}

	if(isDebug)
		cout<<__FUNCTION__<<" vnode: "<< vnode<<" "<<vnode->value()<<endl;
}

void POMDPLite::Update(Ext_QNode* qnode) {
	double value = 0;

	int total_child_count=0;
	double total_child_value=0;
	map<OBS_TYPE, VNode*>& children = qnode->children();
	for (map<OBS_TYPE, VNode*>::iterator it = children.begin();
		it != children.end(); it++) {
		VNode* vnode = it->second;

		value += vnode->value()*vnode->count();
		total_child_count+=vnode->count();
		total_child_value+=vnode->value()*vnode->count();
	}
	qnode->value(qnode->internal_reward()+Globals::config.discount * value/qnode->count());

	if(isDebug){
		cout<<__FUNCTION__<<"_recompute"<<"@"<<__LINE__<<" qnode: "<< qnode<<" "<<qnode->value()
				<<" total child value "<<total_child_value<<endl;
		if(total_child_count!=qnode->count())
			cout<<"wrong counting @ "<< qnode<<" child_count "<<total_child_count<<" count "<< qnode->count()<<endl;
	}
}

void POMDPLite::Update(Ext_QNode* qnode, double value) {
	qnode->value(value);
	if(isDebug)
		cout<<__FUNCTION__<<"_initial"<<"@"<<__LINE__<<" qnode: "<< qnode<<" "<<qnode->value()<<endl;

	if(isDebug){
			Update(qnode);
			if(abs(qnode->value()-value)>1e-4){
				map<OBS_TYPE, VNode*>& children = qnode->children();
				cout<<  " details: "<< qnode<<" internal reward="<<qnode->internal_reward()
						<<" #children="<<qnode->children().size()<<" - ";
				for (map<OBS_TYPE, VNode*>::iterator it = children.begin();
					it != children.end(); it++) {
					VNode* vnode = it->second;
					cout<< vnode->value()<<"("<<vnode->count()<<")"<<", ";
				}
				cout<<endl;
			}
		}
}

void POMDPLite::Update(Ext_QNode* qnode, double old_child_value, double new_child_value,
		int old_vcount, int new_vcount, int old_qcount, int new_qcount) {
	double old_value=0;
	if(isDebug){
		cout<<__FUNCTION__<<"_incremental"<<"@"<<__LINE__<< qnode<<" "
			<<" old value "<<qnode->value()<<endl;
		old_value=qnode->value();
	}
	double value = ((qnode->value()-qnode->internal_reward())*old_qcount-
			Globals::config.discount*old_child_value* old_vcount
			+Globals::config.discount*new_child_value*new_vcount)
			/new_qcount
			+qnode->internal_reward();
	qnode->value(value);
	if(isDebug){
		cout<<  " updated value: "<<qnode->value()
			<<" (old_child_value "<<old_child_value<<" new_child_value "<<new_child_value
			<<" old_vcount "<<old_vcount<<" new_vcount "<<new_vcount
			<<" old_qcount "<<old_qcount<<" new_qcount "<<new_qcount<<")"
			<<endl;

		Update(qnode);
		if(abs(qnode->value()-value)>1e-4){
			map<OBS_TYPE, VNode*>& children = qnode->children();
			cout<<  " details: "<< qnode<<" internal reward="<<qnode->internal_reward()
					<<" #children="<<qnode->children().size()<<" - ";
			for (map<OBS_TYPE, VNode*>::iterator it = children.begin();
				it != children.end(); it++) {
				VNode* vnode = it->second;
				cout<< vnode->value()<<"("<<vnode->count()<<")"<<", ";
			}
			cout<<endl;
			double old_child_sum=(old_value-qnode->internal_reward())*old_qcount;
			double substracted=Globals::config.discount*old_child_value* old_vcount;
			double added=Globals::config.discount*new_child_value*new_vcount;
			cout<<"calculation details: "<<" old child value sum="<<old_child_sum
					<<" subtracted="<<substracted
					<<" added="<<added
					<<" after sub="<< old_child_sum-substracted
					<<" after add=" << old_child_sum-substracted+added
					<<" after divide=" << (old_child_sum-substracted+added)/new_qcount
					<<" after add_reward=" << (old_child_sum-substracted+added)/new_qcount+qnode->internal_reward();
			cout <<endl;
		}
	}
}


void POMDPLite::BeliefUpdate(ACT_TYPE action, OBS_TYPE obs) {
	double start = get_time_second();
	DiscreteBelief* dis_belief=static_cast<DiscreteBelief*>(belief_);
	assert(dis_belief);
	assert(prior_->history().Size());
	//const State* cur_state=model_->IndexToState(obs);
	const State* cur_state=world_->GetCurrentState();
	state_=static_cast<const MeanMDP*>(model_)->CopyForSearch(cur_state);//create a new state for search

	dis_belief->DeepUpdate(prior_->history_states(),
			prior_->history_states_for_search(),
			cur_state,
			state_, action);
	//dis_belief->Update(prior_->history().LastState(),cur_state, action);
	prior_->Add(action, cur_state);
	prior_->Add_in_search(-1, state_);

	//logi << "[POMDPLite::Update] Updated belief, history and root with action "
	//	<< action << ", observation " << obs
	//	<< " in " << (get_time_second() - start) << "s" << endl;
}


double POMDPLite::UCB[UCB_N][UCB_n];
bool POMDPLite::InitialisedFastUCB = true;

static double UCBfunc(int N, int n){
	return sqrt(N) / (1+n);
}

static double UCBfunc(int N, double logN, int n){
	return sqrt(logN / n);
}

void POMDPLite::InitFastPUCB()
{
    cout << "Initializing fast UCB table... ";
    for (int N = 0; N < UCB_N; ++N)
        for (int n = 0; n < UCB_n; ++n)
            if (n == 0)
                UCB[N][n] = Globals::POS_INFTY;
            else
                UCB[N][n] = Globals::config.exploration_constant * UCBfunc(N,n);
    cout << "done" << endl;
    InitialisedFastUCB = true;
}

double POMDPLite::FastPUCB(int N, int n, double logN, Ext_QNode* qnode)
{
    if (InitialisedFastUCB && N < UCB_N && n < UCB_n)
    	return UCB[N][n]*qnode->prior_probability();

    if (n == 0)
    	return Globals::POS_INFTY;
    else
    	return Globals::config.exploration_constant * UCBfunc(N,n) *qnode->prior_probability();
}

double POMDPLite::FastPUCB(int N, int n, Ext_QNode* qnode)
{
    if (InitialisedFastUCB && N < UCB_N && n < UCB_n)
    	return UCB[N][n]*qnode->prior_probability();

    if (n == 0)
    	return Globals::POS_INFTY;
    else
    	return Globals::config.exploration_constant * UCBfunc(N,n) *qnode->prior_probability();
}

} // namespace despot
