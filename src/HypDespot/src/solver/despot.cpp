#include <despot/solver/despot.h>
#include <despot/solver/pomcp.h>
#include <iomanip>      // std::setprecision
#include <despot/core/builtin_upper_bounds.h>

using namespace std;
static double HitCount = 0;

static double InitBoundTime = 0;
static double BlockerCheckTime = 0;
static double TreeExpansionTime = 0;
static double PathTrackTime = 0;
static double AveRewardTime = 0;
static double MakeObsNodeTime = 0;
static double ModelStepTime = 0;
static double ParticleCopyTime = 0;
static double ObsCountTime = 0;
static double TotalExpansionTime = 0;
static long Num_searches = 0;
static int InitialSearch = true;
namespace despot {
bool DESPOT::use_GPU_;
int DESPOT::num_Obs_element_in_GPU;
double DESPOT::Initial_root_gap;
bool DESPOT::Debug_mode = false;
bool DESPOT::Print_nodes = false;

MemoryPool<QNode> DESPOT::qnode_pool_;
MemoryPool<Shared_QNode> DESPOT::s_qnode_pool_;

MemoryPool<VNode> DESPOT::vnode_pool_;
MemoryPool<Shared_VNode> DESPOT::s_vnode_pool_;
static int step_counter = 0;



DESPOT::DESPOT(const DSPOMDP* model, ScenarioLowerBound* lb,
               ScenarioUpperBound* ub, Belief* belief, bool use_GPU) :
	Solver(model, belief), root_(NULL), lower_bound_(lb), upper_bound_(ub) {
	assert(model != NULL);
	model_ = model;

	use_GPU_ = Globals::config.useGPU;

	cout<<"DESPOT GPU mode "<< use_GPU_ << endl;

	QuickRandom::InitRandGen();

	if (Globals::config.exploration_mode == VIRTUAL_LOSS) {
		Globals::config.exploration_constant = abs(model->GetMaxReward());
	}

	if (use_GPU_) {
		PrepareGPUMemory(model, model->NumActions(),
		                 model->NumObservations());
	}

}

DESPOT::~DESPOT() {

	QuickRandom::DestroyRandGen();

	if (use_GPU_)
		ClearGPUMemory(model_);
}

ScenarioLowerBound* DESPOT::lower_bound() const {
	return lower_bound_;
}

ScenarioUpperBound* DESPOT::upper_bound() const {
	return upper_bound_;
}

VNode* DESPOT::Trial(VNode* root, RandomStreams& streams,
                     ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
                     const DSPOMDP* model, History& history, SearchStatistics* statistics) {
	VNode* cur = root;
	//cout<<"Root bounds:"<<root->lower_bound()<<"/"<<root->upper_bound()<<endl;

	int hist_size = history.Size();
	bool DoPrint = false;
	float weu = 0;
	do {
		if (statistics != NULL
		        && cur->depth() > statistics->longest_trial_length) {
			statistics->longest_trial_length = cur->depth();
		}

		double start1 = clock();
		ExploitBlockers(cur);
		BlockerCheckTime += double(clock() - start1) / CLOCKS_PER_SEC;

		if (Gap(cur) == 0) {
			break;
		}

		if (cur->IsLeaf()) {
			double start = clock();
			Expand(cur, lower_bound, upper_bound, model, streams, history);

			if (statistics != NULL) {
				statistics->time_node_expansion += (double) (clock() - start)
				                                   / CLOCKS_PER_SEC;
				statistics->num_expanded_nodes++;
				statistics->num_tree_particles += cur->particles().size();
			}

			TreeExpansionTime += double(clock() - start) / CLOCKS_PER_SEC;
		}

		double start = clock();
		QNode* qstar = SelectBestUpperBoundNode(cur);

		if (DoPrint || (FIX_SCENARIO == 1 && qstar))
		{
			cout.precision(4);
			cout << "thread " << 0 << " " << " trace down to QNode" << " get old node"
			     << " at depth " << cur->depth() + 1
			     << ": weight=" << qstar->Weight() << ", reward=" << qstar->step_reward / qstar->Weight()
			     << ", lb=" << qstar->lower_bound() / qstar->Weight() <<
			     ", ub=" << qstar->upper_bound() / qstar->Weight() <<
			     ", uub=" << qstar->utility_upper_bound() / qstar->Weight() <<
			     ", edge=" << qstar->edge() <<
			     ", v_loss=" << 0;
			cout << endl;
		}

		VNode* next = SelectBestWEUNode(qstar);

		if (DoPrint || (FIX_SCENARIO == 1 && next))
		{
			cout.precision(4);
			cout << "thread " << 0 << " " << " trace down to VNode" << " get old node"
			     << " at depth " << next->depth()
			     << ": weight=" << next->Weight() << ", reward="
			     << "--"
			     << ", lb=" << next->lower_bound() / next->Weight() <<
			     ", ub=" << next->upper_bound() / next->Weight() <<
			     ", uub=" << next->utility_upper_bound() / next->Weight() <<
			     ", edge=" << next->edge() <<
			     ", v_loss=" << 0;
			cout << endl;
		}
		//model->Debug();
		if (statistics != NULL) {
			statistics->time_path += (clock() - start) / CLOCKS_PER_SEC;
		}
		PathTrackTime += double(clock() - start) / CLOCKS_PER_SEC;

		if (next == NULL) {
			if (DoPrint)cout << "Debug end trial: Null next node" << endl;
			break;
		}

		cur = next;
		history.Add(qstar->edge(), cur->edge());


		weu = WEU(cur);

	} while (cur->depth() < Globals::config.search_depth && weu > 0
	         && !Globals::Timeout(Globals::config.time_per_move));

	if (DoPrint)cout << "Debug end trial: WEU=" << WEU(cur) << endl;

	history.Truncate(hist_size);

	return cur;
}
Shared_VNode* DESPOT::Trial(Shared_VNode* root, RandomStreams& streams,
                            ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
                            const DSPOMDP* model, History& history, bool& Expansion_done,
                            Shared_SearchStatistics* statistics) {
	Shared_VNode* cur = root;
	//cout<<"Root bounds:"<<root->lower_bound()<<"/"<<root->upper_bound()<<endl;

	int hist_size = history.Size();
	do {
		if (statistics != NULL
		        && cur->depth() > statistics->Get_longest_trial_len()) {
			statistics->Update_longest_trial_len(cur->depth());
		}
		auto start1 = Time::now();
		ExploitBlockers(cur);

		BlockerCheckTime += Globals::ElapsedTime(start1);

		if (Gap(cur) == 0) {
			Globals::Global_print_value(this_thread::get_id(), 1,
			                            "debug trial end blocker found");
			break;
		}
		try {
			lock_guard < mutex > lck(cur->GetMutex());
			if (((VNode*) cur)->IsLeaf()) {
				auto start = Time::now();
				Expand(((VNode*) cur), lower_bound, upper_bound, model, streams,
				       history);

				if (statistics != NULL) {
					statistics->Add_time_node_expansion( Globals::ElapsedTime(start) );
					statistics->Inc_num_expanded_nodes();
					statistics->Add_num_tree_particles(
					    ((VNode*) cur)->particles().size());
				}

				TreeExpansionTime += Globals::ElapsedTime(start);
				Expansion_done = true;

			}
		} catch (exception &e) {
			cout << "Locking error: " << e.what() << endl;
		}

		auto start = Time::now();
		Shared_QNode* qstar;
		Shared_VNode* next;
		try {
			//lock_guard < mutex > lck(cur->GetMutex());
			if (!cur->IsLeaf()) {
				qstar = SelectBestUpperBoundNode(cur);
				//Globals::Global_print_down(this_thread::get_id(), qstar, ((VNode*)cur)->depth()+1);
				if (qstar)
					Globals::Global_print_node(this_thread::get_id(), qstar,
						   ((VNode*) cur)->depth() + 1, ((QNode*) qstar)->step_reward,
						   ((QNode*) qstar)->lower_bound(),
						   ((QNode*) qstar)->upper_bound(),
						   ((QNode*) qstar)->utility_upper_bound(),
						   qstar->exploration_bonus,
						   ((QNode*) qstar)->Weight(),
						   ((QNode*) qstar)->edge(),
						   -1000,
						   "trace down to QNode");
			}
			else
				cout << "qstar is leaf" << endl;

		} catch (exception &e) {
			cout << "Locking error: " << e.what() << endl;
		}

		try {
			next = static_cast<Shared_VNode*>(SelectBestWEUNode(qstar));
			if (next != NULL) {
				Globals::Global_print_node(this_thread::get_id(), next,
						   ((VNode*) next)->depth(), 0,
						   ((VNode*) next)->lower_bound(),
						   ((VNode*) next)->upper_bound(),
						   ((VNode*) next)->utility_upper_bound(),
						   next->exploration_bonus,
						   ((VNode*) next)->Weight(),
						   ((VNode*) next)->edge(),
						   WEU((VNode*) next),
						   "trace down to VNode");
			}
		} catch (exception &e) {
			cout << "Locking error: " << e.what() << endl;
		}

		if (statistics != NULL) {
			statistics->Add_time_path( Globals::ElapsedTime(start) );
		}
		PathTrackTime += Globals::ElapsedTime(start);

		if (next == NULL) {
			break;
		}
		cur = next;
		history.Add(qstar->edge(), cur->edge());



	} while (cur->depth() < Globals::config.search_depth
	         && WEU((VNode*) cur) > 0 &&
	         !Globals::Timeout(Globals::config.time_per_move));

	history.Truncate(hist_size);

	return cur;
}
void DESPOT::ExploitBlockers(VNode* vnode) {
	if (Globals::config.pruning_constant <= 0) {
		logd << "Return: small pruning " << Globals::config.pruning_constant
		     << endl;
		return;
	}
	VNode* cur = vnode;
	while (cur != NULL) {
		VNode* blocker = FindBlocker(cur);

		if (blocker != NULL) {
			if (cur->parent() == NULL || blocker == cur) {
				double value = cur->default_move().value;
				cur->lower_bound(value);
				cur->upper_bound(value);
				cur->utility_upper_bound(value);
			} else {
				const map<OBS_TYPE, VNode*>& siblings =
				    cur->parent()->children();
				for (map<OBS_TYPE, VNode*>::const_iterator it =
				            siblings.begin(); it != siblings.end(); it++) {
					VNode* node = it->second;
					double value = node->default_move().value;
					node->lower_bound(value);
					node->upper_bound(value);
					node->utility_upper_bound(value);
				}
			}

			Backup(cur, false);

			if (cur->parent() == NULL) {
				cur = NULL;
			} else {
				cur = cur->parent()->parent();
			}
		} else {
			break;
		}
	}
}
void DESPOT::ExploitBlockers(Shared_VNode* vnode) {
	if (Globals::config.pruning_constant <= 0) {
		logd << "Return: small pruning " << Globals::config.pruning_constant
		     << endl;
		return;
	}
	Shared_VNode* cur = vnode;
	while (cur != NULL) {
		Shared_VNode* blocker = static_cast<Shared_VNode*>(FindBlocker(cur));

		if (blocker != NULL) {
			if (cur->parent() == NULL || blocker == cur) {
				lock_guard < mutex > lok(cur->GetMutex());			//lock cur
				double value = ((VNode*) cur)->default_move().value;
				((VNode*) cur)->lower_bound(value);
				((VNode*) cur)->upper_bound(value);
				((VNode*) cur)->utility_upper_bound(value);
			} else {
				const map<OBS_TYPE, VNode*>& siblings =
				    cur->parent()->children();
				for (map<OBS_TYPE, VNode*>::const_iterator it =
				            siblings.begin(); it != siblings.end(); it++) {
					Shared_VNode* node = static_cast<Shared_VNode*>(it->second);
					lock_guard < mutex > lok(node->GetMutex());		//lock node
					double value = ((VNode*) node)->default_move().value;
					((VNode*) node)->lower_bound(value);
					((VNode*) node)->upper_bound(value);
					((VNode*) node)->utility_upper_bound(value);
				}
			}

			Backup(cur, false);

			if (cur->parent() == NULL) {
				cur = NULL;
			} else {
				cur = cur->parent()->parent();
			}
		} else {
			break;
		}
	}
}
void DESPOT::ExpandTreeServer(RandomStreams streams,
                              ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
                              const DSPOMDP* model, History history,
                              Shared_SearchStatistics* statistics, double& used_time,
                              double& explore_time, double& backup_time, int& num_trials,
                              double timeout, MsgQueque<Shared_VNode>& node_queue,
                              MsgQueque<Shared_VNode>& print_queue, int threadID) {
	//cout << "Create expansion thread " << this_thread::get_id() << endl;
	Globals::ChooseGPUForThread();			//otherwise the GPUID would be 0 (default)
	Globals::AddMappedThread(this_thread::get_id(), threadID);
	used_time = 0;
	explore_time = 0;
	backup_time = 0;
	num_trials = 0;
	Shared_VNode* root = node_queue.receive(true, timeout);

	//for(;;)
	do {
		if (root == NULL || Globals::Timeout(timeout)) {
			int the_case = (root == NULL) ? 0 : 1;
			Globals::Global_print_deleteT(this_thread::get_id(), 0, the_case);
			//node_queue.WakeOneThread();
			print_queue.WakeOneThread();
			break;
		}
		if (statistics->num_expanded_nodes > 100000000)
			break;
		auto start = Time::now();
		root->visit_count_++;

		bool Expansion_done = false;
		VNode* cur = Trial(root, streams, lower_bound, upper_bound, model,
		                   history, Expansion_done, statistics);

		used_time += Globals::ElapsedTime(start);
		explore_time += Globals::ElapsedTime(start);

		start = Time::now();
		if (Expansion_done)
			Backup(cur, true);
		else
		{
			while (cur->parent() != NULL) {
				Shared_QNode* qnode = static_cast<Shared_QNode*>(cur->parent());
				if (Globals::config.exploration_mode == VIRTUAL_LOSS)
				{
					if (cur->depth() >= 0)
						qnode->exploration_bonus += CalExplorationValue(cur->depth());
					else
						qnode->exploration_bonus = 0;
				}
				cur = qnode->parent();
				if ( Globals::config.exploration_mode == VIRTUAL_LOSS) //release virtual loss
					static_cast<Shared_VNode*>(cur)->exploration_bonus += CalExplorationValue(((VNode*) cur)->depth());
				else if ( Globals::config.exploration_mode == UCT) //release virtual loss
					static_cast<Shared_VNode*>(cur)->exploration_bonus +=
					    CalExplorationValue(((VNode*) cur)->depth()) * ((VNode*) cur)->Weight();
			}
		}


		if (statistics != NULL) {
			statistics->Add_time_backup(Globals::ElapsedTime(start));
		}
		used_time += Globals::ElapsedTime(start);
		backup_time += Globals::ElapsedTime(start);

		Globals::AddSerialTime(used_time);
		num_trials++;

		print_queue.send(root);

		if (DESPOT::Debug_mode || FIX_SCENARIO == 1)
			if (num_trials == 10)
				break;
	} while (used_time * (num_trials + 1.0) / num_trials < timeout
	         && !Globals::Timeout(Globals::config.time_per_move)
	         && (((VNode*) root)->upper_bound() - ((VNode*) root)->lower_bound())
	         > 1e-6);
	Globals::Global_print_deleteT(this_thread::get_id(), 0, 1);

	Globals::MinusActiveThread();
	print_queue.WakeOneThread();
}

void PrintServer(MsgQueque<Shared_VNode>& print_queue, float timeout) {
	//cout << "Create printing thread " << this_thread::get_id() << endl;
	for (;;) {
		Shared_VNode* node = print_queue.receive(false, timeout);
		if (node == NULL || Globals::Timeout(timeout)) {
			int the_case = (node == NULL) ? 0 : 1;
			Globals::Global_print_deleteT(this_thread::get_id(), 1, the_case);
			print_queue.WakeOneThread();
			break;
		}
	}
}

VNode* DESPOT::ConstructTree(vector<State*>& particles, RandomStreams& streams,
                             ScenarioLowerBound* lower_bound, ScenarioUpperBound* upper_bound,
                             const DSPOMDP* model, History& history, double timeout,
                             SearchStatistics* statistics) {
	if (statistics != NULL) {
		statistics->num_particles_before_search = model->NumActiveParticles();
	}

	Globals::RecordStartTime();

	double used_time = 0;
	double explore_time = 0;
	double backup_time = 0;

	vector<int> particleIDs;
	particleIDs.resize(particles.size());
	for (int i = 0; i < particles.size(); i++) {
		particles[i]->scenario_id = i;
		particleIDs[i] = i;
	}


	VNode* root = NULL;
	if (Globals::config.use_multi_thread_) {
		root = new Shared_VNode(particles, particleIDs);
		if (Globals::config.exploration_mode == UCT)
			static_cast<Shared_VNode*>(root)->visit_count_ = 1.1;
	} else
		root = new VNode(particles, particleIDs);

	if (use_GPU_) {
		PrepareGPUDataForRoot(root, model, particleIDs, particles);
	}

	logd
	        << "[DESPOT::ConstructTree] START - Initializing lower and upper bounds at the root node.";
	if (use_GPU_)
		GPU_InitBounds(root, lower_bound, upper_bound, model, streams, history);
	else
		InitBounds(root, lower_bound, upper_bound, streams, history);

	Initial_root_gap = Gap(root);
	logd
	        << "[DESPOT::ConstructTree] END - Initializing lower and upper bounds at the root node.";

	if (statistics != NULL) {
		statistics->initial_lb = root->lower_bound();
		statistics->initial_ub = root->upper_bound();

		if (FIX_SCENARIO == 1)
			cout << "Root bounds: (" << statistics->initial_lb << "," << statistics->initial_ub << ")" << endl;
	}

	int num_trials = 0;

	used_time = Globals::ElapsedTime();
	cout << std::setprecision(5) << "Root preperation in " << used_time << " s" << endl;

	Globals::ResetSerialTime();

	if (Globals::config.use_multi_thread_) {

		cout << "Start!" << endl << endl;
		double thread_used_time[Globals::config.NUM_THREADS];
		double thread_explore_time[Globals::config.NUM_THREADS];
		double thread_backup_time[Globals::config.NUM_THREADS];
		int num_trials_t[Globals::config.NUM_THREADS];
		for (int i = 0; i < Globals::config.NUM_THREADS; i++) {
			Expand_queue.send(static_cast<Shared_VNode*>(root));
			thread_used_time[i] = used_time;
			thread_explore_time[i] = 0;
			thread_backup_time[i] = 0;
		}

		vector<future<void>> futures;
		for (int i = 0; i < Globals::config.NUM_THREADS; i++) {
			RandomStreams local_stream(streams);
			History local_history(history);
			futures.push_back(
			    async(launch::async, &ExpandTreeServer, local_stream,
			          lower_bound, upper_bound, model, local_history,
			          static_cast<Shared_SearchStatistics*>(statistics),
			          ref(thread_used_time[i]),
			          ref(thread_explore_time[i]),
			          ref(thread_backup_time[i]), ref(num_trials_t[i]),
			          timeout, ref(Expand_queue), ref(Print_queue), i));
		}

		futures.push_back(
		    async(launch::async, &PrintServer, ref(Print_queue), timeout));

		double passed_time = Globals::ElapsedTime();
		cout << std::setprecision(5) << Globals::config.NUM_THREADS << " threads started at the "
		     << passed_time << "'th second" << endl;
		try {
			while (!futures.empty()) {
				auto ftr = std::move(futures.back());
				futures.pop_back();
				ftr.get();
			}
			for (int i = 0; i < Globals::config.NUM_THREADS; i++) {
				used_time = max(used_time, thread_used_time[i]);
				explore_time = max(explore_time, thread_explore_time[i]);
				backup_time = max(backup_time, thread_backup_time[i]);
				num_trials += num_trials_t[i];
			}
		} catch (exception & e) {
			cout << "Exception" << e.what() << endl;
		}

		passed_time = Globals::ElapsedTime();
		cout << std::setprecision(5) << "Tree expansion in "
		     << passed_time << " s" << endl;

	} else {

		do {
			if (statistics->num_expanded_nodes > 10000)
				break;
			double start = clock();
			VNode* cur = Trial(root, streams, lower_bound, upper_bound, model,
			                   history, statistics);
			used_time += double(clock() - start) / CLOCKS_PER_SEC;
			explore_time += double(clock() - start) / CLOCKS_PER_SEC;
			//model->Debug();
			start = clock();
			Backup(cur, true);
			if (statistics != NULL) {
				statistics->time_backup += double(
				                               clock() - start) / CLOCKS_PER_SEC;
			}
			used_time += double(clock() - start) / CLOCKS_PER_SEC;
			backup_time += double(clock() - start) / CLOCKS_PER_SEC;
			//model->Debug();
			num_trials++;
			Globals::AddSerialTime(used_time);

			if (DESPOT::Debug_mode || FIX_SCENARIO == 1)
				if (num_trials == 10)
					break;

		} while (used_time * (num_trials + 1.0) / num_trials < timeout
		         && !Globals::Timeout(Globals::config.time_per_move)
		         && (root->upper_bound() - root->lower_bound()) > 1e-6);
	}

	logi << "[DESPOT::Search] Time for EXPLORE: " << explore_time << "s"
	     << endl;
	logi << "	[DESPOT::Search] Time for BLOCKER_CHECK: " << BlockerCheckTime
	     << "s" << endl;
	logi << "	[DESPOT::Search] Time for TREE_EXPANSION: " << TreeExpansionTime
	     << "s" << endl;
	logi << "		[DESPOT::Search] Time for AVE_REWARD: " << AveRewardTime << "s"
	     << endl;
	logi << "			[DESPOT::Search] Time for STEP_MODEL: " << ModelStepTime << "s"
	     << endl;
	logi << "			[DESPOT::Search] Time for COPY_PARTICLE: " << ParticleCopyTime
	     << "s" << endl;
	logi << "			[DESPOT::Search] Time for COUNT_OBS: " << ObsCountTime << "s"
	     << endl;
	logi << "		[DESPOT::Search] Time for MAKE_NODES: "
	     << MakeObsNodeTime - InitBoundTime << "s" << endl;
	logi << "		[DESPOT::Search] Time for INIT_BOUNDS: " << InitBoundTime << "s"
	     << endl;
	logi << "	[DESPOT::Search] Time for PATH_TRACKING: " << PathTrackTime << "s"
	     << endl;
	logi << "[DESPOT::Search] Time for BACK_UP: " << backup_time << "s" << endl;

	if (statistics != NULL) {
		statistics->num_particles_after_search = model->NumActiveParticles();
		statistics->num_policy_nodes = root->PolicyTreeSize();
		statistics->num_tree_nodes = root->Size();
		statistics->final_lb = root->lower_bound();
		statistics->final_ub = root->upper_bound();
		statistics->time_search = used_time;
		statistics->num_trials = num_trials;
	}

	return root;
}

void DESPOT::Compare() {
	vector<State*> particles = belief_->Sample(Globals::config.num_scenarios);
	SearchStatistics statistics;

	RandomStreams streams = RandomStreams(Globals::config.num_scenarios,
	                                      Globals::config.search_depth);

	VNode* root = ConstructTree(particles, streams, lower_bound_, upper_bound_,
	                            model_, history_, Globals::config.time_per_move, &statistics);

	CheckDESPOT(root, root->lower_bound());
	CheckDESPOTSTAR(root, root->lower_bound());
	delete root;
}

void DESPOT::InitLowerBound(VNode* vnode, ScenarioLowerBound* lower_bound,
                            RandomStreams& streams, History& history) {
	//cout<<vnode->depth()<<" "<<history.Size()<<endl;
	streams.position(vnode->depth());
	/*if(vnode->depth()==89)
		cout<<"Reach depth 89!"<<endl;*/
	ValuedAction move = lower_bound->Value(vnode->particles(), streams,
	                                       history);
	move.value *= Globals::Discount(vnode->depth());
	vnode->default_move(move);
	//cout<<"vnode at depth "<<vnode->depth()<<", default move value="<< move.value
	//	<<" weight="<< vnode->Weight()<< endl;//Debug
	vnode->lower_bound(move.value);
}

void DESPOT::InitUpperBound(VNode* vnode, ScenarioUpperBound* upper_bound,
                            RandomStreams& streams, History& history) {
	streams.position(vnode->depth());
	double upper = upper_bound->Value(vnode->particles(), streams, history);
	vnode->utility_upper_bound(upper * Globals::Discount(vnode->depth()));
	upper = upper * Globals::Discount(vnode->depth())
	        - Globals::config.pruning_constant;
	vnode->upper_bound(upper);
}

void DESPOT::InitBounds(VNode* vnode, ScenarioLowerBound* lower_bound,
                        ScenarioUpperBound* upper_bound, RandomStreams& streams,
                        History& history) {
	//if(vnode->depth()==3)
	//	cout<<vnode->depth()<<" "<<history.Size()<<" "<<streams.position()<<endl;
	InitLowerBound(vnode, lower_bound, streams, history);
	InitUpperBound(vnode, upper_bound, streams, history);
	if (vnode->upper_bound() < vnode->lower_bound()
	        // close gap because no more search can be done on leaf node
	        || vnode->depth() == Globals::config.search_depth - 1) {
		vnode->upper_bound(vnode->lower_bound());
	}
}

ValuedAction DESPOT::Search() {
	if (logging::level() >= logging::DEBUG) {
		model_->PrintBelief(*belief_);
	}
	Num_searches++;
	//model_->PrintBelief(*belief_); // for debugging

	if (Globals::config.time_per_move <= 0) // Return a random action if no time is allocated for planning
		return ValuedAction(Random::RANDOM.NextInt(model_->NumActions()),
		                    Globals::NEG_INFTY);

	double start = get_time_second();

	if (Debug_mode){
		step_counter++;
	}

	vector<State*> particles;
	if (FIX_SCENARIO == 1) {
		ifstream fin;
		string particle_file = "Particles" + std::to_string(step_counter) + ".txt";
		fin.open(particle_file, std::ios::in);
		if(!fin.is_open()){
			cerr << "Unable to open particle file " << particle_file << endl;
			exit(1);
		}
		particles.resize(Globals::config.num_scenarios);
		model_->ImportStateList(particles, fin);
		fin.close();

		logi << "[FIX_SCENARIO] particles read from file "<< particle_file << endl;
	} else {
		if(Debug_mode)
			std::srand(0);
		particles = belief_->Sample(Globals::config.num_scenarios);
	}
	logi << "[DESPOT::Search] Time for sampling " << particles.size()
	     << " particles: " << (get_time_second() - start) << "s" << endl;

	if (FIX_SCENARIO == 2) {
		for (int i = 0; i < particles.size(); i++) {
			particles[i]->scenario_id = i;
		}
		if (InitialSearch)
		{
			ofstream fout;
			string particle_file = "Particles" + std::to_string(step_counter) + ".txt";
			fout.open(particle_file, std::ios::trunc);
			assert(fout.is_open());
			fout << particles.size() << endl;
			for (int i = 0; i < Globals::config.num_scenarios; i++) {
				model_->ExportState(*particles[i], fout);
			}

			fout.close();
			InitialSearch = false;
		}
		else
		{
			ofstream fout;
			string particle_file = "Particles" + std::to_string(step_counter) + ".txt";
			fout.open(particle_file, std::ios::trunc);
			assert(fout.is_open());
			fout << particles.size() << endl;
			for (int i = 0; i < Globals::config.num_scenarios; i++) {
				model_->ExportState(*particles[i], fout);
			}
			fout.close();
		}
	}

	if (Debug_mode)
		;//model_->PrintParticles(particles);
	else{
		if (Globals::config.silence != true) {
			model_->PrintParticles(particles);
		}
	}
	statistics_ = Shared_SearchStatistics();

	start = get_time_second();
	static RandomStreams streams;

	if (FIX_SCENARIO == 1 || Debug_mode) {
		std::ifstream fin;
		string stream_file = "Streams" + std::to_string(step_counter) + ".txt";

		fin.open(stream_file, std::ios::in);
		streams.ImportStream(fin, Globals::config.num_scenarios,
		                     Globals::config.search_depth);

		fin.close();
		cout << "[FIX_SCENARIO] random streams read from file "<< stream_file << endl;
	} else {
		streams = RandomStreams(Globals::config.num_scenarios,
		                        Globals::config.search_depth);
	}

	if (FIX_SCENARIO == 2) {
		std::ofstream fout;
		string stream_file = "Streams" + std::to_string(step_counter) + ".txt";
		fout.open(stream_file, std::ios::trunc);
		fout << streams;
		fout.close();
	}

	LookaheadUpperBound* ub = dynamic_cast<LookaheadUpperBound*>(upper_bound_);
	if (ub != NULL) { // Avoid using new streams for LookaheadUpperBound
		static bool initialized = false;
		if (!initialized) {
			lower_bound_->Init(streams);
			upper_bound_->Init(streams);
			initialized = true;
		}
	} else {
		if (FIX_SCENARIO == 1 || Debug_mode) {
			std::ifstream fin;
			string stream_file = "Streams" + std::to_string(step_counter) + ".txt";

			fin.open(stream_file, std::ios::in);
			streams.ImportStream(fin, Globals::config.num_scenarios,
			                     Globals::config.search_depth);
			fin.close();

		} else {
			streams = RandomStreams(Globals::config.num_scenarios,
			                        Globals::config.search_depth);
		}
		if (FIX_SCENARIO == 2) {
			std::ofstream fout;
			string stream_file = "Streams" + std::to_string(step_counter) + ".txt";

			fout.open(stream_file, std::ios::trunc);
			fout << streams;
			fout.close();
		}
		lower_bound_->Init(streams);
		upper_bound_->Init(streams);
	}


	if (use_GPU_) {
		PrepareGPUStreams(streams);
		for (int i = 0; i < particles.size(); i++) {
			particles[i]->scenario_id = i;
		}
	}


	root_ = ConstructTree(particles, streams, lower_bound_, upper_bound_,
	                      model_, history_, Globals::config.time_per_move, &statistics_);
	logi << "[DESPOT::Search] Time for tree construction: "
	     << (get_time_second() - start) << "s" << endl;
	start = get_time_second();
	root_->Free(*model_);

	if (use_GPU_)
		model_->DeleteGPUParticles(MEMORY_MODE(RESET));

	logi << "[DESPOT::Search] Time for freeing particles in search tree: "
	     << (get_time_second() - start) << "s" << endl;


	start = get_time_second();
	ValuedAction astar = OptimalAction(root_);

	delete root_;


	logi << "[DESPOT::Search] Time for deleting tree: "
	     << (get_time_second() - start) << "s" << endl;

	logi << "[DESPOT::Search] Search statistics:" << endl << statistics_
	     << endl;

	assert(use_GPU_ == Globals::config.useGPU);
	if (use_GPU_) {
		PrintGPUData(Num_searches);
		cout << "[DESPOT::Search] Search statistics:" << endl << statistics_
		     << endl;

	} else {
		PrintCPUTime(Num_searches);
		cout << "[DESPOT::Search] Search statistics:" << endl << statistics_
		     << endl;
	}
	Initial_upper.push_back(statistics_.initial_ub);
	Initial_lower.push_back(statistics_.initial_lb);
	Final_upper.push_back(statistics_.final_ub);
	Final_lower.push_back(statistics_.final_lb);

	return astar;
}

double DESPOT::CheckDESPOT(const VNode* vnode, double regularized_value) {
	cout
	        << "--------------------------------------------------------------------------------"
	        << endl;

	const vector<State*>& particles = vnode->particles();
	vector<State*> copy;
	vector<int> copyID;

	for (int i = 0; i < particles.size(); i++) {
		copy.push_back(model_->Copy(particles[i]));
		copyID.push_back(particles[i]->scenario_id);
	}
	VNode* root = new VNode(copy, copyID);

	double pruning_constant = Globals::config.pruning_constant;
	Globals::config.pruning_constant = 0;

	RandomStreams streams = RandomStreams(Globals::config.num_scenarios,
	                                      Globals::config.search_depth);

	streams.position(0);
	InitBounds(root, lower_bound_, upper_bound_, streams, history_);

	double used_time = 0;
	int num_trials = 0, prev_num = 0;
	double pruned_value;
	do {
		double start = clock();
		VNode* cur = Trial(root, streams, lower_bound_, upper_bound_, model_,
		                   history_);
		num_trials++;
		used_time += double(clock() - start) / CLOCKS_PER_SEC;

		start = clock();
		Backup(cur, true);
		used_time += double(clock() - start) / CLOCKS_PER_SEC;

		if (double(num_trials - prev_num) > 0.05 * prev_num) {
			ACT_TYPE pruned_action;
			Globals::config.pruning_constant = pruning_constant;
			VNode* pruned = Prune(root, pruned_action, pruned_value);
			Globals::config.pruning_constant = 0;
			prev_num = num_trials;

			pruned->Free(*model_);
			delete pruned;

			cout << "# trials = " << num_trials << "; target = "
			     << regularized_value << ", current = " << pruned_value
			     << ", l = " << root->lower_bound() << ", u = "
			     << root->upper_bound() << "; time = " << used_time << endl;

			if (pruned_value >= regularized_value) {
				break;
			}
		}
	} while (true);

	cout << "DESPOT: # trials = " << num_trials << "; target = "
	     << regularized_value << ", current = " << pruned_value << ", l = "
	     << root->lower_bound() << ", u = " << root->upper_bound()
	     << "; time = " << used_time << endl;
	Globals::config.pruning_constant = pruning_constant;
	cout
	        << "--------------------------------------------------------------------------------"
	        << endl;

	root->Free(*model_);
	delete root;

	return used_time;
}

double DESPOT::CheckDESPOTSTAR(const VNode* vnode, double regularized_value) {
	cout
	        << "--------------------------------------------------------------------------------"
	        << endl;

	const vector<State*>& particles = vnode->particles();
	vector<State*> copy;
	vector<int> copyIDs;

	for (int i = 0; i < particles.size(); i++) {
		copy.push_back(model_->Copy(particles[i]));
		copyIDs.push_back(particles[i]->scenario_id);
	}
	VNode* root = new VNode(copy, copyIDs);

	RandomStreams streams = RandomStreams(Globals::config.num_scenarios,
	                                      Globals::config.search_depth);
	InitBounds(root, lower_bound_, upper_bound_, streams, history_);

	double used_time = 0;
	int num_trials = 0;
	do {
		double start = clock();
		VNode* cur = Trial(root, streams, lower_bound_, upper_bound_, model_,
		                   history_);
		num_trials++;
		used_time += double(clock() - start) / CLOCKS_PER_SEC;

		start = clock();
		Backup(cur, true);
		used_time += double(clock() - start) / CLOCKS_PER_SEC;
	} while (root->lower_bound() < regularized_value);

	cout << "DESPOT: # trials = " << num_trials << "; target = "
	     << regularized_value << ", current = " << root->lower_bound()
	     << ", l = " << root->lower_bound() << ", u = "
	     << root->upper_bound() << "; time = " << used_time << endl;
	cout
	        << "--------------------------------------------------------------------------------"
	        << endl;

	root->Free(*model_);
	delete root;

	return used_time;
}

VNode* DESPOT::Prune(VNode* vnode, int& pruned_action, double& pruned_value) {
	vector<State*> empty;
	vector<int> emptyID;
	VNode* pruned_v = new VNode(empty, emptyID, vnode->depth(), NULL,
	                            vnode->edge());

	vector<QNode*>& children = vnode->children();
	ACT_TYPE astar = -1;
	double nustar = Globals::NEG_INFTY;
	QNode* qstar = NULL;
	for (int i = 0; i < children.size(); i++) {
		QNode* qnode = children[i];
		double nu;
		QNode* pruned_q = Prune(qnode, nu);

		if (nu > nustar) {
			nustar = nu;
			astar = qnode->edge();

			if (qstar != NULL) {
				delete qstar;
			}

			qstar = pruned_q;
		} else {
			delete pruned_q;
		}
	}

	if (nustar < vnode->default_move().value) {
		nustar = vnode->default_move().value;
		astar = vnode->default_move().action;
		delete qstar;
	} else {
		pruned_v->children().push_back(qstar);
		qstar->parent(pruned_v);
	}

	pruned_v->lower_bound(vnode->lower_bound()); // for debugging
	pruned_v->upper_bound(vnode->upper_bound());

	pruned_action = astar;
	pruned_value = nustar;

	return pruned_v;
}

QNode* DESPOT::Prune(QNode* qnode, double& pruned_value) {
	QNode* pruned_q = new QNode((VNode*) NULL, qnode->edge());
	pruned_value = qnode->step_reward - Globals::config.pruning_constant;
	map<OBS_TYPE, VNode*>& children = qnode->children();
	for (map<OBS_TYPE, VNode*>::iterator it = children.begin();
	        it != children.end(); it++) {
		ACT_TYPE astar;
		double nu;
		VNode* pruned_v = Prune(it->second, astar, nu);
		if (nu == it->second->default_move().value) {
			delete pruned_v;
		} else {
			pruned_q->children()[it->first] = pruned_v;
			pruned_v->parent(pruned_q);
		}
		pruned_value += nu;
	}

	pruned_q->lower_bound(qnode->lower_bound()); // for debugging
	pruned_q->upper_bound(qnode->upper_bound()); // for debugging

	return pruned_q;
}
//for debugging
void DESPOT::OutputWeight(QNode* qnode) {
	double qnode_weight = 0;
	std::vector<double> vnode_weight;
	if (qnode == NULL) {
		logi << "NULL" << endl;
		return;
	} else {
		logi.precision(3);
		logi << "Qnode depth: " << qnode->parent()->depth() + 1
		     << "  action: " << qnode->edge()
		     << "  value: " << qnode->lower_bound() / qnode->Weight()
		     << "  ub: " << qnode->upper_bound() / qnode->Weight();
		logi.precision(3);
		logi << " weight: " << qnode->Weight() << endl;
		map<OBS_TYPE, VNode*>& vnodes = qnode->children();
		for (map<OBS_TYPE, VNode*>::iterator it = vnodes.begin();
		        it != vnodes.end(); it++) {
			if (it->second == NULL) {
				logi << "child vnode is empty" << endl;
				return;
			}
			logi << "Forward->";// << endl;
			OptimalAction2(it->second);
			logi << "Back<-";// << endl;
			qnode_weight += it->second->Weight();
		}
	}

}

//for debugging
void DESPOT::OptimalAction2(VNode* vnode) {
	if (vnode->depth() <= 2) {
		ValuedAction astar(-1, Globals::NEG_INFTY);
		QNode * best_qnode = NULL;
		for (int action = 0; action < vnode->children().size(); action++) {
			QNode* qnode = vnode->Child(action);
			if (qnode->lower_bound() > astar.value) {
				astar = ValuedAction(action, qnode->lower_bound());
				best_qnode = qnode;
			}
		}
		if (vnode->children().size() == 0)
			astar.value = 0;
		if (vnode->default_move().value > astar.value) {
			//if(vnode->children().size()>0)
			logi.precision(3);
			logi << "Vnode depth: " << vnode->depth()
			     << " value lower than default value: " << astar.value << " "
			     << vnode->default_move().value << endl;
			//else
			//	logi <<"[LEAF] ";
			astar = vnode->default_move();
			best_qnode = NULL;
		}

		if (vnode->children().size() == 0) {
			logi << "[LEAF] ";
		}
		logi.precision(3);
		logi << "Vnode depth: " << vnode->depth() << "  weight: " << vnode->Weight()/*<<" #particle:  "<<vnode->particles().size()*/
		     << "  value: " << vnode->lower_bound() / vnode->Weight()
		     << "  ub: " << vnode->upper_bound() / vnode->Weight()
		     << "  astar: " << astar.action
		     << "  astar_value: " << astar.value / vnode->Weight()
		     << endl;

		if (vnode->depth() != 2 && best_qnode)
		{
			logi << "Forward->" ;//<< endl;
			DESPOT::OutputWeight(best_qnode);
			logi << "Back<-" ;//<< endl;
		}
	}
	return;
}

ValuedAction DESPOT::OptimalAction(VNode* vnode) {
	ValuedAction astar(-1, Globals::NEG_INFTY);
	QNode * best_qnode = NULL;

	for (ACT_TYPE action = 0; action < vnode->children().size(); action++) {
		QNode* qnode = vnode->Child(action);

		logi << "Children of root node: action: " << qnode->edge() << "  lowerbound: "
				     << qnode->lower_bound() << endl;


		if (qnode->lower_bound() > astar.value) {
			astar = ValuedAction(action, qnode->lower_bound());
			best_qnode = qnode;		//Debug
		}
	}

	if (vnode->default_move().value > astar.value + 1e-5) {
		logi.precision(5);
		logi << "Searched value worse than default move: " << astar.value << "(act " << astar.action << ")<" << vnode->default_move().value << endl;		//Debug
		astar = vnode->default_move();
		logi << "Execute default move: " << astar.action << endl;		//Debug
		best_qnode = NULL;		//Debug
	}
	else if (astar.action != vnode->default_move().action )
	{
		logi.precision(5);
		logi << "unused default action: " << vnode->default_move().action
		     << "(" << vnode->default_move().value << ")" << endl;		//Debug
	}

	if (vnode->lower_bound() < -200/*false*/) {
		logi.precision(3);
		logi << "Root depth: " << vnode->depth() << "  weight: "
		     << vnode->Weight() << " #particle:  "
		     << vnode->particles().size() << "  lowerbound: "
		     << vnode->lower_bound() << endl;
		logi << "Forward->" << endl;
		DESPOT::OutputWeight(best_qnode); //debug
		logi << "Back<-" << endl;
	}
	return astar;
}

double DESPOT::Gap(VNode* vnode) {
	return (vnode->upper_bound() - vnode->lower_bound());
}

double DESPOT::Gap(Shared_VNode* vnode, bool use_Vloss) {
	return (vnode->upper_bound((bool)use_Vloss) - vnode->lower_bound());
}

double DESPOT::WEU(VNode* vnode) {
	return WEU(vnode, Globals::config.xi);
}
double DESPOT::WEU(Shared_VNode* vnode) {
	return WEU(vnode, Globals::config.xi);
}
// Can pass root as an argument, but will not affect performance much
double DESPOT::WEU(VNode* vnode, double xi) {
	VNode* root = vnode;
	while (root->parent() != NULL) {
		root = root->parent()->parent();
	}

	return Gap(vnode) - xi * vnode->Weight() * Gap(root);
}

double DESPOT::WEU(Shared_VNode* vnode, double xi) {
	VNode* root = vnode;
	while (root->parent() != NULL) {
		root = root->parent()->parent();
	}
	//cout<< "Gap(vnode)=" <<Gap(vnode)<<endl;
	//cout <<"vnode->Weight()="<<vnode->Weight()<<endl;
	//cout <<"Gap(root)="<<Gap(root)<<endl;
	//return Gap((VNode*) vnode) - xi * vnode->Weight() * Gap(root);
	return Gap(vnode, true) - xi * vnode->Weight() * Gap(root);
}

VNode* DESPOT::SelectBestWEUNode(QNode* qnode) {
	double weustar = Globals::NEG_INFTY;
	VNode* vstar = NULL;
	map<OBS_TYPE, VNode*>& children = qnode->children();
	for (map<OBS_TYPE, VNode*>::iterator it = children.begin();
	        it != children.end(); it++) {
		VNode* vnode = it->second;

		double weu;
		if (Globals::config.use_multi_thread_)
			weu = WEU(static_cast<Shared_VNode*>(vnode));
		else
			weu = WEU(vnode);
		if (weu >= weustar) {
			weustar = weu;
			vstar = vnode->vstar;
		}

		if (FIX_SCENARIO == 1) {
			if (!Globals::config.use_multi_thread_)
			{
				cout.precision(4);
				cout << "thread " << 0 << " " << "   Compare children vnode" << " get old node"
				     << " at depth " << vnode->depth()
				     << ": weight=" << vnode->Weight() << ", reward=-"
				     << ", lb=" << vnode->lower_bound() / vnode->Weight() <<
				     ", ub=" << vnode->upper_bound() / vnode->Weight() <<
				     ", uub=" << vnode->utility_upper_bound() / vnode->Weight() <<
				     ", edge=" << vnode->edge() <<
				     ", v_loss=" << 0;
				if (vnode->Weight() == 1.0 / Globals::config.num_scenarios) cout << ", particle_id=" << "-"/*vnode->particles()[0]->scenario_id*/;
				cout << ", WEU=" << WEU(vnode);
				cout << endl;
			}
			else
			{
				float weight = ((VNode*) vnode)->Weight();

				Globals::Global_print_node(this_thread::get_id(), vnode,
						   ((VNode*) vnode)->depth(), 0,
						   ((VNode*) vnode)->lower_bound(),
						   ((VNode*) vnode)->upper_bound(),
						   ((VNode*) vnode)->utility_upper_bound(),
						   static_cast<Shared_VNode*>(vnode)->exploration_bonus,
						   ((VNode*) vnode)->Weight(),
						   ((VNode*) vnode)->edge(),
						   WEU(((VNode*) vnode)),
						   "  Compare children vnode");
			}
		}
		//Globals::Global_print_value(this_thread::get_id(), weu, "weu");
	}

	if (vstar && Globals::config.use_multi_thread_)
	{
		switch (Globals::config.exploration_mode)
		{
		case VIRTUAL_LOSS:
			static_cast<Shared_VNode*>(vstar)->exploration_bonus -= CalExplorationValue(
			            vstar->depth());
			break;
		case UCT:
			static_cast<Shared_VNode*>(vstar)->visit_count_++;
			//CalExplorationValue(static_cast<Shared_VNode*>(vstar));
			static_cast<Shared_VNode*>(vstar)->exploration_bonus -= CalExplorationValue(
			            vstar->depth()) * ((VNode*) vstar)->Weight();
			break;
		}
	}
	//cout<<"trace obs "<<obs_vstar<<endl;
	return vstar;
}

QNode* DESPOT::SelectBestUpperBoundNode(VNode* vnode) {
	int astar = -1;
	double upperstar = Globals::NEG_INFTY;
	for (ACT_TYPE action = 0; action < vnode->children().size(); action++) {
		QNode* qnode = vnode->Child(action);

		if (qnode->upper_bound() > upperstar + 1e-5) {
			upperstar = qnode->upper_bound();
			astar = action;
		}
	}

	assert(astar >= 0);
	return vnode->Child(astar);
}

Shared_QNode* DESPOT::SelectBestUpperBoundNode(Shared_VNode* vnode) {
	int astar = -1;
	double upperstar = Globals::NEG_INFTY;
	for (int action = 0; action < vnode->children().size(); action++) {
		Shared_QNode* qnode =
		    static_cast<Shared_QNode*>(((VNode*) vnode)->Child(action));
		if (qnode && Globals::config.use_multi_thread_ && Globals::config.exploration_mode == UCT)
		{
			CalExplorationValue(qnode);
		}
		if (qnode->upper_bound(true) > upperstar + 1e-5) {
			upperstar = qnode->upper_bound(true);
			astar = action;
		}

		/*if(astar==action)
			Globals::Global_print_node(this_thread::get_id(), qnode,
				((VNode*) vnode)->depth() + 1, ((QNode*) qnode)->step_reward,
				((QNode*) qnode)->lower_bound(),
				((QNode*) qnode)->upper_bound(), qnode->exploration_bonus,
				((QNode*) qnode)->Weight(),
				((QNode*) qnode)->edge(),
				"   (Selected) compare children qnode");
		else
			Globals::Global_print_node(this_thread::get_id(), qnode,
				((VNode*) vnode)->depth() + 1, ((QNode*) qnode)->step_reward,
				((QNode*) qnode)->lower_bound(),
				((QNode*) qnode)->upper_bound(), qnode->exploration_bonus,
				((QNode*) qnode)->Weight(),
				((QNode*) qnode)->edge(),
				"   Compare children qnode");*/
	}
	assert(astar >= 0);

	Shared_QNode* qstar = static_cast<Shared_QNode*>(((VNode*) vnode)->Child(
	                          astar));
	if (Globals::config.exploration_mode == VIRTUAL_LOSS)
		qstar->exploration_bonus -= CalExplorationValue(((VNode*) vnode)->depth() + 1);
	else if (Globals::config.exploration_mode == UCT)
	{
		qstar->visit_count_++;
		CalExplorationValue(qstar);
	}
	//cout<<"trace action "<<astar<<endl;
	return qstar;
}

void DESPOT::Update(VNode* vnode, bool real) {
	if (vnode->IsLeaf()) {
		return;
	}

	double lower = vnode->default_move().value;
	double upper = vnode->default_move().value;
	double utility_upper = Globals::NEG_INFTY;

	for (ACT_TYPE action = 0; action < vnode->children().size(); action++) {
		QNode* qnode = vnode->Child(action);

		lower = max(lower, qnode->lower_bound());
		upper = max(upper, qnode->upper_bound());
		utility_upper = max(utility_upper, qnode->utility_upper_bound());
	}

	if (lower > vnode->lower_bound()) {
		vnode->lower_bound(lower);
	}
	if (upper < vnode->upper_bound()) {
		vnode->upper_bound(upper);
	}
	if (utility_upper < vnode->utility_upper_bound()) {
		vnode->utility_upper_bound(utility_upper);
	}
}
void DESPOT::Update(Shared_VNode* vnode, bool real) {
	lock_guard < mutex > lck(vnode->GetMutex());//lock v_node during updation
	if (((VNode*) vnode)->depth() > 0 && real) {
		if ( Globals::config.exploration_mode == VIRTUAL_LOSS) //release virtual loss
			vnode->exploration_bonus += CalExplorationValue(((VNode*) vnode)->depth());
		else if ( Globals::config.exploration_mode == UCT) //release virtual loss
			vnode->exploration_bonus += CalExplorationValue(((VNode*) vnode)->depth())
			                            * ((VNode*) vnode)->Weight();
	}

	if (((VNode*) vnode)->IsLeaf()) {
		return;
	}
	double lower = ((VNode*) vnode)->default_move().value;
	double upper = ((VNode*) vnode)->default_move().value;
	double utility_upper = Globals::NEG_INFTY;

	for (int action = 0; action < ((VNode*) vnode)->children().size();
	        action++) {
		Shared_QNode* qnode =
		    static_cast<Shared_QNode*>(((VNode*) vnode)->Child(action));

		lower = max(lower, ((QNode*) qnode)->lower_bound());
		upper = max(upper, ((QNode*) qnode)->upper_bound());
		utility_upper = max(utility_upper,
		                    ((QNode*) qnode)->utility_upper_bound());

		if (((VNode*) vnode)->parent() == NULL)	//IS ROOT
		{
			//Globals::Global_print_value(this_thread::get_id(),
			//		((QNode*) qnode)->upper_bound(), "child qnode upper");
			//Globals::Global_print_value(this_thread::get_id(), qnode->GetVirtualLoss(),
			//		"child qnode v_loss");
		}
	}

	if (lower > ((VNode*) vnode)->lower_bound()) {
		((VNode*) vnode)->lower_bound(lower);
	}
	if (upper < ((VNode*) vnode)->upper_bound()) {
		((VNode*) vnode)->upper_bound(upper);
	}
	if (utility_upper < ((VNode*) vnode)->utility_upper_bound()) {
		((VNode*) vnode)->utility_upper_bound(utility_upper);
	}
}
void DESPOT::Update(QNode* qnode, bool real) {

	double lower = qnode->step_reward;
	double upper = qnode->step_reward;
	double utility_upper = qnode->step_reward
	                       + Globals::config.pruning_constant;

	map<OBS_TYPE, VNode*>& children = qnode->children();
	for (map<OBS_TYPE, VNode*>::iterator it = children.begin();
	        it != children.end(); it++) {
		VNode* vnode = it->second;

		lower += vnode->lower_bound();
		upper += vnode->upper_bound();
		utility_upper += vnode->utility_upper_bound();
	}

	if (lower > qnode->lower_bound()) {
		qnode->lower_bound(lower);
	}
	if (upper < qnode->upper_bound()) {
		qnode->upper_bound(upper);
	}
	if (utility_upper < qnode->utility_upper_bound()) {
		qnode->utility_upper_bound(utility_upper);
	}
}
void DESPOT::Update(Shared_QNode* qnode, bool real) {
	lock_guard < mutex > lck(qnode->GetMutex());//lock v_node during updation

	double lower = qnode->step_reward;
	double upper = qnode->step_reward;
	double utility_upper = qnode->step_reward
	                       + Globals::config.pruning_constant;

	int cur_depth = -1;
	map<OBS_TYPE, VNode*>& children = ((QNode*) qnode)->children();
	for (map<OBS_TYPE, VNode*>::iterator it = children.begin();
	        it != children.end(); it++) {
		Shared_VNode* vnode = static_cast<Shared_VNode*>(it->second);

		lower += ((VNode*) vnode)->lower_bound();
		upper += ((VNode*) vnode)->upper_bound();
		utility_upper += ((VNode*) vnode)->utility_upper_bound();
		cur_depth = vnode->depth();
		//if(cur_depth==2)Globals::Global_print_value(this_thread::get_id(),
		//		((VNode*) vnode)->upper_bound(), "child vnode upper");
	}

	if (Globals::config.exploration_mode == VIRTUAL_LOSS && real)
	{
		if (cur_depth >= 0)
			qnode->exploration_bonus += CalExplorationValue(cur_depth);
		else
			qnode->exploration_bonus = 0;
	}

	if (lower > ((QNode*) qnode)->lower_bound()) {
		((QNode*) qnode)->lower_bound(lower);
	}
	if (upper < ((QNode*) qnode)->upper_bound()) {
		((QNode*) qnode)->upper_bound(upper);
	}
	if (utility_upper < ((QNode*) qnode)->utility_upper_bound()) {
		((QNode*) qnode)->utility_upper_bound(utility_upper);
	}
}

void DESPOT::Backup(VNode* vnode, bool real) {
	int iter = 0;
	logd << "- Backup " << vnode << " at depth " << vnode->depth() << endl;
	while (true) {
		logd << " Iter " << iter << " " << vnode << endl;
		string msg = "backup to VNode";
		msg += real ? "(true)" : "(blocker)";
		if (Globals::config.use_multi_thread_) {
			Update(static_cast<Shared_VNode*>(vnode), real);

			if (real)
				Globals::Global_print_node(this_thread::get_id(), vnode,
				   static_cast<Shared_VNode*>(vnode)->depth(), 0,
				   static_cast<Shared_VNode*>(vnode)->lower_bound(),
				   static_cast<Shared_VNode*>(vnode)->upper_bound(false),
				   -1000,
				   static_cast<Shared_VNode*>(vnode)->GetVirtualLoss(),
				   static_cast<Shared_VNode*>(vnode)->Weight(),
				   static_cast<Shared_VNode*>(vnode)->edge(),
				   -1000,
				   msg.c_str());
		} else
		{
			Update(vnode, real);
			if(FIX_SCENARIO==1){
				cout.precision(4);
				cout<<"thread "<<0<<" "<<msg<<" get old node "<<vnode
						<<" at depth "<<vnode->depth()
						<<": weight="<<vnode->Weight()<<", reward="
						<<"--"
						<<", lb="<<vnode->lower_bound()/vnode->Weight()<<
						", ub="<<vnode->upper_bound()/vnode->Weight()<<
						", edge="<<vnode->edge()<<
						", v_loss="<<0;
				cout<<endl;
			}
		}

		QNode* parentq = vnode->parent();
		if (parentq == NULL) {
			break;
		}
		msg = "backup to QNode";
		msg += real ? "(true)" : "(blocker)";
		if (Globals::config.use_multi_thread_) {
			Update(static_cast<Shared_QNode*>(parentq), real);
			if (real)
				Globals::Global_print_node<int>(this_thread::get_id(), parentq,
				                                static_cast<Shared_VNode*>(vnode)->depth(),
				                                static_cast<Shared_QNode*>(parentq)->step_reward,
				                                static_cast<Shared_QNode*>(parentq)->lower_bound(),
				                                static_cast<Shared_QNode*>(parentq)->upper_bound(false),
				                                -1000,
				                                static_cast<Shared_QNode*>(parentq)->GetVirtualLoss(),
				                                static_cast<Shared_QNode*>(parentq)->Weight(),
				                                static_cast<Shared_QNode*>(parentq)->edge(),
				                                -1000,
				                                msg.c_str());
		} else
		{
			Update(parentq, real);
			if(FIX_SCENARIO==1)
			{
				cout.precision(4);
				cout<<"thread "<<0<<" "<<msg<<" get old node "<<parentq
						<<" at depth "<<vnode->depth()
						<<": weight="<<parentq->Weight()<<", reward="
						<<parentq->step_reward/parentq->Weight()
						<<", lb="<<parentq->lower_bound()/parentq->Weight()<<
						", ub="<<parentq->upper_bound()/parentq->Weight()<<
						", edge="<<parentq->edge()<<
						", v_loss="<<0;
				cout<<endl;
			}
		}

		logd << " Updated Q-node to (" << parentq->lower_bound() << ", "
		     << parentq->upper_bound() << ")" << endl;

		vnode = parentq->parent();

		iter++;
	}
	logd << "* Backup complete!" << endl;
}

VNode* DESPOT::FindBlocker(VNode* vnode) {
	VNode* cur = vnode;
	int count = 1;
	while (cur != NULL) {
		if (cur->utility_upper_bound()
		        - count * Globals::config.pruning_constant
		        <= cur->default_move().value) {
			logd << "Return: blocked." << endl;
			break;
		}
		count++;
		if (cur->parent() == NULL) {
			cur = NULL;
		} else {
			cur = cur->parent()->parent();
		}
	}
	return cur;
}

void DESPOT::Expand(VNode* vnode, ScenarioLowerBound* lower_bound,
                    ScenarioUpperBound* upper_bound, const DSPOMDP* model,
                    RandomStreams& streams, History& history) {
	vector<QNode*>& children = vnode->children();
	logd << "- Expanding vnode " << vnode << endl;

	if (use_GPU_ && !vnode->PassGPUThreshold())
	{
		const vector<State*>& particles = vnode->particles();
		if (particles[0] == NULL) // switching point
		{
			GPU_UpdateParticles(vnode, lower_bound, upper_bound, model, streams,
			                    history);
			vnode->ReadBackCPUParticles(model);

			if (Globals::MapThread(this_thread::get_id()) == 0) {
				logd << " Read-back GPU particle at depth " << vnode->depth() << ": " << endl; //Debugging
			}
		}

		Globals::AddExpanded();
	}
	if (!use_GPU_ || !vnode->PassGPUThreshold())
		Globals::Global_print_expand(this_thread::get_id(), vnode, vnode->depth(), vnode->edge());

	for (ACT_TYPE action = 0; action < model->NumActions(); action++) {
		logd << " Action " << action << endl;

		//Create new Q-nodes for each action
		QNode* qnode;

		if (Globals::config.use_multi_thread_)
			qnode = new Shared_QNode(static_cast<Shared_VNode*>(vnode), action);
		else
			qnode = new QNode(vnode, action);

		children.push_back(qnode);
		if (use_GPU_ && vnode->PassGPUThreshold())
			;
		else
		{
			if (Globals::config.use_multi_thread_ && Globals::config.exploration_mode == UCT)
				static_cast<Shared_QNode*>(qnode)->visit_count_ = 1.1;
			Expand(qnode, lower_bound, upper_bound, model, streams, history);
		}

	}

	if (use_GPU_ && vnode->PassGPUThreshold())
		GPU_Expand_Action(vnode, lower_bound, upper_bound, model, streams,
		                  history);
	else {


		HitCount++;
	}

	logd << "* Expansion complete!" << endl;
}

void DESPOT::EnableDebugInfo(QNode* qnode) {
	if (FIX_SCENARIO == 1 || DESPOT::Print_nodes)
		if (qnode->parent()->depth() == 1 && qnode->edge() == 0) {
			CPUDoPrint = true;
		}
}

void DESPOT::EnableDebugInfo(VNode* vnode, QNode* qnode) {
	if (FIX_SCENARIO == 1 || DESPOT::Print_nodes)
		if (vnode->edge() == /*16617342961956436967*/0 && qnode->edge() == 0) {
			CPUDoPrint = true;
		}
}

void DESPOT::DisableDebugInfo() {
	if (FIX_SCENARIO == 1 || DESPOT::Print_nodes)
		if (CPUDoPrint)
			CPUDoPrint = false;
}


void DESPOT::Expand(QNode* qnode, ScenarioLowerBound* lb,
                    ScenarioUpperBound* ub, const DSPOMDP* model, RandomStreams& streams,
                    History& history) {

	VNode* parent = qnode->parent();
	streams.position(parent->depth());
	map<OBS_TYPE, VNode*>& children = qnode->children();
	auto totalstart = Time::now();

	const vector<State*>& particles = parent->particles();

	double step_reward = 0;

	// Partition particles by observation
	map<OBS_TYPE, vector<State*> > partitions;
	OBS_TYPE obs;
	double reward;
	auto start = Time::now();
	int NumParticles = particles.size();


	for (int i = 0; i < NumParticles; i++) {

		State* particle = particles[i];
		logd << " Original: " << *particle  << endl;
		State* copy = model->Copy(particle);
		assert(copy != NULL);

		logd << " Before step: " << *copy << endl;

		EnableDebugInfo(qnode);

		bool terminal = model->Step(*copy, streams.Entry(copy->scenario_id),
		                            qnode->edge(), reward, obs);

		DisableDebugInfo();

		step_reward += reward * copy->weight;


		logd << " After step: " << *copy << " " << (reward * copy->weight)
		     << " " << reward << " " << copy->weight << endl;

		if (!terminal) {
			partitions[obs].push_back(copy);
		} else {
			model->Free(copy);
		}

	}
	step_reward = Globals::Discount(parent->depth()) * step_reward
	              - Globals::config.pruning_constant;	//pruning_constant is used for regularization

	bool doPrint = DESPOT::Print_nodes;

	if (FIX_SCENARIO == 1 || doPrint) {
		Globals::global_mutex.lock();
		cout.precision(10);
		if (qnode->edge() == 0) cout << endl;
		cout << "step reward (d= " << parent->depth() + 1 << " ): "
		     << step_reward / parent->Weight() << endl;
		Globals::global_mutex.unlock();
	}

	double lower_bound = step_reward;
	double upper_bound = step_reward;


	AveRewardTime += Globals::ElapsedTime(start);

	start = Time::now();
	// Create new belief nodes
	for (map<OBS_TYPE, vector<State*> >::iterator it = partitions.begin();
	        it != partitions.end(); it++) {
		OBS_TYPE obs = it->first;
		logd << " Creating node for obs " << obs << endl;
		vector<int> partition_ID;	//empty ID, no use for CPU codes
		VNode* vnode;
		if (Globals::config.use_multi_thread_)
		{
			vnode = new Shared_VNode(partitions[obs], partition_ID,
			                         parent->depth() + 1, static_cast<Shared_QNode*>(qnode),
			                         obs);
			if (Globals::config.exploration_mode == UCT)
				static_cast<Shared_VNode*>(vnode)->visit_count_ = 1.1;
		}
		else
			vnode = new VNode(partitions[obs], partition_ID,
			                  parent->depth() + 1, qnode, obs);
		logd << " New node created!" << endl;
		children[obs] = vnode;
		auto start1 = Time::now();

		history.Add(qnode->edge(), obs);

		EnableDebugInfo(vnode, qnode);

		InitBounds(vnode, lb, ub, streams, history);

		DisableDebugInfo();

		history.RemoveLast();
		logd << " New node's bounds: (" << vnode->lower_bound() << ", "
		     << vnode->upper_bound() << ")" << endl;

		if (FIX_SCENARIO == 1 || doPrint) {
			Globals::global_mutex.lock();
			cout.precision(10);
			cout << " [CPU Vnode] New node's bounds: (d= " << vnode->depth()
			     << " ,obs=" << obs << " ,lb= "
			     << vnode->lower_bound() / vnode->Weight() << " ,ub= "
			     << vnode->upper_bound() / vnode->Weight() << " ,uub= "
			     << vnode->utility_upper_bound() / vnode->Weight()
			     << " ,weight= " << vnode->Weight() << " )" ;//<< endl;
			if (vnode->Weight() == 1.0 / Globals::config.num_scenarios) cout << ", particle_id=" << "-"/*vnode->particles()[0]->scenario_id*/;
			cout << ", WEU=" << WEU(vnode);
			cout << ", parent_obs=" << vnode->parent()->parent()->edge();
			cout  << endl;

			cout << " [Vnode] New node's particles: ";
			for (int i=0; i<vnode->particles().size();i++)
				cout<< vnode->particles()[i]->scenario_id<<" ";
			cout<<endl;
			Globals::global_mutex.unlock();
		}

		lower_bound += vnode->lower_bound();
		upper_bound += vnode->upper_bound();
		InitBoundTime += Globals::ElapsedTime(start1);
	}

	MakeObsNodeTime += Globals::ElapsedTime(start);
	qnode->Weight();//just to initialize the weight

	qnode->step_reward = step_reward;
	qnode->lower_bound(lower_bound);
	qnode->upper_bound(upper_bound);
	qnode->utility_upper_bound(upper_bound + Globals::config.pruning_constant);
	qnode->Weight();
	qnode->default_value = lower_bound; // for debugging
	if (FIX_SCENARIO == 1 || doPrint) {
		Globals::global_mutex.lock();
		cout.precision(10);
		cout << " [CPU Qnode] New qnode's bounds: (d= " << parent->depth() + 1
		     << " ,action=" << qnode->edge() << ", lb= "
		     << qnode->lower_bound() / qnode->Weight() << " ,ub= "
		     << qnode->upper_bound() / qnode->Weight() << " ,uub= "
		     << qnode->utility_upper_bound() / qnode->Weight()
		     << " ,weight= " << qnode->Weight() << " )" << endl;
		Globals::global_mutex.unlock();
	}

	TotalExpansionTime += Globals::ElapsedTime(totalstart);
}
void DESPOT::PrintCPUTime(int num_searches) {
	cout.precision(5);
	cout << "ExpansionCount (total/per-search)=" << HitCount << "/"
	     << HitCount / num_searches << endl;
	cout.precision(3);
	cout << "TotalExpansionTime=" << TotalExpansionTime / num_searches << "/"
	     << TotalExpansionTime / HitCount << endl;
	cout << "AveRewardTime=" << AveRewardTime / num_searches << "/"
	     << AveRewardTime / HitCount << "/"
	     << AveRewardTime / TotalExpansionTime * 100 << "%" << endl;
	cout << "CopyParticleTime=" << ParticleCopyTime / num_searches << "/"
	     << ParticleCopyTime / HitCount << "/"
	     << ParticleCopyTime / TotalExpansionTime * 100 << "%" << endl;

	cout << "InitBoundTime=" << InitBoundTime / num_searches << "/"
	     << InitBoundTime / HitCount << "/"
	     << InitBoundTime / TotalExpansionTime * 100 << "%" << endl;
	cout << "MakeObsNodeTime="
	     << (MakeObsNodeTime - InitBoundTime) / num_searches << "/"
	     << (MakeObsNodeTime - InitBoundTime) / HitCount << "/"
	     << (MakeObsNodeTime - InitBoundTime) / TotalExpansionTime * 100
	     << "%" << endl;
}
ValuedAction DESPOT::Evaluate(VNode* root, vector<State*>& particles,
                              RandomStreams& streams, POMCPPrior* prior, const DSPOMDP* model) {
	double value = 0;

	for (int i = 0; i < particles.size(); i++) {
		particles[i]->scenario_id = i;
	}

	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
		VNode* cur = root;
		State* copy = model->Copy(particle);
		double discount = 1.0;
		double val = 0;
		int steps = 0;

		while (!streams.Exhausted()) {
			ACT_TYPE action =
			    (cur != NULL) ?
			    OptimalAction(cur).action : prior->GetAction(*copy);

			assert(action != -1);

			double reward;
			OBS_TYPE obs;
			bool terminal = model->Step(*copy, streams.Entry(copy->scenario_id),
			                            action, reward, obs);

			val += discount * reward;
			discount *= Globals::Discount();

			if (!terminal) {
				prior->Add(action, obs);
				streams.Advance();
				steps++;

				if (cur != NULL && !cur->IsLeaf()) {
					QNode* qnode = cur->Child(action);
					map<OBS_TYPE, VNode*>& vnodes = qnode->children();
					cur = vnodes.find(obs) != vnodes.end() ? vnodes[obs] : NULL;
				}
			} else {
				break;
			}
		}

		for (int i = 0; i < steps; i++) {
			streams.Back();
			prior->PopLast();
		}

		model->Free(copy);

		value += val;
	}

	return ValuedAction(OptimalAction(root).action, value / particles.size());
}

void DESPOT::belief(Belief* b) {
	logi << "[DESPOT::belief] Start: Set initial belief." << endl;
	belief_ = b;
	history_.Truncate(0);

	//lower_bound_->belief(b); // needed for POMCPScenarioLowerBound
	logi << "[DESPOT::belief] End: Set initial belief." << endl;
}

void DESPOT::BeliefUpdate(ACT_TYPE action, OBS_TYPE obs) {
	double start = get_time_second();

	belief_->Update(action, obs);
	history_.Add(action, obs);
	//lower_bound_->belief(belief_);

	logi << "[Solver::Update] Updated belief, history and root with action "
	     << action << ", observation " << obs << " in "
	     << (get_time_second() - start) << "s" << endl;
}

double DESPOT::AverageInitLower() const {
	double sum = 0;
	for (int i = 0; i < Initial_lower.size(); i++) {
		double bound = Initial_lower[i];
		sum += bound;
	}
	return Initial_lower.size() > 0 ? (sum / Initial_lower.size()) : 0.0;
}

double DESPOT::StderrInitLower() const {
	double sum = 0, sum2 = 0;
	for (int i = 0; i < Initial_lower.size(); i++) {
		double bound = Initial_lower[i];
		sum += bound;
		sum2 += bound * bound;
	}
	int n = Initial_lower.size();
	return n > 0 ? sqrt(sum2 / n / n - sum * sum / n / n / n) : 0.0;
}
double DESPOT::AverageFinalLower() const {
	double sum = 0;
	for (int i = 0; i < Final_lower.size(); i++) {
		double bound = Final_lower[i];
		sum += bound;
	}
	return Final_lower.size() > 0 ? (sum / Final_lower.size()) : 0.0;
}

double DESPOT::StderrFinalLower() const {
	double sum = 0, sum2 = 0;
	for (int i = 0; i < Final_lower.size(); i++) {
		double bound = Final_lower[i];
		sum += bound;
		sum2 += bound * bound;
	}
	int n = Final_lower.size();
	return n > 0 ? sqrt(sum2 / n / n - sum * sum / n / n / n) : 0.0;
}

double DESPOT::AverageInitUpper() const {
	double sum = 0;
	for (int i = 0; i < Initial_upper.size(); i++) {
		double bound = Initial_upper[i];
		sum += bound;
	}
	return Initial_upper.size() > 0 ? (sum / Initial_upper.size()) : 0.0;
}
double DESPOT::StderrInitUpper() const {
	double sum = 0, sum2 = 0;
	for (int i = 0; i < Initial_upper.size(); i++) {
		double bound = Initial_upper[i];
		sum += bound;
		sum2 += bound * bound;
	}
	int n = Initial_upper.size();
	return n > 0 ? sqrt(sum2 / n / n - sum * sum / n / n / n) : 0.0;
}
double DESPOT::AverageFinalUpper() const {
	double sum = 0;
	for (int i = 0; i < Final_upper.size(); i++) {
		double bound = Final_upper[i];
		sum += bound;
	}
	return Final_upper.size() > 0 ? (sum / Final_upper.size()) : 0.0;
}
double DESPOT::StderrFinalUpper() const {
	double sum = 0, sum2 = 0;
	for (int i = 0; i < Final_upper.size(); i++) {
		double bound = Final_upper[i];
		sum += bound;
		sum2 += bound * bound;
	}
	int n = Final_upper.size();
	return n > 0 ? sqrt(sum2 / n / n - sum * sum / n / n / n) : 0.0;
}

void DESPOT::PrintStatisticResult() {
	ios::fmtflags old_settings = cout.flags();

	//int Width=7;
	int Prec = 3;
	cout.precision(Prec);
	cout << "Average initial lower bound (stderr) = " << AverageInitLower()
	     << " (" << StderrInitLower() << ")" << endl;
	cout << "Average initial upper bound (stderr) = " << AverageInitUpper()
	     << " (" << StderrInitUpper() << ")" << endl;
	cout << "Average final lower bound (stderr) = " << AverageFinalLower()
	     << " (" << StderrFinalLower() << ")" << endl;
	cout << "Average final upper bound (stderr) = " << AverageFinalUpper()
	     << " (" << StderrFinalUpper() << ")" << endl;
	cout.flags(old_settings);
}
} // namespace despot
