/*
 * policy_graph.cpp
 *
 *  Created on: 9 Sep, 2018
 *      Author:panpan
 */


#include <despot/interface/policy_graph.h>
#include <despot/core/builtin_policygraph.h>

#include <despot/interface/pomdp.h>
#include <unistd.h>

using namespace std;

namespace despot {
/* =============================================================================
 * PolicyGraph class
 * =============================================================================*/
bool PolicyGraph::Load_Graph=false;

PolicyGraph::PolicyGraph(const DSPOMDP* model, ParticleLowerBound* particle_lower_bound,
		Belief* belief) :
	ScenarioLowerBound(model),
	particle_lower_bound_(particle_lower_bound) {
	assert(particle_lower_bound_ != NULL);
	graph_size_=0;
	num_edges_per_node_=0;
	initial_depth_=0;
	current_node_=0;
	entry_node_=0;
}

PolicyGraph::~PolicyGraph() {
}

void PolicyGraph::Reset() {
}

ParticleLowerBound* PolicyGraph::particle_lower_bound() const {
	return particle_lower_bound_;
}


ValuedAction PolicyGraph::Value(const vector<State*>& particles,
	RandomStreams& streams, History& history) const {
	vector<State*> copy;
	for (int i = 0; i < particles.size(); i++)
		copy.push_back(model_->Copy(particles[i]));

	initial_depth_ = history.Size();
	int MaxDepth=Globals::config.max_policy_sim_len+initial_depth_;
	int depth;
	if(FIX_SCENARIO)
		entry_node_=0;/*Debug*/
	else
		entry_node_=Random::RANDOM.NextInt(graph_size_);
	int Action_decision=action_nodes_[entry_node_];
	double TotalValue=0;
	int init_pos=streams.position();

	//cout<<" [Vnode] raw lb for "<<copy.size()<<" particles: ";
	for (int i = 0; i < copy.size(); i++)
	{
		current_node_=entry_node_;

		streams.position(init_pos);
		State* particle = copy[i];
		vector<State*> local_particles;
		local_particles.push_back(particle);
		bool terminal=false;
		double value = 0;

		for(depth=initial_depth_;(depth<MaxDepth && !streams.Exhausted());depth++)
		{
			int action = action_nodes_[current_node_];

			OBS_TYPE obs;
			double reward;
			terminal = model_->Step(*particle,
				streams.Entry(particle->scenario_id), action, reward, obs);

			value += reward * particle->weight * Globals::Discount(depth-initial_depth_);

			streams.Advance();

			if(terminal)
				break;
			else
			{
				//vector<int> link= obs_edges_[obs];
				current_node_=obs_edges_[obs][current_node_];
				if(current_node_>10000)
					cout<<"error"<<endl;
			}
		}

		if(!terminal)
		{
			value += Globals::Discount(depth-initial_depth_/*+1*/) * particle_lower_bound_->Value(local_particles).value;
		}
		//cout.precision(3);
		//cout<< value<<" ";
		TotalValue+=value;
	}
	//cout<<endl;
	for (int i = 0; i < copy.size(); i++)
		model_->Free(copy[i]);

	return ValuedAction(Action_decision, TotalValue);
}

void PolicyGraph::ClearGraph()
{
	action_nodes_.resize(0);
	for(int i=0;i<num_edges_per_node_;i++)
		obs_edges_[i].resize(0);
	graph_size_=0;	num_edges_per_node_=0;
	current_node_=0;
	initial_depth_=0;
}

void PolicyGraph::ExportGraph(std::ostream& fout)
{
	cout<<"Export generated graph"<<endl;

	fout<<"action_nodes ";
	for (int j = 0; j < graph_size_; j++)
	{
		fout<<action_nodes_[j]<<" ";
	}
	fout<<endl;

	for (int i = 0; i < num_edges_per_node_; i++)
	{
		fout<<"obs_edges ";
		for (int j = 0; j < graph_size_; j++)
		{
			fout<<obs_edges_[(OBS_TYPE)i][j]<<" ";
		}
		fout<<endl;
	}
}

void PolicyGraph::ImportGraph(std::ifstream& in, int size, int branch)
{
	cout<<"Import external graph"<<endl;

	graph_size_=size;
	num_edges_per_node_=branch;
	action_nodes_.resize(size);
	for (int i = 0; i < num_edges_per_node_; i++)
	{
		for (int j = 0; j < graph_size_; j++)
		{
			obs_edges_[(OBS_TYPE)i].push_back(-1);
		}
	}

	if (in.good())
	{
		string str;
		getline(in, str);
		istringstream ss(str);
		string dummy;
		ss>>dummy;//throw headers
		for (int j = 0; j < graph_size_; j++)
		{
			ss >> action_nodes_[j];
		}
		int obs=0;
		while(getline(in, str))
		{
			if(!str.empty() && obs<num_edges_per_node_)
			{
				istringstream ss(str);
				string dummy;
				ss>>dummy;//throw headers
				int num; int pos=0;
				while(ss >> num)
				{
					if(pos>=graph_size_)
					{
						pos=0;
						cout<<"Import graph error: pos>=graph_size_!"<<endl;
					}
					obs_edges_[(OBS_TYPE)obs][pos]=num;
					pos++;
				}
				obs++;
			}
		}
	}
	else
	{
		cout<<"Empty default policy graph file!"<<endl;
		exit(-1);
	}
	current_node_=0;

	//ExportGraph(cout);
}

} // namespace despot


