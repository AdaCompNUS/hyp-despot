/*
 * builtin_policygraph.cpp
 *
 *  Created on: 9 Sep, 2018
 *      Author: panpan
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
bool Load_Graph=false;

RandomPolicyGraph::RandomPolicyGraph(const DSPOMDP* model, ParticleLowerBound* bound,
	Belief* belief) :
	PolicyGraph(model, bound, belief) {
}
RandomPolicyGraph::~RandomPolicyGraph()
{
	ClearGraph();
}
void RandomPolicyGraph::ConstructGraph(int size, int branch)
{
	if(FIX_SCENARIO==1 || Load_Graph==true)
	{
		ifstream fin;fin.open("Graph.txt", ios::in);

		if(!fin.is_open())
		{
			throw;
			exit(-1);
		}
		ImportGraph(fin,size,branch);
		fin.close();
	}
	else
	{
		graph_size_=size;
		num_edges_per_node_=branch;
		action_nodes_.resize(size);

		//Set random actions for nodes
		for (int i = 0; i < graph_size_; i++)
		{
			action_nodes_[i]=Random::RANDOM.NextInt(model_->NumActions());
		}
		//Link to random nodes for edges
		for (int i = 0; i < num_edges_per_node_; i++)
		{
			for (int j = 0; j < graph_size_; j++)
			{
				int rand_next_node=Random::RANDOM.NextInt(graph_size_);
				obs_edges_[(OBS_TYPE)i].push_back(rand_next_node);
			}
		}
		current_node_=0;
	}

	if(FIX_SCENARIO==2 && !Load_Graph)
	{
		ofstream fout;fout.open("Graph.txt", ios::trunc);
		ExportGraph(fout);
		fout.close();
	}

}

} // namespace despot


