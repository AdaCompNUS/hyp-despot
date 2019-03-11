/*
 * policy_graph.h
 *
 *  Created on: 9 Sep, 2018
 *      Author: panpan
 */

#ifndef POLICY_GRAPH_H_
#define POLICY_GRAPH_H_


#include <despot/random_streams.h>
#include <despot/interface/lower_bound.h>
#include <despot/util/random.h>
#include <despot/core/history.h>

#include <string.h>
#include <queue>
#include <vector>
#include <stdlib.h>
#include <despot/core/globals.h>
#include <despot/interface/pomdp.h>

namespace despot {

/* =============================================================================
 * PolicyGraph class
 * =============================================================================*/


class PolicyGraph: public ScenarioLowerBound {
protected:

	mutable int initial_depth_;
	ParticleLowerBound* particle_lower_bound_;


public:
	mutable int graph_size_;
	mutable int num_edges_per_node_;
	std::vector<int> action_nodes_;
	mutable std::map<OBS_TYPE, std::vector<int> > obs_edges_;

	mutable int current_node_;
	mutable int entry_node_;

public:
	PolicyGraph(const DSPOMDP* model, ParticleLowerBound* particle_lower_bound,
		Belief* belief = NULL);
	virtual ~PolicyGraph();

	void Reset();

	ParticleLowerBound* particle_lower_bound() const;

	virtual void ConstructGraph(int size, int branch)=0;
	void ClearGraph();

	void SetEntry(int node)
	{entry_node_=node;}

	virtual ValuedAction Value(const std::vector<State*>& particles, RandomStreams& streams,
		History& history) const;

	virtual void ExportGraph(std::ostream& fout);
	virtual void ImportGraph(std::ifstream& fin, int size, int branch);

public:
	static bool Load_Graph;
};

}// namespace despot



#endif /* POLICY_GRAPH_H_ */
