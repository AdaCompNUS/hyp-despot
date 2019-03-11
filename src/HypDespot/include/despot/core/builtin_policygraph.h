/*
 * builtin_policygraph.h
 *
 *  Created on: 9 Sep, 2018
 *      Author: panpan
 */

#ifndef BUILTIN_POLICYGRAPH_H_
#define BUILTIN_POLICYGRAPH_H_

#include <despot/interface/policy_graph.h>

namespace despot {

/* =============================================================================
 * RandomPolicyGraph class
 * =============================================================================*/

class RandomPolicyGraph: public PolicyGraph {
private:
	std::vector<double> action_probs_;

public:
	RandomPolicyGraph(const DSPOMDP* model, ParticleLowerBound* ParticleLowerBound,
		Belief* belief = NULL);
	virtual ~RandomPolicyGraph();
	virtual void ConstructGraph(int size, int branch);
};

}// namespace despot




#endif /* BUILTIN_POLICYGRAPH_H_ */
