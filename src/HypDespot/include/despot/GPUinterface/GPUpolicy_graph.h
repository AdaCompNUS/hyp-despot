/*
 * GPUpolicy_graph.h
 *
 *  Created on: 9 Sep, 2018
 *      Author: panpan
 */

#ifndef GPUPOLICY_GRAPH_H_
#define GPUPOLICY_GRAPH_H_

#include <despot/core/globals.h>
#include <vector>

#include <despot/GPUrandom_streams.h>
#include <despot/GPUinterface/GPUlower_bound.h>
#include <despot/GPUutil/GPUrandom.h>
#include <despot/GPUcore/GPUhistory.h>

#include <string.h>
#include <queue>
#include <vector>
#include <stdlib.h>
#include <despot/GPUcore/GPUglobals.h>
#include <despot/GPUinterface/GPUpomdp.h>
#include <despot/GPUinterface/GPUdefault_policy.h>

#include <despot/GPUcore/CudaInclude.h>

namespace despot {

class Dvc_State;
class Dvc_StateIndexer;
class Dvc_StatePolicy;
class Dvc_DSPOMDP;
class Dvc_MMAPInferencer;


/* =============================================================================
 * Dvc_PolicyGraph class
 * =============================================================================*/

class Dvc_PolicyGraph/*: public Dvc_ScenarioLowerBound */{
public:

	DEVICE static Dvc_ValuedAction Value(
		Dvc_State* particles,
		Dvc_RandomStreams& streams,
		Dvc_History& history, int start_node);

	DEVICE static int Edge(int action, OBS_TYPE obs);

};

/*These values need to be passed from the CPU side*/
DEVICE extern int graph_size_;
DEVICE extern int num_edges_per_node_;
DEVICE extern int* action_nodes_;
DEVICE extern int* obs_edges_;/*A flattened pointer*/


DEVICE extern int (*DvcChooseEdge_)( int, OBS_TYPE);

} // namespace despot


#endif /* GPUPOLICY_GRAPH_H_ */
