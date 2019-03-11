#ifndef CONFIG_H
#define CONFIG_H

#include <string>

namespace despot {

struct Config {
	int search_depth;
	double discount;
	unsigned int root_seed;
	double time_per_move;  // CPU time available to construct the search tree
	int num_scenarios;
	double pruning_constant;
	double xi; // xi * gap(root) is the target uncertainty at the root.
	int sim_len; // Number of steps to run the simulation for.
  std::string default_action;
	int max_policy_sim_len; // Maximum number of steps for simulating the default policy
	double noise;
	bool silence;
	bool useGPU;
	std::string rollout_type;
	int GPUid;
	bool use_multi_thread_;
	int NUM_THREADS;
	int exploration_mode;
	double exploration_constant;
	double exploration_constant_o;
	bool experiment_mode;


	Config() :
		search_depth(90),
		discount(0.95),
		root_seed(42),
		time_per_move(1),
		num_scenarios(500),
		pruning_constant(0),
		xi(0.95),
		sim_len(90),
		default_action(""),
		max_policy_sim_len(90),
		noise(0.1),
		silence(false),
	    useGPU(false),
	    rollout_type("BLIND"),
	    GPUid(0),
	    use_multi_thread_(false),
	    NUM_THREADS(0),
	    experiment_mode(false),
	    exploration_mode(0),
	    exploration_constant(0.3),
	    exploration_constant_o(0.3)
	{
		rollout_type = "INDEPENDENT";
	}
};

} // namespace despot

#endif
