/*
 * shared_solver.h
 *
 *  Created on: 20 Jul, 2017
 *      Author: panpan
 */

#ifndef SHARED_SOLVER_H_
#define SHARED_SOLVER_H_

#include <despot/GPUcore/thread_globals.h>
namespace despot {

/* =============================================================================
 * SearchStatistics class (with mutex protection for multi-threading)
 * =============================================================================*/

struct Shared_SearchStatistics: public SearchStatistics {
	std::mutex _mutex;
public:
	Shared_SearchStatistics():SearchStatistics(){;}

	Shared_SearchStatistics& operator=(const Shared_SearchStatistics& other)
	{
		initial_lb=other.initial_lb;
		initial_ub=other.initial_ub;
		final_lb=other.final_lb;
		final_ub=other.final_ub;
		time_search=other.time_search;
		time_path=other.time_path;
		time_backup=other.time_backup;
		time_node_expansion=other.time_node_expansion;
		num_policy_nodes=other.num_policy_nodes;
		num_tree_nodes=other.num_tree_nodes;
		num_expanded_nodes=other.num_expanded_nodes;
		num_tree_particles=other.num_tree_particles;
		num_particles_before_search=other.num_particles_before_search;
		num_particles_after_search=other.num_particles_after_search;
		num_trials=other.num_trials;
		longest_trial_length=other.longest_trial_length;
		return *this;
	}
	int Get_longest_trial_len()
	{
		std::lock_guard<std::mutex> lck(_mutex);
		return longest_trial_length;
	}
	void Update_longest_trial_len(int l)
	{
		std::lock_guard<std::mutex> lck(_mutex);
		longest_trial_length=max(longest_trial_length,l);
	}
	void Set_longest_trial_len(int l)
	{
		std::lock_guard<std::mutex> lck(_mutex);
		longest_trial_length=l;
	}
	void Add_time_node_expansion(double value)
	{
		std::lock_guard<std::mutex> lck(_mutex);
		time_node_expansion+=value;
	}
	void Inc_num_expanded_nodes()
	{
		std::lock_guard<std::mutex> lck(_mutex);
		num_expanded_nodes++;
	}

	void Add_num_tree_particles(int num)
	{
		std::lock_guard<std::mutex> lck(_mutex);
		num_tree_particles+=num;
	}
	void Add_time_path(float value)
	{
		std::lock_guard<std::mutex> lck(_mutex);
		time_path+=value;
	}

	void Add_time_backup(float value)
	{
		std::lock_guard<std::mutex> lck(_mutex);
		time_backup+=value;
	}
};

}
#endif /* SHARED_SOLVER_H_ */
