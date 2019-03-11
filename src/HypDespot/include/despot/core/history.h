#ifndef HISTORY_H
#define HISTORY_H

#include <vector>
#include <despot/util/util.h>
#include <despot/core/globals.h>

namespace despot {

/**
 * Action-observation history.
 */
class History {
public:
	void Add(ACT_TYPE action, OBS_TYPE obs) {
		actions_.push_back(action);
		observations_.push_back(obs);
	}

	void RemoveLast() {
		actions_.pop_back();
		observations_.pop_back();
	}

	ACT_TYPE Action(int t) const {
		return actions_[t];
	}

	const ACT_TYPE* Action() const {
		return actions_.data();
	}

	OBS_TYPE Observation(int t) const {
		return observations_[t];
	}

	const OBS_TYPE* Observation() const {
		return observations_.data();
	}

	size_t Size() const {
		return actions_.size();
	}

	void Truncate(int d) {
		actions_.resize(d);
		observations_.resize(d);
	}

	ACT_TYPE LastAction() const {
		return actions_.back();
	}

	OBS_TYPE LastObservation() const {
		return observations_.back();
	}

	History Suffix(int s) const {
		History history;
		for (int i = s; i < Size(); i++)
			history.Add(Action(i), Observation(i));
		return history;
	}

	friend std::ostream& operator<<(std::ostream& os, const History& history) {
		for (int i = 0; i < history.Size(); i++)
			os << "(" << history.Action(i) << ", " << history.Observation(i)
				<< ") ";
		return os;
	}

	//History(const History& src);
	History(){
		;
	}
private:
  std::vector<ACT_TYPE> actions_;
	std::vector<OBS_TYPE> observations_;
};


class State;
/**
 * Action-observation history. (used for POMDPLite solver)
 */
class ActionStateHistory {
public:
	void Add(ACT_TYPE action, const State* state) {
		actions_.push_back(action);
		states_.push_back(state);
	}

	void RemoveLast() {
		actions_.pop_back();
		states_.pop_back();
	}

	ACT_TYPE Action(int t) const {
		return actions_[t];
	}

	const State* state(int t) const {
		return states_[t];
	}

	const std::vector<const State*>& states() const {
		return states_;
	}

	size_t Size() const {
		return actions_.size();
	}

	void Truncate(int d) {
		actions_.resize(d);
		states_.resize(d);
	}

	ACT_TYPE LastAction() const {
		return actions_.back();
	}

	const State* LastState() const {
		return states_.back();
	}

	ActionStateHistory Suffix(int s) const {
		ActionStateHistory history;
		for (int i = s; i < Size(); i++)
			history.Add(Action(i), state(i));
		return history;
	}


private:
  std::vector<ACT_TYPE> actions_;
  std::vector<const State*> states_;
};

class VariableActionStateHistory{
public:
	void Add(ACT_TYPE action, State* state) {
		actions_.push_back(action);
		states_.push_back(state);
	}

	void RemoveLast() {
		actions_.pop_back();
		states_.pop_back();
	}

	ACT_TYPE Action(int t) const {
		return actions_[t];
	}

	State* state(int t) const {
		return states_[t];
	}

	std::vector<State*>& states() {
		return states_;
	}

	size_t Size() const {
		return actions_.size();
	}

	void Truncate(int d) {
		actions_.resize(d);
		states_.resize(d);
	}

	ACT_TYPE LastAction() const {
		return actions_.back();
	}

	State* LastState() const {
		return states_.back();
	}

	ActionStateHistory Suffix(int s) const {
		ActionStateHistory history;
		for (int i = s; i < Size(); i++)
			history.Add(Action(i), state(i));
		return history;
	}


private:
  std::vector<ACT_TYPE> actions_;
  std::vector<State*> states_;
};

} // namespace despot

namespace std {
/*
// NOTE: disabled C++11 feature
template<>
struct hash<History> {
	size_t operator()(const History& h) const {
		size_t seed = 0;
		for (int i = 0; i < h.Size(); i++) {
			hash_combine(seed, h.Action(i));
			hash_combine(seed, h.Observation(i));
		}
		return seed;
	}
};
*/

template<>
struct less<despot::History> {
	bool operator()(const despot::History& h1, const despot::History& h2) const {
		int N = h1.Size() < h2.Size() ? h1.Size() : h2.Size();

		for (int i = 0; i < N; i++) {
			if (h1.Action(i) < h2.Action(i))
				return true;
			if (h1.Action(i) > h2.Action(i))
				return false;
			if (h1.Observation(i) < h2.Observation(i))
				return true;
			if (h1.Observation(i) > h2.Observation(i))
				return false;
		}
		return false;
	}
};

}
#endif
