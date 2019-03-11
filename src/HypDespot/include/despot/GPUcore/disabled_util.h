#ifndef HASH_UTIL_H
#define HASH_UTIL_H

#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <math.h>
#include <chrono>
#include <locale>

using namespace std;
using namespace chrono;

// NOTE: disabled C++11 feature
// Functions for hashing data structs
namespace std {
	template<class T>
	inline void hash_combine(size_t& seed, const T& v) {
		std::hash<T> hasher;
		seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
	}

	template<typename S, typename T>
	struct hash<pair<S, T>> {
		inline size_t operator()(const pair<S, T>& v) const {
			size_t seed = 0;
			::hash_combine(seed, v.first);
			::hash_combine(seed, v.second);
			return seed;
		}
	};

	template<typename T>
	struct hash<vector<T>> {
		inline size_t operator()(const vector<T>& v) const {
			size_t seed = 0;
			for (const T& ele : v) {
				::hash_combine(seed, ele);
			}
			return seed;
		}
	};
}

// NOTE: disabled C++11 feature
template<typename T>
void write(ostringstream& os, T t) {
	os << t;
}

template<typename T, typename ... Args>
void write(ostringstream& os, T t, Args ... args) {
	os << t;
	write(os, args...);
}

template<typename T, typename ... Args>
string concat(T t, Args ... args) {
	ostringstream os;
	write(os, t, args...);
	return os.str();
}

template<typename T>
string concat(vector<T> v) {
	ostringstream os;
	for (int i = 0; i < v.size(); i++) {
		os << v[i];
		os << " ";
	}
	return os.str();
}

#endif