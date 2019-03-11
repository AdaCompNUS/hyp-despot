#ifndef GPURANDOM_STREAMS_H
#define GPURANDOM_STREAMS_H

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <despot/util/random.h>

#include <despot/GPUcore/CudaInclude.h>

#include <despot/random_streams.h>

namespace despot {

/**
 * A Dvc_RandomStreams object represents multiple random number sequences, where each
 * entry is independently and identically drawn from [0, 1].
 */
class Dvc_RandomStreams {
public:
	int num_streams_; int length_;
  double** streams_; // streams_[i] is associated with i-th particle
	mutable int position_;

	bool externel_streams;
public:
	/**
	 * Constructs multiple random sequences of the same length.
	 *
	 * @param num_streams number of sequences
	 * @param length sequence length
	 */
	DEVICE Dvc_RandomStreams(int num_streams, int length, double** stream);
	DEVICE Dvc_RandomStreams(int num_streams, int length, double** stream, int pos);
	DEVICE Dvc_RandomStreams()
	{
		num_streams_=0; length_=0;
		streams_=0; // streams_[i] is associated with i-th particle
		position_=0;
		externel_streams=false;
	}

	DEVICE ~Dvc_RandomStreams();

	/**
	 * Returns the number of sequences.
	 */
	DEVICE int NumStreams() const;

	/**
	 * Returns the length of the sequences.
	 */
	DEVICE int Length() const;

	DEVICE void Advance() const;
	DEVICE void Back() const;

	DEVICE void position(int value) const;
	DEVICE int position() const;

	DEVICE bool Exhausted() const;

	DEVICE double Entry(int stream) const;
	DEVICE double Entry(int stream, int position) const;

	//DEVICE friend std::ostream& operator<<(std::ostream& os, const Dvc_RandomStreams& stream);

	HOST void assign(RandomStreams* host_stream);
	HOST void Alloc(int num_streams, int length);
	HOST static void CopyToGPU(Dvc_RandomStreams* Dvc, const RandomStreams* Hst, void* cudaStream=NULL);
	HOST static void ShareStreamsAmongThreads(Dvc_RandomStreams** streams);
	HOST static void Init(Dvc_RandomStreams* Dvc, int num_streams,int length, bool do_step2=false);
	HOST static void Clear(Dvc_RandomStreams* Dvc);
};

} // namespace despot

#endif
