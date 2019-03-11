#include <despot/random_streams.h>
#include <despot/util/seeds.h>
#include <vector>
#include <sstream>
#include <cstring>
#include <assert.h>
using namespace std;

namespace despot {


RandomStreams::RandomStreams(int num_streams, int length) :
	position_(0) {
	vector<unsigned> seeds = Seeds::Next(num_streams);

	streams_.resize(num_streams);
	for (int i = 0; i < num_streams; i++) {
		Random random(seeds[i]);
		streams_[i].resize(length);
		for (int j = 0; j < length; j++)
			streams_[i][j] = random.NextDouble();
	}
}
RandomStreams::RandomStreams(const RandomStreams& src)
{
	position_=src.position_;
	streams_.resize(src.streams_.size());
	for(int i=0;i<streams_.size();i++)
	{
		streams_[i].resize(src.streams_[i].size());
		memcpy(streams_[i].data(),src.streams_[i].data(), src.streams_[i].size()*sizeof(double));
		/*for(int j=0;j<10;j++)
			cout<<streams_[i][j]<<" ";
		cout<<endl;*/
	}
}
int RandomStreams::NumStreams() const {
	return streams_.size();
}

int RandomStreams::Length() const {
	return streams_.size() > 0 ? streams_[0].size() : 0;
}

void RandomStreams::Advance() const {
	position_++;
}
void RandomStreams::Back() const {
	position_--;
}

void RandomStreams::position(int value) const {
	position_ = value;
}
int RandomStreams::position() const {
	return position_;
}

bool RandomStreams::Exhausted() const {
	return position_ > Length() - 1;
}

double RandomStreams::Entry(int stream) const {
	return streams_[stream][position_];
}

double RandomStreams::Entry(int stream, int position) const {
	return streams_[stream][position];
}

ostream& operator<<(ostream& os, const RandomStreams& stream) {
	for (int i = 0; i < stream.NumStreams(); i++) {
		os << "Stream " << i << ":";
		for (int j = 0; j < stream.Length(); j++) {
			os << " " << stream.Entry(i, j);
		}
		os << endl;
	}
	return os;
}

void RandomStreams::ImportStream(std::istream& in,int num_streams, int length)
{
	streams_.resize(num_streams);
	for (int i = 0; i < num_streams; i++) {
		streams_[i].resize(length);
	}
	if (in.good())
	{
		string str;
		int SID=0;
		while(getline(in, str))
		{
			if(!str.empty() && SID<num_streams)
			{
				istringstream ss(str);

				float num; string dummy; int pos=0;
				ss >> dummy >>dummy;//Remove headers
				while(ss >> num)
				{
					if(pos>=length)
					{
						pos=0;
						cout<<"Import stream error: pos>=length!"<<endl;
					}
					streams_[SID][pos]=num;
					pos++;
				}
			}
			SID++;
		}
	}
	else
	{
		cout<<__FUNCTION__<<": Empty stream file!"<<endl;
		exit(-1);
	}
	position_=0;

	cout<< this;
}

} // namespace despot
