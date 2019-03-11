#ifndef GPUUTIL_H
#define GPUUTIL_H

#include <despot/GPUcore/CudaInclude.h>

namespace despot {


DEVICE inline bool Dvc_CheckFlag(int flags, int bit) {
	return (flags & (1 << bit)) != 0;
}
DEVICE inline void Dvc_SetFlag(int& flags, int bit) {
	flags = (flags | (1 << bit));
}
DEVICE inline void Dvc_UnsetFlag(int& flags, int bit) {
	flags = flags & ~(1 << bit);
}



} // namespace despot

#endif
