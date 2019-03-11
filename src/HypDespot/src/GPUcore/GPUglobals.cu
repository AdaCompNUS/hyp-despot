#include <despot/GPUcore/GPUglobals.h>
#include <float.h>
using namespace std;

namespace despot {
namespace Dvc_Globals {

Dvc_Config* config;
DEVICE const double Dvc_INF = 1e8;
DEVICE const double Dvc_TINY = 1e-8;
DEVICE const double Dvc_POS_INFTY = DBL_MAX;
DEVICE const double Dvc_NEG_INFTY = -Dvc_POS_INFTY;

} // namespace Dvc_Globals
} // namespace despot
