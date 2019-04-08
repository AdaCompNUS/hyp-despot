# HyP-DESPOT

The HyP-DESPOT package is developed based on [the DESPOT package](https://github.com/AdaCompNUS/despot). The API in HyP-DESPOT closely follows that in the DESPOT package. See [here](https://github.com/AdaCompNUS/despot/tree/API_redesign/doc) for detailed documentations of the DESPOT package.

The algorithm was initially published in our RSS paper:

Cai, P., Luo, Y., Hsu, D. and Lee, W.S., HyP-DESPOT: A Hybrid Parallel Algorithm for Online Planning under Uncertainty. Robotics: Science & System 2018. [(PDF)](http://motion.comp.nus.edu.sg/wp-content/uploads/2018/06/rss18hyp.pdf)

## Getting Started

### Pre-requisites
* Cmake 
  * Version >=3.8 is required for CUDA integration
* CUDA and an NVIDIA GPU with computational capacity >=3.0
### 1. Download the HyP-DESPOT package:
```bash
cd ~/workspace
git clone https://github.com/AdaCompNUS/HyP-DESPOT.git
```
### 2. Compile HyP-DESPOT and examples:
```bash
cd HyP-DESPOT
mkdir build; cd build
cmake ..
make
```
## Main Extensions in HyP-DESPOT from the DESPOT Package
The source files of HyP-DESPOT and examples are in folder [src/HypDespot](src/HypDespot). Main extensions from DESPOT include:
```
include/despot/GPUinterface/             Header files: GPU versions of interface classes in DESPOT
include/despot/GPUcore/                  Header files: GPU versions of core classes in DESPOT
include/despot/GPUutil/                  Header files: GPU versions of utility classes in DESPOT
src/GPUinterface                         Source files: GPU versions of interface classes in DESPOT
src/GPUcore                              Source files: GPU versions of core classes in DESPOT
src/GPUutil                              Source files: GPU versions of utility classes in DESPOT
src/solvers/Hyp_despot.cu                Main file of the HyP-DESPOT solver
src/Parallel_planner.cu                  Parallel extension of the planner class in DESPOT
src/GPUrandom_streams.cu                 GPU version of the RandomStreams class in DESPOT
```
See this [GPU model documentation](doc/Build_GPU_POMDP_model_with_CUDA.md) for detailed descriptions on these extensions and how to build a custom GPU POMDP model. Refer to this [guide](doc/Nsight_guide.md) for integration with NVidia Nsight.

## Examples
The HyP-DESPOT package contains two examples presented in our [RSS paper](http://motion.comp.nus.edu.sg/wp-content/uploads/2018/06/rss18hyp.pdf). They include:

* Navigation in a partially known map [(HyP_examples/unkown_navigation/)](src/HyP_examples/unkown_navigation/src). The key files in this example are:
```
Unc_Navigation/UncNavigation.cpp       CPU POMDP model of the navigation problem
GPU_Unk_nav/GPU_UncNavigation.cu       GPU POMDP model of the navigation problem
Unc_Navigation/main.cu                 The custom planner and the main function
```

* Multi-agent RockSample [(HyP_examples/ma_rock_sample/)](src/HyP_examples/ma_rock_sample/src). The key files in this example are:
```
ma_rock_sample/ma_rock_sample.cpp       CPU POMDP model of the car driving problem
GPU_MA_RS/GPU_ma_rock_sample.cu         GPU POMDP model of the MARS problem
ma_rock_sample/main.cu                  The custom planner and the main function
```

## (Optional) Debugging Tools in HyP-DESPOT Package
The  [tools](tools) folder provides tools for debugging HyP-DESPOT when implementing new problems, including:
```
Particles*.txt                 Text files: particles (starting states of scenarios) for different simulation steps to be loaded and used to fix scenarios in HyP-DESPOT.
Streams*.txt                   Text files: random streams in scenarios for different simulation steps to be loaded and used to fix scenarios in HyP-DESPOT
draw_car_cross.py              Script: to visualize the execution record output by HyP-DESPOT (through cout and cerr)
run_Car_hyp_debug.sh           Script: to run experiments with HyP-DESPOT
```
The best way to debug is to fix the scenarios and output the search process. This can be acheived by setting the **FIX_SCENARIO** flag defined in [GPUcore/thread_globals.h](src/HypDespot/include/despot/GPUcore/thread_globals.h). Possible vaues to be set are:
```
0         Normal mode
1         Read scenarios from Particles*.txt and Streams*.txt
2         Run in normal mode and export Particles*.txt and Streams*.txt during each simulation step
```
Alternatively, setting the **DESPOT::Debug_mode** defined in [despot.cpp](src/HypDespot/src/solver/despot.cpp) to be **true** will fix all random seeds used in HyP-DESPOT, and thus the search will be fully determinized for easier debugging.
