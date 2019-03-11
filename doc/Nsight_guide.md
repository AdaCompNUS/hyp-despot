
# Import HyP-DESPOT code into nsight

## 1. Create project

Create a C++ -> Makefile Project with Existing Code. Choose CUDA Toolkit toolchain. Set the root folder as the code location.

## 2. CMake generator

Create Release folder in the project.

Go to "Make Target" view (Ctrl+3 and then type "Make Target" if it's hard to find). "Make Target" view looks just like the project view.

Right click on the "Release" folder and "New...".

Uncheck "Same as target name", uncheck "Use builder settings".

Type in "Release" into "Target name" field, leave "Make target" empty, "Build command" is "cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Release ../". Click ok.

## 3.Build

Go to Project Properties, create "Release" configuration.

Make "Release" configuration active.

For "Release" configuration uncheck "Generate Makefiles automatically".

Set Build directory to "Release".

Enable parallel build.

Double click on this "Release" make target that was just created in the Release folder. It should run cmake.

It should build now in "Release" directory. If it doesn't, remove all from the "Release" directory and rerun cmake by double-clicking on "Release" target in the "Make Target" view as before.

"Debug" configuration can be done similarly by repeating 2 and 3.
