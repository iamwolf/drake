# Kuka LWR Simulation in Drake
There are three different options to simulate the LWR arm in Drake.

## Simulating through the Matlab interface

### TODO
1. Add a simple PDController example
1. Implement the HandState and HandInput analog to IRB140

## Simulating with the runDynamics C++ simulator

## Starting a basic simulator with ``urdfLCMNode``
1. Launch a visualizer: ``drake-visualizer &&``
1. Launch the LCM simulator: ``urdfLCMNode --base FIXED  ~/oh-distro/software/models/lwr_defs/robots/lwr_drake.urdf``