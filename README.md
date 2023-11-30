# Proportional Formation Control 
Using proportional control and graph theory to control a swarm of robots to create various formations
_summer project I did in 2023_

In this repo, you will find a folder for MATLAB simulations and python programs meant to run on the Robomaster SDK. This was an independent summer project under Dr. Hassan Jaleel where we used some elements of graph theory
to create a proportional formation control algorithm for multiple robots. 

## Installation

simple clone the repository:
```
git clone https://github.com/synonymous01/multiagent_formation_control/
```

You will be able to run the MATLAB simulations directly from the folder if you have MATLAB installed.

For running the Robomaster SDK programs, you'll need the python library as a prerequisite. Make sure you have a python version between 3.6 and 3.10 for installing this package.
```
pip install robomaster
```

## Setting up the robomaster equipment for these experiments
We used centralized control to do all these formations. Therefore, all the robots were connected to a central router. Our computer was also connected to the same network.
Please see the *Networking Connection* section of this [documentation](https://robomaster-dev.readthedocs.io/en/latest/python_sdk/connection.html)

## Reference
Graph Theoretic Methods in Multiagent Networks by Mesbahi and Egerstedt
