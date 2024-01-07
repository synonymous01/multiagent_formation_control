# Proportional Formation Control 
Using proportional control and graph theory to control a swarm of robots to create various formations  
_summer project I did in 2023_

In this repo, you will find a folder for MATLAB simulations and python programs meant to run on the Robomaster SDK. This was an independent summer project under [Dr. Hassan Jaleel](https://scholar.google.com/citations?user=T7kqUskAAAAJ&hl=en) where we used some elements of graph theory
to create a proportional formation control algorithm for multiple robots. 

This exercise went on to become the basis of our research in Distributed Control Theory, source for which you can find [here](https://github.com/synonymous01/multiagent_control).

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

You will also have to modify the `self.robots_sn` array to the serial numbers associated with your EP Core robots for each program. 

## Setting up the robomaster equipment for these experiments
We used centralized control to do all these formations. Therefore, all the robots were connected to a central router. Our computer was also connected to the same network.
Please see the *Networking Connection* section of this [documentation](https://robomaster-dev.readthedocs.io/en/latest/python_sdk/connection.html)

## Credits
**Ibrahim Arif (me)**: coded iMaSS logo choreography, moving formation control (MATLAB and robomaster)  
**Ali Rehman**: coded consensus algorithms for robomaster and formation control with collision avoidance MATLAB simulations  
**Ibrahim Rana**: coded MATLAB simulations for consensus algorithm and formation control  

## Reference
Graph Theoretic Methods in Multiagent Networks by Mesbahi and Egerstedt
