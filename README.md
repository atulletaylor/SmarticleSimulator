# SmarticleSimulation

Preliminaries:
1. Python version must be python 3
2. Install pybullet

other dependencies:
sys, os, time, pybullet_data, numpy, matplotlib, datetime

Example simulation (3 smarticles in a ring on a table top):
python simple_sim.py

General-use simulation:
This file loads a user specified number of permanently active and inactive smarticles and a ring object. Optionally one can load a flashlight object that activates the light sensors on the smarticles, causing active smarticles to become inactive. After the prespecified run time has passed, data will be saved in a folder called Trial_MM-DD_HH:MinMin. Each episode will be saved in a csv file called Episode_ITERATIONNUMBER.

Variables located in simulation.py
	1. To use gui, GUI = True
	2. To use flashlight FLASHLIGHT = True
	3. Variables R, L for defining right and left wing gait
	4. Variables num_smart_active, inactive for setting number of active and inactive smarticles
	5. Varible sim_time controls simulation length in seconds
	6. Number of episodes can be changed in the variable runs (set to 10 currently)
	 
	



