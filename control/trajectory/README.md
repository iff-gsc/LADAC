## Trajectory

This project is about autonomous flight along given waypoints.
It contains function to compute smooth trajectorys from waypoints.
Functions for determining the closest point of the trajectory to the current aircraft position.
And to determine the reference variables for attitude and acceleration as a function of the current airspeed in order to be able to follow the trajectory exactly.

## Motivation

There are many tools to generate trajectories from waypoints. However, this one is specifically optimized for the requirements of highly dynamic trajectories.
It works on few and clearly defined structs and offers functions with intuitive handling to generate trajectories from given waypoints very easily.
In addition, the definition of the trajectory is mathematically chosen to be defined by piecewise polynomials which can be of arbitrary order.
Trajectories with smooth derivatives of higher order are especially important for highly dynamic controllers, since they also provide reference quantities for acceleration and jerk.
In addition, various functions are integrated, which determine the current offset from the aircraft to the trajectory. 
Depending on the flight speed, the required acceleration can also be calculated for a rigid body, which is necessary to follow the trajectory exactly.
This acceleration vector already considers the earth gravity. From it, a position reference can be derived unambiguously at any time.
All in all, these tools under both Simulink and Matlab drastically simplify the creation of position controllers for airplanes and helicopters as well as UAVs.

## Tests

1. Checkout the example script to see how the trajectory generation for a few trajectorys from waypoints work with matlab.
	```
	traj_from_waypoints_example
	```
2. Open and run the Simulink example to see how a basic trajectory controller for a closed trajectory works.
	```
	traj_example_rigid_body_init
	```


## How to use?

- Read through the [Tests](#Tests) section.
- additonally documentation will be added soon -work in progress-
