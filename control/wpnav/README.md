# Waypoint Navigation

This project is about automatic navigation along given waypoints.
It contains functions to connect waypoints with line segments and connect the line segments with circle segments in the proximity of a waypoint.
There are matching functions that perform the matching to the waypoint track.
Functions for determining the closest point of the trajectory to the current aircraft position and functions that return the required velocity and acceleration to follow the desired flight path.

## Motivation

This is an efficient implementation to follow waypoints.
There is also a different implementation in the [Trajectory](../trajectory) project, where the waypoints are connected with splines.
Compared to the Trajectory project, this project is much more computationally efficient since only line and circle segments are used to connect the waypoints.

## Example

Run the example script to see how the waypoint navigation from waypoints works.
```
wpnav_example
```
