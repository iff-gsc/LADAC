# Map-based propeller

This project provides functions and Simulink models to compute the forces and moments of a propeller based on a map.
Data of multiple maps of APC propellers is also provided.

## Motivation

The propeller thrust and torque are not easy to model. 
However, there are available maps that provide good results.

## Installation

- You must install [LADAC](../../README.md) (you need the Curve Fitting Toolbox).


## Example

Run the example script and read the comments.
  ```
  propMap_example
  ```

## How to use?

Simulink:  
You should initialize a propeller struct that contains
the propeller map and also the inertia and the direction of rotation.
  ```
  prop = propMapLoadParams( 'propMap_params_default' );
  ```
Then you can use the struct to parametrize the available blocks in the library
(e.g. `Map-based propeller fixed to airframe`).
