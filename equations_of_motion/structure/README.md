# Structural Dynamics (Beam-Element Model)

This project implements a linear structural dynamics beam-element model.

## Motivation

Often aircraft structural dynamics can approximated with a beam-element model.
With this implementation such beam-element structural dynamics model can be created, analyzed, and reduced to a mode shapes model.

## Installation

- You need [LADAC](../../README.md)

## Example

- Create the default structural dynamics model:  
  ``
  structure = structureCreate( 'structure_params_default' );
  ``
- Visualize the structural dynamics model:  
  ``
  figure
  structurePlot( structure )
  ``
- Visualize the first eigenmode:  
  ``
  figure
  structurePlotEigenmode( structure, 1, 'Scaling', 0.2 )
  ``

## How to use

First, create a parameters file.
Have a look at the template `structure_params_default`.
Then, create the structural dynamics model using the function `structureCreate`.
For more information, have a look at the function documentation.
There is also a function to import structural dynamics models from NASTRAN: `structureCreateFromNastran`.

After that, you can use the various functions provided by this project.
For example, you can create a (reduced-order) modal model by using the function `structureGetReduced`.
For more information, have a look at the function documentations.

If you also want to simulate the global motion of the structure (for example the motion of a flexible aircraft), have a look at the project [Flexible Body (simple)](../flexible_body_simple), which is based on this project.

