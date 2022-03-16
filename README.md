# Library for Aircraft Dynamics And Control (LADAC)

This project is a library with functions for aircraft dynamics and control
for Matlab/Simulink.


## Motivation

This is the main motivation and objective of LADAC:

- Functions in LADAC can be used in different projects. Functions
    that are generally usable in multiple projects should be located in
    LADAC instead of copied/developed in multiple projects.
- LADAC is related to the Matlab Aerospace Blockset but also contains
    several functions that are not available in the Aerospace Blockset.
- The Simulink blocks are based on Matlab functions saved as m-files. This
    improves the use of version control. Diffs can be made easier and
    the data size of the repository is reduced.
- It is possible to generate C/C++ code of the library functions. This is
    assured by using the %#codegen directive.
- All functions are carefully documented using comments in the code. This
    includes literature references to make scientific use easier.


## Installation

- MATLAB:
  1. You need MATLAB/Simulink 2018b (later is also possible). If you want to use older versions,
        you will have to export the .slx file in older version format before.
  2. You may also need some MATLAB toolboxes like Curve Fitting Toolbox,
	   Aerospace Blockset, Aerospace Toolbox, MATLAB Coder, MATLAB Compiler, 
	   Simulink Control Design, Simulink Coder depending on what you want to do
- Clone LADAC (see Tests section).


## Tests

1. Add LADAC to your project
   - If your project is a Git repository, you should add LADAC as a
    submodule:
        ```
        git add submodule https://github.com/iff-gsc/ladac.git
        ```
     OR: clone LADAC to the desired directory:
        ```
        git clone --recursive https://github.com/iff-gsc/ladac.git
        ```
   - Generally, the master branch should be used.
        
2. Add LADAC to the Matlab path:
    ```
    addpath(genpath('LADAC'));
    ```
3. Test if LADAC functions work by e.g. calling the ISA atmosphere function or its documentation:
    ```
    isAtmosphere(0)  
    help isAtmosphere
    ```
4. Test if LADAC appears in the Simulink Library Browser:
   - Open a Simulink file.
   - Open the Simulink Library Browser.
   - Look for "LADAC" at the highest level (you may have to refresh the Library Browser by right-clicking or pressing F5). This will only work if you use the supported Matlab version. You can use the library blocks in newer version but they will not appear in the Library Browser.
   - Check whether all sub-areas of LADAC appear and if you can add blocks to your Simulink file. This will only work if you use the supported Matlab version. You can use the library blocks in newer version but they will not appear in the Library Browser.


## How to use?

### Introduction

- Get to know the range of functions.
LADAC is divided into several sections.
These areas are partly subdivided again into subareas.
Each area and subarea is located in a folder or subfolder (see [Contents](#Contents)).
- Usually each folder or subfolder contains a separate README with specific information.
- Most of the implementations are m-functions that are documented inside the code.
- Each subdirectory contains a Simulink library file (.slx or .mdl) with the same name as the subdirectory. These libraries contain blocks which call the m-functions. There are also more complex blocks, but they are always based on the m-functions.

### Contents

- [**Actuators**](actuators)  
The actuators library contains typical models for actuator dynamics
and other functions related to actuators.
- [**Aerodynamics**](aerodynamics)  
The aerodynamics library contains multiple models to compute aerodynamic forces and moments
for wings, rotorcrafts, airfoils and fuselages based on different methods.
- [**Aircraft**](aircraft)  
The aircraft library contains complete aircraft models as well as subsystems
of aircraft.
- [**Control**](control)  
The control library contains basic modules to control different types of aircraft
with different control methods.
- [**Environment**](environment)  
The environment library contains modules to compute environmental
parameters such as wind, atmospheric parameters and ground forces.
- [**Equations of motion**](equations_of_motion)  
The equations of motion library contains multiple implementations of rigid-body equations of motion.
- [**Flight parameters**](flight_parameters)  
The flight parameters library allows the computation of flight parameters
such as aerodynamic angles, flight path angles or time-derivatives of Euler angles.
- [**Propulsion**](propulsion)  
The propulsion library contains sublibraries to model propellers, motors or batteries.
- [**Sensors**](sensors)  
The sensor library contains models that compute outputs for different sensors.
- [**Structure**](structure)  
The structure library contains an implementation of flexible aircraft equations of motion
that can also be coupled with an aerodynamics model (aeroelasticity).
- [**Utilities**](utilities)  
The utilities library contains multiple sublibraries with miscellaneous functions
such as axes transformation, interfaces to external programs and unit conversions

### Modeling and control of systems
- Use LADAC as Git submodule.
- Build your models using the LADAC library blocks
  - EITHER: by drag and drop the blocks from the Simulink Library Browser
  - OR: by copy and paste them from the corresponding Simulink library file (.slx).
- Make yourself familiar with working with libraries:
  - Read the Matlab documentation, e.g. https://www.mathworks.com/help/simulink/ug/creating-and-working-with-linked-blocks.html
  - **Hint:** display links of library blocks: Display --> Library Links --> All

### Anything missing in LADAC?
- Make sure you did not overlook the required function.
- Consider modifying existing functions if it is about input data handling or consider contributing new functions (see Contribute section).


## Contribute

Contributions are welcome and encouraged. You can contribute in many ways:

- implement a new feature in the software  
  --> [read developers guide](docs/Development.md).
- fix a bug  
  --> [read developers guide](docs/Development.md).
- documentation updates and corrections  
  --> [read developers guide](docs/Development.md).
- report a bug   
  --> [create an issue](https://github.com/iff-gsc/ladac/issues/new)
- new feature ideas & suggestions  
  --> [create an issue](https://github.com/iff-gsc/ladac/issues/new)

Before creating new issues please check to see if there is an existing one.
