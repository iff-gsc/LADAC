# Library for Aircraft Dynamics And Control (LADAC)

This project is a library with functions for aircraft dynamics and control
for Matlab/Simulink.
Examples of how to use LADAC are available in [LADAC-Examples](https://github.com/iff-gsc/LADAC-Examples).


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
- It is possible to generate C/C++ code of the library functions.
- All functions are carefully documented using comments in the code. This
    includes literature references to make scientific use easier.


## Installation

- MATLAB:
  1. You need MATLAB/Simulink 2018b (later is also possible). If you want to use older versions,
        you will have to export the .slx file in older version format before.
  2. You may also need some MATLAB toolboxes like Curve Fitting Toolbox,
	   MATLAB Coder, Embedded Coder, Simulink Coder
	   Simulink Control Design depending on what you want to do
- Clone LADAC including its submodules (generally the main branch should be used).
  - If your project is a Git repository, you should add LADAC as a submodule:
    ```
    git add submodule https://github.com/iff-gsc/ladac.git
    ```
  - Otherwise, just clone LADAC to the desired directory:
    ```
    git clone --recursive https://github.com/iff-gsc/ladac.git
    ```
- Add LADAC folder to the Matlab path (in Matlab Command Window):
    ```
    addpath(genpath('ladac'));
    ```


## Tests

1. Check if LADAC functions work by calling the automatic test framework (in Matlab Command Window):
    ```
    check_ladac
    ```
2. Check if LADAC appears in the Simulink Library Browser:
   - Open a Simulink file.
   - Open the Simulink Library Browser.
   - Look for "LADAC" at the highest level (you may have to refresh the Library Browser by right-clicking or pressing F5).
     This will probably only work if you use the supported Matlab version.
   - Check whether all sub-areas of LADAC appear and if you can add blocks to your Simulink file.
     Alternatively, you can add blocks by opening `ladac_lib.slx` (you can click through the sub-areas and add blocks by copy and paste).


## How to use?

### Introduction

- Get to know the range of functions.
LADAC is divided into several sections.
These areas are partly subdivided again into subareas.
Each area and subarea is located in a folder or subfolder (see [Contents](#Contents)).
- Usually each folder or subfolder contains a separate README with specific information.
- Most of the implementations are m-functions that are documented inside the code.
- Each subdirectory contains a Simulink library file (.slx) with the same name as the subdirectory. These libraries contain blocks which call the m-functions. There are also more complex blocks, but they are always based on the m-functions.

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
The equations of motion library contains multiple implementations of rigid-body
and flexible-body equations of motion.
- [**Flight parameters**](flight_parameters)  
The flight parameters library allows the computation of flight parameters
such as aerodynamic angles, flight path angles or time-derivatives of Euler angles.
- [**Propulsion**](propulsion)  
The propulsion library contains sublibraries to model propellers, motors or batteries.
- [**Sensors**](sensors)  
The sensor library contains models that compute outputs for different sensors.
- [**Utilities**](utilities)  
The utilities library contains multiple sublibraries with miscellaneous functions
such as axes transformation, interfaces to external programs and unit conversions

### Modeling and control of systems
- Use LADAC as Git submodule.
- Build your models using the LADAC library blocks.
- Make yourself familiar with working with libraries:
  - Read the Matlab documentation, e.g. https://www.mathworks.com/help/simulink/ug/creating-and-working-with-linked-blocks.html
  - **Hint:** display links of library blocks: Display &#8594; Library Links &#8594; All

### Anything missing in LADAC?
- Make sure you did not overlook the required function.
- Consider modifying existing functions if it is about input data handling or consider contributing new functions (see Contribute section).


## Contribute

Contributions are welcome and encouraged. You can contribute in many ways:

- implement a new feature in the software  
  &#8594; [read developers guide](docs/Development.md).
- fix a bug  
  &#8594; [read developers guide](docs/Development.md).
- documentation updates and corrections  
  &#8594; [read developers guide](docs/Development.md).
- report a bug   
  &#8594; [create an issue](https://github.com/iff-gsc/ladac/issues/new)
- new feature ideas & suggestions  
  &#8594; [create an issue](https://github.com/iff-gsc/ladac/issues/new)

Before creating new issues please check to see if there is an existing one.
