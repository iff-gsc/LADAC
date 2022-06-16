# Tornado VLM Interface

The Tornado interface allows easy use of the third-party VLM code Tornado [1].


## Motivation 

Tornado is a well-known VLM code implemented in Matlab.
It has already been used in several publications.
Therefore, it can be used, for example, to validate other aerodynamic models or to calculate aerodynamic derivatives.


## Installation

- You need [LADAC](../../README.md)
- You need Tornado and add the folder to your Matlab path:
  ```
  git clone https://github.com/iff-gsc/Tornado.git
  ```

## Example

Run an example computation by running `tornado_interface_example.m`:
  ```
  tornado_interface_example
  ```
The spanwise lift coefficient distribution should appear in a figure.


## How to use

The wing geometry is defined based on the same parameters file as in the
[VLM wing](../vlm_wing) project.


## Literature
[1] User's Guide Tornado 1.0, Release 2.3 2001-01-21, http://tornado.redhammer.se/images/manual.pdf
