# Quaternions

This directory contains an elementary collection of functions for calculating quaternions, angular distances and rotation vectors.

## Motivation

Quaternions are widely used in the field of aviation as attitude representation.
Especially in the context of high dynamic control or in transformation aircraft, they have great advantages over Euler angles, because they are free of singularities (no gimbal lock). Moreover, they have very favorable mathematical properties. A number of problems can be solved very efficiently and clearly by using them.

## Usage

All functions have example calls in their documentation.
Important is the definition of the quaternions which corresponds to the Hamilton quaternions. The order is the scalar part followed by the 3 vector parts. So a quaternion is defined as column vector:

    % Scalar part 
    qw = 1.0;

    % Vector parts
    qx = 0.0;
    qy = 0.0;
    qz = 0.0;

    % Zero rotation quaternion (equivalent to the unit matrix as a rotation matrix)
    quat = [qw; qx; qx; qz];

To calculate two quaternions:

    q  = [0.7071; 0.7071; 0; 0];
    qr = [0.7071; -0.7071; 0; 0];
    ans = quatMultiply(q, qr)
