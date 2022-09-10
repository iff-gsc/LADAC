% ** Parameters for motor-propeller control effectiveness (default) **

% thrust factor, N/(rad/s)^2
param.k   = 1.0177e-6;
% drag factor, Nm/(rad/s)^2
param.d   = 9.5376e-9;
% propeller FRD x positions w.r.t. CG, m
param.x   = [1,1,-1,-1]*0.0795;
% propeller FRD y position w.r.t. CG, m
param.y   = [-1,1,1,-1]*0.099;
% propeller FRD z position w.r.t. CG, m
param.z   = [1,1,1,1]*0.01;
% direction of all propeller rotations, from top view: -1 = right, 1 = left 
param.a   = [-1,1,-1,1];
% propeller unit vector thrust component in FRD x direction
param.nx  = [0,0,0,0];
% propeller unit vector thrust component in FRD y direction
param.ny  = [0,0,0,0];
% propeller moment of inertia, kg*m^2
param.ip  = 2e-6;
% motor torque constant (inverse of KV), Nm/A
param.kt  = 0.0056;
param.vb  = 22.2;
% motor internal resistance, Ohm
param.ri  = 0.32;
