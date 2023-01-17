function [ v_i, V_z, v_i0 ] = inducedVelocityWithUnitsAngleOfAttack( ...
    V_A, alpha, T, rho, A )
% inducedVelocityWithUnitsAngleOfAttack computes all important velocities
% of a rotor.
%
%   The computation is based on the momentum theory. The equation is
%   solved according to [1, page 153]. 
%   This function eliminates the singularity of momentum theory in
%   vortex ring state. This Vortex Ring State model uses third-order
%   polynomials, that connects stepwise the two branches of climb and
%   fast decent. [2, page 14]. This model is based on real rotor test data.
%   This model creates an instability for the vortex ring state. Within
%   the defined boundary the real part of the eigenvalue of heave damping
%   become positive.
%
% Literature:
% [1]   Van der Wall, B. G. (2015): Grundlagen der Hubschrauber-
%       Aerodynamik. Springer.
%
% Inputs:
%   V_A         absolute velocity of the propeller relative to the air
%               (scalar), in m/s
%   alpha       angle between the rotor plane and the velocity vector 
%               relative to the air (scalar), alpha = 0 means that the 
%               airspeed vector is parallel to the propeller plane, 
%               alpha = -pi/2 means the airspeed vector is perpendicular to
%               the propeller plane, negative alpha means that the 
%               propeller is ascending, positive alpha means that the 
%               propeller is descending, in rad
%   T           thrust (scalar), in N
%   rho         air density (scalar), in kg/m^3
%   A           surface of the propeller (scalar), in m^2
%
% Outputs:
%   v_i         induced velocity of the propeller (scalar), in m/s
%   V_z         velocity of the propeller relative to the air perpendicular
%               to the propeller plane (scalar), in m/s
%   v_i0        induced velocity for zero velocity perpendicular to the
%               propeller plane (scalar), m/s
%

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2020-2022 Fabian Gücker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% compute the induced velocity for hover according to [1, page 115]
v_i0 = sqrt( abs( T ) / ( 2*rho*A ) );

% compute the velocity of the propeller relative to the air
% perpendicular to the propeller plane according to [1, page 151]
V_z = -V_A*sin(alpha);

% compute the velocity of the propeller relative to the air
% tangential to the propeller plane according to [1, page 151]
V_x = V_A*cos(alpha);

% induced velocity in hover without climb speed V_c
sign_v_h = sign(T);
v_h = sqrt( abs(T) / (2 * rho * A) );
v_h = v_h * sign_v_h;

% check for division by zero 
if(v_h == 0 )
    
    % calculate forward velocity scaled with induced velocity in hover
    Vx = 0;
    Vz = 0;
    
else
    % calculate forward velocity scaled with induced velocity in hover
    Vx = V_x / v_h;
    
    % climb velocity scaled with induced velocity in hover
    Vz = V_z / v_h;
end

% calculate ratio of induced velocity to induced velocity in hover
vi_vh = inducedVelocityVortexRingState(Vx, Vz);

% scale induced velocity with ratio of vi_vh
v_i = vi_vh * v_h;

end