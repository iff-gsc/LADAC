function [v_i, v_h, v_total, V_z, P_i, P_c] = ...
    inducedVelocityWithUnits(T, V_kb, R, rho) %#codegen
% inducedVelocityWithUnits computes all important velocities of a rotor,
% as well as the needed induced power and power for climb.
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
% [2]   Johnson, Wayne (2005): Model for Vortex Ring State Influence on
%       Rotorcraft Flight Dynamics. NASA Ames Research Center.
%
% Syntax: [v_i, v_h, v_total, V_z, P_i, P_c] =
%       inducedVelocityWithUnits(T, V_kb, R, rho)
%
% Inputs:
%   T           rotor thrust, positive for regular helicopter operation.
%               (scalar), in N
%   V_kb        velocity of rotor hub. right handed coordinate system.
%               for regular helicopter in hover:
%               x-axis postive in flight direction, y-axis positive to
%               right side and z-axis positive from rotor looking down at
%               the earth. V_kb = [Vx; Vy; Vz].
%               positive Vz means rotor plane is descending;
%               (3x1 vector), in m/s
%   R           rotor radius, 
%               (scalar), in m         
%   rho         density of air, 
%               (scalar), in kg/m^3    
%
% Outputs:
%   v_i         induced velocity
%               (scalar), in m/s
%   v_h         induced velocity in hover
%               (scalar), in m/s
%   v_total     total flow velocity through rotor plane
%               (scalar), in m/s
%   V_z         climb speed
%               (scalar), in m/s
%   P_i         induced power
%               (scalar), in Watt
%   P_c         climb power
%               (scalar), in Watt
%
% See also: InducedVelocity

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% rotor disk area
A = pi * R^2;

% calculate forward velocity scaled with induced velocity in hover
V_x = sqrt(V_kb(1)^2 + V_kb(2)^2);

% vertical veloctiy
V_z = -V_kb(3);

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

%calculate induced velocity induced velocity in hover
vi_vh = inducedVelocityVortexRingState(Vx, Vz);

% scale induced velocity with 
v_i = vi_vh * v_h;

% total inflow velocity
v_total = v_i + V_z;

% Induced power
P_i = T * v_i;

% Climb power
P_c = T * V_z;


end
