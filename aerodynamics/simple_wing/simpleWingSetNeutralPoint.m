function wing = simpleWingSetNeutralPoint( wing )
% simpleWingSetNeutralPoint sets the neutral point for the simple wing
%   structs that will be applied for small aerodynamic angles.
% 
% Inputs:
%   wing            simple wing struct (see simpleWingLoadParams)
% 
% Outputs:
%   wing            simple wing struct (see simpleWingLoadParams)
% 
% Literature:
%   [1] Schlichting, H., & Truckenbrodt, E. A. (2013). Aerodynamik des
%       Flugzeuges: Zweiter Band: Aerodynamik des Tragfl�gels (Teil II), 
%       des Rumpfes, der Fl�gel-Rumpf-Anordnungen und der Leitwerke. 
%       Springer-Verlag.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

b = wing.geometry.b;
phi = wing.geometry.phi;
c = wing.geometry.c;


% spanwise distance from center (own formula that says that the centroid of
% the lift is at 0.42 of the span; to do: make variable w.r.t. aspect 
% ratio, taper, ...)
distance = 0.42*b/cos(phi)*0.5;
% correction of aerodynamic neutral point vs. geometric neutral point
% [1], page 68
distance = distance * ( 1 + 0.09/cos(pi/4)*sin(phi) * c / (eps(1)+sin(phi)) );
% position of the neutral point of the left wing side and the right wing
% side in wing frame, m
xyz_wing_np_left = [ -distance*sin(phi); -distance*cos(phi); 0];
xyz_wing_np_right = [ -distance*sin(phi); distance*cos(phi); 0 ];

% create matrix from vectors
wing.xyz_wing_np = [ xyz_wing_np_left, xyz_wing_np_right ];

end
