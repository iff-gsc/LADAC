function [p_gravity,q_gravity] = structureGetGravityLoadVector( ...
    m_nodes, g, M_bg, T )
% structureGetGravityLoadVector compute the load vector of a structural
% dynamics model due to gravity
% 
% Synatx:
%   [p_gravity,q_gravity] = structureGetGravityLoadVector( m_nodes, g, ...
%       M_bg, T )
% 
% Inputs:
%   m_nodes         masses of the nodes (1xM array for M nodes), in kg
%   g               gravitational acceleration (scalar), in m/s^2
%   M_bg            rotation 3x3 matrix (DCM) from the earth frame (g) to 
%                   the body-fixed frame (b), in 1   
%   T               modal transformation matrix ((6*M)xN array for N mode
%                   shapes)
% 
% Outputs:
%   p_gravity       node load vector due to gravity ((6*M)x1 array)
%   q_gravity   	generalized force vector due to gravity (Nx1 array)
% 
% See also:
%   structureGetNodeMass, structureGetReduced

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% gravity vector in earth frame (g)
g_g = [0;0;g];

% transform gravity to body frame
g_b = M_bg * g_g;

% node load vector due to gravity
p_gravity = structureGetLoadVectorFromNodeLoad( g_b * m_nodes, ...
    zeros(3,size(m_nodes,2)) );

% generalized load vector due to gravity
q_gravity = T' * p_gravity;

end