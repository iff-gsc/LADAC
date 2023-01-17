function [A,B,C7,D7] = unstAirfoilAeroSplit( V, Ma, c, C_L_alpha, x_ac ) %#codegen
% unstAirfoilAeroSplit computes the matrices of a state-space representation for
%   unsteady transsonic airfoil behavior according to [1] splitted into
%   the circulatory and noncirculatory part. The model has 8
%   states, two inputs and seven outputs.
%   The outputs are:
%   1. total normal force coefficient
%   2. total pitching moment coefficient
%   3. effective angle of attack, alpha_E, due to the shed wake
%       (circulatory terms)
%   4. circulatory part of the normal force coefficient
%   5. circulatory part of the pitching moment coefficient
%   6. noncirculatory part of the normal force coefficient
%   7. noncirculatory part of the pitching moment coefficient
%   Important note:
%   The state vector is defined differently than in [1], because otherwise
%   the airspeed should not be varied! Transformation: x_better = A * x
%   Thanks to the transformation the amplitude of the state vector does not
%   really depend on the airspeed anymore.
% 
% Inputs:
%   V           airspeed (scalar), in m/s
%   Ma          Mach number (scalar), in 1
%   c           chord (scalar), in m
%   C_L_alpha   lift curve slope (scalar), in 1/rad
%   x_ac        aerodynamic center measured from the nose of the profile
%               (scalar), in m
% 
% Outputs:
%   A           state matrix (8x8)
%   B           input matrix (8x2)
%   C7          output matrix (7x8)
%   D7          feedthrough matrix (7x2)
% 
% Authors:
%   Yannic Beyer
% 
% Literature:
%   [1] Leishman, J. G., and Nguyen, K. Q. (1990). State-space 
%       representation of unsteady airfoil behavior. AIAA journal, 28(5),
%       836-844 (https://arc.aiaa.org/doi/pdf/10.2514/3.25127).
% 
% See also: unstProfileAero
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    % compute the state space model according to [1]
    [A,B,C,D] = unstAirfoilAero( V, Ma, c, C_L_alpha, x_ac );

    % modify the output equation so that it splits the normal force
    % coefficient and pitching moment coefficient into the circulatory and
    % into the noncirculatory part (this should be comprehensible after
    % reading the appendix of [1])
    C7 = zeros( 7, size(C,2), size(C,3) );
    D7 = zeros( 7, size(D,2), size(D,3) );
    
    C7(1:end,1:end,1:end) = [ C; C(1:2,:,:); C(1:2,:,:) ];
    C7(4,[3,4],:) = 0;
    C7(5,[5,6,8],:) = 0;
    C7(6,[1,2],:) = 0;
    C7(7,[1,2,7],:) = 0;
    
    D7(1:end,1:end,1:end) = [ D; zeros(2,size(D,2),size(D,3)); D(1:2,:,:) ];
    
end