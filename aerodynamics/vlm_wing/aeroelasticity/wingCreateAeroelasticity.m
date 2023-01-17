function aeroelasticity = wingCreateAeroelasticity( n_panels, n_structure_eig )
% wingCreateAeroelasticity defince and initialize aeroelasticity struct of
% wing struct
% 
% Inputs:
%   n_panels            number of wing panels (scalar)
%   n_structure_eig     number of structure eigenmodes (scalar)
% 
% Outputs:
%   aeroelasticity      aeroelasticity struct as defined by this function
% 
% See also:
%   wingCreate, wingSetAeroelasticity
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% transformation matrix that maps from structure deformation state to wing
% vortex position deformation (see wingSetGeometryState)
aeroelasticity.T_vs = zeros( 3 * (n_panels+1), n_structure_eig );
% transformation matrix that maps from structure deformation state to wing
% control point deformation (see wingSetGeometryState)
aeroelasticity.T_cs = zeros( 4 * n_panels, n_structure_eig );
% transformation matrix that maps from load vector for each control point
% (note that the load is applied at c/4 which is important for the pitching
% moment) to structure generalized load vector
aeroelasticity.T_sc = zeros( n_structure_eig, 4 * n_panels );

end