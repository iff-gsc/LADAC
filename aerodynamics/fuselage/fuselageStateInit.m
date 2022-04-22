function state = fuselageStateInit( n_segments, geometry )
%fuselageStateInit define and initialize fuselage state struct
% 
% Syntax:
%   state = fuselageStateInit( n_segments, geometry )
% 
% Inputs:
%   n_segments      number of segments for the discretized aerodynamics
%   geometry        fuselage geometry struct (see fuselageGeometryInit)
% 
% Outputs:
%   state           fuselage state struct as defined by this function
% 
% See also:
%   fuselageInit, fuselageCreate, fuselageSetState
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% body state struct
state.body = fuselageBodyStateInit();

% local inflow struct
state.aero.local_inflow	= fuselageLocalInflowInit(n_segments);
% unsteady aerodynamic angles
state.aero.unsteady.alpha_dt = state.aero.local_inflow.alpha(1:end-1);
state.aero.unsteady.beta_dt = state.aero.local_inflow.beta(1:end-1);
state.aero.unsteady.alpha   = state.aero.local_inflow.alpha(1:end-1);
state.aero.unsteady.beta    = state.aero.local_inflow.beta(1:end-1);
% aerodynamic force coefficients in body frame at each control point (3xS
% array, where S is the number of segments), dimensionless
state.aero.C_XYZ_b_i  	= zeros( 3, n_segments );
% aerodynamic forces in body frame at each control point (3xS array), in N
state.aero.R_Ab_i     	= zeros( 3, n_segments );
% total aerodynamic force coefficients in body frame (3x1 array),
% dimensionless
state.aero.C_XYZ_b      = zeros( 3, 1 );
% total force in body frame (3x1 array), in N
state.aero.R_Ab         = zeros( 3, 1 );
% total moment w.r.t. center of gravity (3x1 array), in Nm
state.aero.Q_Ab         = zeros( 3, 1 );

% fuselage geometry struct (see fuselageGeometryInit)
state.geometry = geometry;
% velocity of border points (3x(S+1) array), in m/s
state.geometry_deriv.border_pos_dt = geometry.border_pos;
state.geometry_deriv.border_pos_dt(:) = 0;
% acceleration of border points (3x(S+1) array), in m/s
% state.geometry_deriv.border_pos_dt2 = state.geometry_deriv.border_pos_dt;
% state.geometry_deriv.alpha_dt = zeros( 1, n_segments+1 );
% state.geometry_deriv.beta_dt = zeros( 1, n_segments+1 );

% fuselage external struct
state.external = fuselageExternalInit( n_segments );

end

