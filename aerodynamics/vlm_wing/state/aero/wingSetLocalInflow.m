function wing = wingSetLocalInflow( wing, xyz_cg )
% wingSetLocalInflow sets the wing.state.aero.local_inflow struct of a wing
% struct.
%   The local inflow for each control point is computed. The rotation of
%   the rigid body, the structural motion and the local wind are
%   considered.
% 
% Inputs:
% 	wing           	wing struct, see wingCreate
%   xyz_cg          vehicle center of gravity position in wing coordinate
%                   system (3x1 array), in m
% 
% Outputs:
% 	wing           	wing struct, see wingCreate
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% local airspeed from rotation
r_cntrl_cg = wing.state.geometry.ctrl_pt.pos ...
    - repmat( xyz_cg, 1, size(wing.state.geometry.ctrl_pt.pos,2) );
V_Ab = dcmBaFromAeroAngles( wing.state.body.alpha, wing.state.body.beta ) ...
    * [ wing.state.body.V; 0; 0 ];

% total local airspeed including local wind (external) and structure motion
wing.state.aero.local_inflow.V = velocityFromRot( V_Ab, wing.state.body.omega, r_cntrl_cg ) ...
    + wing.state.geometry.ctrl_pt_dt.pos + wing.state.external.V_Wb;

% local aerodynamic angles
[ wing.state.aero.local_inflow.alpha, wing.state.aero.local_inflow.beta ] = ...
    aeroAngles( wing.state.aero.local_inflow.V );

% local airspeed time derivative
wing.state.aero.local_inflow.V_dt = wing.state.external.V_Wb_dt ...
    + velocityFromRot( wing.state.body.V_Kb_dt, wing.state.body.omega_dt, r_cntrl_cg ) ...
    + wing.state.geometry.ctrl_pt_dt2.pos;

end