function body = rigidBodyCreate(mass, inertia, varargin)
% create rigid body struct and bus object

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if ~isempty(varargin)
    busName = varargin{1};
else
    busName = 'body';
end
 
% mass and inertia
body.m = mass;
body.I = inertia;

% linear and angular accelerations
body.V_Kb_dt = zeros(3,1);
body.omega_Kb_dt = zeros(3,1);
% linear and angular velocities
body.V_Kb = zeros(3,1);
body.V_Kg = zeros(3,1);
body.s_Kg_dt = zeros(3,1);
body.omega_Kb = zeros(3,1);
body.q_bg_dt = zeros(4,1);
% position and attitude
body.s_Kg = zeros(3,1);
body.q_bg = [1;0;0;0];
body.EulerAngles = zeros(3,1);
body.M_bg = zeros(3,3);

% bus object
struct2slbus( body, busName );

end

