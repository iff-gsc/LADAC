
% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

geometry.vortex.x = [-0.5,-0.2,0,-0.2,-0.5];
geometry.vortex.y = [-1,-0.6,0,0.6,1];
geometry.vortex.z = [-0.1,-0.08,0,-0.08,-0.1];
geometry.vortex.c = [0.1,0.2,0.3,0.2,0.1];

geometry.cntrl_pt.local_incidence = [0,0,0,0];

aircraft.wing.geometry = geometry;

aircraftBus = struct2bus(aircraft);

[aircraftDoubleArray,aircraftDoubleIdx] = struct2double(aircraft,9999);
