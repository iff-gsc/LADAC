% Example for FlexiFlightVis Simulink library block

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2023 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

clear all

% init wing and fuselage structs
wing_1 = wingCreate('wing_params_default',20);
wing_2 = wingCreate(wing_parametric(5,0.5,0.1,0),10);
fuselage = fuselageCreate('fuselage_params_default',5, 10);

% pick wing and fuselage state structs
wing_1_state = wing_1.state;
wing_2_state = wing_2.state;
fuselage_state = fuselage.state;

% create config and rigid body struct
config.xyz_ref_c = [-10;0;0];
rigid_body = rigidBodyCreate();

% create bus objects from structs
struct2bus(wing_1_state,'wing_1_state_bus');
struct2bus(wing_2_state,'wing_2_state_bus');
struct2bus(fuselage_state,'fuselage_state_bus');
struct2bus(config,'config_bus');
struct2bus(rigid_body,'rigid_body_bus');

% adjust positions so that it looks a bit like an airplane
wing_1_state.geometry.origin = [-6;0;0];
wing_2_state.geometry.origin=[-12;0;0];
config.xyz_ref_c = [-10;0;0];

% open Simulink example file
open('flexiFlightVis_lib_example')

