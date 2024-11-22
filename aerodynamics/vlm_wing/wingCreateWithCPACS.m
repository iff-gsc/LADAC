function wing = wingCreateWithCPACS( tiglHandle, wing_idx, n_panel, n_panel_x, varargin )

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% default parameters
is_unsteady = 0;
is_flexible = 0;
is_stall    = 1;
is_le_shock = 0;
spacing = 'like_chord';
is_infl_recomputed = 0;
method = 'IVLM';
n_trail = 1;
Ma = 0;
alt = 0;
Delta_jig_twist_data = [linspace(-1,1,n_panel);zeros(1,n_panel)];
if wing_idx == 1
    controls_filename = 'wingControls_params_mainDefault';
elseif wing_idx == 2
    controls_filename = 'wingControls_params_htpDefault';
elseif wing_idx == 3
    controls_filename = 'wingControls_params_vtpDefault';
end
scale = 1;

% set user parameters
for i = 1:length(varargin)
    if ~ischar(varargin{i})
        continue;
    end
    switch varargin{i}
        case 'spacing'
            if strcmp(varargin{i+1},'constant') || strcmp(varargin{i+1}, 'like_chord')
                spacing = varargin{i+1};
            else
                error('Invalid option for parameter spacing.')
            end
        case 'is_unsteady'
            if islogical(varargin{i+1})
                is_unsteady = varargin{i+1};
            else
                error('Invalid option for parameter is_unsteady.')
            end
        case 'flexible'
            if isstruct(varargin{i+1})
                structure = varargin{i+1};
                is_modal = 0;
                if isfield(structure,'modal')
                    is_modal = 1;
                end
                is_flexible = 1;
            else
                error('Invalid option for parameter flexible.')
            end
        case 'stall'
            if islogical(varargin{i+1})
                is_stall = varargin{i+1};
            else
                error('Invalid option for parameter is_stall.')
            end
        case 'le_shock'
            if islogical(varargin{i+1})
                is_le_shock = varargin{i+1};
            else
                error('Invalid option for parameter le_shock.')
            end
        case 'is_infl_recomputed'
            if islogical(varargin{i+1})
                is_infl_recomputed = varargin{i+1};
            else
                error('Invalid option for parameter is_infl_recomputed.')
            end
        case 'controlsdef'
            if ischar(varargin{i+1})
                controls_filename = varargin{i+1};
            else
                error('Invalid option for parameter control_filename.');
            end
        case 'DLM'
            method = 'DLM';
            n_trail = varargin{i+1};
        case 'Mach'
            Ma = varargin{i+1};
        case 'Alt'
            alt = varargin{i+1};
        case 'AdjustJigTwist'
            Delta_jig_twist_data(:) = varargin{i+1};
        case 'Scale'
            scale = varargin{i+1};
    end
end


%% define aircraft parameters

% load wing parameters
prm = wingGetParamsFromCPACS( tiglHandle, wing_idx, controls_filename );

prm.b = prm.b * scale;
prm.c = prm.c * scale;
prm.S = prm.S * scale^2;
prm.x = prm.x * scale;
prm.z = prm.z * scale;
prm.xyz_25 = prm.xyz_25 * scale;
prm.xyz_75 = prm.xyz_75 * scale;

% set further wing parameters
wing.params = wingSetParams(prm);


%% compute geometry

wing.n_panel = n_panel;
wing.n_panel_x = n_panel_x;
wing.n_trail = n_trail;
wing.geometry = wingSetGeometryCoord( wing.params, wing.n_panel, spacing );

Delta_jig_twist = interp1( Delta_jig_twist_data(1,:), Delta_jig_twist_data(2,:), ...
    wing.geometry.ctrl_pt.pos(2,:), 'linear', 'extrap' );
wing.geometry.ctrl_pt.local_incidence(:) = ...
        wing.geometry.ctrl_pt.local_incidence + Delta_jig_twist;

cntrl_prm = loadParams(controls_filename);
if contains(cntrl_prm.flap_mode,'everywhere')
    flap_mode_split = strsplit(cntrl_prm.flap_mode,'-');
    rel_border = str2double(flap_mode_split{2});
    is_flap = abs(wing.geometry.ctrl_pt.pos(2,:)/wing.params.b*2) > rel_border;
    num_flaps = sum( is_flap );
    flap_idx_min = 1;
    wing.geometry.segments.control_input_index_local(1,is_flap) = ...
        flap_idx_min:(num_flaps+flap_idx_min-1);
    wing.geometry.segments.control_input_index_local(1,~is_flap) = 0;
    wing.geometry.segments.flap_depth(1,~is_flap) = 0;
    wing.params.num_flaps = num_flaps;
    wing.params.num_actuators = wing.params.num_flaps;
else
    num_flaps = max(cntrl_prm.control_input_index(1,:));
end
if contains(cntrl_prm.lad_mode,'everywhere')
    lad_mode_split = strsplit(cntrl_prm.lad_mode,'-');
    rel_border = str2double(lad_mode_split{2});
    is_lad = abs(wing.geometry.ctrl_pt.pos(2,:)/wing.params.b*2) > rel_border;
    num_lads = sum( is_lad );
    lad_idx_min = max(min(wing.geometry.segments.control_input_index_local(2,:)),num_flaps+1);
    wing.geometry.segments.control_input_index_local(2,is_lad) = ...
        lad_idx_min:(num_lads+lad_idx_min-1);
    wing.geometry.segments.control_input_index_local(2,~is_lad) = 0;
    wing.params.num_lads = num_lads;
    wing.params.num_actuators = wing.params.num_flaps + wing.params.num_lads;
end

%% init state
wing.state = wingCreateState( wing.params.num_actuators, wing.n_panel, wing.geometry );
wing.state.external.atmosphere = isAtmosphere(alt);

%% set airfoil aerodynamics
wing.airfoil.simple = airfoilAnalyticSimpleInit();
if contains( wing.params.section(1,:), 'airfoilAnalytic0515' )
    wing.airfoil.analytic = airfoilAnalytic0515LoadParams(wing.params.section(1,:));
    airfoil_method = 'analytic';
elseif contains( wing.params.section(1,:), 'airfoilAnalyticSimple' )
    wing.airfoil.analytic = airfoilAnalytic0515LoadParams( 'airfoilAnalytic0515_params_empty' );
    wing.airfoil.simple = airfoilAnalyticSimpleLoadParams(wing.airfoil.simple,wing.params.section(1,:));
    airfoil_method = 'simple';
else
    error('airfoil section not specified correctly');
end

if strcmp( wing.params.actuator_2_type(1,:), 'none' )
    wing.airfoil.micro_tab = airfoilMicroTabLoadParams( 'airfoilMicroTab_params_empty' );
    actuator_2_type = 'none';
elseif contains( wing.params.actuator_2_type(1,:), 'airfoilMicroTab' )
    wing.airfoil.micro_tab = airfoilMicroTabLoadParams(wing.params.actuator_2_type(1,:));
    actuator_2_type = 'micro-tab';
elseif contains( wing.params.actuator_2_type(1,:), 'custom' )
    actuator_2_type = 'custom';
else
    error('second actuator type not specified correctly');
end


%% structure coupling
if is_flexible
    % structure nodes - wing nodes - coupling
    wing.aeroelasticity = ...
        wingSetAeroelasticity( wing.geometry, structure, is_modal );
else
    wing.aeroelasticity = wingCreateAeroelasticity( wing.n_panel, 1 );
end

%% configuration

wing.config.is_unsteady = double(is_unsteady);
wing.config.is_flexible = double(is_flexible);
wing.config.method      = method;
wing.config.is_stall    = double(is_stall);
wing.config.is_le_shock = double(is_le_shock);
if wing.config.is_unsteady
    wing.config.is_circulation_iteration = 0;
else
    wing.config.is_circulation_iteration = 1;
end
wing.config.airfoil_method = airfoil_method;
wing.config.actuator_2_type = actuator_2_type;
wing.config.is_infl_recomputed = is_infl_recomputed;

%% set interim results
wing.interim_results = wingSetInterimResults( wing, Ma );

%% set custom actuator
custom_path = which('wingCustomActuator','-all');
if length(custom_path) > 1
    custom_act_split = strsplit(wing.params.actuator_2_type(1,:),'-');
    custom_act_name = custom_act_split{end};
    for i = 1:length(custom_path)
        folder_names_split = strsplit(custom_path{i},{'/','\'});
        custom_folder_name = folder_names_split{end-1};
        if ~strcmp(custom_folder_name,custom_act_name)
            rmpath(fileparts(custom_path{i}));
        end
    end
end
custom_path = which('wingCustomActuator','-all');
if length(custom_path) > 1
    error('Custom actuator was not specified correctly.');
end
wing = wingCustomActuatorSetup(wing);

end