function wing = wingCreateWithCPACS( tiglHandle, wing_idx, n_panel, varargin )

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
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
if wing_idx == 1
    controls_filename = 'wingControls_params_mainDefault';
elseif wing_idx == 2
    controls_filename = 'wingControls_params_htpDefault';
elseif wing_idx == 3
    controls_filename = 'wingControls_params_vtpDefault';
end

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
    end
end


%% define aircraft parameters

% load wing parameters
prm = wingGetParamsFromCPACS( tiglHandle, wing_idx, controls_filename );

% set further wing parameters
wing.params = wingSetParams(prm);


%% compute geometry

wing.n_panel = n_panel;
wing.geometry = wingSetGeometryCoord( wing.params, wing.n_panel, spacing );


%% init state
wing.state = wingCreateState( wing.params.num_actuators, n_panel, wing.geometry );


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
wing.interim_results = wingSetInterimResults( wing );

end