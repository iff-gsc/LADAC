function wing = wingCreate( params_file, n_panel, varargin )
% wingCreate generates a wing struct from a paramters file and additional
%   parameters and options.
% 
% Inputs:
%   params_file    	file name of the parameters file (string), see
%                  	wing_params_default.m
%   n_panel        	number of spanwise panels (scalar)
%   flag            Indicates that the next input is a specific variable
%                   that can be passed optionally:
%                       'spacing'           Next variable is spacing.
%                       'is_unsteady'       Next variable is is_unsteady.
%                       'is_wagner'         Next variable is is_wagner.
%                       'flexible'          Next variable is structure.
%                       'is_infl_recomputed'Next variable is
%                                           is_infl_recomputed.
%                       'is_elliptical'     Next variable is is_elliptical.
% 
%   spacing             there are the following options for the spacing of
%                       the panels:
%                           'like_chord'    Relation of panel chord and
%                                           panel span is equal for all
%                                           panels (default).
%                           'constant'      All panel spans are equal.
%   is_unsteady    	boolean that defines if unsteady computation (true) or
%                   steady computation (false) is required.
%                   Default is false.
%   is_wagner       boolean that defines if the approximated Wagner
%                   function is used for unsteady airfoil aerodynamics
%                   (suitable for low Mach numbers)
%   structure     	a structure struct (see structureCreateFromNastran)
%                  	that must have nodes at similar positions at the
%                  	configured wing (else the coupling will not work).
%                  	This will enable flexible/aeroelastic computations.
%                  	By default the wing is rigid.
%   is_infl_recomputed  boolean that defines if the influence coefficients
%                  	are recomputed (true) or only adjusted for different
%                   sideslip angles (false). Default is false.
%   is_elliptical  	boolean that defines if the wing geometry is
%                  	elliptical (true) or not (false). Default is false.
% 
% Outputs:
%   wing          	wing struct as defined by this function
% 
% Syntax:
%   wing = wingCreate( params_file, n_panel )
%   wing = wingCreate( params_file, n_panel, 'spacing', spacing )
%   wing = wingCreate( params_file, n_panel, 'is_unsteady', is_unsteady )
%   wing = wingCreate( params_file, n_panel, 'flexible', structure )
%   wing = wingCreate( params_file, n_panel, 'is_infl_recomputed', is_infl_recomputed )
%   wing = wingCreate( params_file, n_panel, 'is_elliptical', is_elliptical )
% 
% Example:
%   wing = wingCreate( 'wing_params_default', 50 )
% 
% See also:
%   wingPlotGeometry, wingSetState
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% default parameters
is_unsteady = 0;
is_wagner   = 0;
is_flexible = 0;
is_stall    = 1;
is_le_shock = 0;
spacing = 'like_chord';
is_infl_recomputed = 0;
is_elliptical = false;
Ma = 0;

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
        case 'is_wagner'
            if islogical(varargin{i+1})
                is_wagner = varargin{i+1};
            else
                error('Invalid option for parameter is_wagner.')
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
                error('Invalid option for parameter is_stall.');
            end
        case 'le_shock'
            if islogical(varargin{i+1})
                is_le_shock = varargin{i+1};
            else
                error('Invalid option for parameter le_shock.');
            end
        case 'is_infl_recomputed'
            if islogical(varargin{i+1})
                is_infl_recomputed = varargin{i+1};
            else
                error('Invalid option for parameter is_infl_recomputed.')
            end
        case 'is_elliptical'
            if islogical(varargin{i+1})
                is_elliptical = varargin{i+1};
            else
                error('Invalid option for parameter is_elliptical.')
            end
        case 'Mach'
            Ma = varargin{i+1};
    end
end


%% define aircraft parameters

% load wing parameters
if ischar(params_file)
    wing.params = wingLoadParameters( params_file );
elseif isstruct(params_file)
    prm = params_file;
    wing.params = wingSetParams(prm);
else
    error('Parameters not specified correctly.')
end

%% compute geometry

wing.n_panel = n_panel;
wing.geometry = wingSetGeometry( wing.params, wing.n_panel, ...
    'is_elliptical', is_elliptical, 'spacing', spacing );


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

%% wake
% wing.wake = wingCreateWake( wing.params, wing.geometry, n_trail );

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
wing.config.is_wagner   = double(is_wagner);
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
wing.interim_results = wingSetInterimResults( wing, Ma );

%% set custom actuator
custom_path = which('wingCustomActuator','-all');
if length(custom_path) > 1
    for i = 1:length(custom_path)
        if contains(custom_path{i},'/ladac/') || contains(custom_path{i},'/LADAC/')
            rmpath(fileparts(custom_path{i}));
        end
    end
end

end