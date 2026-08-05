function apParAddTunability2Model(model)
% APPARADDTUNABILITY2MODEL provides a minimal guided configuration of a
%   Simulink model for code generation with tunability in ArduPilot.
% 
% Inputs:
%   model           Handle or path to a Simulink model root (e.g. 'bdroot()')
% 
% See also:
%   APPARPROCESSCODEEXPORT

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


%% Config
ap_pars = loadParams( 'apPar_params_dependent' );

model_reqs.param = {'IsERTTarget'; ...
                    'SystemTargetFile'; ...
                    'TargetLang'; ...
                    'DefaultParameterBehavior'};

model_reqs.value = {'on'; ...
                    'ert.tlc'; ...
                    'C++'; ...
                    'Inlined'};
                
model_params2change = {'TunableVars'; ...
                       'CustomHeaderCode'; ...
                       'CustomSourceCode'};
                

%% Check input
if nargin == 0 || ~bdIsRoot(model)
    error('MATLAB:apParAddTunability2Model:inputError', ...
          '\nPlease provide a path or a handle to a loaded top-level Simulink model as input (e.g. using ''bdroot()'').');
end


%% Display headline
fprintf('%s\n\n', boldWithFrame(sprintf('Configuring ''%s'' for code export with tunability in ArduPilot:', model)));


%% Check model
try
    checkModelReqs(model, model_reqs);
catch ME
    ME.throw;
end

if bdIsDirty(model)
    prompt = sprintf('The model already has unsaved changes. Do you really want to continue?');
    if ~getUserConfirmation(prompt)
        return
    end
    fprintf('\n');
end

if any(cellfun(@(a) ~isempty(get_param(model, a)), model_params2change))
    prompt = sprintf('The model already has non-empty configurations that will be overwritten by this tool. Do you really want to continue?');
    if ~getUserConfirmation(prompt)
        return
    end
    fprintf('\n');
end


%% Read variables from base workspace (dirty)
base_vars       = evalin('base', 'whos');
model_base_vars = Simulink.findVars(model, 'WorkspaceType', 'base');

% filter base_vars for model_base_vars
[~,ia,~] = intersect({base_vars.name}, {model_base_vars.Name}, 'stable');
base_vars = base_vars(ia);

% filter for supported variables (scalar structs)
idxs_bool_class = arrayfun(@(a) strcmp(a.class, 'struct'), base_vars);
idxs_bool_size  = arrayfun(@(a) prod(a.size) == 1, base_vars);
idxs_bool = idxs_bool_class & idxs_bool_size;
base_vars = base_vars(idxs_bool);

% exclude variables that contain 'notune' in variable names
idxs_bool_name = arrayfun(@(a) contains(a.name, 'notune'), base_vars);
idxs_bool = ~idxs_bool_name;
base_vars = base_vars(idxs_bool);

if isempty(base_vars)
   error('MATLAB:apParAddTunability2Model:ModelParameterError', ...
         'No supported variables (see also %s) found in the base workspace, please make sure that the model is fully initialized!', getLink2URL([ap_pars.readme_url '#supported-data-types'], 'README'));
end


%% Ask which workspace variables should be used for tunability
prompt = 'The following supported variables were found in the workspace. Please select the variables to be added to the model as tunable';
idxs_select = getUserSelection(prompt, {base_vars.name}.');


%% Write settings to model

% write tunable variables
if ~isempty(idxs_select)
    base_vars_select = base_vars(idxs_select);
    
    tunable_vars    = strjoin({base_vars_select.name}, ',');
    tunable_vars_sc = strjoin(repmat({'Auto'}, size(base_vars_select)), ',');
    tunable_vars_tq = repmat(',', 1, numel(base_vars_select)-1);
    
    set_param(model, 'TunableVars',              tunable_vars);
    set_param(model, 'TunableVarsStorageClass',  tunable_vars_sc);
    set_param(model, 'TunableVarsTypeQualifier', tunable_vars_tq);    
end

% write custom source / header file imports
set_param(model, 'CustomHeaderCode', '//custom_header_file_import_identifier');
set_param(model, 'CustomSourceCode', '//custom_source_file_import_identifier');


fprintf('\nSettings have been written to the model. Please make sure to save the model manually to make the changes permanent!\n')

end
