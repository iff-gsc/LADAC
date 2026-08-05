function apParGenerateQGCParFile(varargin)
% APPARGENERATEQGCPARFILE
% 
% Inputs:
%   apPar_params_file   Name of the apPar configuration parameters file

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


% ToDo:
% - This file contains copied sections from apParProcessCodeExport.
%   -> Combine both into one solution?


%% Input Parser
p = inputParser;
p.addRequired('apPar_params_file', @(a) exist(a, 'file'));
p.parse(varargin{:});

apPar_params_file = p.Results.apPar_params_file;


%% Config
ap_pars = loadParams( apPar_params_file );
time_stamp = datetime;


%% Select opened and active model
bds_open = Simulink.allBlockDiagrams('model');
bd = bdroot;
bd_link = getLink2bdroot(bd);

if isempty(bds_open)
    error('\n%s', 'No opened Simulink models found. Do not close the Simulink model after model initialization.')
end

if numel(bds_open) > 1
    prompt = sprintf('\nMore than one opened model found. %s is the active model, do you want to proceed?', bd_link);
    if ~getUserConfirmation(prompt)
        return
    end
    fprintf('\n');
end


%% Display headline
fprintf('%s\n\n', boldWithFrame(sprintf('Generating ArduPilot parameters file for ''%s'':', bd)));


%% Query required model settings
model_pars.name                      = get_param(bd, 'Name');
model_pars.version                   = get_param(bd, 'ModelVersion');
model_pars.tunable_vars              = get_param(bd, 'TunableVars');

folder_struct                        = RTW.getBuildDir(bd);
model_pars.build_directory           = folder_struct.BuildDirectory;


%% Do processing of model settings
model_pars.tunable_vars = strsplit(model_pars.tunable_vars, ',')';


%% Load tunable parameters of model from base workspace (dirty)
tunable_vars = struct;
err_msg = [];
for idx = 1:numel(model_pars.tunable_vars)
    var_name = model_pars.tunable_vars{idx};
    if ~evalin('base', ['exist(''' var_name ''', ''var'')'])
        msg = sprintf(' - ''%s'' is not loaded into the base workspace\n', var_name);
        err_msg = [err_msg, msg];
    else
        tunable_vars.(var_name) = evalin('base', var_name);
    end
end

if ~isempty(err_msg)
    catg = sprintf('Model tunable parameters not loaded');
    hint = sprintf('Have you run the model init script?');
    error('MATLAB:apParGenerateQGCParFile:ModelParameterError', '\n%s:\n%s\n%s', catg, err_msg, hint);
end


%% Check if the tunable variables are of supported type
%   ToDo: right now, only scalar structs are supported
tunable_vars_type_check = structfun(@(a) isstruct(a) && isscalar(a), tunable_vars);
var_names = fieldnames(tunable_vars);
err_msg = [];
for idx_var = 1:numel(var_names)
    if ~tunable_vars_type_check(idx_var)
        msg = sprintf(' - tunable variable ''%s'' has unsupported type\n', var_names{idx_var});
        err_msg = [err_msg, msg];
    end
end

if ~isempty(err_msg)
    catg = sprintf('Model tunable parameter error');
    hint = sprintf('Please have a look at the %s for further details.', ...
        getLink2URL([ap_pars.readme_url '#supported-data-types'], 'README'));
    error('MATLAB:apParGenerateQGCParFile:ModelParameterError', '\n%s:\n%s\n%s', catg, err_msg, hint);
end


%% Configure remaining code_pars
code_pars.version = "unknown";

code_pars.par_file_header = apPar_generateParFileHeader(model_pars, code_pars, 'time_stamp', datestr(time_stamp, 'dd-mm-yyyy HH:MM:SS'));

% parameters file export path
filename = [datestr(time_stamp, 'yyyymmdd_HH-MM-SS') '_QGC_Params'];
if ~isfolder(model_pars.build_directory)
    fprintf('Build directory for model does not exist, saving the parameters file ''%s'' to the current working directory.\n', filename);
    export_path = pwd;
else
    fprintf('Saving the parameters file ''%s'' to the build directory of the model.\n', filename);
    export_path = model_pars.build_directory;
end
code_pars.qgc_par_file    = fullfile(export_path, [datestr(time_stamp, 'yyyymmdd_HH-MM-SS') '_QGC_Params']);


%% Generate parameters file for QGroundControl
tunable_vars_proc = apPar_processTunableVars(ap_pars, tunable_vars);
apPar_writeQGCParFile(ap_pars, code_pars, tunable_vars_proc);


end
