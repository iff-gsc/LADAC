function apParProcessCodeExport(varargin)
% APPARPROCESSCODEEXPORT
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
% - Add more (optional) input arguments, e.g.
%   - p.addOptional('CodeExportDir')
%   - p.addOptional('TunableVars')
%   - p.addOptional('Model')


%% Input Parser
p = inputParser;
p.addRequired('apPar_params_file', @(a) exist(a, 'file'));
p.parse(varargin{:});

apPar_params_file = p.Results.apPar_params_file;


%% Config
ap_pars = loadParams( apPar_params_file );
header_file_ext = 'h';      % hardcoded is not nice
source_file_ext = 'cpp';    % hardcoded is not nice


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
fprintf('%s\n\n', boldWithFrame(sprintf('Post-processing ''%s''s'' code export for ArduPilot tunability:', bd)));


%% Check model
try
    checkModelReqs(bd, 'apPar_modelReqs_default');
catch ME
    if strcmp(ME.identifier, 'MATLAB:checkModelReqs:requirementsFailed')
        hint = sprintf('Please have a look at %s for further details.', ...
                        getLink2File('apPar_modelReqs_default'));
        ME = MException(ME.identifier, '%s\n%s', ME.message, hint);
    end
    ME.throw;
end


%% Query required model settings
model_pars.name                      = get_param(bd, 'Name');
model_pars.version                   = get_param(bd, 'ModelVersion');
model_pars.tunable_vars              = get_param(bd, 'TunableVars');
model_pars.custom_header_code        = get_param(bd, 'CustomHeaderCode');
model_pars.custom_source_code        = get_param(bd, 'CustomSourceCode');
model_pars.ert_header_file_root_name = get_param(bd, 'ERTHeaderFileRootName');
model_pars.ert_source_file_root_name = get_param(bd, 'ERTSourceFileRootName');
model_pars.ert_data_file_root_name   = get_param(bd, 'ERTDataFileRootName');
model_pars.indent_size               = get_param(bd, 'IndentSize');

cpp_class_interface                  = RTW.getClassInterfaceSpecification(bd);
model_pars.class_name                = getClassName(cpp_class_interface);

folder_struct                        = RTW.getBuildDir(bd);
model_pars.build_directory           = folder_struct.BuildDirectory;


%% Do processing of model settings
model_pars.tunable_vars = strsplit(model_pars.tunable_vars, ',')';

% see also: https://de.mathworks.com/help/ecoder/ug/specify-identifier-formats.html
model_pars.ert_header_file_root_name = regexprep(model_pars.ert_header_file_root_name, '\$R', bd);
model_pars.ert_source_file_root_name = regexprep(model_pars.ert_source_file_root_name, '\$R', bd);

model_pars.ert_header_file_root_name = regexprep(model_pars.ert_header_file_root_name, '\$E', '');
model_pars.ert_source_file_root_name = regexprep(model_pars.ert_source_file_root_name, '\$E', '');

if contains(model_pars.ert_header_file_root_name, '$') || ...
   contains(model_pars.ert_source_file_root_name, '$')
    error('Cannot parse code export filenames from model settings. Please implement unknown identifiers!');
end


%% Check if build directory and code files exist
err_msg = char.empty;

if ~isfolder(model_pars.build_directory)
    msg = sprintf(' - %s\n', 'build directory does not exist');
    err_msg = [err_msg, msg];
end

% expected code export source and header file paths
code_pars.header_file = fullfile(model_pars.build_directory, [model_pars.ert_header_file_root_name '.' header_file_ext]);
code_pars.source_file = fullfile(model_pars.build_directory, [model_pars.ert_source_file_root_name '.' source_file_ext]);

if isempty(err_msg)
%     header_files = dir(fullfile(model_par.build_directory, '*.h'));
%     source_files = dir(fullfile(model_par.build_directory, '*.cpp'));
    
    files = fieldnames(code_pars);
    for idx = 1:numel(files)
        file_full = (code_pars.(files{idx}));
        file = getFileFromPath(file_full);
        
        if ~isfile(file_full)
            msg = sprintf(' - %s ''%s'' not found in build directory\n', strrep(files{idx}, '_', ' '), file);
            err_msg = [err_msg, msg];
        end
    end
end

if ~isempty(err_msg)
    catg = sprintf('Model build directory is not correctly recognized');
    hint = sprintf('Have you already run code generation (Build Model)?');
    error('MATLAB:apParProcessCodeExport:ModelBuildError', '\n%s:\n%s\n%s', catg, err_msg, hint);
end


%% Check if model and code export version match
% read version from header file
fid = fopen(code_pars.header_file, 'r');
f = fread(fid, '*char')';
fclose(fid);
code_pars.version = regexp(f, '(?<=\n//\s*Model version\s*\:\s*)[0-9.]+', 'match', 'once');

% compare versions
if ~strcmp(model_pars.version, code_pars.version)
    prompt = sprintf('Version mismatch (model: %s / code: %s). Do you want to proceed?', model_pars.version, code_pars.version);
    if ~getUserConfirmation(prompt)
        return
    end
    fprintf('\n');
end


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
    error('MATLAB:apParProcessCodeExport:ModelParameterError', '\n%s:\n%s\n%s', catg, err_msg, hint);
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
    error('MATLAB:apParProcessCodeExport:ModelParameterError', '\n%s:\n%s\n%s', catg, err_msg, hint);
end


%% Configure remaining code_pars
% additional ArduPilot files
code_pars.struct_override_file    = fullfile(model_pars.build_directory, [model_pars.ert_header_file_root_name 'StructOverride.' header_file_ext]);
code_pars.model_ap_param_file     = fullfile(model_pars.build_directory, [model_pars.ert_source_file_root_name 'Params.' source_file_ext]);
code_pars.interface_ap_param_file = fullfile(model_pars.build_directory, [ap_pars.interface.class_name 'Params.' source_file_ext]);

% ground control station parameter files
code_pars.par_file_header = apPar_generateParFileHeader(model_pars, code_pars);
code_pars.qgc_par_file    = fullfile(model_pars.build_directory, 'QGC_Params');

% model_pars depending settings
code_pars.indent             = repmat(' ', 1, str2double(model_pars.indent_size));
code_pars.class_name         = model_pars.class_name;
code_pars.custom_source_code = model_pars.custom_source_code;
code_pars.custom_header_code = model_pars.custom_header_code;


%% Run code export processing
tunable_vars_proc = apPar_processTunableVars(ap_pars, tunable_vars);
var_infos         = apPar_generateAdditionalFiles(ap_pars, code_pars, tunable_vars_proc);
apPar_modifyCodeExportFiles(ap_pars, code_pars, var_infos);

end
