function tunable_vars_proc = apPar_processTunableVars(ap_pars, tunable_vars)
% APPAR_PROCESSTUNABLEVARS

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


tunable_vars_proc = struct();

var_names = fieldnames(tunable_vars);

for idx_var = 1:numel(var_names)
    var_name = var_names{idx_var};
    
    %% Unpack tunable parameter struct
    if ap_pars.use_variable_name
        tln_name = var_name;
    else
        tln_name = char.empty;
    end
    unpkd_struct = struct;
    [unpkd_struct.fields, unpkd_struct.classes, unpkd_struct.sizes, unpkd_struct.values] = unpackStruct(tunable_vars.(var_name), 'tln_name', tln_name);
    
    
    %% Check parameter dimensions
    %  only 1D arrays are allowed
    idxs_bool_ = ~cellfun(@(a) checkParSize(a), unpkd_struct.sizes);
    if any(idxs_bool_)
        sizes_str = cellfun(@(a) num2str(a), unpkd_struct.sizes, 'UniformOutput', false);
        title = sprintf('\nWrong parameter dimensions:\n\n');
        detail = printData({'Parameter','Dimensions'}, [unpkd_struct.fields(idxs_bool_), sizes_str(idxs_bool_)], 'indent', 4, 'column_spacing', 2, 'headerstyle', 'bold');
        error([title detail]);
    end
    
    
    
    %% # Convert paramter names
    
    %% ## Underscores
    %   Underscores don't carry much information and are wasting our
    %   available name length space. Additionaly, they should be usable as
    %   delimiter and therefore removed from variable names.
    flds = unpkd_struct.fields;
    
    idxs = find(contains(flds, '_'));
    found_underscore = any(idxs);
    
    flds = flds(idxs);
    
    % AutoTransform 1: test_var -> testVar (lowercase letters connected with underscore)
    flds_s = regexprep(flds, '[a-z]_[a-z]', '${[lower($0(1)) upper($0(end))]}');
    
    % AutoTransform 2: W_v: (uppercase letter with lowercase indice)
    %  ToDo: Make this a more precise match?
    flds_s = regexprep(flds_s, '[A-Z]_[a-z]', '${[$0(1) $0(end)]}');
    
    % Check if there are any remaining underscores, if the underscore is
    % also used as a delimiter
    if strcmp(ap_pars.delimiter, '_') || strcmp(ap_pars.array_delimiter)
        idxs_ = contains(flds_s, '_');
        check_underscore = any(idxs_);
        if ap_pars.debug && found_underscore && ~check_underscore
            fprintf('Automatic parameter underscore conversion for ''%s'':\n\n', var_name);
            printData({'Original','Converted'}, [flds, flds_s], 'indent', 4, 'column_spacing', 2, 'headerstyle', 'bold');
            fprintf('\n');
        elseif check_underscore
            title = fprintf('\nAutomatic parameter underscore conversion for ''%s'' failed:\n\n', var_name);
            detail = printData({'Original','Converted'}, [flds(idxs_), flds_s(idxs_)], 'indent', 4, 'column_spacing', 2, 'headerstyle', 'bold');
            error([title detail]);
        end
    end
    
    flds_processed = unpkd_struct.fields;
    flds_processed(idxs) = flds_s;
    
    
    %% ## Name length
    %   The maximum length of a parameter name is limited in ArduPilot.
    %   Therefore, the names must be checked and shortened.
    flds = flds_processed;
    max_len = ap_pars.name.max_len - numel(ap_pars.name.prefix);
    
    % calculate length penalties due to array indices
    flds_size = cellfun(@(a) prod(a), unpkd_struct.sizes);
    idxs_bool = flds_size>1;
    
    flds_penalty = flds_size(idxs_bool);
    flds_penalty = arrayfun(@(a) length(num2str(a)), flds_penalty);
    flds_penalty = flds_penalty + length(ap_pars.array_delimiter);
    
    len_penalties = zeros(size(flds));
    len_penalties(idxs_bool) = flds_penalty;
    
    max_len_per_fld = max_len - len_penalties;
    
    % shorten field names
    [flds_s, idxs_s] = apPar_shortenFieldpaths(flds, max_len_per_fld);
    
    % check if any fields are too long
    flds_s_length = cellfun(@(a) length(a), flds_s);
    idxs_ = flds_s_length > max_len_per_fld;
    if any(idxs_)
        title = sprintf('\nAutomatic parameter name shortening for ''%s'' failed:\n\n', var_name);
        detail = printData({'Original','Converted'}, [flds(idxs_), flds_s(idxs_)], 'indent', 4, 'column_spacing', 2, 'headerstyle', 'bold');
        error([title detail]);
    elseif ap_pars.debug && ~isempty(idxs_s)
        fprintf('Automatic parameter name shortening for ''%s'':\n\n', var_name);
        printData({'Original','Shortened'}, [flds(idxs_s), flds_s(idxs_s)], 'indent', 4, 'column_spacing', 2, 'headerstyle', 'bold');
        fprintf('\n');
    end
    
    tunable_vars_proc.(var_name)          = unpkd_struct;
    tunable_vars_proc.(var_name).fields_s = flds_s;
    tunable_vars_proc.(var_name).sizes_l  = flds_size;
end



%% Check for duplicates in shortened names

% merge shortened names of all tunable vars
fields_s = {};
for idx_var = 1:numel(var_names)
    var_name = var_names{idx_var};
    fields_s = [fields_s; tunable_vars_proc.(var_name).fields_s];
end

% check for duplicates
[C, ia] = unique(fields_s, 'stable');

if numel(fields_s) ~= numel(C)
    duplicate_idxs = setdiff( 1:numel(fields_s), ia );
    
    title = sprintf('Found duplicates in shortened names');
    err_msg = sprintf('    %s\n', fields_s{duplicate_idxs});
    hint  = sprintf('Please have a look at the README for further details.');
    error('MATLAB:apPar_processTunableVars:detectedDuplicates', ...
          '\n%s:\n%s\n%s', title, err_msg, hint);
end

end





%% LOCAL FUNCTIONS
function res = checkParSize(par_size)
% CHECKPARSIZE Checks if the dimensions of a parameter fulfill the
% requirements for processing
%
% good dimensions  = true
% wrong dimensions = false
% 
% Requirements:
% - scalar or 1D array
    
    if numel(size(par_size)) > 2 || all(par_size ~= 1)
        res = false;
    else
        res = true;
    end
end
