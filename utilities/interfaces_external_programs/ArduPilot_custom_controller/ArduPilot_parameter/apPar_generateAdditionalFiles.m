function var_infos = apPar_generateAdditionalFiles(ap_pars, code_pars, tunable_vars_proc)
% APPAR_GENERATEADDITIONALFILES

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


%% Read structs and typedef structs from the header file
[cpp.structs, cpp.tstructs] = apPar_getStructsFromHeaderFile(code_pars.header_file);
struct_defs = mergeStructDefs(cpp.structs, cpp.tstructs);



%% Try to find and resolve structs for tunable variables
struct_overrides = struct;

var_names   = fieldnames(tunable_vars_proc);
top_structs = cell(size(var_names));

for idx = 1:numel(var_names)
    tvar_name = var_names{idx};
    
    [fields, ~, ~, values] = unpackStruct(struct_defs, 'tln', false);
    
    % Search for tunable variable name in struct definitions
    idxt = find(endsWith(fields, tvar_name));
    if ~isscalar(idxt)
        error('Could not find tunable variable name ''%s'' in parsed struct definitions', tvar_name);
    end
    sub_typedef     = getfieldNested(struct_defs, fields{idxt});
    
    if ~strcmp(fields{idxt}, tvar_name)
        top_struct_name = strrep(fields{idxt}, ['.' tvar_name], '');
    end
    
    if isempty(top_struct_name)
        error('Should not happen, please check!')
    end
    
    % Reverse search if a typedef exists for the top struct name
    if ~isempty(top_struct_name)
        while true
            idxt = find(arrayfun(@(a) strcmp(a.type, top_struct_name), values));
            
            if isscalar(idxt)
                % if an typedef exists for the top struct name, override the
                % name because the typedef is used in the code
                top_struct_name = fields{idxt};
                if contains(top_struct_name, '.')
                    error('')
                end
            elseif isempty(idxt)
                break;
            else
                error('Should not happen, please check!');
            end
        end
    end
    top_structs{idx} = top_struct_name;
    
    
    
    todo = {sub_typedef.type};
    
    while ~isempty(todo)
        sub_struct_name = todo{1};
        todo(1) = [];
        
        % Lookup sub struct
        if ~isfield(struct_defs, sub_struct_name)
            % Must be final type, need to check for that
            error()
        end
        
        if isa(struct_defs.(sub_struct_name), 'apPar_TypeDef')
            error()
        end
        
        fnames = fieldnames(struct_defs.(sub_struct_name));
        for idx4 = 1:numel(fnames)
            var_name = fnames{idx4};
            var = struct_defs.(sub_struct_name).(var_name);
            
            if isfield(struct_defs, var.type)
                todo = [todo, {var.type}];
                var_type = var.type;
            else
                try
                    var_type = ap_pars.type_translation.sl2ap(var.type);
                catch ME
                    if strcmp(ME.identifier, 'MATLAB:Containers:Map:NoKey')
                        if ~ap_pars.use_variable_name
                            var_name_full = [tvar_name '.' var_name];
                        else
                            var_name_full = var_name;
                        end
                        err_msg = sprintf('\nIn ''%s'' data type ''%s'' is unsupported!', var_name_full, var.type);
                        catg = sprintf('Model tunable parameter error');
                        hint = sprintf('Please have a look at the %s for further details.', ...
                            getLink2URL([ap_pars.readme_url '#technical-background'], 'README'));
                        error('MATLAB:apPar_generateAdditionalFiles:ModelParameterError', '\n%s:\n%s\n%s', catg, err_msg, hint);
                    else
                        ME.rethrow;
                    end
                end
            end
            struct_overrides.(sub_struct_name).(var_name).type = var_type;
            struct_overrides.(sub_struct_name).(var_name).size = var.size;
        end
    end
end



%% Search for variable declarations of type top_struct in header file
[C,~,ic] = unique(top_structs);
var_decs = getVarDecFromFile(code_pars.header_file, C, 'once');
top_vars = var_decs(ic);



%% Generate struct override
apPar_writeStructOverrideFile(code_pars, struct_overrides);



%% Generate AP param files
var_infos = apPar_writeModelAPParamFile(ap_pars, code_pars, tunable_vars_proc, top_vars);
apPar_writeInterfaceAPParamFile(ap_pars, code_pars);



%% Generate parameter files for Ground Control Stations
apPar_writeQGCParFile(ap_pars, code_pars, tunable_vars_proc);

end





%% LOCAL FUNCTIONS
function merged_struct = mergeStructDefs(a, b)
    
    in_structs = {a; b};
    
    a_fnames = fieldnames(a);
    b_fnames = fieldnames(b);
    
    % Check for duplicates
    f_duplicates      = intersect(a_fnames, b_fnames);
    f_duplicates_elim = false(size(f_duplicates));
    
    for idx = 1:numel(f_duplicates)
        f = f_duplicates{idx};
        
        for idx2 = 1:numel(in_structs)            
            in_struct = in_structs{idx2};
            
            if isa(in_struct.(f), 'apPar_TypeDef')
                if strcmp(in_struct.(f).type, f) && in_struct.(f).size == 1
                    in_structs{idx2} = rmfield(in_struct, f);
                    f_duplicates_elim(idx) = true;
                end
            end
        end
    end
    
    if ~all(f_duplicates_elim)
        error('Could not eliminate duplicate structure definitions!');
    end
    
    
    % Taken from: https://de.mathworks.com/matlabcentral/answers/96973-how-can-i-concatenate-or-merge-two-structures#answer_401067
    mergestructs = @(x,y) cell2struct([struct2cell(x);struct2cell(y)],[fieldnames(x);fieldnames(y)]);
    
    merged_struct = mergestructs(in_structs{1}, in_structs{2});
    
end


function var_matches = getVarDecFromFile(file, var_type, varargin)
    
    if ~iscell(var_type)
        var_type = {var_type};
    end
    
    fid = fopen(file,'r');
    f = fread(fid, '*char')';
    fclose(fid);
    
    var_matches = cell(size(var_type));
    for idx = 1:numel(var_type)
        % This only finds a single variable declaration in one line, e.g. '  double a;'
        var_matches{idx} = regexp(f, ['(?<=\n\s*' regexptranslate('escape', var_type{idx}) '\s*)[a-zA-Z_0-9_\[\]]+(?=;\n)'], 'match', varargin{:});
    end
    
end
