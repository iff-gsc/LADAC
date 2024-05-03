function apPar_writeQGCParFile(ap_pars, code_pars, tunable_vars_proc)
% APPAR_WRITEQGCPARFILE generates a QGroundControl parameter file

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


% Config (centralized)
ml2qgc = ap_pars.type_translation.ml2qgc;
file   = code_pars.qgc_par_file;
header = code_pars.par_file_header;

% Config (QGroundControl specific)
vehicle_id        = 1;
component_id      = 1;
decimal_places    = 18;
line_comment_char = '#';
file_extension    = 'params';


% Check file extension
[~, ~, file_ext] = fileparts(code_pars.qgc_par_file);
if isempty(file_ext)
    file = [file '.' file_extension];
elseif ~strcmp(file_extension, file_ext)
    warning('For QGC parameter files a file extension of ''.%s'' is prefered (currently: ''.%s'')', file_extension, file_ext);
end


% Open file
fid = fopen(file, 'w');


% Write header
header = regexprep(header, '((?<=^).|(?<=\n).)', [line_comment_char ' $1']);    % transform to comment
fprintf(fid, '%s\n#\n', header);


% Write parameters
fprintf(fid, '# %s\n', 'Vehicle-Id Component-Id Name Value Type');    % headline

var_names = fieldnames(tunable_vars_proc);

for idx_var = 1:numel(var_names)
    var_name = var_names{idx_var};
    
    for idx = 1:numel(tunable_vars_proc.(var_name).fields_s)
        
        % Param name
        name = strrep(tunable_vars_proc.(var_name).fields_s{idx}, '.', ap_pars.delimiter);
        name = [ap_pars.name.prefix name];
        
        sz = tunable_vars_proc.(var_name).sizes_l(idx);
        
        if sz > 1
            % array
            val = tunable_vars_proc.(var_name).values{idx};
            for idx2 = 1:sz
                name_arr = [name ap_pars.array_delimiter num2str(idx2)];     % ToDo: if sz > 10 -> 01, 02, 03? AND CENTRALIZE!
                addParam(name_arr, val(idx2))
            end
        else
            % scalar
            addParam(name, tunable_vars_proc.(var_name).values{idx});
        end
    end
end


function addParam(name, value)
    
    try
        type = ml2qgc(class(value));
    catch ME
        if strcmp(ME.identifier, 'MATLAB:Containers:Map:NoKey')
            err_msg = sprintf(' - Param ''%s'' has unsupported type: ''%s''', name, class(value));
            catg = sprintf('Model tunable parameter error');
            hint = sprintf('Please have a look at the %s for further details.', ...
                getLink2URL([ap_pars.readme_url '#technical-background'], 'README'));
            error('MATLAB:apPar_writeQGCParFile:ModelParameterError', '\n%s:\n%s\n%s', catg, err_msg, hint);
        else
            ME.rethrow;
        end
    end
    
    
    if isfloat(value)
        value_str = sprintf(['%.' num2str(decimal_places) 'f'], value);
    elseif isinteger(value)
        value_str = sprintf('%i', value);
    end
    
    fprintf(fid, '%i\t%i\t%s\t%s\t%i\n', ...
            vehicle_id, component_id, name, value_str, type);
end

end
