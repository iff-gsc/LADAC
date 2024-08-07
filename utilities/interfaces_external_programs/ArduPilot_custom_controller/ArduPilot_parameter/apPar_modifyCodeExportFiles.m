function apPar_modifyCodeExportFiles(ap_pars, code_pars, var_infos)
% APPAR_MODIFYCODEEXPORTFILES applies the necessary changes to the header
% and source file generated by code generation.

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************



%% Process source file

% Read source file
fid = fopen(code_pars.source_file, 'r');
f = fread(fid, '*char')';
fclose(fid);

f_mod = f;

% Replace custom source file import identifier
custom_source_code = regexptranslate('escape', code_pars.custom_source_code);
search_rgx  = ['(?<=\n)' custom_source_code '(?=\n)'];
replacement = sprintf('#include "%s"', getFileFromPath(code_pars.model_ap_param_file));

%f_mod = regexprep(f_mod, search_rgx, replacement);
[idx_a, idx_b] = regexp(f_mod, search_rgx);
if isempty(idx_a)
    warning('Custom source file import identifier in ''%s'' not found! This can happen if the script has already been executed!', getFileFromPath(code_pars.source_file))
else
    f_mod = [f_mod(1:idx_a-1) replacement, f_mod(idx_b+1:end)];
end



% Override constructor
class_name = regexptranslate('escape', code_pars.class_name);
search_rgx  = ['(?<=\n' class_name '::' class_name '\(\)\s{\n).*?(?=\n}\n)'];
replacement = sprintf([code_pars.indent 'AP_Param::setup_object_defaults(this, %s);\n'], var_infos{:});

f_mod = regexprep(f_mod, search_rgx, replacement);

% Override source file file
fid = fopen(code_pars.source_file, 'w');
fprintf(fid, '%s', f_mod);
fclose(fid);



%% Process header file

% Read header file
fid = fopen(code_pars.header_file, 'r');
f = fread(fid, '*char')';
fclose(fid);

f_mod = f;

% Replace custom header file import identifier
custom_header_code = regexptranslate('escape', code_pars.custom_header_code);
search_rgx  = ['(?<=\n)' custom_header_code '(?=\n)'];
replacement = sprintf('#include "%s"\n%s', ...
                  getFileFromPath(code_pars.struct_override_file), ...
                  ap_pars.interface.preprocessor_directive);

%f_mod = regexprep(f_mod, search_rgx, replacement);
[idx_a, idx_b] = regexp(f_mod, search_rgx);
if isempty(idx_a)
    warning('Custom header file import identifier in ''%s'' not found! This can happen if the script has already been executed!', getFileFromPath(code_pars.header_file))
else
    f_mod = [f_mod(1:idx_a-1) replacement, f_mod(idx_b+1:end)];
end

% Remove already existing variables of type AP_Param::GroupInfo
% - just in case the script was already executed...
f_mod = regexprep(f_mod, '(?<=\n) *static const struct AP_Param::GroupInfo \w*\[\];\n', '');
f_mod = regexprep(f_mod, '(?<=\n *public:\n)\n*', '', 'once');

% Find start position for variable insertion
[~,b] = regexp(f_mod, '\n *public:\n', 'once');
if numel(b) ~= 1
    error("Could not find 'public:' section in '%s'", getFileFromPath(code_pars.header_file));
end

% Insert variables
variables = sprintf([code_pars.indent 'static const struct AP_Param::GroupInfo %s[];\n'], var_infos{:});
f_mod = sprintf('%s%s\n%s', f_mod(1:b), variables, f_mod(b+1:end));

% Override header file
fid = fopen(code_pars.header_file, 'w');
fprintf(fid, '%s', f_mod);
fclose(fid);

end
