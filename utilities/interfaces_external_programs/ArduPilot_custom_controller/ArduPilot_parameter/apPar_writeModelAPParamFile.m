function var_info_names = apPar_writeModelAPParamFile(ap_pars, code_pars, tune_vars_proc, par_tlns)
% APPAR_WRITEMODELAPPARAMFILE

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% ToDo:
% - Use correct C++ literal?
% - It is unclear, if it has an performance impact how the group splitting
%   is done


indent          = code_pars.indent;
class_name      = code_pars.class_name;
ap_param_file   = code_pars.model_ap_param_file;
header_file     = getFileFromPath(code_pars.header_file);



% Combine all tunable parameters
flds_ap_path  = {};
flds_cpp_path = {};
flds_value    = {};
sizes         = [];

var_names = fieldnames(tune_vars_proc);
for idx_var = 1:numel(var_names)
    var_name = var_names{idx_var};
    
    par_struct = tune_vars_proc.(var_name);
    
    flds    = par_struct.fields;
    flds_s  = par_struct.fields_s;
    sizes_l = par_struct.sizes_l;
    values  = par_struct.values;
    
    
    if ~ap_pars.use_variable_name
        flds = cellfun(@(a) [var_name, '.', a], flds, 'UniformOutput', false);
    end
    flds = cellfun(@(a) [par_tlns{idx_var}, '.', a], flds, 'UniformOutput', false);
    
    
    flds_ap_path  = [flds_ap_path;  flds_s];
    flds_cpp_path = [flds_cpp_path; flds];
    flds_value    = [flds_value; values];
    sizes         = [sizes; sizes_l];
    
end

flds_ap_name  = strrep(flds_ap_path, '.', ap_pars.delimiter);
nesting_depth = max(cellfun(@(a) length(a) + 1, strfind(flds_cpp_path, '.')));
sizes_total   = sum(sizes);


%% Split into subgroups
% ToDo: This logic could get overhaul!
if sizes_total > ap_pars.group.max_elements
    is_par_split_success = false;
    
    for idx_nst = 1:nesting_depth
        
        [grps, idxs_bool, idxs, is_group] = apPar_getGroups(flds_cpp_path, idx_nst);
        
        % Check total number of parameters and split them into subgroups if required
        frst_layer_idxs_bool = idxs_bool(~is_group);
        scnd_layer_idxs_bool = idxs_bool(is_group);
        
        frst_layer_grp_size = sum(cellfun(@(a) sum(sizes(a)), frst_layer_idxs_bool)) + length(scnd_layer_idxs_bool);
        if isempty(frst_layer_grp_size)
            frst_layer_grp_size = length(scnd_layer_idxs_bool);
        end
        scnd_layer_grp_size = cellfun(@(a) sum(sizes(a)), scnd_layer_idxs_bool);
        
        if frst_layer_grp_size <= ap_pars.group.max_elements && all(scnd_layer_grp_size <= ap_pars.group.max_elements)
            is_par_split_success = true;
            break;
        end
    end
    
    scnd_layer_grps = grps(is_group);
    frst_layer_idxs = idxs(~is_group);
    scnd_layer_idxs = idxs(is_group);
    
    if ~is_par_split_success
        error('Automatic parameter splitting did not succeed. Too many parameters per group! Please implement a more advanced method.')
    end
else
    scnd_layer_grps = cell.empty;
    scnd_layer_idxs = cell.empty;
    
    frst_layer_idxs = num2cell([1:numel(flds_ap_path)].');
    is_group        = false(size(flds_ap_path));
end



%% Generate var_info names
var_info_prfx  = 'var_info_';
var_info_names = cellfun(@(a) [var_info_prfx a], strrep(scnd_layer_grps, '.', '_'), 'UniformOutput', false);



%% Generate AP parameter file
fid = fopen(ap_param_file, 'w');

includes = {'<AP_Param/AP_Param.h>'; ...
           ['"' header_file '"']};

fprintf(fid, '#include %s\n', includes{:});


% Second layer groups
for idx_g = 1:length(scnd_layer_grps)
    
    fprintf(fid, '\n\nconst AP_Param::GroupInfo %s::%s[] = {\n', class_name, var_info_names{idx_g});
    
    idx_d = 0;
    for idx_scnd = scnd_layer_idxs{idx_g}(:).'
        
        fld_cpp_path = flds_cpp_path{idx_scnd};
        fld_ap_name  = flds_ap_name{idx_scnd};
        fld_value    = flds_value{idx_scnd};
        fld_size     = sizes(idx_scnd);
        
        idx_d = addField(ap_pars, code_pars, fid, idx_d, fld_cpp_path, fld_ap_name, fld_value, fld_size);
        
    end
    fprintf(fid, '%sAP_GROUPEND\n};\n', indent);
end


% First layer group
fprintf(fid, '\n\nconst AP_Param::GroupInfo %s::var_info[] = {\n', class_name);

% V1 -> first parameters, then groups
%{
idx_d = 0;
for idx_g = 1:length(frst_layer_grps)
    for idx_frst = frst_layer_idxs{idx_g}(:).'

        fld_cpp_path = flds_cpp_path{idx_frst};
        fld_ap_name  = flds_ap_name{idx_frst};
        fld_value    = flds_value{idx_frst};
        fld_size     = sizes(idx_frst);

        idx_d = addField(ap_pars, code_pars, fid, idx_d, fld_cpp_path, fld_ap_name, fld_value, fld_size);
    end
end

idx_offset = idx_d;

for idx_g = 1:length(scnd_layer_grps)
    fprintf(fid, '%sAP_SUBGROUPEXTENSION("", %i, %s, var_info_%s),\n', ...
        indent, idx_g-1+idx_offset, class_name, var_info_names{idx_g});
end
%}

% V2 -> the same order as the original struct
idx_d    = 0;
idx_scnd = 1;
idx_frst = 1;
for idx_g = 1:length(is_group)
    
    if is_group(idx_g)
        fprintf(fid, '%sAP_SUBGROUPEXTENSION("", %i, %s, %s),\n', ...
            indent, idx_d, class_name, var_info_names{idx_scnd});
        
        idx_d = idx_d + 1;        
        idx_scnd = idx_scnd + 1;
    else
        idx_fld      = frst_layer_idxs{idx_frst};
        fld_cpp_path = flds_cpp_path{idx_fld};
        fld_ap_name  = flds_ap_name{idx_fld};
        fld_value    = flds_value{idx_fld};
        fld_size     = sizes(idx_fld);
        
        idx_d = addField(ap_pars, code_pars, fid, idx_d, fld_cpp_path, fld_ap_name, fld_value, fld_size);
        
        idx_frst = idx_frst + 1;
    end
end

fprintf(fid, '%sAP_GROUPEND\n};\n', indent);

fclose(fid);

var_info_names = [var_info_names; 'var_info'];

end





%% LOCAL FUNCTIONS
function idx_d = addField(ap_pars, code_pars, fid, idx_d, fld_cpp_path, fld_ap_name, fld_value, fld_size)
    
    % array
    if fld_size > 1
        for idx_arr = 1:fld_size
            cpp_idx = ['[' num2str(idx_arr-1) ']'];
            ap_idx  = [ap_pars.array_delimiter num2str(idx_arr)];     % ToDo: if sz > 10 -> 01, 02, 03?
            
            fprintf(fid, '%sAP_GROUPINFO("%s", %i, %s, %s, %d),\n', ...
                code_pars.indent, [fld_ap_name ap_idx], idx_d, code_pars.class_name, [fld_cpp_path cpp_idx], fld_value(idx_arr));
            
            idx_d = idx_d + 1;
        end
        
%     % Experimental usage of AP_ParamA
%         fprintf(fid, '%sAP_GROUPINFO("%s", %i, MatlabControllerClass, %s, %d),\n\n', ...
%             indent, name, idx_d, fld, 0);
%         idx_d = idx_d + 1;
        
    % scalar
    else
        fprintf(fid, '%sAP_GROUPINFO("%s", %i, %s, %s, %d),\n', ...
            code_pars.indent, fld_ap_name, idx_d, code_pars.class_name, fld_cpp_path, fld_value);
        
        idx_d = idx_d + 1;
    end
    
end
