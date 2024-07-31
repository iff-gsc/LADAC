function apPar_writeStructOverrideFile(code_pars, structs)
% APPAR_WRITESTRUCTOVERRIDEFILE

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


includes = {'<AP_Param/AP_Param.h>'};   % ToDo: central config?


% Generate header file struct overrides
fid = fopen(code_pars.struct_override_file,'w');

fprintf(fid, '#include %s\n', includes{:});

fnames = fieldnames(structs);
for idx = numel(fnames):-1:1
    tstruct_name = fnames{idx};
    tstruct = structs.(tstruct_name);
    
    % #define statement
    fprintf(fid, '\n\n#define DEFINED_TYPEDEF_FOR_%s_\n', tstruct_name);    % ToDo: dynamic config? -> Search in Simulink Coder template?
    
    % typedef struct definition
    fprintf(fid, '\ntypedef struct {\n');
    
    fnames2 = fieldnames(tstruct);
    for idx2 = 1:numel(fnames2)
        var_name = fnames2{idx2};
        var = tstruct.(var_name);
        
        if var.size > 1
            fprintf(fid, '%s%s %s[%i];\n', code_pars.indent, var.type, var_name, var.size);
            
%             % Experimental usage of AP_ParamA
%             fprintf(fid, '%sAP_ParamA<float, %i, AP_PARAM_ARRAYF> %s;\n', code_pars.indent, var.size, var_name);
        else
            fprintf(fid, '%s%s %s;\n', code_pars.indent, var.type, var_name);
        end
    end
    
    fprintf(fid, '} %s;\n', tstruct_name);
end

fclose(fid);

end
