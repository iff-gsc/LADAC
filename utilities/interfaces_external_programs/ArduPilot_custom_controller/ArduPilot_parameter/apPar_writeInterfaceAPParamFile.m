function apPar_writeInterfaceAPParamFile(ap_pars, code_pars)
% APPAR_WRITEINTERFACEAPPARAMFILE

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


%% ModeCustom.cpp file
fid = fopen(code_pars.interface_ap_param_file, 'w');

% Includes
switch ap_pars.target
    case 'ArduCopter'
        includes = {'"../../ArduCopter/Copter.h"'};
        includes_info = {'- libraries/AP_Param/AP_Param.h'; ...
                         '- ArduCopter/mode.h'; ...
                         '- libraries/AC_AttitudeControl/MatlabController.h'};
    case 'ArduPlane'
        includes = {'"../../ArduPlane/Plane.h"'};
        includes_info = {'- libraries/AP_Param/AP_Param.h'; ...
                         '- ArduPlane/mode.h'; ...
                         '- libraries/AC_AttitudeControl/MatlabController.h'};
    otherwise
        error('MATLAB:apPar_writeInterfaceAPParamFile:inputError', ...
              '\nTarget ''%s'' is not supported', ap_pars.target);
end

fprintf(fid, '#include %s\n', includes{:});
fprintf(fid, '// %s\n', includes_info{:});

% ModeCustom::var_info[]
fprintf(fid, '\n\nconst AP_Param::GroupInfo ModeCustom::var_info[] = {\n');
fprintf(fid, '%sAP_SUBGROUPINFO(custom_controller, "", 0, ModeCustom, MatlabControllerClass),\n', code_pars.indent);
fprintf(fid, '\tAP_GROUPEND\n};\n');

fclose(fid);

end
