% ** Parameters for tunability post-processing of code export (dependent) **
%
%   The following parameters are dependent on external sources and are
%   therefore separate from the user-configurable parameters
%   (apPar_params_default).
%   The external sources are mentioned in the comments for each parameter.

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************



%% ArduPilot dependent settings
%   The following settings are dependent on the ArduPilot source code

param.name.prefix = 'ML_';   % Configured in:   ArduPilot/ArduCopter/Parameters.cpp

param.name.max_len = 16;     % See also:        ArduPilot/libraries/AP_Param/AP_Param.cpp
                                %                   - Keywords: "suffix is too long"
                                %                   ArduPilot/libraries/AP_Param/AP_Param.h
                                %                   - Keywords: AP_MAX_NAME_SIZE

param.group.max_nesting = 3;     % See also:    ArduPilot/libraries/AP_Param/AP_Param.h:
                                    %               - Keywords: _group_bits, _group_level_shift
param.group.max_elements = 2^6-1;  % See also:    ToDo:
                                    %               ToDo: Consider remaining nesting capacity due to implementation...



%% Supported datatypes type translations
%   The following settings translate the data types between different
%   worlds (e.g. Simulink, MATLAB, ArduPilot, QGroundControl)

sl_set = {'real32_T'; ...
          'int8_T'; ...
          'int16_T'; ...
          'int32_T'};
%   ToDo: read this from the Simulink model?

ml_set = {'single'; ...
          'int8'; ...
          'int16'; ...
          'int32'};
%   ToDo: read this from MATLAB?

%   See also:
%   - ArduPilot/libraries/AP_Param/AP_Param.h -> ap_var_type
ap_set = {'AP_Float'; ...
          'AP_Int8'; ...
          'AP_Int16'; ...
          'AP_Int32'};

%   See also:
%   - https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/file_formats/parameters.html
%   - https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE
qgc_set = {9; ...
           2; ...
           4; ...
           6};

param.type_translation.sl2ap  = containers.Map(sl_set, ap_set);
param.type_translation.ml2qgc = containers.Map(ml_set, qgc_set);



%% Code export settings
%   The following settings are dependent on the ArduPilot <-> Simulink
%   interface source-code

%   This preprocessor directive is used in the interface to allow tunable
%   and non-tunable code exports with the same ArduPilot codebase.
%   See also:
%    - ArduCopter/mode_custom.cpp -> MODE_CUSTOM_VAR_INFO
%    - ArduPlane/mode_custom.cpp  -> MODE_CUSTOM_VAR_INFO
param.interface.preprocessor_directive = '#define MODE_CUSTOM_VAR_INFO';

%   The interfaces class name
%   See also:
%   - ArduCopter/mode.h; ArduCopter/mode_custom.cpp
%   - ArduPlane/mode.h;  ArduPlane/mode_custom.cpp
param.interface.class_name = 'ModeCustom';



%% README
param.readme_url = 'https://github.com/iff-gsc/LADAC/blob/main/ardupilot-tunable-parameter-postprocessing/utilities/interfaces_external_programs/ArduPilot_custom_controller/ArduPilot_parameter/README.md';
