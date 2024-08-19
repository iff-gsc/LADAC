function list = propMapGetNameList()
% getPropellerMapNameList name list of all available propeller maps
% 
% Syntax:
%   list = propMapGetNameList()
% 
% Inputs:
%   -
% 
% Outputs:
%   list                    names of all propellers in the propeller map
%                           database (cell array of strings)
% 
% See also:
%   PROPMAPFITCREATE, PROPMAPGRIDCREATE, PROPMAPSCATTERCREATE


% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

load('DATA_APC');
list = unique([DATA_APC(:,1)]);
    
end