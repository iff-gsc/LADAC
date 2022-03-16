function list = propMapGetNameList( DATA_APC )
% getPropellerMapNameList returns a list of propeller names available in a
%   propeller database cell array.
% 
% Syntax:
%   list = propMapGetNameList( DATA_APC )
% 
% Inputs:
% 	 DATA_APC               An array of cell array which contains propeller
%                           maps from APC for several propeller types.
%                           This can be load from a mat file:
%                           load('DATA_APC');
% 
% Outputs:
%   list                    names of all propellers in the propeller map
%                           database (cell array of strings)
% 
% Example:
%   load('DATA_APC')
%   list = getPropellerMapNameList('DATA_APC');
% 
% See also:
%   PROPMAPFITCREATE, PROPMAPGRIDCREATE, PROPMAPSCATTERCREATE


% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

list = unique([DATA_APC(:,1)]);
    
end