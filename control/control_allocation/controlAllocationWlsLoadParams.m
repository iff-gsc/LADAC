function ca = controlAllocationWlsLoadParams( filename )
% controlAllocationWlsLoadParams loads a wls control allocation parameters
%   struct
% 
% Example:
%  ca = controlAllocationWlsLoadParams( ...
%   'control_allocation_wls_params_default' );
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

run(filename);

end