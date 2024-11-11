function aero = conventionalAirplaneAeroLoadParams( filename )
% the variable loaded from filename must be equal to
% conventionalAirplaneAero_params_default
% 
% Example:
%   aero = conventionalAirplaneAeroLoadParams( ...
%       'conventionalAirplaneAero_params_default' );
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% load parameters from file 
run(filename);

if ~isfield(aero,'downwash')
    aero.downwash.alpha_htp_dalpha = 0;
    aero.downwash.alpha_htp_deta = zeros(size(aero.wingMain.flap.dalpha_deta));
end

end