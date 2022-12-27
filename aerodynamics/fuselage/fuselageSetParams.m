function fuselage = fuselageSetParams( fuselage, filename )
% fuselageSetParams set params struct of fuselage struct from parameters file
% 
% Inputs:
%   fuselage        fuselage struct (see fuselageInit)
%   filename        name of fuselage parameters file (e.g.
%                   fuselage_params_default')
% 
% Outputs:
%   fuselage        fuselage struct (see fuselageInit)
% 
% See also:
%   fuselageInit, fuselageCreate, fuselageSetState
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

run(filename);

fuselage.params.total_length(:) = total_length;
fuselage.params.xi_segments(:) = xi_segments;
fuselage.params.width(:) = border_width;
fuselage.params.center_line_height(:) = center_line_height;
fuselage.params.C_L_alpha(:) = C_L_alpha;
fuselage.params.C_D0(:) = C_D0;
fuselage.params.is_straight(:) = is_straight;

end