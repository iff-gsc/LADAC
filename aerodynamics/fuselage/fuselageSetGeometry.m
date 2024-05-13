function fuselage = fuselageSetGeometry( fuselage, n_segments )
% fuselageSetGeometry set geometry struct in fuselage struct
%   This functions sets the discretized geometry of the fuselage based on
%   the fuselage parameters.
% 
% Inputs:
%   fuselage        fuselage struct (see fuselageInit)
%   n_segments      number of fuselage segments for discretization
% 
% Outputs:
%   fuselage        fuselage struct (see fuselageInit)
% 
% See also:
%   fuselageInit, fuselageCreate, fuselageSetState
% 
% Literature:
%   [1] Schlichting, H., & Truckenbrodt, E. (2001). Aerodynamik des
%       Flugzeuges. Zweiter Band: Aerodynamik des Tragfluegels (Teil II),
%       des Rumpfes, der Fluegel-Rumpf-Anordnung und der Leitwerke. 3.
%       Auflage. Springer-Verlag Berlin Heidelberg.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

xi_interp = 0:(1/n_segments):1;

if fuselage.params.is_straight
    interp_method = 'pchip';
else
    interp_method = 'makima';
end

fuselage.geometry.border_pos(1,:) = -interp1( fuselage.params.xi_segments, ...
    fuselage.params.xi_segments*fuselage.params.total_length, xi_interp, interp_method );
fuselage.geometry.border_pos(2,:) = 0;
fuselage.geometry.border_pos(3,:) = interp1( fuselage.params.xi_segments, ...
    -fuselage.params.center_line_height, xi_interp, interp_method );

fuselage.geometry.cntrl_pos(1,:) = fuselage.geometry.border_pos(1,1:end-1) ...
    + diff( fuselage.geometry.border_pos(1,:) ) / 2;
fuselage.geometry.cntrl_pos(2,:) = 0;
fuselage.geometry.cntrl_pos(3,:) = fuselage.geometry.border_pos(3,1:end-1) ...
    + diff( fuselage.geometry.border_pos(3,:) ) / 2;

fuselage.geometry.alpha(:) = atan( -diff(fuselage.geometry.border_pos(3,:)) ...
    ./ diff(fuselage.geometry.border_pos(1,:)) );
fuselage.geometry.beta(:) = 0;

fuselage.geometry.width(:) = interp1( fuselage.params.xi_segments, fuselage.params.width, xi_interp, interp_method );

fuselage = fuselageSetGeometryWidthVisc(fuselage,xi_interp,interp_method);

end