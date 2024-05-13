function fuselage = fuselageSetGeometryWidthVisc( fuselage, xi_interp, interp_method )
% fuselageSetGeometryWidthVisc set viscous width of fuselage
%   Normally (if the front width and aft width of the fuselage is zero),
%   a fuselage generates a free moment but no lift (and no side force).
%   If the fuselage is supposed to create lift, modifications are required.
%   Take a look at [1], page 265f and Abb. 9.17.
%   I did not find a proper solution but it seemed easy and reasonable to
%   adjust (mostly increase) the width at the fuselage aft.
%   In order to assure a reasonable transition from the thickest fuselage
%   position to the rear, some interpolations and iterations were
%   implemented.
% 
% Inputs:
%   fuselage        fuselage struct (see fuselageInit)
%   xi_interp       position of the width along the centerline from front
%                   to back relative to the total length (1x(N+1) array, where
%                   N is the number of fuselage segments, dimensionless
%   interp_method   interpolation method (see interp1, e.g. 'makima' or
%                   'pchip')
% 
% Outputs:
%   fuselage        fuselage struct (see fuselageInit)
% 
% See also:
%   fuselageInit, fuselageSetGeometry, fuselageCreate, fuselageSetState
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

dx = abs( diff( fuselage.geometry.border_pos(1,:) ) );
radius = fuselage.geometry.width/2;
volume = pi/3 * dx .* ( radius(1:end-1).^2 + radius(1:end-1).*radius(2:end) + radius(2:end).^2 );
total_volume = sum( volume );
width_visc_end = 2 * sqrt( fuselage.params.C_L_alpha*total_volume^(2/3) / (2*pi) );
n_section = length(fuselage.params.width)-1;
for i = n_section:-1:floor(round(n_section/2))
    if fuselage.params.width(i+1) < width_visc_end
        continue;
    else
        break;
    end
end
width_visc = fuselage.params.width;
is_width_decreasing = diff(fuselage.params.width)<0;
width_max = max(fuselage.params.width);
idx_width_max_vec = find( fuselage.params.width == width_max );
idx_width_max = idx_width_max_vec(end);
idx = [false,is_width_decreasing];
width_visc(idx) = fuselage.params.width(idx) + ...
    (width_max-fuselage.params.width(idx)) * width_visc_end / width_max;
shift_factor = 0.1;
while true
    xi_segments_ext = [ fuselage.params.xi_segments(1:idx_width_max), ...
        fuselage.params.xi_segments(idx_width_max) + diff(fuselage.params.xi_segments(idx_width_max:idx_width_max+1)) * shift_factor, ...
        fuselage.params.xi_segments(end-1) + diff(fuselage.params.xi_segments(end-1:end))*0.8, fuselage.params.xi_segments(end) ];
    width_visc_ext = [ width_visc(1:idx_width_max), ...
        width_visc(idx_width_max) + diff(width_visc(idx_width_max:idx_width_max+1))*shift_factor, ...
        width_visc(end).*[1,1] ];
    fuselage.geometry.width_visc(:) = interp1( xi_segments_ext, width_visc_ext, xi_interp, interp_method );
    width_interp_q = interp1( -fuselage.geometry.border_pos(1,:), fuselage.geometry.width_visc, fuselage.params.xi_segments(idx_width_max+1:end)*fuselage.params.total_length );
    if any( width_interp_q < fuselage.params.width(idx_width_max+1:end) )
        shift_factor = shift_factor + 0.01;
    else
        break;
    end
end

end