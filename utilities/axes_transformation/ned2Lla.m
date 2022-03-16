function [lat, lon, alt] = ned2Lla(lat_ref, lon_ref, alt_ref, position_NED)
% to do: documentation
% to do: prevent that lat and lon became > 90 deg or 180 deg
% source: https://www.mathworks.com/help/aeroblks/flatearthtolla.html

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

dN = position_NED(1);
dE = position_NED(2);

R = 6378137;

du = atan(1 / R) * dN;
dl = atan(1 / (R * cos(lat_ref)) ) * dE;

lat = lat_ref + du;
lon = lon_ref + dl;

alt = alt_ref - position_NED(3) ;

end
