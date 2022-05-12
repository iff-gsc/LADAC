function fuselage = fuselageSetGeometryFromCpacs( fuselage, tiglHandle, fuse_UID, axis_reversed )
% fuselageSetGeometryFromCpacs create fuselage from CPACS file.
% 
% Inputs:
%   fuselage        fuselage struct (see fuselageInit)
%   tiglHandle      tiglHandle, see tiglOpenCPACSConfigurationTry
%   fuse_UID        UID of fuselage (string) as defined in CPACS
%   axis_reversed   indicates if any axis orientation is reversed in the
%                   CPACS file (-1) or not (1) (3x1 array)
% 
% Outputs:
%   fuselage        fuselage struct (see fuselageInit)
% 
% See also: fuselageInit
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% get fuselage index
fuse_index = tiglFuselageGetIndex( tiglHandle, fuse_UID );

% compute fuselage center line and circumfence at sections
fuse_num_sections = tiglFuselageGetSectionCount(tiglHandle,fuse_index);
points_right = zeros(3,fuse_num_sections);
points_left = zeros(3,fuse_num_sections);
circumfence = zeros(1,fuse_num_sections);
for i=1:fuse_num_sections
    if i < fuse_num_sections
        eta = 0;
        idx = i;
    else
        eta = 1;
        idx = i-1;
    end
    [points_right(1,i),points_right(2,i),points_right(3,i)] = ...
        tiglFuselageGetPoint(tiglHandle,fuse_index,idx,eta,0);
    [points_left(1,i),points_left(2,i),points_left(3,i)] = ...
        tiglFuselageGetPoint(tiglHandle,fuse_index,idx,eta,0.5);
    if i == 1
        eta = 0.2;
    end
    circumfence(i) = tiglFuselageGetCircumference(tiglHandle,fuse_index,idx,eta);
end

% assume circle cross section
diameter = circumfence/pi;
% fuselage center line (mean)
points = ( points_right + points_left ) / 2;
% points and width
pos_x_ext = linspace(points(1,1),points(1,end),fuselage.n_segments+1);
pos_x = pos_x_ext(1:end-1) + diff(pos_x_ext)/2;
fuselage.geometry.cntrl_pos(:) = [ ...
    pos_x; ...
    interp1(points(1,:),points(2,:),pos_x); ...
    interp1(points(1,:),points(3,:),pos_x); ...
    ] .* repmat(axis_reversed,1,fuselage.n_segments);
fuselage.geometry.border_pos(:) = [ ...
    pos_x_ext; ...
    interp1(points(1,:),points(2,:),pos_x_ext); ...
    interp1(points(1,:),points(3,:),pos_x_ext); ...
    ] .* repmat(axis_reversed,1,fuselage.n_segments+1);
fuselage.geometry.width(:) = interp1(points(1,:),diameter,pos_x_ext);

fuselage.geometry.alpha(:) = atan( -diff(fuselage.geometry.border_pos(3,:)) ...
    ./ diff(fuselage.geometry.border_pos(1,:)) );

fuselage.geometry.beta(:) = 0;

% params
fuselage.params.total_length = abs( fuselage.geometry.border_pos(1,end) - fuselage.geometry.border_pos(1,1) );
fuselage.params.xi_segments = abs(fuselage.geometry.border_pos(1,:)) / fuselage.params.total_length;
fuselage.params.width = fuselage.geometry.width;
fuselage.params.center_line_height = -fuselage.geometry.border_pos(3,:);

if fuselage.params.is_straight
    interp_method = 'pchip';
else
    interp_method = 'makima';
end
fuselage = fuselageSetGeometryWidthVisc(fuselage,fuselage.params.xi_segments,interp_method);

end