function fuselagePlotGeometry( fuselage )
% fuselagePlotGeometry visualize geometry of fuselage
% 
% Syntax:
%   fuselagePlotGeometry( fuselage )
% 
% Inputs:
%   fuselage        fuselage struct (see fuselageInit)
% 
% See also:
%   fuselageInit, fuselageCreate, fuselageSetGeometry
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

pos = zeros( 3, size(fuselage.geometry.cntrl_pos,2) * 2 + 1 );
pos(:,1:2:end) = fuselage.geometry.border_pos;
pos(:,2:2:end) = fuselage.geometry.cntrl_pos;
plot3( fuselage.geometry.cntrl_pos(1,:), fuselage.geometry.cntrl_pos(2,:), ...
    fuselage.geometry.cntrl_pos(3,:), 'x' )
hold on
plot3( pos(1,:), pos(2,:), pos(3,:) )


for i = 1:fuselage.n_segments + 1

    circleYz( fuselage.geometry.border_pos(1,i), fuselage.geometry.border_pos(2,i), ...
        fuselage.geometry.border_pos(3,i), fuselage.geometry.width(i)/2 );

end

axis equal

xlabel('x in m')
ylabel('y in m')
zlabel('z in m')

% perspective
view(-150,25); % azimuth, elevation

set(gca, 'YDir','reverse')
set(gca, 'ZDir','reverse')

end

function h = circleYz(x,y,z,r)
    hold on
    th = 0:pi/50:2*pi;
    xunit = repmat(x,1,length(th));
    yunit = r * cos(th) + y;
    zunit = r * sin(th) + z;
    h = plot3(xunit, yunit, zunit, 'k' );
    hold off
end