function fuselagePlotGeometry( fuselage, varargin )
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
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

CircRes = 20;
CntrlPtsColor = 'b';
is_cntrl_pt = true;
LineColor = [0,0,0];
for i = 1:length(varargin)
    if strcmp(varargin{i},'CircRes')
        CircRes = varargin{i+1};
    elseif strcmp(varargin{i},'CntrlPtsColor')
        CntrlPtsColor = varargin{i+1};
    elseif strcmp(varargin{i},'CntrlPts')
        if strcmp(varargin{i+1},'on')
            is_cntrl_pt = true;
        elseif strcmp(varargin{i+1},'off')
            is_cntrl_pt = false;
        end
    elseif strcmp(varargin{i},'LineColor')
        LineColor = varargin{i+1};
    end
end

pos = zeros( 3, size(fuselage.geometry.cntrl_pos,2) * 2 + 1 );
pos(:,1:2:end) = fuselage.geometry.border_pos;
pos(:,2:2:end) = fuselage.geometry.cntrl_pos;
if is_cntrl_pt
    plot3( fuselage.geometry.cntrl_pos(1,:), fuselage.geometry.cntrl_pos(2,:), ...
        fuselage.geometry.cntrl_pos(3,:), 'Color', CntrlPtsColor, ...
        'LineStyle', 'none', 'Marker', 'o' )
end
hold on
plot3( pos(1,:), pos(2,:), pos(3,:), 'Color', LineColor )

for i = 1:fuselage.n_segments + 1

    circleYz( fuselage.geometry.border_pos(1,i), fuselage.geometry.border_pos(2,i), ...
        fuselage.geometry.border_pos(3,i), fuselage.geometry.width(i)/2, CircRes, LineColor );

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

function h = circleYz(x,y,z,r,CircRes,LineColor)
    hold on
    th = linspace(0,2*pi,CircRes);
    xunit = repmat(x,1,length(th));
    yunit = r * cos(th) + y;
    zunit = r * sin(th) + z;
    h = plot3(xunit, yunit, zunit, 'Color', LineColor );
    hold off
end