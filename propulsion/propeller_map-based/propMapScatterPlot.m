function [] = propMapScatterPlot(prop_map_scat,varargin)
%PROPMAPSCATTERPLOT plot the map of a propeller map scatter
% 
% Syntax:
%   propMapScatterPlot( prop_map_scat )
%   propMapScatterPlot( prop_map_scat, output_name )
% 
% Inputs:
%   prop_map_scat       propeller map scatter (struct) as defined by
%                       propMapScatterCreate
%   output_name         (optional) specify the z axis:
%                           'thrust'
%                           'torque'
%                           'power'
%                           empty:   thrust (default)
% 
% See also:
%   PROPMAPFITCREATE, PROPMAPFITGETZ, PROPMAPFITGETZDERIV

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if isempty(varargin)
    scatter3( prop_map_scat.RPM, prop_map_scat.V, prop_map_scat.thrust );
    label = 'thrust, N';
else
    if strcmp(varargin{1},'thrust')
        scatter3( prop_map_scat.RPM, prop_map_scat.V, prop_map_scat.thrust );
        label = 'thrust, N';
    elseif strcmp(varargin{1},'torque')
        scatter3( prop_map_scat.RPM, prop_map_scat.V, prop_map_scat.torque );
        label = 'torque, Nm';
    elseif strcmp(varargin{1},'power')
        scatter3( prop_map_scat.RPM, prop_map_scat.V, prop_map_scat.power );
        label = 'power, W';
    else
        error('The z axis was not specified correctly.')
    end
end

xlabel('rotational speed, RPM')
ylabel('airspeed, m/s')
zlabel(label)

end
