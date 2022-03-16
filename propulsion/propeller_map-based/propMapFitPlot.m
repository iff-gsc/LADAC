function [] = propMapFitPlot( prop_map_fit, varargin )
%PROPMAPFITPLOT plot the map of a propeller
% 
% Syntax:
%   propMapFitPlot( prop_map_fit )
%   propMapFitPlot( prop_map_fit, output_name )
%   
% Inputs:
%   prop_map_fit        propeller map fit (struct), see propMapFitCreate
%   output_name         (optional) specify the z axis:
%                           'thrust'
%                           'torque'
%                           'power'
%                           empty:   thrust (default)
% 
% See also:
%   PROPMAPFITCREATE, PROPMAPFITGETZ, PROPMAPFITGETZDERIV


% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

rel_lim = 0.2;
num_pt = 50;
Delta_RPM = prop_map_fit.boarders.RPM_max - prop_map_fit.boarders.RPM_min;
V_max = max( prop_map_fit.boarders.V_max );
V_min = min( prop_map_fit.boarders.V_min );
Delta_V = V_max - V_min;
RPM_vec = linspace( prop_map_fit.boarders.RPM_min - rel_lim*Delta_RPM, ...
                    prop_map_fit.boarders.RPM_max + rel_lim*Delta_RPM, num_pt );
V_vec   = linspace( V_min - rel_lim*Delta_V, ...
                    V_max + rel_lim*Delta_V, num_pt );

[RPM_grid,V_grid] = meshgrid( RPM_vec, V_vec );

if isempty(varargin)
    Z = propMapFitGetZ(prop_map_fit,RPM_grid,V_grid,'thrust');
    label = 'thrust, N';
else
    if strcmp(varargin{1},'thrust')
        Z = propMapFitGetZ(prop_map_fit,RPM_grid,V_grid,'thrust');
        label = 'thrust, N';
    elseif strcmp(varargin{1},'torque')
        Z = propMapFitGetZ(prop_map_fit,RPM_grid,V_grid,'torque');
        label = 'torque, Nm';
    elseif strcmp(varargin{1},'power')
        Z = propMapFitGetZ(prop_map_fit,RPM_grid,V_grid,'power');
        label = 'power, W';
    else
        error('The z axis was not specified correctly.')
    end
end

surf(RPM_grid,V_grid,Z)
xlabel('rotational speed, RPM')
ylabel('airspeed, m/s')
zlabel(label)

end
