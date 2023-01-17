function [] = propMapGridPlot( prop_grid, varargin )
%propMapGridPlot plots the map of a propeller.
% 
% Syntax:
%   propMapGridPlot( prop_grid )
%   propMapGridPlot( prop_grid, output_name )
% 
% Inputs:
%   prop_grid           propeller map grid (struct) as defined by the
%                       function propMapGridCreate
%   output_name         (optional) specify the z axis:
%                           'thrust'
%                           'torque'
%                           'power'
%                           empty:   thrust (default)
% 
% See also:
%   PROPMAPGRIDCREATE, PROPMAPFITPLOT

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

[RPM_grid,V_grid] = meshgrid( prop_grid.RPM, prop_grid.V );

if isempty(varargin)
    Z = prop_grid.T;
    label = 'thrust, N';
else
    if strcmp(varargin{1},'thrust')
        Z = prop_grid.T;
        label = 'thrust, N';
    elseif strcmp(varargin{1},'torque')
        Z = prop_grid.Tau;
        label = 'torque, Nm';
    elseif strcmp(varargin{1},'power')
        Z = prop_grid.P;
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
