function airfoil_map_scell = wingAirfoilMapSetSim( airfoil_map )
% wingAirfoilMapSetSim converts multiple profile aerodynamics maps (array
% of structs) into an Scell array that is supported for code generation
% 
% Inputs:
%   section             Array of structs containing the profile maps as
%                       loaded by wingSetProfileAerodynamics
% 
% Outputs:
%   airfoil_map_scell   Scell array containing multiple profile maps (see
%                       cell2scellArr, mat_file_format.md)
% 
% See also: wingAirfoilMapSetSim, cell2scell
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2021 Lucas Schreer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% get field names
fields = fieldnames( airfoil_map );

% number of dependent variables
num_variables = 5;

% initialization
map_cl = cell(1,length(fields));
map_cd = cell(1,length(fields));
map_cm = cell(1,length(fields));
alpha = cell(1,length(fields));
Reynolds = cell(1,length(fields));
Mach = cell(1,length(fields));
actuator_1 = cell(1,length(fields));
actuator_2 = cell(1,length(fields));

for j = 1:length(fields)
    
    % get grid names
    grid_names = airfoil_map.(fields{j}).grid.name;
    grid_fieldnames = fieldnames(grid_names);
    num_grid_names = numel(fieldnames(grid_names));
    
    alpha_sub = [0];
    Mach_sub = [0];
    Reynolds_sub = [0];
    actuator_1_sub = [0];
    actuator_2_sub = [0];
    idx_al_Ma_Re_de = zeros(1,num_variables);
    is_independent = zeros(1,num_variables);
    
    for i = 1:num_variables
        if i > num_grid_names
            is_independent(i) = 1;
        else
            grid_name_i = grid_names.(grid_fieldnames{i});
            grid_val = airfoil_map.(fields{j}).grid.val.(['x',num2str(i)]);
            switch grid_name_i
                case 'alpha'
                    alpha_sub = grid_val;
                    idx = 1;
                case 'Mach'
                    Mach_sub = grid_val;
                    idx = 2;
                case 'Reynolds'
                    Reynolds_sub = grid_val;
                    idx = 3;
                case 'actuator_1'
                    actuator_1_sub = grid_val;
                    idx = 4;
                case 'actuator_2'
                    actuator_2_sub = grid_val;
                    idx = 5;
            end
            idx_al_Ma_Re_de(idx) = i;
        end
    end
    
    
    [alpha{j}, Mach{j}, Reynolds{j}, actuator_1{j}, actuator_2{j} ] = ndgrid(...
        alpha_sub, Mach_sub, Reynolds_sub, actuator_1_sub, actuator_2_sub );
	
    idx_al_Ma_Re_de(idx_al_Ma_Re_de == 0) = (max(idx_al_Ma_Re_de) + 1):num_variables;
    
    map_cl{j} = permute( airfoil_map.(fields{j}).data.c_L, idx_al_Ma_Re_de );
    map_cd{j} = permute( airfoil_map.(fields{j}).data.c_D, idx_al_Ma_Re_de );
    map_cm{j} = permute( airfoil_map.(fields{j}).data.c_m, idx_al_Ma_Re_de );
    
end

% create scells 
airfoil_map_scell.map_cl = cell2scellArr( map_cl );
airfoil_map_scell.map_cd = cell2scellArr( map_cd );
airfoil_map_scell.map_cm = cell2scellArr( map_cm );
% profile_aero_scell.map_cl.dim(end+1:5) = 1;
% profile_aero_scell.map_cd.dim(end+1:5) = 1;
% profile_aero_scell.map_cm.dim(end+1:5) = 1;
airfoil_map_scell.alpha = cell2scellArr( {alpha_sub} );
airfoil_map_scell.Mach = cell2scellArr( {Mach_sub} );
airfoil_map_scell.Reynolds = cell2scellArr( {Reynolds_sub} );
airfoil_map_scell.actuator_1 = cell2scellArr( {actuator_1_sub} );
airfoil_map_scell.actuator_2 = cell2scellArr( {actuator_2_sub} );

end