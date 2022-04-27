function geo = tornadoCreateGeo( params_file, xyz_cg_c, xyz_ref_c, ...
    n_panel_y, n_panel_x, n_panel_x_flap )
% tornadoCreateGeo create geometry struct as used by Tornado [1] from
% parameters file
% 
% Inputs:
%   params_file    	file name of the parameters file (string), see
%                  	wing_params_default.m
%   xyz_cg_c        position of the center of gravity in c frame (3x1
%                   array) used for velocity computation due to angular
%                   velocity, in m
%   xyz_ref_c       reference position for the moments in c frame (3x1
%                   array), in m
%   n_panel_y       number of panels in y direction (scalar int)
%   n_panel_x       number of panels in x direction (scalar int)
%   n_panel_x_flap  number of panels in x direction for flaps
% 
% Outputs:
%   geo             geometry (geo) struct as used by Tornado
% 
% Literature:
%   [1] User's Guide Tornado 1.0, Release 2.3 2001-01-21,
%       http://tornado.redhammer.se/images/manual.pdf
% 
% See also:
%   tornadoCreateState

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

params = wingLoadParameters(params_file);

ny = (n_panel_y+1)/2;

eta_partitions = unique( [ params.eta_segments_wing, ...
    params.eta_segments_device ] );

n_partitions = length( eta_partitions ) - 1;

% the number of panels for each partition is defined as follows:
% - detect the panel of the whole wing with the highest y-length
% - divide this panel in the middle
% - repeat until the desired number of panels in y direction is reached
ny_vec = ones( 1, n_partitions );
eta_partitions_ny = eta_partitions;
while length(eta_partitions_ny) < ny
    [max_diff,idx_max_diff] = max(diff(eta_partitions_ny));
    idx_max_diff_ny = max(find(eta_partitions<eta_partitions_ny(idx_max_diff+1)));
    ny_vec(idx_max_diff_ny) = ny_vec(idx_max_diff_ny) + 1;
    eta_partitions_ny = [eta_partitions_ny(1:idx_max_diff),mean(eta_partitions_ny(idx_max_diff:idx_max_diff+1)),eta_partitions_ny(idx_max_diff+1:end)];
end

is_flapped = false(1,n_partitions);
flap_depth = zeros(1,n_partitions);
for i = 1:length(params.eta_segments_device)-1
    idx = find(eta_partitions >= params.eta_segments_device(i) & ...
        eta_partitions < params.eta_segments_device(i+1));
    flap_depth(idx) = params.flap_depth(i);
    is_flapped(idx) = params.control_input_index(1,end/2+i) > 0 & flap_depth(i) > 0;
end

% number of wings
geo.nwing = 1;
% number of wing partitions (partitions are divided by kinks or flap
% borders)
geo.nelem = n_partitions;
% does the wing have flaps?
geo.flapped = is_flapped;
% see function input
geo.CG = xyz_cg_c(:)';
% see function input
geo.ref_point = xyz_ref_c(:)';
% is the wing symmetrical?
geo.symetric = params.is_symmetrical;
% apex x-coordinate in c frame, in m
geo.startx = params.x;
% apex y-coordinate in c frame, in m
geo.starty = 0;
% apex z-coordinate in c frame, in m
geo.startz = params.z;
% root chord, in m
geo.c = params.c(1);
% airfoil (disabled)
geo.foil=cell(1,geo.nelem,2);
geo.foil(1,1:end,1:end) = {'0'};
% number of panels in x direction of c frame (equal for all partitions)
geo.nx = n_panel_x * ones(1,geo.nelem);
% number of panels in y direction per partition (computed above)
geo.ny = ny_vec;
% twist matrix (number of partitions x 2) showing inner and outer twist of
% each partition, inner first, in rad
TW = interp1( params.eta_segments_wing, [0,params.epsilon], eta_partitions, 'linear', 'extrap' ) + params.i;
geo.TW(1,:,1) = TW(1,1:end-1);
geo.TW(1,:,2) = TW(1,2:end);
% dihedral of each partition, in rad
geo.dihed = interp1( params.eta_segments_wing(2:end), params.nu, eta_partitions(2:end), 'linear', 'extrap' );
% span of each partition (in combined y-z direction depending on the
% dihedral angle), in m
geo.b = diff( eta_partitions * params.b/2 ) ./ cos(geo.dihed);
c = interp1( params.eta_segments_wing, params.c, eta_partitions, 'linear', 'extrap' );
% taper ratio of each partition, dimensionless
geo.T = c(2:end) ./ c(1:end-1);
% sweep angle of each partition, in rad
geo.SW = interp1( params.eta_segments_wing(2:end), params.lambda, eta_partitions(2:end), 'linear', 'extrap' );
geo.meshtype = ones(1,geo.nelem);
% relative flap depth of each partition, dimensionless
geo.fc = flap_depth;
% number of panels in x direction of the flap for each partition
geo.fnx = n_panel_x_flap * ones(1,geo.nelem) .* geo.flapped;
% are flaps deflected symmetrically (per partition)?
geo.fsym = zeros(1,geo.nelem);

% unknown and hard coded
geo.flap_vector = zeros(1,geo.nelem);
geo.name = 'Undefined';
geo.project = 'Undefined';
geo.version = 136;
geo.allmove = 0;
geo.allmove_origin = 0;
geo.allmove_axis = 0;
geo.allmove_symetric = 0;
geo.allmove_def = 0;

end
