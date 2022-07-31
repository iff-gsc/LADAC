function prop_map_scatter = propMapScatterCreate(prop_name)
% PROPMAPSCATTERCREATE create propeller map scatter from propeller map data
%   The propeller map scatter stores all data points from the propeller map
%   data plus one point at RPM=0, V=0, where thrust and torque are zero.
% 
% Syntax:
%   prop_map_scatter = propMapScatterCreate( prop_name )
% 
% Inputs:
%    prop_name              The name of one specific propeller type within
%                           the first column of DATA_APC.
%                           Use the following command to get all available
%                           names:
%                           name_list = propMapGetNameList();
% 
% Outputs:
%   prop_map_scatter        propeller map scatter as defined by this
%                           function (struct)
% 
% See also:
%   PROPMAPSCATTERPLOT, PROPMAPFITCREATE, PROPMAPGETNAMELIST

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

load('DATA_APC');

ind = find(strcmp(DATA_APC(:,1),prop_name));
DATAp = DATA_APC(ind,:);

num_rpm = size(DATAp,1)-1;

num_data_points = numel( [0, DATAp{2:end,3}] ) + 1;
V = zeros( 1, num_data_points );
RPM = zeros( 1, num_data_points );
T = zeros( 1, num_data_points );
P = zeros( 1, num_data_points );

idx_beg = 2;
for i = 1:num_rpm
    V_sub = [0,DATAp{i+1,3}];
    idx_end = idx_beg + length( V_sub )-1;
    V(idx_beg:idx_end) = V_sub;
    RPM(idx_beg:idx_end) = DATAp{i+1,2};
    T(idx_beg:idx_end) = [DATAp{1,4}(i), DATAp{i+1,4}];
    P(idx_beg:idx_end) = [DATAp{1,5}(i), DATAp{i+1,5}];
    idx_beg = idx_end + 1;
end

Tau = P./(RPM*2*pi/60);
Tau(isnan(Tau)) = 0;

prop_map_scatter.RPM = RPM;
prop_map_scatter.V = V;
prop_map_scatter.thrust = T;
prop_map_scatter.power = P;
prop_map_scatter.torque = Tau;

end