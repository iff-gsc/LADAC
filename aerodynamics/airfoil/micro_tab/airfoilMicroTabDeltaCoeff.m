function [Delta_c_L,Delta_c_D,Delta_c_m] = airfoilMicroTabDeltaCoeff( ...
    micro_tab, circulation, state )
% airfoilMicroTabDeltaCoeff compute the Delta lift, drag and moment
% coefficient for a micro-tab [1]
% 
% Inputs:
%   micro_tab           micro-tab struct (see airfoilMicroTabCreate)
%   circulation         circulation struct (see wingSetCirculationUnsteady)
%   state               state of the micro-tab (1xN array for N tabs;
%                       the arrays inside circulation must also have N
%                       elements)
% 
% Outputs:
%   Delta_c_L           Delta lift coefficient (1xN array)
%   Delta_c_D           Delta drag coefficient (1xN array)
%   Delta_c_m           Delta pitching moment coefficient (1xN array)
% 
% See also:
%   airfoilMicroTabLoadParams, airfoilMicroTabStateDeriv
% 
% Literature:
%   [1] Khalil, K., Asaro, S., & Bauknecht, A. (2021). Active flow control
%       devices for wing load alleviation. Journal of Aircraft, 1-17.
%       https://arc.aiaa.org/doi/pdf/10.2514/1.C036426?casa_token=ev6VDwEIkCIAAAAA:4g1h23YzMzEHPzJskme5LC45Gs0FkqwqlveAhscZVpJXFK0Do3Mh9TEMuuIvwbp9SPNOZkFzlA
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

fdcl = airfoilAnalytic0515Ma( micro_tab.net.wcl, circulation.Ma, ...
    micro_tab.net.ncl, micro_tab.net.ocl );
fdcd = airfoilAnalytic0515Ma( micro_tab.net.wcd, circulation.Ma, ...
    micro_tab.net.ncd, micro_tab.net.ocd );
fdcm = airfoilAnalytic0515Ma( micro_tab.net.wcm, circulation.Ma, ...
    micro_tab.net.ncm, micro_tab.net.ocm );
Delta_c_L = airfoilAnalytic0515De( fdcl, [circulation.alpha_eff;state] );
Delta_c_D = airfoilAnalytic0515De( fdcd, [circulation.alpha_eff;state] );
Delta_c_m = airfoilAnalytic0515De( fdcm, [circulation.alpha_eff;state] );
    
end