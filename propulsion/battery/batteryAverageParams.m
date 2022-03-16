function [ SoC_full, SoC_nom, SoC_exp, V_full, V_exp, V_nom, C_rate ] = ...
    batteryAverageParams( DATA )
% batteryAverageParams   creates average values for the standardised 
%   battery cell
%  
%   For the standardised battery cell the values of the state of charge
%   (SoC) for all three measuring points (full, nom and exp) as well as the
%   voltage (full, exp and nom) are beeing averaged with arithmetic mean.
%   The measuring points were determined at a discharge rate of 10 1/h [1].
%
% Literature: 
%   [1] Gerd Giese. Elektromodellflug - Datenbank. 
%       URL: https://www.elektromodellflug.de/oldpage/datenbank.htm
%       [last downloaded 20.02.2019]. 2012.
%
% Syntax:  [ SoC_full, SoC_nom, SoC_exp, V_full, V_exp, V_nom, C_rate ] = 
%   batteryAverageParams( DATA )
%
% Inputs:
%   DATA        matrix containing the data and four measuring points of the
%               discharge curve of every battery
%
% Outputs:
%   SoC_full    State of Charge with no current applied (scalar), -
%   SoC_nom     nominal State of Charge (scalar), -
%   SoC_exp     exponential State of Charge (scalar), -
%   V_full      cell voltage with no current applied (scalar), in V
%   V_exp       exponential cell voltage (scalar), in V
%   V_nom       nominal cell voltage (scalar), in V
%   C_rate      discharge rate (scalar), in 1/h
%
% See also: batteryDischargeParams, batteryVoltage
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Extract the necessary discharge points and the capacity from the data
% base
discharge_points = cell2mat(DATA(:,3));
capacity = cell2mat(DATA(:,5));

% Standardising the battery capacity to the State of Charge
% The capacity values are given in As, ./3.6 transforms them into
% mAh, finally the figures are standardised with the overall battery
% capacity in mAh
SoC_full = mean( discharge_points(:,1) ./ 3.6 ./ capacity ); 
SoC_nom = mean( discharge_points(:,2) ./ 3.6 ./ capacity );
SoC_exp = mean( discharge_points(:,3) ./ 3.6 ./ capacity );

% Voltage discharge points in V
V_full = mean( discharge_points(:,4) );
V_exp = mean( discharge_points(:,5) );
V_nom = mean( discharge_points(:,6) );

% All points of the discharge curve have been determined at a discharge
% rate (C-Rate) of 10 1/h
C_rate = 10;


end

