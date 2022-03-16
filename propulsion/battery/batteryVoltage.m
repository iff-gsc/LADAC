function U_cell = batteryVoltage(SoC,C_rate,params)
% batteryVoltage computes the voltage of the battery
%   
%   The battery voltage  of one cell is computed according to a discharge 
%   curve depending on the state of charge (SoC) and the discharge rate 
%   (C rate) of the battery.
%
% Syntax:  U_cell = batteryVoltage(SoC,C_rate,params)
%
% Inputs:
%   SoC         state of charge (scalar), -
%   C_rate      discharge rate (scalar), in 1/h
%   params      a struct containing several parameters
%
% Outputs:
%   U_cell      the voltage of one cell (scalar), in V
%
%
% See also: batteryDischargeParams, batteryAverageParams
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% get parameters
Eo = params.Eo;
SoC_full = params.SoC_full;
R_over_C = params.R_over_C;
K = params.K;
A = params.A;
B = params.B;
% compute voltage
if isinf(C_rate)
    U_cell = 1e-10;
else
    U_cell = Eo - R_over_C*C_rate - K * SoC_full ./ ( SoC_full -  (1- SoC ) ) .* ...
        ((1-SoC) + C_rate*0) + A * exp(-B*(1-SoC));
end

end