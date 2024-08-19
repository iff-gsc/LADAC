function [V_bat,V_cell] = batteryVoltage(bat,SoC,C_rate,C_rate_filt)
% batteryVoltage computes the voltage of the battery
%   
%   The battery voltage  of one cell is computed according to a discharge 
%   curve depending on the state of charge (SoC) and the discharge rate 
%   (C rate) of the battery.
%
% Syntax: 
%   [V_bat,V_cell] = batteryVoltage(bat,SoC,C_rate,C_rate_filt)
%
% Inputs:
%   bat         battery struct, see batteryCreate
%   SoC         state of charge (scalar), -
%   C_rate      discharge rate (scalar), in 1/s
%   C_rate_filt filtered discharge rate (scalar), in 1/s
%
% Outputs:
%   V_bat       battery voltage (scalar), in V
%
% Literature:
%   [1] Tremblay, O., & Dessaint, L. A. (2009). Experimental validation of
%       a battery dynamic model for EV applications. World electric vehicle
%       journal, 3(2), 289-298.
% 
% See also: 
%   batteryDischargeParams, batteryAverageParams

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2018-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

SoC_full = bat.SoC_full;
% compute voltage
if SoC > 1 - bat.SoC_full
    V_cell = ...
        bat.E_0 ...
        - bat.R_times_C*C_rate ...
        - bat.K * SoC_full ./ (SoC_full-(1-SoC)) .* ((1-SoC)+C_rate_filt) ...
        + bat.A * exp(-bat.B*(1-SoC));
    V_cell(V_cell<bat.V_min) = bat.V_min;
else
    V_cell = bat.V_min;
end

V_bat = bat.num_serial * V_cell;

end