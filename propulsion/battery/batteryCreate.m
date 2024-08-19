function bat = batteryCreate( filename, capacity, num_serial, c_rate, varargin )
% batteryLoadParams loads a battery struct.
% 
% Syntax:
%   bat = batteryCreate( filename, capacity, num_serial, c_rate )
%   bat = batteryCreate( filename, capacity, num_serial, c_rate, V_min )
% 
% Inputs:
%   filename            battery cell parameters filename (string), e.g.
%                       'battery_params_default'
%   capacity            battery capacity (scalar), Ah
%   num_serial          number of cells in series (scalar)
%   c_rate              battery C rating, 1/h
%   V_min               (optional) minimum battery model output voltage, V
% 
% Outputs:
%   bat                 battery struct as defined by this function
% 
% Example:
% 	bat = batteryCreate( 'battery_params_default', 4, 4, 20, 3 );
% 
% Literature:
%   [1] Tremblay, O., & Dessaint, L. A. (2009). Experimental validation of
%       a battery dynamic model for EV applications. World electric vehicle
%       journal, 3(2), 289-298.
% 
% See also
%   batteryVoltage, batteryDischargeParams

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% load cell parameters
run(filename);

% number of serial cells
bat.num_serial  = num_serial;
% capacity, Ah
bat.capacity 	= capacity;
% C rate, 1/h
bat.C_rate      = c_rate;

% typical value for LiPo batteries (R*C*C_rate should be about constant, 
% see batteryAverageParams)
R_times_C_times_C_rate = 0.5128;
% typical value for LiPo batteries based on C rating (R*C should be about
% constant for batteries of same C rating)
bat.R_times_C = R_times_C_times_C_rate / (bat.C_rate/3600);

% minimum batterie cell voltage
if isempty(varargin)
    bat.V_min = 3.0;
else
    bat.V_min = varargin{1};
end

end