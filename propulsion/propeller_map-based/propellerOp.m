function [ Omega, tau ] = propellerOp( v, Thrust, RPM_map, V_map, ...
    T_map, TAU_map )
% propellerOP interpolates the propeller rotational speed and torque
%   
%   By means of linear interpolation the propeller rotational speed and
%   torque is computated using the aircrafts airspeed (v) and the required
%   thrust (Thrust) as well as the propellers characteristic maps [1] for 
%   the rotational speed (RPM_map), the velocity of the propeller 
%   relatively to the air (V_map), the thrust (T_map) and the torque 
%   (TAU_map).
%
% Literature
%   [1] Landing Products Inc. Performance Data | APC Propellers. URL: 
%       https://www.apcprop.com/technical-information/performance-data/
%       [abgerufen am 19.02.2019]. 2019.
%
% Syntax:  [ Omega, tau ] = propellerOp( v, Thrust, RPM_map, V_map, ...
%   T_map, TAU_map )
%
% Inputs:
%    v          velocity of the propeller relative to the air (scalar), 
%               in m/s
%    Thrust     required thrust per propeller (scalar), in Nm
%    RPM_map	characteristics map of the propellers rotational speed
%               (column vector), in RPM
%    V_map      characteristics map of the propellers velocity relatively
%               to the air (row vector), in m/s
%    T_map      characteristics map of the propellers generated thrust 
%               (matrix), in N
%    TAU_map	characteristics map of the torque at the propeller shaft
%               (matrix), in Nm
%
% Outputs:
%    Omega		rotational speed of the propeller (scalar), in 1/(V*s)
%    tau		torque at propeller shaft (scalar), in Nm
%
%
% See also: propellerMap
%

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% If the propeller velocity relative to the air exceeds the characteristics
% map the rotational velocity and propeller shaft torque become infinite
if v > max(V_map) || v < min(V_map)
    Omega = Inf;
    tau = Inf;
else
    % Rotational Speed
    % Initialising a result column vector for the first interpolated values
    T_interp = zeros(length(RPM_map),1);                                                                    
    % Torque 
    % Initialising a result column vector for the first interpolated values
    Tau_interp = zeros(length(RPM_map),1);			 						

    % 1. Interpolation
    for i = 1:length(RPM_map)
        % Interpolation of every row to generate an interpolated column for
        % the velocity v
        t_row = T_map(i,:); 
        T_interp(i) = interp1(V_map,t_row,v);
        tau_row = TAU_map(i,:);
        Tau_interp(i) = interp1(V_map,tau_row,v);
    end
    
    % If the propeller thrust exceeds the characteristics map the 
    % rotational velocity and propeller shaft torque become infinite
    if Thrust > max(T_interp) || Thrust < min(T_interp)
        Omega = Inf;
        tau = Inf;
    else
        % 2. Interpolation
        % Interpolating the rotational speed and torque out of the result 
        % column vector
        % Rotational speed in (U/min)
        rpm = interp1(T_interp,RPM_map,Thrust);  
        % Rotational speed in (U/s)
        Omega = rpm/60*2*pi;    			
        % Propeller torque
        tau = interp1(RPM_map,Tau_interp,rpm);			  
    end
    
end

end

