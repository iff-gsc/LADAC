function accel_b = accelerometer( V_Kb_dt, V_Kb, omega_Kb, M_bg, g, varargin ) %#codegen
% accelerometer computes the acceleration measured by an IMU.
% 
% Syntax:
%   IMU placed in center of gravity:
%   accel_b = accelerometer( V_Kb_dt, V_Kb, omega_Kb, M_bg, g )
% 
%   IMU placed with offset from center of gravity:
%   accel_b = accelerometer( V_Kb_dt, V_Kb, omega_Kb, M_bg, g, ...
%                           Omega_Kb_dt, pos )
% 
% Inputs:
%   V_Kb_dt         time derivative (3x1) vector of the flight path
%                   velocity in body fame V_Kb, in m/s^2
%   V_Kb            flight path velocity (3x1) vector in body frame, in m/s
%   omega_Kb        angular velocity (3x1) vector of the body relative to
%                   the earth in body frame, in rad/s
%   M_bg            direction cosine (3x3) matrix for the transformation
%                   of g frame to b frame
%   g               scalar fravity of earth, in m/s^2
%   Omega_Kb_dt     (optional) time-derivative of omega_Kb, in rad/s/s
%   pos             (optional) Sensor position offset from center of
%                   gravity represented in body frame, in m
% 
% Outputs:
%   accel_b         acceleration (3x1) vector measured by an IMU in body 
%                   frame, in m/s^2
% 
% Literature:
%   [1] Brockhaus, R. et al. (2011): Flugregelung. 3rd ed. Springer-Verlag.
% 
% See also: euler2Dcm
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if isempty(varargin)
    Omega_Kb_dt = zeros(3,1);
    pos         = zeros(3,1);
elseif length(varargin) == 2
    Omega_Kb_dt = varargin{1};
    pos         = varargin{2};
else
    error('accelerometer: Number of inputs incorrect.');
end

% compute the acceleration according to [1, p. 213]
accel_b = V_Kb_dt + cross(omega_Kb,V_Kb) - M_bg * [ 0; 0; g ];

% [1], Eq. (2.6.12)
Delta_V = cross( omega_Kb, pos );
accel_b = accel_b + cross(Omega_Kb_dt,pos) + cross(omega_Kb,Delta_V);

end