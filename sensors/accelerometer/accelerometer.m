function accel_b = accelerometer( V_Kb_dt, V_Kb, omega_Kb, M_bg, g ) %#codegen
% accelerometer computes the acceleration measured by an IMU placed in the
%   center of gravity of the body.
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
% 
% Outputs:
%   accel_b         acceleration (3x1) vector measured by an IMU placed in
%                   the center of gravity of the body in body frame, in
%                   m/s^2
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
   
% compute the acceleration according to [1, p. 213]
accel_b = V_Kb_dt + cross(omega_Kb,V_Kb) - M_bg * [ 0; 0; g ];

end