function [ y_dt, Ku_y ] = pdt2( u,y,int_Ku_y_dt,K,omega_0,D,T_D ) %#codegen
% pdt2 computes computes the time derivative of the output of a PDT2
%   transfer function.
%   The following transfer function is used:
% 
%   Y(s)        K * ( T_D*s + 1 )
%   ---- = -------------------------------------
%   U(s)    1/omega_0*s^2 + 2*D/omega_0^2*s + 1
% 
% Inputs:
%   u               input of the transfer function
%   y               output of the transfer function
%   int_Ku_y_dt     integral of (K*u-y) over time
%   K               P gain (static gain) of the transfer function
%   omega_0         natural frequency of the T2 transfer function
%   D               damping of the T2 transfer function
%   T_D             time constant of D transfer function
% 
% Outputs:
%   y_dt            time-derivative of the output of the transfer function
%   Ku_y            (K*u-y)
% 
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    Ku_y = zeros(size(K));
    y_dt = zeros(size(K));
    Ku_y(1:end,1:end) = K.*u - y;
    y_dt(1:end,1:end) = omega_0.^2 .* ( K.*T_D.*u + int_Ku_y_dt - 2*D./max(abs(omega_0),1e-10).*sign(omega_0).*y );

end
