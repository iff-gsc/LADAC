function [ y_dt ] = pt1( u,y,K,T ) %#codegen
% pt1 computes computes the time derivative of the output of a PT1
%   transfer function.
%   The following transfer function is used:
% 
%   Y(s)       K
%   ---- = ---------
%   U(s)    T*s + 1
% 
% Inputs:
%   u               input of the transfer function
%   y               output of the transfer function
%   K               P gain (static gain) of the transfer function
%   T               time constant of the transfer function
% 
% Outputs:
%   y_dt            time-derivative of the output of the transfer function
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    y_dt = zeros(size(K));
    y_dt(1:end,1:end) = 1./T .* ( K.*u - y );

end
