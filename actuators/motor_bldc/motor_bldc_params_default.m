% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%  ** default brushless direct current motor parameters **

% torque constant of the motor (KT=60/(2*pi*KV)), N.m/A
% with KV = 1280 RPM/V
param.KT = 60/(2*pi*1280);
% motor internal resistance, Ohm
param.R = 0.07;