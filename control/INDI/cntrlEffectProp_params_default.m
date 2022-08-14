% ** Parameters for control effectiveness (default) **

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% k is the number of control inputs
% m is the number of pseudo control inputs

% derivative of the pseudo control kx1 vector ny w.r.t. the control input
% mx1 vector u (usually actuator dynamics are reduced) at the trim
% point (jacobi kxm matrix)
cntrl_effect.ny_du_red = [ ...
 -201.6832 -201.6832  201.6832  201.6832; ...
  253.9542 -253.9542 -253.9542  253.9542; ...
   12.7391  -12.7391   12.7391  -12.7391; ...
   -8.4799   -8.4799   -8.4799   -8.4799 ...
    ];

% derivative of the pseudo control kx1 vector ny w.r.t. the time derivative
% of the control input mx1 vector u_dt at the trim point (jacobi kxm 
% matrix)
cntrl_effect.ny_du_dt = [ ...
   -0.0000    0.0000   -0.0000    0.0000; ...
    0.0000   -0.0000    0.0000   -0.0000; ...
    4.8581   -4.8581    4.8581   -4.8581; ...
         0         0         0         0 ...
     ];

% trim values mx1 vector of the control inputs, 0~1
cntrl_effect.u_trim = [0.5620;0.5620;0.5620;0.5620];
