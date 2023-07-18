function [K,A,B] = ndiFeedbackGainPlace( p, T_h, varargin )
% ndiFeedbackGainPlace compute feedback gain for system with linearized
% input-output dynamics with pole placement.
%   With this function you can compute the state feedback gain for a SISO
%   system with feedback linearization (or NDI/INDI) that exhibits the
%   the specified eigenvalues/poles.
%   The error dynamics of the system ideally are a chain of integrators.
%   Additionally, we consider first-order delay to account for higher order
%   dynamics that can not be captured by the feedback linearization (e.g. 
%   actuator dynamics, sensor filter or inner loop dynamics of cascaded
%   controllers). Thus, the following error dynamics are considered (see
%   [2], Abb. 4.4 and Abb. 4.9):
% 
%                -----------     -----     -----            -----
%               |     1     |   |  1  |   |  1  |          |  1  |
%   Delta_ny -->| --------- |-->| --- |-->| --- |-- ... -->| --- |--> e_y
%               | T_h*s + 1 |   |  s  |   |  s  |          |  s  |
%                -----------     -----     -----            -----
% 
%   where Delta_ny is the Delta pseudo-control input, e_y is the error of
%   the output and T_h is the first-order delay time constant to account
%   for higher order dynamics.
%   For control the state vector of the above system is supposed to be fed
%   back. The error of the state vector is defined as:
%   e_x = [ e_y; e_y_dt; e_y_dt2; ...] with N elements where N is the order
%   of the above system (note that it includes the output of the first-
%   order time delay (except the first-order time delay was omitted)).
% 
% Syntax:
%   [K,A,B] = ndiFeedbackGainPlace( p, T_h )
%   [K,A,B] = ndiFeedbackGainPlace( p, T_h, is_fb_deriv )
%   [K,A,B] = ndiFeedbackGainPlace( p, T_h, is_fb_deriv, w )
% 
% Inputs:
%   p               desired poles of the closed-loop system (1xN array),
%                   where the first element corresponds to e_y, the second
%                   element corresponds to e_y_dt and so on.
%   T_h             first-order delay time constant to account for higher
%                   order dynamics; if no first-order dynamics should be
%                   taken into account, set T_h <= 0
%   is_fb_deriv     defines whether the highest derivative of e_y (e_y_dtN)
%                   should be included in the feedback (boolean; default: 
%                   true). If it is set to false, the last column of K will
%                   be zero. This parameter is only considered if T_h > 0.
%   w               weights for all desired poles p in case of output
%                   feedback; The weighting is required because in case of
%                   output feedback the desired poles can not be reached
%                   perfectly.
% 
% Outputs:
%   K               feedback gain matrix where the first column corresponds
%                   to the first state and so on:
%                   Delta_ny_cntrl = K * e_x (with e_x(1) = e_y, e_x(2) = 
%                   e_y_dt, e_x(3) = e_y_dt2, ...)
%   A               system matrix of the above system (NxN array)
%   B               input matrix of the above system (Nx1 array)
% 
% Example:
%   K = ndiFeedbackGainPlace([-15,-10,-5],0.015)
% 
% See also:
%   ndiPrintBlocksFromSs, ndiOpenLoopSs, ndiPlotClosedLoopErrorDynamics
% 
% Literature:
%   [1] Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
%   [2]	Holzapfel, F. (2004). Nichtlineare adaptive Regelung eines 
%       unbemannten Fluggeraetes (Doctoral dissertation, Technische 
%       Universitaet Muenchen).
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if isempty(varargin)
    is_fb_deriv = true;
else
    is_fb_deriv = varargin{1};
end

sys_order = length(p);

is_first_order_delay = T_h > 0;

if is_first_order_delay
    relative_degree = sys_order - 1;
else
    relative_degree = sys_order;
end

% create state-space model (same as above block diagram)
[A,B] = ndiOpenLoopSs( relative_degree, T_h );

if is_fb_deriv
    % compute full state feedback with pole placement
    K = ssPlace(A,B,p);
else
    % define all states as outputs except of e_y_dtN
    C = [ eye(relative_degree), zeros(relative_degree,1) ];
    % output feedback (output is the state vector without e_y_dtN)
    if length(varargin) == 1
        Ky = ssPlaceY(A,B,C,p);
    else
        w = varargin{2};
        Ky = ssPlaceY(A,B,C,p,w);
    end
    % transform output feedback to state feedback
    K = Ky*C;
end

end
