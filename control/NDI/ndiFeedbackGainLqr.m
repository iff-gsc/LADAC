function [K,A,B] = ndiFeedbackGainLqr( e_x_max, Delta_ny_max, T_h, varargin )
% ndiFeedbackGainLqr compute feedback gain for system with linearized
% input-output dynamics.
%   With this function you can compute an optimal state feedback gain for a
%   SISO system with feedback linearization (or NDI/INDI).
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
%   [K,A,B] = ndiFeedbackGainLqr( e_x_max, Delta_ny_max, T_h )
%   [K,A,B] = ndiFeedbackGainLqr( e_x_max, Delta_ny_max, T_h, is_fb_deriv )
% 
% Inputs:
%   e_x_max         maximum desired error of all states; LQR tuning
%                   parameters q_i (see [1], p. 406, "MaximumDesired Values
%                   of z(t) and u(t)")
%   Delta_ny_max    maximum desired control input (pseudo-control input
%                   ny); LQR tuning parameter r_i (see [1], p. 406, Maximum
%                   Desired Values of z(t) and u(t))
%   T_h             first-order delay time constant to account for higher
%                   order dynamics; if no first-order dynamics should be
%                   taken into account, set T_h <= 0
%   is_fb_deriv     defines whether the highest derivative of e_y (e_y_dtN)
%                   should be included in the feedback (boolean; default: 
%                   true). If it is set to false, the last column of K will
%                   be zero. This parameter is only considered if T_h > 0.
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
%   K = ndiFeedbackGainLqr([0.1,1,10],3,0.015)
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
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if isempty(varargin)
    is_fb_deriv = true;
else
    is_fb_deriv = varargin{1};
end

% detect system order
sys_order = length(e_x_max);

is_first_order_delay = T_h > 0;

if is_first_order_delay
    relative_degree = sys_order - 1;
else
    relative_degree = sys_order;
end

% create weighting matrices from the "maximum allowable deviations" (see
% [1], p. 406)
Q = diag( 1./(e_x_max.^2) );
R = diag( 1./(Delta_ny_max.^2) );

% create state-space model (same as above block diagram)
[A,B] = ndiOpenLoopSs( relative_degree, T_h );

% compute LQR state feedback
if is_fb_deriv || ~is_first_order_delay
    % full state feedback
    [~,~,K] = care(A,B,Q,R);
else
    % define all states as outputs except of e_y_dtN
    C = zeros(sys_order-1,sys_order);
    C(1:sys_order-1,1:sys_order-1) = eye(sys_order-1);
    % output feedback (output is the state vector without e_y_dtN)
    Ky = lqrY(A,B,C,Q,R,0);
    % transform output feedback to state feedback
    K = Ky*C;
end

end
