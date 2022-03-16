function [A,B] = ndiOpenLoopSs( relative_degree, varargin )
% ndiOpenLoopSs creates the matrices of the state-space representation
% of a transformed SISO system with feedback linearization (chain of 
% integrators) including a first-order delay to account for higher order
% dynamics.
%   The error dynamics of the system ideally are a chain of integrators.
%   Additionally, we consider first-order delay to account for higher order
%   dynamics that can not be captured by the feedback linearization (e.g. 
%   actuator dynamics, sensor filter or inner loop dynamics of cascaded
%   controllers). Thus, the following error dynamics are considered (see
%   [1], Abb. 4.4 and Abb. 4.9):
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
% 
% Syntax:
%   [A,B] = ndiTransformedSs( sys_order )
%   [A,B] = ndiTransformedSs( sys_order, T_h )
% 
% Inputs:
%   relative_degree relative degree of the system (number of integrator
%                   blocks)
%   T_h             first-order delay time constant to account for higher
%                   order dynamics
% 
% Outputs:
%   A               system matrix of the above system
%   B               input matrix of the above system
% 
% Example:
%   [A,B] = ndiTransformedSs( sys_order, T_h )
% 
% See also:
%   ndiPrintBlocksFromSs, ndiFeedbackGainLqr
% 
% Literature:
%   [1]	Holzapfel, F. (2004). Nichtlineare adaptive Regelung eines 
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
    T_h = 0;
else
    T_h = varargin{1};
end

is_first_order_delay = T_h > 0;

if is_first_order_delay
    sys_order = relative_degree + 1;
else
    sys_order = relative_degree;
end

% create state-space model (same as above block diagram)
A = zeros(sys_order,sys_order);
A_1_diag = eye(sys_order-1);
A(1:end-1,2:end) = A_1_diag;

B = zeros(sys_order,1);

if is_first_order_delay
    A(end,end) = -1./T_h;
    B(end) = 1./T_h;
else
    B(end) = 1;
end

end
