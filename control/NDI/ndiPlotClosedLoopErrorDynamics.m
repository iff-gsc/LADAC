function [] = ndiPlotClosedLoopErrorDynamics( A, B, K )
% ndiPlotClosedLoopErrorDynamics visualize the closed-loop error dynamics
% of a transformed SISO system with feedback linearization and state
% feedback.
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
%   of the above system.
% 
% Syntax:
%   ndiPlotClosedLoopErrorDynamics( A, B, K )
% 
% Inputs:
%   A               system matrix of the above system (NxN array)
%   B               input matrix of the above system (Nx1 array)
%   K               feedback gain matrix where the first column corresponds
%                   to the first state and so on:
%                   Delta_ny_cntrl = K * e_x (with e_x(1) = e_y, e_x(2) = 
%                   e_y_dt, e_x(3) = e_y_dt2, ...)
% 
% Outputs:
%   none
% 
% Example:
%   [K,A,B] = ndiFeedbackGainLqr([0.1,1,10],3,0.015)
%   ndiPlotClosedLoopErrorDynamics( A, B, K )
% 
% See also:
%   ndiFeedbackGainLqr, ndiPrintBlocksFromSs
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

sys_order = size(A,1);

% plot eigenvalues
subplot(2,2,1)
set(0,'DefaultLegendAutoUpdate','off')
plot(complex(eig(A)),'rx')
hold on
% new system matrix: A-B*K (see [1], Eq. (5.2-4))
plot(complex(eig(A-B*K)),'bx')
legend('open loop','closed loop')
axis equal
sgrid
xlabel('Re')
ylabel('Im')
title('Eigenvalues')
hold off

% plot step response
% x_dt = (A-BK)*x + B*K*x_cmd --> x_dt = (A-BK)*x + B*K*pick_only_y*y_cmd
pick_only_y = zeros(sys_order,1);
pick_only_y(1) = 1;
Bp = B*K*pick_only_y;
C = [ 1, zeros(1,sys_order-1) ];
D = zeros(1,1);
sys = ss(A-B*K,Bp,C,D);
sys.OutputName = {'y'};
sys.InputName = {'y_{cmd}'};
[y,tOut] = step(sys);
subplot(2,2,2)
plot(tOut,1-y)
xlabel('Time in s')
ylabel('Output error e_y')
title('Error in time domain')
grid on
xl = xlim;

% bode plot
subplot(2,2,3)
bode(sys)
title('Bode plot')
grid on

% plot pseudo-control input in case of step response
% u = K*(x_cmd-x)
sys = ss(A-B*K,Bp,-K,D);
[u,tOut] = step(sys);
subplot(2,2,4)
plot(tOut,u(end)-u)
xlabel('Time in s')
ylabel('Pseudo-control input command')
title({'Pseudo-control input in time domain','(corresponding to above diagram)'})
grid on
xlim(xl)

end
