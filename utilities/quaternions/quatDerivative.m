function [q_bg_dt2,q_bg_dt] = quatDerivative( q_bg, omega_Kb, omega_Kb_dt )
% quatDerivative computes the second (and first) derivative of the
%   quaternion from the angular rate and acceleration.
%   To check the correctness of the result, can integrate (double
%   integrate) the outputs of this function over time and compare them with
%   the q_bg input of this function (don't forget the initial condition).
%   If there are large differences, you should check if the inputs are
%   defined in the correct axis systems (small drifting deviations are ok).
%
% Inputs:
%   q_bg            quaternion 4x1 vector (q0, q1, q2, q3) for attitude
%                   representation (see dcm2Quat, dcm2QuatCont)
%                   note that there should be no discontinuities in the
%                   signal.
%   omega_Kb        angular velocity 3x1 vector (between the same frames as
%                   the quaternion)
%   omega_Kb_dt     time derivative of the angular velocity vector (3x1)
% 
% Outputs:
%   q_bg_dt2        second time derivative of the input q_bg
%   q_bg_dt         first time derivative of the input q_bg
% 
% Literature:
%   [1] Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
% 
% See also: dcm2Quat, DCM2_q_bg_cont
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% time derivative of quaternion according to [1], eq. (1.8-14) on page 51
q_bg_dt = 0.5 * quatMultiply( q_bg, [0;omega_Kb] );
% The econd time derivative can be obtained by applying the chain rule of
% eq. (1.8-14).
q_bg_dt2 = 0.5 * ( quatMultiply( q_bg, [0;omega_Kb_dt] ) ...
    + quatMultiply( q_bg_dt, [0;omega_Kb] ) );

end