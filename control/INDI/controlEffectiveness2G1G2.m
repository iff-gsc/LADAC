function [G1,G2] = controlEffectiveness2G1G2( ny_du_red_trim, ny_du_dt_trim, u_trim, u, T_s) %#codegen
% controlEffectiveness2G1G2 computes the matrices G1 and G2 (see [1]) from 
%   the control effectiveness of a trim point and compensates that the
%   thrust and torque are usually quadratic functions w.r.t. the control
%   input.
%   G1 and G2 can be used for incremental inversion according to eq. (19)
%   (neglecting G3).
% 
% Inputs:
%   ny_du_red_trim          derivative of the pseudo control kx1 vector ny 
%                           w.r.t. the control input mx1 vector u (usually
%                           actuator dynamics are reduced) at the trim
%                           point (jacobi kxm matrix)
%   ny_du_dt_trim           derivative of the pseudo control kx1 vector ny 
%                           w.r.t. the time derivative of the control input
%                           mx1 vector u_dt at the trim point (jacobi kxm 
%                           matrix)
%   u_trim                  control input mx1 vector at the trim point 0~1
%   u                       current control input mx1 vector 0~1
%   T_s                     sample time (scalar), s
% 
% Outputs:
%   G1                      control effectiveness matrix according to [1]
%   G2                      control effectiveness matrix to compensate the 
%                           control input derivative according to [2]
% 
% Literature:
%   [1] Smeur, E. J., Chu, Q., & de Croon, G. C. (2016). Adaptive
%       incremental nonlinear dynamic inversion for attitude control of
%       micro air vehicles. Journal of Guidance, Control, and Dynamics,
%       39(3), 450-461.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

G2 = ny_du_dt_trim/T_s;

G1 = ny_du_red_trim * diag(u./u_trim) + G2;

end