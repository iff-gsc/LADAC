function c_m_flap = airfoilFlapMoment(c_L_flap,lambda_k) %#codegen
% airfoilFlapMoment computes the flap moment from the flap lift [1].
%   The pitching moment reference point is at 25% of the airfoil chord.
% 
% Inputs:
%   c_L_flap    lift coefficient due to flap deflection (1xN array)
%   lambda_k    depth of the flap relative to the airfoil chord (1xN array),
%               dimensionless
% 
% Outputs:
%   c_m_flap    pitching moment coefficient due to flap deflection (1xN
%               array)
% 
% Literature:
%   [1] Schlichting, H. & Truckenbrodt, E. (2001): Aerodynamik des
%       Flugzeuges - Teil 1, Springer.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% [1], eq. (6.106)
c_m_d_eta = -2*sqrt(lambda_k.*powerFast(1-lambda_k,3));

% [1], eq. (6.105)
alpha_d_eta = -2/pi*( sqrt(lambda_k.*(1-lambda_k))+asin(sqrt(lambda_k)) );

% lift curve slope @c_L/@alpha
c_L_d_alpha = 2*pi;

% derivative of pitching moment coeff. w.r.t lift coeff. @c_m/@c_L
c_m_d_c_L = c_m_d_eta .* 1./max(eps(c_L_flap),abs(alpha_d_eta)) .* 1./c_L_d_alpha;

% now the flap pitching moment coeff. can be computed from lift coeff.
c_m_flap = c_m_d_c_L .* c_L_flap;

end

