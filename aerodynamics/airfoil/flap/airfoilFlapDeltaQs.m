function delta_qs = airfoilFlapDeltaQs(F_10,F_11,V,c,delta,delta_dt)
% airfoilFlapDeltaQs computes the quasisteady equivalent angle of attack
%   due to flap deflection delta_qs, see [1].
% 
% Inputs:
%   F_10        parameter according to [1], eq. (3), see
%               airfoilFlapEffectiveness (scalar)
%   F_11        parameter according to [1], eq. (3), see
%               airfoilFlapEffectiveness (scalar)
%   V           freestream velocity (scalar), in m/s
%   c           airfoil chord (scalar), in m
%   delta       flap deflection angle (scalar), in rad
%   delta_dt    flap deflection angular rate (scalar), in rad/s
% 
% Outputs:
%   delta_qs    quasisteady equivalent angle of attack due to flap
%               deflection (see [1], eq. (5)), in rad
% 
% Literature:
%   [1] Leishman, J. G. (1994). Unsteady lift of a flapped airfoil by
%       indicial concepts. Journal of Aircraft, 31(2), 288-297.
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% [1], eq. (5)
delta_qs = F_10.*delta/pi + F_11.*delta_dt.*c./(4*pi*V);

end

