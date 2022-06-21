function [F_10,F_11]= airfoilFlapEffectiveness(E)
% airfoilFlapEffectiveness computes geometric flap effectiveness parameters
%   F_10 and F_11 are parameters that are needed to compute the quasisteady
%   equivalent angle of attack due to flap deflection (F_10) and deflection
%   rate (F_11), see [1].
% 
% Inputs:
%   E           flap length relative to the chord
% 
% Outputs:
%   F_10        geometric parameter for the flap deflection effectiveness,
%               see [1], eq. (3)
%   F_11        geometric parameter for the flap deflection rate
%               effectiveness, see [1], eq. (3)
% 
% Literature:
%   [1] Leishman, J. G. (1994). Unsteady lift of a flapped airfoil by
%       indicial concepts. Journal of Aircraft, 31(2), 288-297.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% convert relative flap length to "e", see [1], fig. 1
e = 1-2*E;

% [1], eq. (3)
F_10 = sqrt(1-powerFast(e,2)) + acos(e);
F_11 = (1-2*e).*acos(e) + (2-e).*sqrt(1-powerFast(e,2));

end

