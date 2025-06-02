function F = airfoilFlapEffectiveness(E)
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
%   [2] Theodorsen, T. (1935). General Theory of Aerodynamic Instability
%       and the Mechanism of Flutter. NACA Rept. 496.
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% convert relative flap length to "e", see [1], fig. 1
e = 1-2*E;

e2 = powerFast(e,2);
sqrt_1_e2 = sqrtReal(1-e2);
acos_e = acosReal(e);

% [1], eq. (3)
F.e = e;
F.F_1 = -1/3*sqrt_1_e2.*(2+e2)+e.*acos_e;
F.F_4 = -acos_e+e.*sqrt_1_e2;
% [2], p. 5
F.F_8 = -1/3*sqrt_1_e2.*(2*e2+1)+e.*acos_e;
% [1], eq. (3)
F.F_10 = sqrt_1_e2 + acos_e;
F.F_11 = (1-2*e).*acos_e + (2-e).*sqrt_1_e2;

end

