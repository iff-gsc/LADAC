function c_L = airfoilAnalytic9090AlCl(beta,alpha)
% analytic function for the lift coefficient for angle of attack / alpha 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

alpha0 = beta(1);
f1 = beta(2);
f2 = beta(3);
s1l = beta(4);
s1r = beta(5);
b1l = beta(6);
b1r = beta(7);
b2l = beta(8);
b2r = beta(9);
srdelta = beta(10);
sldelta = beta(11);


s2l = s1l + sldelta;
s2r = s1r + srdelta;

c_L = f1.*90/pi.*sin(pi/90*(alpha-alpha0)) ./ ( (exp(b1l.*(-alpha-s1l))+1) .*(exp(b1r.*(alpha-s1r))+1) ) ...
    + f2 * sin(pi/90*alpha) .* (1-1./( (exp(b2l.*(-alpha-s2l))+1) .* (exp(b2r.*(alpha-s2r))+1)) );

% c_L_alpha_max = 

end