function c_D = airfoilAnalytic9090AlCd(beta,alpha)
% analytic function for the drag coefficient for angle of attack / alpha 
% 
% Inputs:
%   beta            parameter vector
%   xy              concentrated vectors of angle of attack (first column)
%                   and drag coefficient (second column)
% 
% Outputs:
%   c_D             drag coefficient vector
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************



c_D0 = beta(1);

f1 = beta(2);
alpha0 = beta(3);
s1l = beta(4);
s1delta = beta(5);

b1l = beta(6);
b1r = beta(7);

f2 = beta(8);
aexp = beta(9);

b2r = beta(10);
s2delta = beta(11);
c_D1 = beta(12);

c_D2 = -f2 + c_D1;

b2l = b1l;

s1r = s1l + s1delta;
s2l = s1l;
s2r = s1r + s2delta;


c_D = c_D0 + (f1*0.001*((alpha-alpha0).^2)) ./ ( (exp(b1l*(-alpha-s1l))+1) .* (exp(b1r*(alpha-s1r))+1) ) ...
    + max(0,f2*cos(pi/90.^aexp*abs(alpha).^aexp)+c_D2) .* ( 1 - 1 ./ ( (exp(b2r*(-alpha-s2l))+1) .* (exp(b2l*(alpha-s2r))+1)) );


end