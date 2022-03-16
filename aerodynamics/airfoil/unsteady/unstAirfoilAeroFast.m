function [c_L_c,c_m_c,c_L_nc,c_m_nc,alpha_E,x_dt] = unstAirfoilAeroFast( ...
    V, Ma, c, C_L_alpha, x_ac, x, alpha, q ) %#codegen
% unstAirfoilAeroFast computes the outputs and the state derivative of the
%   unsteady transsonic airfoil behavior according to [1]. The model has 8
%   states, two inputs and five outputs.
%   This function is optimized for the computation of multiple
%   airfoils/sections simultaneously. In this case the use of a state-space
%   model with concentrated matrices (A,B,C,D) is not fast.
% 
% Inputs:
%   V           airspeed (1xn vector), in m/s
%   Ma          Mach number (1xn vector), in 1
%   c           chord (1xn vector), in m
%   C_L_alpha   lift curve slope (1xn vector), in 1/rad
%   x_ac        aerodynamic center measured from the nose of the profile
%               (1xn vector), in m
%   x           state (8xn matrix)
%   alpha       angle of attack (1xn vector), in rad
%   q           dimensionless pitch rate (1xn vector)
% 
% Outputs:
%   c_L_c       circulatory lift coefficient (1xn vector)
%   c_m_c       circulatory pitching moment coefficient (1xn vector)
%   c_L_nc      non-circulatory lift coefficiehnt (1xn vector)
%   c_m_nc      non-circulatory pitching moment coefficient (1xn vector)
%   alpha_E     effective angle of attack (1xn vector) corresponding to 
%               c_L_c, rad
%   x_dt        time-derivative of the state x (8xn matrix)
% 
% Literature:
%   [1] Leishman, J. G., and Nguyen, K. Q. (1990). State-space 
%       representation of unsteady airfoil behavior. AIAA journal, 28(5),
%       836-844 (https://arc.aiaa.org/doi/pdf/10.2514/3.25127).
% 
% Authors:
%   Yannic Beyer
% 
% See also:
%   unstProfileAero
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% speed of sound
a = V./Ma;

% [1], below eq. (16), below eq. (A6) and below eq. (A10)
% A_1     = 0.3;
% A_2     = 0.7;
A_3     = 1.5;
A_4     = -0.5;
% b_1     = 0.14;
% b_2     = 0.53;
b_3     = 0.25;
b_4     = 0.1;
b_5     = 0.5;

% https://arc.aiaa.org/doi/pdf/10.2514/3.46340?casa_token=aPPVCTXUrfQAAAAA:t6WQNNjPPaLV3ZqyXi3m5WHwMBIFNyQSchK1pl-CqKQjUqcYchQOiUA8UyNRjyA_V2yJXNjrFQ
A_1     = 0.625;
A_2     = 0.375;
b_1     = 0.310;
b_2     = 0.312;
kappa_1 = 0.85;
kappa_2 = 0.73;

% [1], see Nomenclature
beta2   = 1 - Ma.^2;
beta    = sqrtReal(beta2);

% summarization of variables
fac     = 2*V./c.*beta2;

% [1], between eq. (16) and eq. (17)
term_2  = pi*beta.*Ma.^2*(A_1*b_1+A_2*b_2);
K_alpha = kappa_1 ./ ( (1-Ma) + term_2 );
T_I = c./a;
% [1], eq. (A2)
K_q     = kappa_2 ./ ( (1-Ma) + 2*term_2 );
% [1], eq. (A6)
K_alpha_M = (A_3*b_4+A_4*b_3)./(b_3*b_4*(1-Ma));
% [1], eq. (A12)
K_q_M   = 7./(15*(1-Ma)+3*pi*beta.*Ma.^2*b_5);

% [1], eq. (17)
a_11    = fac * -b_1;
a_22    = fac * -b_2;
% [1], eq. (19)
a_33    = -1./K_alpha./T_I;
% [1], eq. (A3)
a_44    = -1./(K_q.*T_I);
% [1], eq. (A9)
a_55    = -1./(b_3*K_alpha_M.*T_I);
a_66    = -1./(b_4*K_alpha_M.*T_I);
% [1], eq. (A13)
a_77    = -b_5*fac;
% [1], eq. (A14)
a_88    = -1./(K_q_M.*T_I);

% [1], eq. (18)
c_11    = C_L_alpha .* fac * A_1 * b_1;
c_12    = C_L_alpha .* fac * A_2 * b_2;
% [1], eq. (19) and (20)
c_13    = 4./Ma .* a_33;
% [1], eq. (A3) and (A4)
c_14    = 1./Ma .* a_44;
% [1], eq. (11) and (12)
c_21    = c_11 .* ( 0.25 - x_ac );
c_22    = c_12 .* ( 0.25 - x_ac );
% [1], eq. (A14) and (A16)
c_28    = -7./(12*Ma) .* a_88;
% [1], eq. (A8)
c_25    = -1./Ma*A_3.*a_55;
c_26    = -1./Ma*A_4.*a_66;
% [1], eq. (A15)
c_27    = -pi./(8*beta)*b_5.*fac;

% https://arc.aiaa.org/doi/pdf/10.2514/6.1989-1319
% eq. (14)
c_31 = fac .* A_1 * b_1;
c_32 = fac .* A_2 * b_2;

% [1], below eq. (21)

num_vectors = length(V);

A = [ a_11; a_22; a_33; a_44; a_55; a_66; a_77; a_88 ];

B_alpha = repmat( [ 1, 1, 1, 0, 1, 1, 0, 0 ]', 1, num_vectors );
            
B_q = repmat( [ 0.5, 0.5, 0, 1, 0, 0, 1, 1 ]', 1, num_vectors );

% state equation
x_dt = A.*x + B_alpha.*repmat(alpha,8,1) + B_q.*repmat(q,8,1);

% outputs
c_L_c = c_11 .* x(1,:) + c_12 .* x(2,:);
c_m_c = c_21 .* x(1,:) + c_22 .* x(2,:) + c_27 .* x(7,:);
c_L_nc = c_13 .* x(3,:) + c_14 .* x(4,:) + 4./Ma .* alpha + 1./Ma .* q;
c_m_nc = c_25 .* x(5,:) + c_26 .* x(6,:) + c_28 .* x(8,:) - 1./Ma .* alpha - 7/12*Ma .* q;
alpha_E = c_31 .* x(1,:) + c_32 .* x(2,:);

end