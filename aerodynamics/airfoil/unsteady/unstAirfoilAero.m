function [A,B,C,D] = unstAirfoilAero( V, Ma, c, C_L_alpha, x_ac ) %#codegen
% unstAirfoilAero computes the matrices of a state-space representation for
%   unsteady transsonic airfoil behavior according to [1]. The model has 8
%   states, two inputs and three outputs (the effective angle of attack was
%   added as a third output).
% 
% Inputs:
%   V           airspeed (scalar), in m/s
%   Ma          Mach number (scalar), in 1
%   c           chord (scalar), in m
%   C_L_alpha   lift curve slope (scalar), in 1/rad
%   x_ac        aerodynamic center measured from the nose of the profile
%               (scalar), in m
% 
% Outputs:
%   A           state matrix (8x8)
%   B           input matrix (8x2)
%   C           output matrix (2x8)
%   D           feedthrough matrix (2x2)
% 
% Literature:
%   [1] Leishman, J. G., and Nguyen, K. Q. (1990). State-space 
%       representation of unsteady airfoil behavior. AIAA journal, 28(5),
%       836-844 (https://arc.aiaa.org/doi/pdf/10.2514/3.25127).
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
A_1     = 0.636;
A_2     = 0.364;
b_1     = 0.339;
b_2     = 0.249;
kappa_1 = 0.77;
kappa_2 = 0.70;

% [1], see Nomenclature
beta2   = 1 - Ma.^2;
beta    = sqrt( beta2 );

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
A       = zeros(8,8,num_vectors);
B       = zeros(8,2,num_vectors);
C       = zeros(3,8,num_vectors);
D       = zeros(3,2,num_vectors);
for i = 1:num_vectors
    A(:,:,i) = diag( [ a_11(i), a_22(i), a_33(i), a_44(i), a_55(i), a_66(i), a_77(i), a_88(i) ] );
    B(:,:,i) = [ ...
                1, 1, 1, 0, 1, 1, 0, 0; ...
                0.5, 0.5, 0, 1, 0, 0, 1, 1 ...
                ]';
    C(:,:,i) = [ ...
                c_11(i), c_12(i), c_13(i), c_14(i), 0, 0, 0, 0; ...
                c_21(i), c_22(i), 0, 0, c_25(i), c_26(i), c_27(i), c_28(i); ...
                c_31(i), c_32(i), 0, 0, 0, 0, 0, 0 ...
                ];
    D(:,:,i) = [ ...
                4/Ma(i), 1/Ma(i); ...
                -1/Ma(i), -7/12*Ma(i); ...
                0, 0 ...
                ];
end
Bv = [ ...
    1, 1, 1, 0, 1, 1, 0, 0; ...
    0.5, 0.5, 0, 1, 0, 0, 1, 1 ...
    ]';
B = repmat( Bv, 1, 1, num_vectors );

end