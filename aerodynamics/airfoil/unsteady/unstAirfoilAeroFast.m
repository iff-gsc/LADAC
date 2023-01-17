function [c_L_c,c_m_c,c_L_nc,c_m_nc,alpha_E,x_dt,A,B_alpha,B_q] = unstAirfoilAeroFast( ...
    V, Ma, c, C_L_alpha, x_ac, x, alpha, q ) %#codegen
% unstAirfoilAeroFast computes the outputs and the state derivative of the
%   unsteady transsonic airfoil behavior according to [1]. The model has 8
%   states, two inputs and five outputs.
%   Important note:
%   The state vector is defined differently than in [1], because otherwise
%   the airspeed should not be varied! Transformation: x_better = A * x
%   Thanks to the transformation the amplitude of the state vector does not
%   really depend on the airspeed anymore.
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
%   SPDX-License-Identifier: GPL-3.0-only
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

Ma2 = powerFast(Ma,2);

% [1], see Nomenclature
beta2   = 1 - Ma2;
beta    = sqrtReal(beta2);

% summarization of variables
fac     = 2*V./c.*beta2;

Ma_inv  = 1./Ma;
Ma1     = 1-Ma;

% [1], between eq. (16) and eq. (17)
term_2  = pi*beta.*Ma2*(A_1*b_1+A_2*b_2);
K_alpha = kappa_1 ./ ( Ma1 + term_2 );
T_I = c./a;
% [1], eq. (A2)
K_q     = kappa_2 ./ ( Ma1 + 2*term_2 );
% [1], eq. (A6)
K_alpha_M = (A_3*b_4+A_4*b_3)./(b_3*b_4*Ma1);
% [1], eq. (A12)
K_q_M   = 7./(15*Ma1+3*pi*beta.*Ma2*b_5);

num_vectors = length(V);
A = zeros(8,num_vectors);

% [1], eq. (17)
B1      = fac * -b_1;
B2      = fac * -b_2;
A(1,:)  = B1;
A(2,:)  = B2;
% [1], eq. (19)
A(3,:)  = -1./K_alpha./T_I;
% [1], eq. (A3)
A(4,:)  = -1./(K_q.*T_I);
% [1], eq. (A9)
A(5,:)  = -1./(b_3*K_alpha_M.*T_I);
A(6,:)  = -1./(b_4*K_alpha_M.*T_I);
% [1], eq. (A13)
A(7,:)  = -b_5*fac;
% [1], eq. (A14)
A(8,:)  = -1./(K_q_M.*T_I);

% [1], eq. (18)
c_11    = C_L_alpha * A_1;
c_12    = C_L_alpha * A_2;
% [1], eq. (19) and (20)
c_13    = -4*Ma_inv;
% [1], eq. (A3) and (A4)
c_14    = -Ma_inv;
% [1], eq. (11) and (12)
c_21    = c_11 .* ( 0.25 - x_ac );
c_22    = c_12 .* ( 0.25 - x_ac );
% [1], eq. (A14) and (A16)
c_28    = 7/12*Ma_inv;
% [1], eq. (A8)
c_25    = Ma_inv*A_3;
c_26    = Ma_inv*A_4;
% [1], eq. (A15)
c_27    = -pi./(8*beta);

% https://arc.aiaa.org/doi/pdf/10.2514/6.1989-1319
% eq. (14)
c_31 = A_1;
c_32 = A_2;

B_alpha = -A;
B_alpha([4,7,8],:) = 0;
            
B_q = -A;
B_q([3,5,6],:) = 0;
B_q(1,:) = -B1*0.5;
B_q(2,:) = -B2*0.5;

% state equation
x_dt = A.*x;

for i = 1:8
    x_dt(i,:) = x_dt(i,:) + B_alpha(i,:).*alpha + B_q(i,:).*q;
end

% outputs
x1 = x(1,:);
x2 = x(2,:);
c_L_c = c_11 .* x1 + c_12 .* x2;
c_m_c = c_21 .* x1 + c_22 .* x2 + c_27 .* x(7,:);
c_L_nc = c_13 .* x(3,:) + c_14 .* x(4,:) + 4*Ma_inv .* alpha + Ma_inv .* q;
c_m_nc = c_25 .* x(5,:) + c_26 .* x(6,:) + c_28 .* x(8,:) - Ma_inv .* alpha - 7/12*Ma_inv .* q;
alpha_E = c_31 .* x1 + c_32 .* x2;

end