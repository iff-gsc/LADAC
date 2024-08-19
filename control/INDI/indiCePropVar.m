function [G1,G2] = indiCePropVar( cep, G10, G20, u ) %#codegen
% indiCePropVar computes the matrices G1 and G2 (see [1]) from 
%   the control effectiveness of a trim point and compensates that the
%   thrust and torque are usually quadratic functions w.r.t. the control
%   input.
%   G1 and G2 can be used for incremental inversion according to eq. (19)
%   (neglecting G3).
% 
% Inputs:
%   cep             propeller control effectiveness parameters struct, see
%                   indiCeProp_params_default
%   G10             control effectiveness matrix, see indiCePropFix
%   G20             control effectiveness matrix, see indiCePropFix
% 
% Outputs:
%   G1              control effectiveness matrix according to [1]
%   G2              control effectiveness matrix to compensate the control
%                   input derivative according to [2]
% 
% Literature:
%   [1] Smeur, E. J., Chu, Q., & de Croon, G. C. (2016). Adaptive
%       incremental nonlinear dynamic inversion for attitude control of
%       micro air vehicles. Journal of Guidance, Control, and Dynamics,
%       39(3), 450-461.
%   [2] Beyer, Y., Guecker, F., Bremers, E., Steen, M., & Hecker, P.
%       (2022). Incremental Passive Fault-Tolerant Control for Quadrotors
%       With up to Three Successive Rotor Failures. In 6th CEAS Conference
%       on Guidance, Navigation and Control (EuroGNC) 2022.

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

omega = motorStaticSpeed( cep.kt, cep.ri, cep.vb, cep.d, u );

omega_du = motorStaticSpeedDeriv( cep.kt, cep.ri, cep.vb, cep.d, u );

G_omega = diag( omega_du );

G1 = G10 * diag( omega ) * G_omega;

G2 = G20 * G_omega;

end