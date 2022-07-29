function [phi,delta] = dcm2Lean( M_bg )
% This function computes to angles of the reduced attitude which is often
%   used for multicopter control [1], [2].
%   However, usually these angles are not used for control directly.
%   Good luck to understand how this function works.
%   Note to myself: I made some notes on a sheet on 11 December 2020.
% 
% Inputs:
%   M_bg                DCM from g frame to b frame
% 
% Outputs:
%   phi                 lean angle or tilt angle (angle between the z_b and
%                       z_g), rad
%   delta               direction of the lean angle (angle between -z_b and
%                       x_g; but the angle is not 3D, it is the projection
%                       in the x_g,y_g-plane), rad
% 
% Literature:
%   [1] https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=8556372&casa_token=bYMNvUlwVyoAAAAA:L2g4BCxyT6HEt2OVcvhEkfwvhz2EtKX_E91_7dIC2ETEniGquvCiHLj_e1O2-EXDRt3rVkQT&tag=1
%   [2] https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
% 
% See also:
%   quatReduced, euler2Dcm, quat2Dcm
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

phi = acosReal(M_bg(3,3));

if M_bg(3,2) >= 0
    sign_m32 = -1;
else
    sign_m32 = 1;
end
delta = atan2( sign_m32*sqrtReal( sin(phi)^2 - M_bg(3,1)^2 ), -M_bg(3,1) );

end