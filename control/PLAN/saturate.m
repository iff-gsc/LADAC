function y = saturate( u, minS, maxS ) %#codegen
%SATURATE   This function is a continuous replacement for the Saturate block
%   A Taylor expension is used for the approximation of the saturation.
%   Depending on the computing performance and accuracy you can choose the order
%   of terms with the variable taylorOrd. The default value is 8.
%   This function is embedded capable. The use for this function is provided as
%   a matlab function block inside simulink.
%
% Syntax:  saturate( u, minS, maxS )
%
% Required Inputs:
%      u:       The signal which should be saturated.
%      minS:    The minimal saturation value.
%      maxS:    The maximal saturation value.
%
% Output Arguments:
%      y:       The saturated input value u.
%
% Example:
%    y = saturate( u, -2, .8 )
%
% See also: saturationReplacer,  RateLimiterReplacer,  getObjects

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    taylorOrd = 8;
    taylorU = 0;

    for i = 1:taylorOrd
        taylorU = taylorU + u.^( 2*(i-1) + 1 ) / ( 2*(i-1) + 1 );
    end

    y = tanh( taylorU );
    y = ( maxS - minS )/2*( y + 1 ) + minS;
end
