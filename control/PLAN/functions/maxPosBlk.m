function [ xMax, yMax ] = maxPosBlk( blockHandle )

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    handles = find_system( blockHandle, 'FollowLinks', 'on', 'SearchDepth', 1,...
        'LookUnderMasks', 'on' );

    posMax = zeros(1, 4);
    for i = 2:length( handles )
        pos = get_param( handles{i}, 'Position');
        isBigger = pos > posMax;
        posMax(isBigger) = pos(isBigger);
    end
    xMax = max( posMax( [ 1 3 ] ) );
    yMax = max( posMax( [ 2 4 ] ) );
end
