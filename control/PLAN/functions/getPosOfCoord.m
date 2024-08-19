function [ newPos ] = getPosOfCoord( pos, srcBlk, xShift, yShift )

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    srcBlkPos = get_param( srcBlk, 'Position');
    width = srcBlkPos(3) - srcBlkPos(1);
    height = srcBlkPos(4) - srcBlkPos(2);

    lShift = xShift < 0;

    newPos = [ ...
        ( pos(1) + sign( xShift ) * width * lShift + xShift ), ...
        floor( (pos(2) - yShift - height/2) ),...
        ( pos(1) + sign( xShift ) * width * ~lShift + xShift ), ...
        floor( (pos(2) - yShift + height/2) ) ];
end
