function [ newPos ] = getHorizPosOf( destPortHandle, srcBlk, xShift )

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    destPortpos = get_param( destPortHandle, 'Position');
    srcBlkPos = get_param( srcBlk, 'Position');
    width = srcBlkPos(3) - srcBlkPos(1);
    height = srcBlkPos(4) - srcBlkPos(2);

    lShift = xShift < 0;

    newPos = [ ...
        ( destPortpos(1) + sign( xShift ) * width * lShift + xShift ), ...
        floor( (destPortpos(2) - height/2) ),...
        ( destPortpos(1) + sign( xShift ) * width * ~lShift + xShift ), ...
        floor( (destPortpos(2) + height/2) ) ];
end
