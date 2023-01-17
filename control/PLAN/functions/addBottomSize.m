function addBottomSize( blkHandle, size )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    pos = get_param( blkHandle, 'Position' );
    pos(4) = pos(4) + size;
    set_param( blkHandle, 'Position', pos);

end
