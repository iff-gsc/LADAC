function saturationHandle = createSaturation( simulinkPth, saturationName, ...
    xyPos, srcPort, dstPort, minS, maxS )
%CREATESATURATION   This function creates a Saturation block inside Simulink
%   This function uses the saturate function when creating the saturation block.
%   It connects the in and outports automatically.
%
% Syntax:  createSaturation( simulinkPth, saturationName, xyPos,...
%               srcPort, dstPort, minS, maxS )
%
% Required Inputs:
%      simulinkPth:         Given as a string which specifies the Simulink path,
%                           where the saturation should be placed.
%      saturationName:      A string that specifies the name.
%      xyPos:               The block position given as an array
%                           [ xPosition, yPosition ].
%      srcPort:             The source port handle that provide the signal which
%                           should be saturated.
%      dstPort:             The destiniation port handle, where the saturated
%                           signal is used.
%      minS:                The minimal saturation value.
%      maxS:                The maximal saturation value.
%
% Output Arguments:
%      saturationHandle:    The handle of the created saturation block.
%
% Example:
%    saturationHandle = createSaturation( 'saturationTester', 'Saturation',...
%       [0,0], aH.Outport, bH.Inport, -.5, 9 )
%    saturationHandle = createSaturation( 'Planner_input/rate_limit_ldyn', 'Saturation',...
%       [0,0], 1.5166e+04, 1.0149e+04, -.5, 9 )
%
% See also: saturate,  saturationReplacer,  rateLimiterReplacer,  getObjects

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%     simulinkPth = 'saturationTester';
%     saturationName = 'Saturation';
%     xyPos = [ 420,   190 ];
%     srcPort =
%     dstPort =
%     minS = -.15;
%     maxS = 4.2;

    % Saturation block size
    xWidth = 100;
    yWidth = 130;

    saturationFullPth = [ simulinkPth, '/', saturationName ];

    % Create the block
	saturationHandle = add_block( 'simulink/User-Defined Functions/MATLAB Function',...
    saturationFullPth, 'MAKENAMEUNIQUE', 'ON' );

    saturationFullPth = [ get_param( saturationHandle, 'Parent'), '/',...
            get_param( saturationHandle, 'Name') ];
    saturationSFHandle = find( slroot, '-isa', 'Stateflow.EMChart', 'Path',...
            saturationFullPth );
    saturationSFHandle.Script = fileread( 'saturate.m' );
    % Set the block position
    position = [ xyPos(1),   xyPos(2),   xyPos(1) + xWidth,   xyPos(2 ) +  yWidth ];
    set_param( saturationHandle, 'Position', position );

    % Connect the lines
    saturationPortHandles = get_param( saturationHandle, 'PortHandles');
    add_line( simulinkPth, srcPort, saturationPortHandles.Inport(1),...
            'autorouting','on');
    for j = 1:length( dstPort )
        add_line( simulinkPth, saturationPortHandles.Outport, dstPort(j),...
            'autorouting','on');
    end

    % Create min max saturation constant blocks
    minSBlkName = [ simulinkPth, '/', 'minS' ];
    maxSBlkName = [ simulinkPth, '/', 'maxS' ];

    minSBlkHandle = add_block( 'simulink/Sources/Constant',...
    minSBlkName, 'MAKENAMEUNIQUE', 'ON' );
	maxSBlkHandle = add_block( 'simulink/Sources/Constant',...
    maxSBlkName, 'MAKENAMEUNIQUE', 'ON' );

    % Set the position of the constants
    minSPos = getHorizPosOf( saturationPortHandles.Inport(2), minSBlkHandle, -30 );
    set_param( minSBlkHandle, 'Position', minSPos );

    maxSPos = getHorizPosOf( saturationPortHandles.Inport(3), maxSBlkHandle, -30 );
    set_param( maxSBlkHandle, 'Position', maxSPos );
    % Connect the min max blocks to the saturation replacement
    minSPortHandles = get_param( minSBlkHandle, 'PortHandles' );
    add_line( simulinkPth, minSPortHandles.Outport, saturationPortHandles.Inport(2),...
        'autorouting','on');
    maxSPortHandles = get_param( maxSBlkHandle, 'PortHandles' );
    add_line( simulinkPth, maxSPortHandles.Outport, saturationPortHandles.Inport(3),...
        'autorouting','on');
    % Set the saturation values
    set_param( minSBlkHandle, 'Value', num2str(minS) );
    set_param( maxSBlkHandle, 'Value', num2str(maxS) );
end
