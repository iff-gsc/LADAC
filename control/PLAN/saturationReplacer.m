function saturationReplacer( simulinkMdl )
%SATURATIONREPLACER   This function replaces all saturations with a saturation replacement
%   This function searches for all saturations inside the given path simulinkMdl
%   and replaces them with a saturaion replacement (saturate function).
%
% Syntax:  saturationReplacer( simulinkMdl )
%
% Required Inputs:
%      simulinkMdl:         A string which specifies the Simulink model path,
%                           where the saturation should be replaced.
%
% Example:
%    saturationReplacer( 'Norbert_linearize_sim/Referenz_Fzg_Modell' )
%    saturationReplacer( 'saturationTester' )
%
% See also: saturate,  createSaturation,  rateLimiterReplacer,  getObjects

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%     simulinkMdl = 'saturationTester';
%     setColor = true;
    satObjects = find_system( simulinkMdl, 'BlockType', 'Saturate' );

    for i = 1:length( satObjects )
%     i = 1;
       %% Reset view that all blocks fit to view
        set_param( gcs, 'Zoomfactor','fit to view' );
       %% Initialize variables for the saturation replacement
        % Get the block path
        blkPath = get_param( satObjects{i}, 'Parent' );
        % Get the RateLimiter block handle
        satBlkHandle = get_param( satObjects{i}, 'handle');
        % Get the block name
        satBlkName = get_param( satBlkHandle, 'Name' );
        % Get the rate limiter port handles
        satPortHandles = get_param( satObjects{i}, 'PortHandles' );

        % Get signal line and port handles
        srcSignal = get_param( satPortHandles.Inport, 'Line' );
        srcPort = get_param( srcSignal, 'SrcPortHandle' );

        dstSignal = get_param( satPortHandles.Outport, 'Line' );
        dstPort = get_param( dstSignal, 'DstPortHandle' );

        lowerLimit = get_param( satBlkHandle, 'LowerLimit' );
        upperLimit = get_param( satBlkHandle, 'UpperLimit' );


        satPosition = get_param( satBlkHandle, 'Position' );
        xyPos = [ satPosition(1), satPosition(2) ];

        %% Delete the saturation block and the connected lines
        % delete the saturation block
        delete_block( satObjects{i} );
        % delete the lines
        delete_line( srcSignal );
        delete_line( dstSignal );

        createSaturation( blkPath, satBlkName, xyPos, srcPort, dstPort, ...
            lowerLimit, upperLimit );
    end
end
