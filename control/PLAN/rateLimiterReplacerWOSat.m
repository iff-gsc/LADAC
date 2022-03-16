function rateLimiterReplacerWOSat( simulinkMdl, gainStrEntry, setColor )
%RATELIMITERREPLACER   This function creates a rate limiter replacement using a
%   PT-1 elemrnt with a discrete time integrator as an additional state.
%   This rate limiter replacement was created as an approximation for model
%   linearizations. Furthermore, it uses a continuous replacement for the
%   saturation block.
%
% Syntax:  rateLimiterReplacer( simulinkMdl, gainStrEntry, setColor )
%
% Required Inputs:
%      simulinkMdl:         String that specifies in which path and all blocks
%                           inside the rate limiter must be replaced.
%      gainStrEntry:        String which is placed in the gain of the PT-1,
%                           i.e. '1 / ( 50 * 5.E-3 )' or '50'.
%      setColor:            Boolean value which colors the new in- and output
%                           block backrounds with green for input and red for
%                           outputs, respectively, if true is given.
%
% Example:
%    rateLimiterReplacer( 'RateLimiterTest', '1 / ( 10 * sampletime )', true )
%
% See also: RateLimiterReplacer,  createSaturation

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    %% Debug settings
%     simulinkMdl = 'RateLimiterTest';
%     setColor = true;
%     gainStrEntry = '1 / ( 10 * sampletime )';

    RLObjects = find_system( simulinkMdl, 'BlockType', 'RateLimiter' );

    for i = 1:length( RLObjects )
%     i = 1;
       %% Reset view that all blocks fit to view
        set_param( gcs, 'Zoomfactor','fit to view' );
       %% Initialize variables of the rate limiter
        % Get the block path
        blkPath = get_param( RLObjects{i}, 'Parent' );
        % Get the RateLimiter block handle
        RLBlkHandle = get_param( RLObjects{i}, 'handle');
        % Get the block name
        RLBlkName = get_param( RLBlkHandle, 'Name' );
        RLBlkName = regexprep( RLBlkName, '\/', '|' );
        RLBlkName = matlab.lang.makeValidName( RLBlkName );

        % Get the rate limiter port handles
        RLPortHandles = get_param( RLObjects{i}, 'PortHandles' );

        % Get signal line and port handles
        srcSignal = get_param( RLPortHandles.Inport, 'Line' );
        srcPort = get_param( srcSignal, 'SrcPortHandle' );

        dstSignal = get_param( RLPortHandles.Outport, 'Line' );
        dstPort = get_param( dstSignal, 'DstPortHandle' );

        %% Setup and create the new blocks
        % delete the lines
        delete_line( srcSignal );
        delete_line( dstSignal );

        % Get the rate limiter parameter
        xyPos = get_param( RLPortHandles.Inport, 'Position' );
        risingSlewRate = get_param( RLBlkHandle, 'RisingSlewLimit' );
        fallingSlewRate = get_param( RLBlkHandle, 'FallingSlewLimit' );
        % Default Rate Limiter is 30 - half of it 15
        xyPos = xyPos + [ -85+15, -15 ];
        % delete the rate limiter
        delete_block( RLObjects{i} );

        % Create the subsystem block
        RLRplcHandle = createRateLimiterWOSat( blkPath, RLBlkName, xyPos,...
            gainStrEntry, risingSlewRate, fallingSlewRate, setColor );

        RLRplcPortHandle = get_param( RLRplcHandle, 'PortHandles' );
        %% Connect the rate limiter replacement
        % Input -- RateLimiter
        add_line( blkPath, srcPort, RLRplcPortHandle.Inport,...
            'autorouting','on');
        % RateLimiter -- Output
        for j = 1:length( dstPort )
            add_line( blkPath, RLRplcPortHandle.Outport, dstPort(j),...
                'autorouting','on');
        end
    end
end
