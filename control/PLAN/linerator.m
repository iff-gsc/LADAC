function linerator( simulinkMdl, blkToBeReplaced, setColor )
%linerator   Replaces the integrators in the given simulink block path by in-/outputs
%   This function searches for integrators and replaces them with
%   corresponding input and output blocks named by the integrator name and a
%   prefix "x_" and the suffix "P_out" for the derivativ output and the suffix
%   "in" for the value input.
%   1. This function could be used for linearization of a block.
%   2. Furthermore it can be used to transform the model, where all states are
%   defined at the in and output where the integrator could be placed outside
%   the model in the form
%       \dot{x} = f( x, u ).
%
% Syntax:  linerator( simulinkMdl, blkToBeReplaced, setColor )
%           
% Inputs:
% 	simulinkMdl         String which contains a Simulink model path which should be
%                       transformed in the form \dot{x} = f( x, u ).
%   blkToBeReplaced     Simulink 'BlockType' that shall be replaced,
%                       usually 'DiscreteIntegrator' or 'Integrator'.
% 	setColor            Boolean value which colors the new in- and output block
%                       backrounds with green for input and red for outputs,
%                       respectively, if true is given.
%
% Example:
%    linerator( 'Norbert_linearize_sim/Referenz_Fzg_Modell', ...
%       'DiscreteIntegrator', true )
%    linerator( 'my_model/commonRail', 'Integrator', false )
%
% See also: createIntegratorOutBus

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2021 Alexander Kuzolap
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%LINERATOR Summary of this function goes here
%   Detailed explanation goes here
    %simulinkMdl = 'linearizerTester';
    IntObjects = find_system( simulinkMdl, 'BlockType', blkToBeReplaced );
    %i = 1;
    %setColor = true;
    for i = 1:length( IntObjects )
        % Reset view that all fit
        set_param( gcs, 'Zoomfactor','fit to view' );

        % Get the block path
        blkPath = get_param( IntObjects{i}, 'Parent' );
        % Create block names
        outPortName = [ 'x_', get_param(IntObjects{i}, 'Name'),'P_out' ];
        inPortName = [ 'x_', get_param(IntObjects{i}, 'Name'),'_in' ];
        outPortName = regexprep( outPortName, '\/', '|' );
        inPortName = regexprep( inPortName, '\/', '|' );
        % Convert potential output and later bus (field name) into one that is valid
        outPortName = matlab.lang.makeValidName( outPortName );
        inPortName = matlab.lang.makeValidName( inPortName );

        outPortBlk = [ blkPath, '/', outPortName ];
        inPortBlk = [ blkPath, '/', inPortName ];

        % Get the integrator port handles
        intPortHandles = get_param( IntObjects{i}, 'PortHandles' );

        % Add the blocks with the correct position
        outPortBlkHandle = add_block( 'simulink/Sinks/Out1', outPortBlk, ...
            'MAKENAMEUNIQUE', 'ON' );
        inPortBlkHandle = add_block( 'simulink/Sources/In1', inPortBlk, ...
            'MAKENAMEUNIQUE', 'ON' );

        % Give some warnings
        outPortNameNew = get_param( outPortBlkHandle, 'Name' );
        if not( isequal( outPortName, outPortNameNew ) )
            warning( [ sprintf( 'A output block %s already exists.\n', outPortName ), ...
                 sprintf( 'The output port was renamed to %s.', outPortNameNew ) ] );
             outPortName = outPortNameNew;
        end
        inPortNameNew = get_param( inPortBlkHandle, 'Name' );
        if not( isequal( inPortName, inPortNameNew ) )
            warning( [ sprintf( 'A input block %s already exists.\n', inPortName ), ...
                 sprintf( 'The input port was renamed to %s.', inPortNameNew ) ] );
             inPortName = inPortNameNew;
        end

        % Set the block position
        outPortBlkHandlePos = getHorizPosOf( intPortHandles.Inport(1), outPortBlkHandle, -1 );
        inPortblkPos = getHorizPosOf( intPortHandles.Outport, outPortBlkHandle, 1 );

        set_param( outPortBlkHandle, 'Position', outPortBlkHandlePos );
        set_param( inPortBlkHandle, 'Position', inPortblkPos );

        if setColor
            set_param(  outPortBlkHandle, 'BackgroundColor', 'red' );
            set_param(  inPortBlkHandle, 'BackgroundColor', 'green' );
        end

        % Get port handles of added in out ports
        outPortHandle = get_param( outPortBlkHandle, 'PortHandles' );
        inPortHandle = get_param( inPortBlkHandle, 'PortHandles' );

        % Get the line handles of the integrator
        intLineHandles = get_param( IntObjects{i}, 'LineHandles' );

        % Check if we have more than one input into the integrator
        ExternalReset =  get_param( IntObjects{i}, 'ExternalReset' );
        InitialConditionSource = get_param( IntObjects{i}, 'InitialConditionSource' );

        isExternalReset = ~isequal( ExternalReset, 'none' );
        isExternal = isequal( InitialConditionSource, 'external' );
        if isExternalReset || isExternal
            % Create a terminator block name
            terminatorBlk = [ blkPath, '/', 'Terminator' ];

            % for all source port
            for iports = 2:length( intPortHandles.Inport )
                % Get signal line and port handles
                srcSignal = get_param( intPortHandles.Inport( iports ), 'Line' );
                srcPort = get_param( srcSignal, 'SrcPortHandle' );

                % delete the line
                delete_line( srcSignal );

                % Add a terminator
                terminatorHandle = add_block( 'simulink/Sinks/Terminator', terminatorBlk, ...
                    'MAKENAMEUNIQUE', 'ON' );
                % Get the a position for the terminator
                terminatorPos = getHorizPosOf( ...
                    intPortHandles.Inport( iports ), terminatorHandle, -1 );
                set_param( terminatorHandle, 'Position', terminatorPos );
                set_param( terminatorHandle, 'ShowName', 'off' );

                % Get the terminator input port handle
                terminatorPortHandles = get_param( terminatorHandle, 'PortHandles' );

                % Connect the terminator to the src
                add_line( blkPath, srcPort, terminatorPortHandles.Inport, ...
                    'autorouting','on' );
            end
        end
        % Get source port
        srcSignal = get_param( intPortHandles.Inport(1), 'Line' );
        srcPort = get_param( srcSignal, 'SrcPortHandle' );

        % Get destination port
        destSignal = get_param( intPortHandles.Outport, 'Line' );
        destPort = get_param( destSignal, 'DstPortHandle' );

        delete_block( IntObjects{i} );
        signalName = get_param( intLineHandles.Outport, 'Name' );
        delete_line( intLineHandles.Inport(1) )
        delete_line( intLineHandles.Outport )

        % Reconnect
        add_line( blkPath, srcPort, outPortHandle.Inport, 'autorouting', 'on' );
        for j = 1:length( destPort )
            lineHandle = add_line( blkPath, inPortHandle.Outport, ...
                destPort(j), 'autorouting', 'on');
%             signalNameCell = get_param( outPortBlkHandle, 'InputSignalNames' );
%             signalNameCell = erase( signalNameCell, "<" );
%             signalNameCell = erase( signalNameCell, ">" );
            set_param( lineHandle, 'Name', signalName );
        end
        
        blkPathParent = get_param( blkPath, 'Parent' );
        while ~isempty( blkPathParent )

            % Switch to the system and fit zoom
            open_system( blkPathParent );
            set_param( gcs, 'Zoomfactor', 'fit to view' );

            % Resize the subblock
            addBottomSize( blkPath, 42 );

        % One possible, but maybe ugly way to obtain the portnames and get
        % the handle by the port name
%         handles = find_system(blkPathParent,...
%             'LookUnderMasks', 'on',...
%             'FollowLinks', 'on', 'SearchDepth', 1, 'BlockType', 'Inport');
%         portNames = cellstr(get_param(handles, 'Name'))

            % It is maybe easier to get the port number and hope that the
            % porthandles are sorted in the same order

            % Get the port numbers of the ports
            inPortBlkHandlePortNum = str2num( get_param( inPortBlkHandle, 'Port' ) );
            outPortBlkHandlePortNum = str2num( get_param( outPortBlkHandle, 'Port' ) );

            % Get the destination and source port handles
            blkPathPortHandle = get_param( blkPath, 'PortHandles' );
            destPort = blkPathPortHandle.Inport( inPortBlkHandlePortNum );
            srcPort = blkPathPortHandle.Outport( outPortBlkHandlePortNum );

            % Create the parent in out blocks
            outPortBlkParent = [ blkPathParent, '/', outPortName ];
            inPortBlkParent = [ blkPathParent, '/', inPortName ];

            outPortBlkHandleParent = add_block( 'simulink/Sinks/Out1', outPortBlkParent, ...
                'MAKENAMEUNIQUE', 'ON' );
            inPortBlkHandleParent = add_block( 'simulink/Sources/In1', inPortBlkParent, ...
                'MAKENAMEUNIQUE', 'ON' );

            % Give some warnings
            outPortNameNew = get_param( outPortBlkHandleParent, 'Name' );
            if not( isequal( outPortName, outPortNameNew ) )
                warning( [ sprintf( 'A output block %s already exists.\n', outPortName ), ...
                     sprintf( 'The output port was renamed to %s.', outPortNameNew ) ] );
                 outPortName = outPortNameNew;
            end
            inPortNameNew = get_param( inPortBlkHandleParent, 'Name' );
            if not( isequal( inPortName, inPortNameNew ) )
                warning( [ sprintf( 'A input block %s already exists.\n', inPortName ), ...
                     sprintf( 'The input port was renamed to %s.', inPortNameNew ) ] );
                 inPortName = inPortNameNew;
            end

            if setColor
                set_param( outPortBlkHandleParent, 'BackgroundColor', 'red' );
                set_param( inPortBlkHandleParent, 'BackgroundColor', 'green' );
            end

            % Obtain the position for the in and out blocks
            inPortParentBlkPos = getHorizPosOf( destPort, inPortBlkHandleParent, -10 );
            outPortParentBlkPos = getHorizPosOf( srcPort, outPortBlkHandleParent, 10 );

            % Set the obtained position of the in and out block
            set_param( inPortBlkHandleParent, 'Position', inPortParentBlkPos );
            set_param( outPortBlkHandleParent, 'Position', outPortParentBlkPos );

            % Get the in and output handles
            outPortHandle = get_param( outPortBlkHandleParent, 'PortHandles' );
            inPortHandle = get_param( inPortBlkHandleParent, 'PortHandles' );

            % Connect in and output to the subsystem
            add_line( blkPathParent, inPortHandle.Outport, destPort, 'autorouting','on' );
            add_line( blkPathParent, srcPort, outPortHandle.Inport, 'autorouting','on' );

            blkPath = blkPathParent;
            blkPathParent = get_param( blkPath, 'Parent' );
            inPortBlkHandle = inPortBlkHandleParent;
            outPortBlkHandle = outPortBlkHandleParent;
        end
    end
end
